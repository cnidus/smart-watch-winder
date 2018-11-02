//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                          //
//                                            Smart Watch Winder                                            //
//                                                                                                          //
//                                    By Doug Youd (doug@cnidus.net).                                       //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Notes: 
// * Designed for Heltec WiFi Kit 32 w/ OLED (ESP32 based): https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series
// * Heltec Pinout: https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series/blob/master/PinoutDiagram/WIFI%20Kit%2032.pdf
// * Driver ULN2003 and engine reduced to 1:64
// * Winder hardware based on 3d-printed winder: https://cults3d.com/en/3d-model/fashion/gyro-winder-watch-winder-remontoir-montre
// * Learnt how to drive OLED and get time from NTP: https://www.instructables.com/id/WiFi-Kit-32-NTP-Clock/
// * AWS IOT w/ ESP32 https://exploreembedded.com/wiki/AWS_IOT_with_Arduino_ESP32
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Includes
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Winder components
#include                              <Stepper.h>

// Wifi, NTP & OLED
#include                              <time.h>                              // for time calculations
#include                              <WiFi.h>                              // for wifi
#include                              <WiFiUdp.h>                           // for udp via wifi
#include                              <U8g2lib.h>                           // see https://github.com/olikraus/u8g2/wiki/u8g2reference

// ArduinoCam Camera
// TODO: Figure this part out... doesnt look simple.

// AWS IoT stuff
#include <AWS_IOT.h>     //ExploreEmbedded version

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Constants.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Winder components
const int   STEPS =                        100;                                 // 100 steps (for stepper class)... not sure why (yet)
const int   WINDER_ROTATIONS_PER_CYCLE =   20;                                  // # of full winder rotations each cycle
const int   WINDER_DELAY_ROTATIONS =       3000;                                // Delay between each rotation
const int   WINDER_DELAY_CYCLE =           300000;                              // ~5m
const bool  WIND_BOTH_DIRECTION =          true;                                // Enable / disable winding counter-clockwise during each cycle
//TODO: Calculate Turns Per Day from above constants (4.5sec / rotation) @ speed 300)

// For an engine of this type: http://tiptopboards.com/151-motor-not-not-relating-driving-5v-4-fils-driver-.html
// 64 steps per revolution, 4 phases, 5.625 ° angle according to engine specifications
// 1:64 gear ratio for this mechanically reduced motor
// 360 ° / 5.625 ° * 64 = 4096 angles with the gear ratio
// 360 ° / 5.625 ° * 64 * 4 coils / 2 bipolar = 2048 step / turn
const int   STEPS_PER_REVOLUTION =         2048;                                // 360 ° / 5.625 ° * 64 = 4096 angles with the gear ratio
const int   WINDER_SPEED =                 300;                                 // Range 100(slow, have high torque) > 300. Above this the motor vibrates without turning

// Wifi, NTP & OLED
const int   FONT_ONE_HEIGHT =              8;                                   // font one height in pixels
const int   FONT_TWO_HEIGHT =              20;                                  // font two height in pixels
const int   NTP_DELAY_COUNT =              20;                                  // delay count for ntp update
const int   NTP_PACKET_LENGTH =            48;                                  // ntp packet length
const int   TIME_ZONE =                    (-8);                                // offset from utc
const int   UDP_PORT  =                    4000;                                // UDP listen port (for NTP)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Variables.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Winder components
int       WinderLoopCounter=               0;
char      WinderStatus[]=                  "Idle";
// The motor (wires 1 2 3 4) is connected to the GPIO 32 12 13 14 of the ESP32 (and on GND, +5v)
Stepper   winder(STEPS, 14, 12, 13, 32);                                    // Counterclockwise by inverting 3 and 14 (if preferred)

// Wifi, NTP & OLED
char      chBuffer[128];                                                    // general purpose character buffer
char      chPassword[] =                  "<YourWifiPasswordHere>";         // your network password
char      chSSID[] =                      "<YourWifiSSIDHere>";             // your network SSID
bool      bTimeReceived =                 false;                            // time has not been received
U8G2_SSD1306_128X64_NONAME_F_HW_I2C       u8g2(U8G2_R0, 16, 15, 4);         // OLED graphics: Pins GPIO16 (OLED_RST), GPIO15 (OLED_SCL) & GPIO4 (OLED_SDA)
int       nWifiStatus =                   WL_IDLE_STATUS;                   // wifi status
WiFiUDP   Udp;                                                              // Used for NTP

// AWS IoT stuff
AWS_IOT   iot;
char      IOT_HOST_ADDRESS[]=             "<YourAWSIoTHere>.iot.us-east-1.amazonaws.com";     // "AWS host address"
char      IOT_CLIENT_ID[]=                "SmartWatchWinder";                                 // "client id"
char      IOT_PUB_TOPIC_NAME[]=           "$aws/things/SmartWatchWinder/WinderStats";         // "your thing/topic name"
char      IOT_SUB_TOPIC_NAME[]=           "$aws/things/SmartWatchWinder/CloudStats";          // "your thing/topic name"
int       msgCount=0,msgReceived = 0;                                                         // Counts of MQTT messages sent/received
char      payload[512];                                                                       // MQTT publish payload
char      rcvdPayload[512];                                                                   // MQTT subscribe payload
int       IoTWinderCount=                 0;                                                  // Count of watch winder cycles since last successful publish
char      IoTStatus[]=                    "Not Connected";                                    // String for the IoT Status to display


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// Handlers
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
void mySubCallBackHandler (char *topicName, int payloadLen, char *payLoad) // For AWS IoT
{
    strncpy(rcvdPayload,payLoad,payloadLen);
    rcvdPayload[payloadLen] = 0;
    msgReceived = 1;
}

void checkIotStatus ()
{
    //TODO: Implement this, to reconnect when IoT drops out during operation.
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// Setup.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////  
void setup() 
{
  // Serial
  Serial.begin(115200);
  while(!Serial)
  {
    Serial.print('.');
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Winder Setup
  Serial.println("Initializing up smart watch winder"); 
  //pinMode(5, OUTPUT); // Declare the Pin 5 as output for the LED

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Wifi, NTP & OLED

  // OLED graphics. 
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
 
  // Wifi.
    // Display title 
    u8g2.clearBuffer();
    sprintf(chBuffer, "%s", "Connecting to:");
    u8g2.drawStr(64 - (u8g2.getStrWidth(chBuffer) / 2), 0, chBuffer);
    sprintf(chBuffer, "%s", chSSID);
    u8g2.drawStr(64 - (u8g2.getStrWidth(chBuffer) / 2), 31 - (FONT_ONE_HEIGHT / 2), chBuffer);
    u8g2.sendBuffer();

    // Connect to wifi.
    Serial.print("SmartWatchWinder: Connecting to wifi");
    WiFi.begin(chSSID, chPassword);
    while(WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(500);
    }
    Serial.println();
    sprintf(chBuffer, "SmartWatchWinder: WiFi connected to %s.", chSSID);
    Serial.println(chBuffer);

//  // Udp.
//  Udp.begin(UDP_PORT);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // AWS IoT
  u8g2.clearBuffer();
  sprintf(chBuffer, "%s", "Connecting to IoT:");
  u8g2.drawStr(64 - (u8g2.getStrWidth(chBuffer) / 2), 0, chBuffer);
  sprintf(chBuffer, "%s", IOT_HOST_ADDRESS);
  u8g2.drawStr(64 - (u8g2.getStrWidth(chBuffer) / 2), 31 - (FONT_ONE_HEIGHT / 2), chBuffer);
  u8g2.sendBuffer();
  
  if(iot.connect(IOT_HOST_ADDRESS,IOT_CLIENT_ID)== 0)
  {
      Serial.println("Connected to AWS");
      delay(1000);

      if(0==iot.subscribe(IOT_SUB_TOPIC_NAME,mySubCallBackHandler))
      {
          Serial.println("Subscribe Successfull");
          sprintf(IoTStatus, "Sub Success");
      }
      else
      {
          Serial.println("Subscribe Failed, Check the Thing Name and Certificates");
          sprintf(IoTStatus, "Sub Fail");
          while(1);
      }
  }
  else
  {
      Serial.println("AWS connection failed, Check the HOST Address");
      sprintf(IoTStatus, "Conn Fail");
      while(1);
  }

  // Display setup status to OLED.
  u8g2.clearBuffer();

  sprintf(chBuffer, "%s", "Setup status:");
  u8g2.drawStr(64 - (u8g2.getStrWidth(chBuffer) / 2), 0, chBuffer);     // Display the title.

  char  chIp[81];
  WiFi.localIP().toString().toCharArray(chIp, sizeof(chIp) - 1);      // Note this works on ESP32, but not 
  sprintf(chBuffer, "IP  : %s", chIp);
  u8g2.drawStr(0, FONT_ONE_HEIGHT * 2, chBuffer);                     // Display the ip address assigned by the wifi router.
  sprintf(chBuffer, "SSID: %s", chSSID);                              // Display the ssid of the wifi router.
  u8g2.drawStr(0, FONT_ONE_HEIGHT * 3, chBuffer);
  sprintf(chBuffer, "IoT: %s", IoTStatus);                            // Display IoT Connection status
  u8g2.drawStr(0, FONT_ONE_HEIGHT * 4, chBuffer);  
  // Now send the display buffer to the OLED.
  u8g2.sendBuffer();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Main loop.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() 
{
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //  Winder Loop code
  delay(100);  
  Serial.println("Winder running");
  winder.setSpeed(WINDER_SPEED);                                            // Run the Winder

  if (WinderLoopCounter<=WINDER_ROTATIONS_PER_CYCLE)
  {
      sprintf(WinderStatus, "Winding");
      Serial.println(WinderLoopCounter);
      winder.step(STEPS_PER_REVOLUTION);                                   // Counter-Clockwise rotation
      Serial.println("Completed Clockwise winding");
      WinderLoopCounter++;                                                  // Add 1 to the Counter  
      if (WIND_BOTH_DIRECTION)
      {  
          delay(WINDER_DELAY_ROTATIONS);
          winder.step(-STEPS_PER_REVOLUTION);                                   // Clockwise Rotation
          Serial.println("Completed Counter-Clockwise winding");
          WinderLoopCounter++;
      }
  }
  else
  {
      Serial.println("Done winding for a while. Sending stats to IoT....");
      sprintf(WinderStatus, "Idle");
      IoTWinderCount += WinderLoopCounter;
      WinderLoopCounter = 0;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // AWS IoT 
  
  // publish to topic when the cycle completes, or if there are outstanding winds that have not been reported.
  if (IoTWinderCount > 0)
  {
      sprintf(payload,"Rotations: %d", IoTWinderCount);
      msgCount++;
      if(iot.publish(IOT_PUB_TOPIC_NAME,payload) == 0)
      {        
          Serial.print("Publish Message:");
          Serial.println(payload);
          sprintf(IoTStatus, "Pub Success");
          IoTWinderCount = 0;                 // Reset the IoT count once successfully reported
      }
      else
      {
          Serial.println("Publish failed");
          sprintf(IoTStatus, "Pub Failed");
          Serial.println(payload);
      }
  }
  
  //TODO: Parse and display the received message. Thinking it will contain WatchId (fingerprint of the watchface), observed skew, accuracy over period etc.
  // For now, just print it to serial
  if(msgReceived == 1)
  {
      msgReceived = 0;
      Serial.print("Received Message:");
      Serial.println(rcvdPayload);
      sprintf(IoTStatus, "msg rcvd");
  }

  // Update OLED.
  u8g2.clearBuffer();                                                   // Clean the display buffer.
  sprintf(chBuffer, "%s", "SmartWinder Status");
  u8g2.drawStr(64 - (u8g2.getStrWidth(chBuffer) / 2), 0, chBuffer);     // Display the title.
  sprintf(chBuffer, "Winder: %s", WinderStatus);                         // Display Winder status
  u8g2.drawStr(0, FONT_ONE_HEIGHT * 2, chBuffer);      
  sprintf(chBuffer, "Rotations: %d", WinderLoopCounter);                // Display Winder count
  u8g2.drawStr(0, FONT_ONE_HEIGHT * 3, chBuffer);
  sprintf(chBuffer, "IoT: %s", IoTStatus);                              // Display IoT Status
  u8g2.drawStr(0, FONT_ONE_HEIGHT * 4, chBuffer);
  u8g2.sendBuffer();                                                    // Now send the display buffer to the OLED.

  
  // All done! Fire ze missiles!... but I am le tired.... ok, first have a nap. THEN FIRE ZE MISSILES!
  if (WinderLoopCounter == 0)
  {
      Serial.println("Idle for a while");    
      delay(WINDER_DELAY_CYCLE);   // Pre determined delay between cycles.

  }
  else
  {
      Serial.println("Delay between rotations");    
      delay(WINDER_DELAY_ROTATIONS);                         // delay between loops.
  }
}
