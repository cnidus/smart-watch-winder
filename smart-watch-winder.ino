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
#include                              <Stepper.h>                           // Stepper motor driver
#include                              <math.h>                              // Used for roundup etc
#include                              <string>                              // Honestly.... no default string type.. fuuuu C. :P

// Wifi, NTP & OLED
#include                              <time.h>                              // for time calculations
#include                              <WiFi.h>                              // for wifi
#include                              <WiFiUdp.h>                           // for udp via wifi
#include                              <U8g2lib.h>                           // see https://github.com/olikraus/u8g2/wiki/u8g2reference

// ArduinoCam OV2640 Camera
// TODO: Figure this part out... doesnt look simple.
#include                              <esp_camera.h>                        // Thanks expressif! https://github.com/espressif/esp32-camera
#include                              <esp_http_server.h>
#include                              <esp_timer.h>
#include                              <esp_err.h>                           // May need to do as a fully defined path?


// AWS IoT stuff
#include                              <AWS_IOT.h>     //ExploreEmbedded version

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Constants.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Winder components
const int   STEPS =                        100;                                 // 100 steps (for stepper class)... not sure why (yet)
const int   TARGET_TPD =                   650;                                 // Select Turns per day for your watch. https://www.barringtonwatchwinders.com/us/turns-per-day/
const bool  WIND_BOTH_DIRECTION =          true;                                // Enable / disable winding counter-clockwise during each cycle
const int   WINDER_DELAY_ROTATIONS =       1000;                                // Delay between winding directions
const unsigned long   WINDER_DELAY_CYCLE =           900000;                              // Delay between each cycle in ms. (900000 = 15min)

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
const int   NTP_SEND_DELAY =               3600000;                             // Delay between each cycle in ms. (3600000 = 60min)
const int   NTP_CHECK_DELAY =              1000;                                // Delay between NTP packet send and checking for receive
const int   NTP_PACKET_LENGTH =            48;                                  // ntp packet length
const int   TIME_ZONE =                    (-8);                                // offset from utc
const int   UDP_PORT  =                    4000;                                // UDP listen port (for NTP)

// ArduinoCam OV2640 Camera
#define CAM_PIN_PWDN    -1 //power down is not used
#define CAM_PIN_RESET   2 //software reset will be performed
#define CAM_PIN_XCLK    27
#define CAM_PIN_SIOD    25
#define CAM_PIN_SIOC    23

#define CAM_PIN_D7      19
#define CAM_PIN_D6      36
#define CAM_PIN_D5      18
#define CAM_PIN_D4      39
#define CAM_PIN_D3       5
#define CAM_PIN_D2      34
#define CAM_PIN_D1      17
#define CAM_PIN_D0      35
#define CAM_PIN_VSYNC   22
#define CAM_PIN_HREF    26
#define CAM_PIN_PCLK    21


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Variables.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Winder components
int       WinderLoopCounter=               0;
char      WinderStatus[]=                  "Idle";
unsigned long prevCycle =                  WINDER_DELAY_CYCLE;              // will store last time winder was run
int       WinderTurnsPerCycle =            1;                               // # of full winder rotations each cycle. Gets updated according to TARGET_TPD & WINDER_CYCLE_DELAY in the code
unsigned long WinderTotalTurns =           0;                               // Keep track of total winds TODO: s
// The motor (wires 1 2 3 4) is connected to the GPIO 14 12 13 32 of the ESP32 (and on GND, +5v)
Stepper   winder(STEPS, 14, 12, 13, 32);                                    // Counterclockwise by inverting the pins (if preferred)

// Wifi, NTP & OLED
char      chBuffer[128];                                                    // general purpose character buffer
unsigned long prevNTPUpdate =             NTP_SEND_DELAY;                   // will store last time winder was run
unsigned long NTPLastSent =               0;                                // When the NTP packet was previously sent
char      chPassword[] =                  "<YourWifiPasswordHere>";         // your network password
char      chSSID[] =                      "<YourWifiSSIDHere>";             // your network SSID
bool      bTimeReceived =                 false;                            // time has not been received
bool      NTPPacketSent =                 false;                            // Havent yet got NTP time.
U8G2_SSD1306_128X64_NONAME_F_HW_I2C       u8g2(U8G2_R0, 16, 15, 4);         // OLED graphics: Pins GPIO16 (OLED_RST), GPIO15 (OLED_SCL) & GPIO4 (OLED_SDA)
int       nWifiStatus =                   WL_IDLE_STATUS;                   // wifi status
WiFiUDP   Udp;                                                              // Used for NTP

// AWS IoT stuff
AWS_IOT   iot;
char      IOT_HOST_ADDRESS[]=             "<YourAWSIoTHere>.iot.us-east-1.amazonaws.com";     // "AWS host address"
char      IOT_CLIENT_ID[]=                "SmartWatchWinder";                                 // "client id"
char      IOT_PUB_TOPIC_NAME[]=           "SmartWatchWinder/WinderStats";                     // "your thing/topic name"
char      IOT_SUB_TOPIC_NAME[]=           "SmartWatchWinder/CloudStats";                      // "your thing/topic name"
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
    if(iot.connect(IOT_HOST_ADDRESS,IOT_CLIENT_ID)== 0)
    {
        Serial.println("Connected to AWS");
        sprintf(IoTStatus, "Connected");
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
}

void updateTime()
{
  // Local variables.
  static  byte  chNtpPacket[NTP_PACKET_LENGTH];
  unsigned long currentMillis = millis();                      // Time right now

  // Check for time to send ntp request.
  if(!NTPPacketSent)
  {
      // Send ntp time request.
      // Initialize ntp packet.
      memset(chNtpPacket, 0, NTP_PACKET_LENGTH);          // Zero out chNtpPacket.

      // Set the ll (leap indicator), vvv (version number) and mmm (mode) bits.
      // These bits are contained in the first byte of chNtpPacker and are in
      // the following format:  llvvvmmm
      //
      // where:
      //    ll  (leap indicator) = 0
      //    vvv (version number) = 3
      //    mmm (mode)           = 3

      chNtpPacket[0]  = 0b00011011;

      // Send the ntp packet.
      IPAddress ipNtpServer(129, 6, 15, 29);              // https://tf.nist.gov/tf-cgi/servers.cgi
      Udp.beginPacket(ipNtpServer, 123);
      Udp.write(chNtpPacket, NTP_PACKET_LENGTH);
      Udp.endPacket();

      NTPPacketSent = true;
      bTimeReceived = false;
      NTPLastSent = currentMillis;

      Serial.println("NTP clock: ntp packet sent to ntp server.");
      Serial.print("NTP clock: awaiting response from ntp server");
  }
  Serial.print("*");

  if (currentMillis - NTPLastSent >= NTP_CHECK_DELAY)
  {
      // Time to check for a server response.
      if(Udp.parsePacket())
      {
          // Server responded, read the packet.
          Udp.read(chNtpPacket, NTP_PACKET_LENGTH);

          // Obtain the time from the packet, convert to Unix time, and adjust for the time zone.
          struct  timeval tvTimeValue = {0, 0};
          tvTimeValue.tv_sec = ((unsigned long)chNtpPacket[40] << 24) +       // bits 24 through 31 of ntp time
                               ((unsigned long)chNtpPacket[41] << 16) +       // bits 16 through 23 of ntp time
                               ((unsigned long)chNtpPacket[42] <<  8) +       // bits  8 through 15 of ntp time
                               ((unsigned long)chNtpPacket[43]) -             // bits  0 through  7 of ntp time
                               (((70UL * 365UL) + 17UL) * 86400UL) +          // ntp to unix conversion
                               (TIME_ZONE * 3600UL) +                         // time zone adjustment
                               (5);                                           // transport delay fudge factor

          settimeofday(& tvTimeValue, NULL);                                  // Set the ESP32 rtc.
          bTimeReceived = true;                                               // Time has been received.
          NTPPacketSent = false;

          // Output date and time to serial.
          struct tm * tmPointer = localtime(& tvTimeValue.tv_sec);
          strftime (chBuffer, sizeof(chBuffer), "%a, %d %b %Y %H:%M:%S",  tmPointer);
          bTimeReceived = true;
          prevNTPUpdate = currentMillis;
          Serial.println();
          Serial.print("NTP clock: response received, time written to ESP32 rtc: ");
          Serial.println(chBuffer);
      }
      else
      {
          // Server did not respond.
          Serial.println("NTP clock: packet not received.");
          NTPLastSent = currentMillis;                                        // Delay for another cycle
      }
  }
}


// ArduinoCam OV2640 Camera
static camera_config_t camera_config = {
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,//YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_UXGA,//QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1 //if more than one, i2s runs in continuous mode. Use only with JPEG
};

esp_err_t camera_init(){
    //power up the camera if PWDN pin is defined
    if(CAM_PIN_PWDN != -1){
        pinMode(CAM_PIN_PWDN, OUTPUT);
        digitalWrite(CAM_PIN_PWDN, LOW);
    }

    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}

esp_err_t camera_capture(){
    //acquire a frame
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera Capture Failed");
        return ESP_FAIL;
    }
    //replace this with your own function
    process_image(fb->width, fb->height, fb->format, fb->buf, fb->len);

    //return the frame buffer back to the driver for reuse
    esp_camera_fb_return(fb);
    return ESP_OK;
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

  // Udp for NTP
  Udp.begin(UDP_PORT);
  updateTime();

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
  u8g2.drawStr(64 - (u8g2.getStrWidth(chBuffer) / 2), 0, chBuffer);   // Display the title.

  char  chIp[81];
  WiFi.localIP().toString().toCharArray(chIp, sizeof(chIp) - 1);      // Note: this works on ESP32, but not reguar arduino boards
  sprintf(chBuffer, "IP  : %s", chIp);
  u8g2.drawStr(0, FONT_ONE_HEIGHT * 2, chBuffer);                     // Display the ip address assigned by the wifi router.
  sprintf(chBuffer, "SSID: %s", chSSID);                              // Display the ssid of the wifi router.
  u8g2.drawStr(0, FONT_ONE_HEIGHT * 3, chBuffer);
  sprintf(chBuffer, "IoT: %s", IoTStatus);                            // Display IoT Connection status
  u8g2.drawStr(0, FONT_ONE_HEIGHT * 4, chBuffer);
  u8g2.sendBuffer();                                                  // Now send the display buffer to the OLED.
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Main loop.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  unsigned long currentMillis = millis();                      // Time right now

  delay(100);

  if (currentMillis - prevNTPUpdate >= NTP_SEND_DELAY)
  {
      updateTime();
      if (bTimeReceived)
      {
          prevNTPUpdate == currentMillis;
      }
  }


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //  Winder Loop code

  if (currentMillis - prevCycle >= WINDER_DELAY_CYCLE)
  {
      Serial.println("Winder running");
      winder.setSpeed(WINDER_SPEED);                                                             // Run the Winder

      //Calculate number of steps per cycle based on params.
      int CyclesPerDay = (86400000 / WINDER_DELAY_CYCLE);
      WinderTurnsPerCycle = ceil(TARGET_TPD / CyclesPerDay);

      if (WinderLoopCounter < WinderTurnsPerCycle)
      {
          sprintf(WinderStatus, "Winding");
          if (WIND_BOTH_DIRECTION)
          {
              if (WinderLoopCounter < (WinderTurnsPerCycle /2))                                  // Wind clockwise for first half of WinderTurnsPerCycle
              {
                  winder.step(STEPS_PER_REVOLUTION);                                             // Clockwise rotation
                  Serial.println("Completed Clockwise winding");
                  WinderLoopCounter++;                                                           // Add 1 to the Counter
                  WinderTotalTurns++;;
              }
              else
              {
                      delay(WINDER_DELAY_ROTATIONS);
                      winder.step(-STEPS_PER_REVOLUTION);                                        // Counter-Clockwise Rotation
                      Serial.println("Completed Counter-Clockwise winding");
                      WinderLoopCounter++;                                                       // Add 1 to the Counter
                      WinderTotalTurns++;;
              }
          }
          else
          {
              winder.step(STEPS_PER_REVOLUTION);                                                 // Clockwise rotation
              Serial.println("Completed Clockwise winding");
              WinderLoopCounter++;                                                               // Add 1 to the Counter
              WinderTotalTurns++;;
          }
      }
      else
      {
          Serial.println("Done winding for a while. Sending stats to IoT....");
          sprintf(WinderStatus, "Idle");
          IoTWinderCount += WinderLoopCounter;                                                   // Keep a count of turns to publish to IoT
          //WinderTotalTurns += WinderLoopCounter;                                                 // Keep a total count since last powered on. //TODO: Make this a daily count
          WinderLoopCounter = 0;
          prevCycle = currentMillis;                                                             // save the last time you started winding
      }
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // AWS IoT

  // publish to topic when the cycle completes, or if there are outstanding winds that have not been reported.
  if (IoTWinderCount > 0)
  {
      // checkIotStatus();                       // Reconnect if need be.
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

  /////////// Update OLED ///////////////////////////
  //
  u8g2.clearBuffer();                                                 // Clean the display buffer.
  if(bTimeReceived)
  {
      // Ntp time has been received, ajusted and written to the ESP32 rtc, so obtain the time from the ESP32 rtc.
      struct  timeval tvTimeValue;
      gettimeofday(& tvTimeValue, NULL);

      // Obtain a pointer to local time.
      struct tm * tmPointer = localtime(& tvTimeValue.tv_sec);

      // Display the date.
//      strftime(chBuffer, sizeof(chBuffer), "%a, %d %b %Y",  tmPointer);

      // Display the time.
      strftime(chBuffer, sizeof(chBuffer), "%I:%M:%S",  tmPointer);
  }
  else
  {
      sprintf(chBuffer, "%s", "SmartWinder Status");
  }
  u8g2.drawStr(64 - (u8g2.getStrWidth(chBuffer) / 2), 0, chBuffer);     // Display the title.
  sprintf(chBuffer, "Winder: %s", WinderStatus);                        // Display Winder status
  u8g2.drawStr(0, FONT_ONE_HEIGHT * 2, chBuffer);
  sprintf(chBuffer, "Turns: %d", WinderTotalTurns);                     // Display Winder count
  u8g2.drawStr(0, FONT_ONE_HEIGHT * 3, chBuffer);
  sprintf(chBuffer, "IoT: %s", IoTStatus);                              // Display IoT Status
  u8g2.drawStr(0, FONT_ONE_HEIGHT * 4, chBuffer);

  if (WinderLoopCounter)
  {
      sprintf(chBuffer, "Cycle: ");                                                                             // Display cycle progress bar
      u8g2.drawStr(0, FONT_ONE_HEIGHT * 5, chBuffer);
      float temp = WinderLoopCounter;
      int BarWidth = ceil((128- u8g2.getStrWidth(chBuffer)) * (temp / WinderTurnsPerCycle));                    // Calculate box size based on delay timer.
      Serial.println(BarWidth);
      u8g2.drawBox(u8g2.getStrWidth(chBuffer), (FONT_ONE_HEIGHT * 5) + 1, (BarWidth), (FONT_ONE_HEIGHT -1));    // Draw a progress bar
  }
  else
  {
      sprintf(chBuffer, "Idle: ");                                                                              // Display idle progress bar
      u8g2.drawStr(0, FONT_ONE_HEIGHT * 5, chBuffer);
      float temp = (currentMillis - prevCycle);
      int BarWidth = ceil((128- u8g2.getStrWidth(chBuffer)) * (temp / WINDER_DELAY_CYCLE));                     // Calculate box size based on delay timer.
      Serial.println(BarWidth);
      u8g2.drawBox(u8g2.getStrWidth(chBuffer), (FONT_ONE_HEIGHT * 5) + 1, (BarWidth), (FONT_ONE_HEIGHT -1));    // Draw a progress bar
  }
  u8g2.sendBuffer();                                                                                            // Now send the display buffer to the OLED.


  // All done! Fire ze missiles!... but I am le tired.... ok, first have a nap. THEN FIRE ZE MISSILES!
//  Serial.print(".");

}
