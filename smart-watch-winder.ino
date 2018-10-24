//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                          //
//                                            Smart Watch Winder                                            //
//                                                                                                          //
//                                    By Doug Youd (doug@cnidus.net).                                       //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Notes: 
// * Designed for Heltec WiFi Kit 32 w/ OLED (ESP32 based)
// * Heltec instructions: https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series#instructions
// * Driver ULN2003 and engine reduced to 1:64
// * Hardware based on 3d-printed winder: https://cults3d.com/en/3d-model/fashion/gyro-winder-watch-winder-remontoir-montre
// * Learnt how to drive OLED and get time from NTP: https://www.instructables.com/id/WiFi-Kit-32-NTP-Clock/
// * AWS IOT w/ ESP32 https://exploreembedded.com/wiki/AWS_IOT_with_Arduino_ESP32 && https://github.com/Schm1tz1/aws-sdk-arduino-esp8266/blob/master/examples/SimpleExample/SimpleExample.ino
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Includes.
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

// AWS IoT stuff
// #include <AWS_IOT.h>     //ExploreEmbedded version
//#include <WiFiClient.h>
//#include <HTTPClient.h> //ESP32 case
//#include <AmazonIOTClient.h>
//#include "EspAWSImplementations.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Constants.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Winder components
const int   STEPS =                        100;                                 // 100 steps... not sure why (yet)
const int   WINDER_ROTATIONS_PER_CYCLE =   20;                                  // # of full winder rotations each cycle
// For an engine of this type: http://tiptopboards.com/151-motor-not-not-relating-driving-5v-4-fils-driver-.html
// 64 steps per revolution, 4 phases, 5.625 ° angle according to engine specifications
// 1:64 gear ratio for this mechanically reduced motor
// 360 ° / 5.625 ° * 64 = 4096 angles with the gear ratio
// 360 ° / 5.625 ° * 64 * 4 coils / 2 bipolar = 2048 step / turn
const int   STEPS_PER_REVOLUTION =         4096;                                // 360 ° / 5.625 ° * 64 = 4096 angles with the gear ratio
const int   WINDER_SPEED =                 300;                                 // Range 100(slow, have high torque) > 300. Above this the motor vibrates without turning

// Wifi, NTP & OLED
const int   FONT_ONE_HEIGHT =              8;                                   // font one height in pixels
const int   FONT_TWO_HEIGHT =              20;                                  // font two height in pixels
const int   NTP_DELAY_COUNT =              20;                                  // delay count for ntp update
const int   NTP_PACKET_LENGTH =            48;                                  // ntp packet length
const int   TIME_ZONE =                    (-8);                                // offset from utc
const int   UDP_PORT  =                    4000;                                // UDP listen port

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Variables.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Winder components
int       WinderLoopCounter;
// The motor (wires 1 2 3 4) is connected to the outputs 8 9 10 11 of the Arduino (and on GND, + V)
Stepper   small_stepper(STEPS, 8, 10, 9, 11);                               // Counterclockwise by inverting 8 and 11 (if preferred)
int       Steps2Take =                    0;                                // Number of rotation steps requested from the motor

// Wifi, NTP & OLED
char      chBuffer[128];                                                    // general purpose character buffer
char      chPassword[] =                  "YourWifiPassword";               // your network password
char      chSSID[] =                      "YourWifiSsid";                   // your network SSID
bool      bTimeReceived =                 false;                            // time has not been received
U8G2_SSD1306_128X64_NONAME_F_HW_I2C       u8g2(U8G2_R0, 16, 15, 4);         // OLED graphics
int       nWifiStatus =                   WL_IDLE_STATUS;                   // wifi status
WiFiUDP   Udp;

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
  pinMode(5, OUTPUT); // Declare the Pin 5 as output for the LED

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Wifi, NTP & OLED

  // OLED graphics. 
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
 
//  // Wifi.
//    // Display title 
//    u8g2.clearBuffer();
//    sprintf(chBuffer, "%s", "Connecting to:");
//    u8g2.drawStr(64 - (u8g2.getStrWidth(chBuffer) / 2), 0, chBuffer);
//    sprintf(chBuffer, "%s", chSSID);
//    u8g2.drawStr(64 - (u8g2.getStrWidth(chBuffer) / 2), 31 - (FONT_ONE_HEIGHT / 2), chBuffer);
//    u8g2.sendBuffer();
//
//    // Connect to wifi.
//    Serial.print("NTP clock: connecting to wifi");
//    WiFi.begin(chSSID, chPassword);
//    while(WiFi.status() != WL_CONNECTED)
//    {
//        Serial.print(".");
//        delay(500);
//    }
//    Serial.println();
//    sprintf(chBuffer, "NTP clock: WiFi connected to %s.", chSSID);
//    Serial.println(chBuffer);
//    
//    // Display connection stats.
//      // Clean the display buffer.
//      u8g2.clearBuffer();
//      // Display the title.
//      sprintf(chBuffer, "%s", "WiFi Stats:");
//      u8g2.drawStr(64 - (u8g2.getStrWidth(chBuffer) / 2), 0, chBuffer);
//      // Display the ip address assigned by the wifi router.
//      char  chIp[81];
//      //WiFi.localIP().toString().toCharArray(chIp, sizeof(chIp) - 1);      //TODO: Fix this bug. May need staging IDE version
//      sprintf(chBuffer, "IP  : %s", chIp);
//      u8g2.drawStr(0, FONT_ONE_HEIGHT * 2, chBuffer);
//      // Display the ssid of the wifi router.
//      sprintf(chBuffer, "SSID: %s", chSSID);
//      u8g2.drawStr(0, FONT_ONE_HEIGHT * 3, chBuffer);
//      // Display the rssi.
//      sprintf(chBuffer, "RSSI: %d", WiFi.RSSI());
//      u8g2.drawStr(0, FONT_ONE_HEIGHT * 4, chBuffer);
//      // Display waiting for ntp message.
//      u8g2.drawStr(0, FONT_ONE_HEIGHT * 6, "Awaiting NTP time...");
//      // Now send the display buffer to the OLED.
//      u8g2.sendBuffer();
//
//  // Udp.
//  Udp.begin(UDP_PORT);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Main loop.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() 
{
  // Winder Loop code
  delay(100);  
  Serial.println("Winder running");  
  // Run the Winder
  small_stepper.setSpeed(WINDER_SPEED);

  if (WinderLoopCounter<=WINDER_ROTATIONS_PER_CYCLE){
    Serial.println(WinderLoopCounter);
    small_stepper.step(-STEPS_PER_REVOLUTION);  // Counter-Clockwise rotation
    Serial.println("Completed Counter-Clockwise winding");
    delay(2000);  //pause
      
    small_stepper.step(-STEPS_PER_REVOLUTION);  // Clockwise Rotation
    Serial.println("Completed Clockwise winding");
    delay(2000);  //pause
    WinderLoopCounter++; // Add 1 to the Counter
  }
  else{
    Serial.println("Done winding for a while. pausing....");
    delay(30000); // 1 minute delay?
    WinderLoopCounter = 0;
  }
}
