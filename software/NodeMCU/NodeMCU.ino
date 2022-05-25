/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************
  This example runs directly on NodeMCU.

  Note: This requires ESP8266 support package:
    https://github.com/esp8266/Arduino

  Please be sure to select the right NodeMCU module
  in the Tools -> Board menu!

  For advanced settings please follow ESP examples :
   - ESP8266_Standalone_Manual_IP.ino
   - ESP8266_Standalone_SmartConfig.ino
   - ESP8266_Standalone_SSL.ino

  Change WiFi ssid, pass, and Blynk auth token to run :)
  Feel free to apply it to any other example. It's simple!
 *************************************************************/

/* Comment this out to disable prints and save space */
// #define BLYNK_PRINT Serial

/* Fill-in your Template ID (only if using Blynk.Cloud) */
//#define BLYNK_TEMPLATE_ID   "YourTemplateID"
#define BLYNK_TEMPLATE_ID "TMPLddsQgzgp"
#define BLYNK_DEVICE_NAME "Temperature Control"
#define BLYNK_AUTH_TOKEN "QmZ0ozni5rHRDzknAr6cT6kGkhPQRmm8"


#include <ESP8266WiFi.h>
//#include <Blynk.h>
#include <BlynkSimpleEsp8266.h>
//#include <BlynkEdgent.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "QmZ0ozni5rHRDzknAr6cT6kGkhPQRmm8";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "HCMUS-VLDT-SV";
char pass[] = "svvldt38300595";
//int aboveThres;
//int belowThres;
//uint8_t aboveThres;
//uint8_t belowThres = 15;
uint8_t thershold_buffer[2] = {0};
int Hot = 0; 
int Cool = 0; 
int Cold = 0;
int Temp = 0;
void setup()
{
  // Debug console
  Serial.begin(115200);

  Blynk.begin(auth, ssid, pass);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);
  Blynk.syncVirtual(V1);
  Blynk.syncVirtual(V2);
}
BLYNK_WRITE(V1) {
  thershold_buffer[0] = param.asInt(); // Truyen tu Blynk xuong
}
BLYNK_WRITE(V2) {
  thershold_buffer[1] = param.asInt(); //below
}

void loop()
{
  Blynk.run();

  //Transmit Threshold from nodeMCU to ATMega
  //Serial.println(aboveThres);
  Serial.write(thershold_buffer,2);

  //Recieve Current Tempurature from ATMega
    if (Serial.available())
    {
      Temp = Serial.read();
    }
  //   int testtemp = 0;
  //   testtemp = Temp;
//  Temp = 20;
  // Transmit Current Tempurature to Blynk
  Blynk.virtualWrite(V3, Temp);

  //Compare Curren Tempurature with Threshold
  if (Temp > thershold_buffer[0]) {
    Hot = 1;
    Cold = 0;
    Cool = 0;
    Blynk.virtualWrite(V4, Hot);
    Blynk.virtualWrite(V5, Cold);
    Blynk.virtualWrite(V6, Cool);
  }
  if (thershold_buffer[1] < Temp &&  Temp < thershold_buffer[0]) {
    Hot = 0;
    Cold = 0;
    Cool = 1;
    Blynk.virtualWrite(V4, Hot);
    Blynk.virtualWrite(V5, Cold);
    Blynk.virtualWrite(V6, Cool);
  }
  if (Temp < thershold_buffer[1]) {
    Hot = 0;
    Cold = 1;
    Cool = 0;
    Blynk.virtualWrite(V4, Hot);
    Blynk.virtualWrite(V5, Cold);
    Blynk.virtualWrite(V6, Cool);
  }

}
