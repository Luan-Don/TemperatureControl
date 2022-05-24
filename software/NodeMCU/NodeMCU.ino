
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
#define BLYNK_PRINT Serial

/* Fill-in your Template ID (only if using Blynk.Cloud) */
//#define BLYNK_TEMPLATE_ID   "YourTemplateID"
#define BLYNK_TEMPLATE_ID "TMPLddsQgzgp"
#define BLYNK_DEVICE_NAME "Temperature Control"
#define BLYNK_AUTH_TOKEN "QmZ0ozni5rHRDzknAr6cT6kGkhPQRmm8"


#include <ESP8266WiFi.h>
//#include <Blynk.h>
#include <BlynkExampleEsp8266.h>
//#include <BlynkEdgent.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "QmZ0ozni5rHRDzknAr6cT6kGkhPQRmm8";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Truc Mai 3";
char pass[] = "10031953456";
int Temp;
int aboveThres;
int belowThres;
uint8_t u8aboveThres, u8belowThres;

void setup()
{
  // Debug console
  Serial.begin(9600);

  Blynk.begin(auth, ssid, pass);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);
  Blynk.syncVirtual(V1);
  Blynk.syncVirtual(V2);
}
BLYNK_WRITE(V1) {
  aboveThres = param.asInt(); // Truyen tu Blynk xuong
}
BLYNK_WRITE(V2) {
  belowThres = param.asInt(); // Truyen tu Blynk xuong
}

void loop()
{
  Blynk.run();
  Serial.println(aboveThres | 0x00000100);
  Serial.println(belowThres | 0x00000100);
  if (Serial.available()) {
    Temp = Serial.read();
  }
  Blynk.virtualWrite(V0, Temp); // Truyen len Blynk
  if (Temp >= aboveThres){
    Hot = 1;
    Cold = 0;
    Cool = 0;
    Blynk.virtualWrite(V4, Hot);
    Blynk.virtualWrite(V5, Cold);
    Blynk.virtualWrite(V6, Cool);
  }
  if (belowThres < Temp &&  Temp < aboveThres) {
    Hot = 0;
    Cold = 0;
    Cool = 1;
    Blynk.virtualWrite(V4, Hot);
    Blynk.virtualWrite(V5, Cold);
    Blynk.virtualWrite(V6, Cool);
  }
    if(Temp <= belowThres){
    Hot = 0;
    Cold = 0;
    Cool = 1;
    Blynk.virtualWrite(V4, Hot);
    Blynk.virtualWrite(V5, Cold);
    Blynk.virtualWrite(V6, Cool);
  }
  
}
