/* Comment this out to disable prints and save space */
// #define BLYNK_PRINT Serial

/* Fill-in your Template ID */
//#define BLYNK_TEMPLATE_ID   "YourTemplateID"
#define BLYNK_TEMPLATE_ID "TMPLddsQgzgp"
#define BLYNK_DEVICE_NAME "Temperature Control"
#define BLYNK_AUTH_TOKEN "QmZ0ozni5rHRDzknAr6cT6kGkhPQRmm8"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

char auth[] = "QmZ0ozni5rHRDzknAr6cT6kGkhPQRmm8";
char ssid[] = "HCMUS-VLDT-SV";
char pass[] = "svvldt38300595";

//Declare variables
uint8_t thershold_buffer[2] = {0}; 
int Temp = 0;
int Hot = 0;
int Cool = 0;
int Cold = 0;

void setup(){
  // Debug console
  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass);
  Blynk.syncVirtual(V1);
  Blynk.syncVirtual(V2);
}
BLYNK_WRITE(V1) {
  thershold_buffer[0] = param.asInt(); //Recieve aboveThreshold from Blynk
}
BLYNK_WRITE(V2) {
  thershold_buffer[1] = param.asInt(); //Recieve belowThreshold from Blynk
}

void loop(){
  Blynk.run();
  //Transmit Threshold from nodeMCU to ATMega
  Serial.write(thershold_buffer, 2);
  //Recieve Current Tempurature from ATMega
  if (Serial.available()){
    Temp = Serial.read();
  }
  //Transmit Current Tempurature to Blynk
  Blynk.virtualWrite(V3, Temp); 
  //Compare Current Tempurature with Threshold
  if (Temp > thershold_buffer[0]) {
    Hot = 1;
    Cool = 0;
    Cold = 0;
    Blynk.virtualWrite(V4, Hot);
    Blynk.virtualWrite(V5, Cold);
    Blynk.virtualWrite(V6, Cool);
  }
  if (thershold_buffer[1] < Temp &&  Temp < thershold_buffer[0]) {
    Hot = 0;
    Cool = 1;
    Cold = 0;
    Blynk.virtualWrite(V4, Hot);
    Blynk.virtualWrite(V5, Cold);
    Blynk.virtualWrite(V6, Cool);
  }
  if (Temp < thershold_buffer[1]){
    Hot = 0;
    Cool = 0;
    Cold = 1;
    Blynk.virtualWrite(V4, Hot);
    Blynk.virtualWrite(V5, Cold);
    Blynk.virtualWrite(V6, Cool);
  }
}
