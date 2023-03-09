/*
* Team Id: HB#1254
* Author List: Rishav
* Filename: espSection.ino
* Theme: Hola Bot -- Specific to eYRC 2022-23
* Functions: setup(), loop()
* Global Variables: 
*    ssid, password, port, host, previousMillis, interval, client, receivedMsg
*/

#include <WiFi.h>

#define BAUD_RATE 115200

// WiFi credentials
const char* ssid = "Dark_Demon";                   //WiFi hotspot ssid
const char* password =  "apna_use_kar";            //WiFi hotspot password
const uint16_t port = 8002;
const char * host = "192.168.43.129";             //The ip address of laptop(base station) after connecting it to wifi hotspot

unsigned long previousMillis = 0;
unsigned long interval = 8000;

WiFiClient client;

String receivedMsg = "0";

void setup(){
   
  Serial.begin(BAUD_RATE);                          //Serial to print data on Serial Monitor
  Serial.print("Hello Rishav!\n");
  Serial1.begin(BAUD_RATE, SERIAL_8N1, 33, 32);     //Serial to transfer data between ESP and AVR. The Serial connection is inbuilt.
    
  //Connecting to wifi
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("...");

    unsigned long currentMillis = millis();
    // if WiFi is down, try reconnecting
    if (currentMillis - previousMillis >= interval) {
      Serial.print(millis());
      Serial.println("Reconnecting to WiFi...");
      ESP.restart();
      previousMillis = currentMillis;
    }
  }
 
  Serial.print("WiFi connected with IP: ");
  Serial.println(WiFi.localIP());
}


void loop() {

  if (!client.connect(host, port)) {
    Serial.println("Connection to host failed");
    delay(200);
    return;
  }
  
  while(1){
      receivedMsg = client.readStringUntil('\n');     //Read the message through the socket until new line char(\n)
      client.print("Hello from ESP32!");              //Send an acknowledgement to host(laptop)
      Serial.println(receivedMsg);                    //Print data on Serial monitor
      Serial1.println(receivedMsg);                   //Send data to AVR
      Serial1.flush();
  }
}
