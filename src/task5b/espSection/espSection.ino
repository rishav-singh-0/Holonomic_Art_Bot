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

// ssid: WiFi hotspot ssid
const char* ssid = "Dark_Demon";

// password: WiFi hotspot password
const char* password =  "apna_use_kar";            

// port: common port for ost and client
const uint16_t port = 8002;

//host: The ip address of laptop(base station) after connecting it to wifi hotspot
const char * host = "192.168.43.129";             

// previousMillis: store iteration time
unsigned long previousMillis = 0;

// interval: in milliseconds
unsigned long interval = 8000;

//client: WiFi Client object for socket communication
WiFiClient client;

//receivedMsg: String holder for received messages from socket connection
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
      //Read the message through the socket until new line char(\n)
      receivedMsg = client.readStringUntil('\n');     

      //Send an acknowledgement to host(laptop)
      client.print("Hello from ESP32!");              
      
      //Print data on Serial monitor
      Serial.println(receivedMsg);                    
      
      //Send data to AVR
      Serial1.println(receivedMsg);                   
      Serial1.flush();
  }
}
