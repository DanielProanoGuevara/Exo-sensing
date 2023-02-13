#include <Arduino.h>

#define LED 0

void setup() {
  Serial.begin(115200);
  Serial.println("ESP Listening...");
  pinMode(LED, OUTPUT);
}

void loop() {
  if(Serial.available()){ //Listen to serial
    String command = Serial.readStringUntil('\n'); // Reads string untill \n
    if (command == "change")
    {
      Serial.println("LED change status");
      digitalWrite(LED, !digitalRead(LED));
    } 
  }
}