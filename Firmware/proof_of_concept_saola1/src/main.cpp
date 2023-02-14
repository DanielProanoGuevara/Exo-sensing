#include <Arduino.h>
#include <HardwareSerial.h>

#define LED 0

// Use serial 1
#define RXD1 18
#define TXD1 17
HardwareSerial BLE (1);

void setup() {
  // Start serial com
  Serial.begin(115200);
  // Start serial 1
  BLE.begin(115200, SERIAL_8N1, RXD1, TXD1);
}

void loop() {
  //Serial pass through
  if (Serial.available()){
    BLE.write(Serial.read());
  }

  if (BLE.available()){
    Serial.write(BLE.read());
  }

}