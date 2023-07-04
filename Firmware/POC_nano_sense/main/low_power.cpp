#include "low_power.h"


void setupLowPower(){
  digitalWrite(LED_PWR, LOW); // "ON" LED turned off:
  digitalWrite(PIN_ENABLE_SENSORS_3V3, LOW); //PIN_ENABLE_SENSORS_3V3 set to LOW:
  digitalWrite(PIN_ENABLE_I2C_PULLUP, LOW); // PIN_ENABLE_I2C_PULLUP set to LOW:
}