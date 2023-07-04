#ifndef BT_LOW_ENERGY_H
#define BT_LOW_ENERGY_H

/***************INCLUDES**********************/
#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Arduino_HS300x.h>
#include <Arduino_LPS22HB.h>



void setupBluetoothLE();
void updateTemperature();
void updatePressure();
void updateHumidity();
void poll_BLE();

#endif