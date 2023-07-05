#ifndef BT_LOW_ENERGY_H
#define BT_LOW_ENERGY_H

/***************INCLUDES**********************/
#include <Arduino.h>
#include <ArduinoBLE.h>
#include <arm_math.h>


void setupBluetoothLE();
void poll_BLE();

void updateAmbTemperature(float32_t AmbTemp);
void updateAmbPressure(float32_t AmbPres);
void updateAmbHumidity(float32_t AmbHum);

void updateMinBodyTemp(float32_t T);
void updateAvgBodyTemp(float32_t T);
void updateMaxBodyTemp(float32_t T);

void updateMinForce(float32_t F);
void updateAvgForce(float32_t F);
void updateMaxForce(float32_t F);

void updateONTime(uint32_t time);

void updateComfort(uint8_t comf);

#endif