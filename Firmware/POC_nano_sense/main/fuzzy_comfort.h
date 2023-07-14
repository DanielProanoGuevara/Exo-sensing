#ifndef FUZZY_COMFORT_H
#define FUZZY_COMFORT_H

#include <Arduino.h>
#include <Fuzzy.h>

void setupFuzzy();

float get_comfort(float t_amb, float t_skin, float force);

#endif