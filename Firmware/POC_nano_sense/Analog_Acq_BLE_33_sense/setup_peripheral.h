#ifndef SETUP_PERIPHERAL_H
#define SETUP_PERIPHERAL_H


/***************INCLUDES**********************/
#include <Arduino.h>
#include <nrfx.h>
#include <nrf.h>
#include <nrf_timer.h>
#include <SPI.h>

#include "dsp.h"
#include "sys.h"

/***************FUNCTION PROTOTYPES**********************/
void setupTimer();
#ifdef DEBUG_DAC
void setupDAC();
void writeDAC(uint16_t val);
#endif


#endif