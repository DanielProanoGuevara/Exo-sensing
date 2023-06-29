#ifndef DSP_H
#define DSP_H


/***************INCLUDES**********************/
#include <Arduino.h>
#include <arm_math.h>

/***************FUNCTION PROTOTYPES**********************/
// Time Filter
void movingAverage(float32_t ringBuffer[],
                   uint8_t win_size,
                   uint8_t &i,
                   float32_t acq,
                   float32_t *average,
                   float32_t *min,
                   float32_t *max);

// Frequency Filter
void IIR(uint16_t Ain,
         float32_t x[],
         float32_t y[],
         float32_t A[],
         float32_t B[]);

// Regression
float32_t cubicFit(float32_t x, float32_t params[]);

// Convertions
float32_t vToTemp(float32_t v);
#endif