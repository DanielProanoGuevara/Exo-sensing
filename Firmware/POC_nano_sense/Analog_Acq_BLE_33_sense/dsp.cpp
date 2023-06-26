#include "dsp.h"

static float32_t int_res;  // Auxiliar dot product result for IIR filter
static uint32_t idx;       //  Auxiliar min and max index (not used)

void movingAverage(float32_t ringBuffer[],
                   uint8_t win_size,
                   uint8_t &i,
                   float32_t acq,
                   float32_t *average,
                   float32_t *min,
                   float32_t *max) {

  if (i == win_size) i = 0;  // Resets the iterator
  ringBuffer[i] = acq;       // Replaces the oldest element of the ring buffer with the new one
  arm_mean_f32(ringBuffer, win_size, average);
  arm_min_f32(ringBuffer, win_size, min, &idx);
  arm_max_no_idx_f32(ringBuffer, win_size, max);
  i++;  // Move the iterator
}


void IIR(uint16_t Ain,
         float32_t x[],
         float32_t y[],
         float32_t A[],
         float32_t B[]) {
  // Convert from uint16 to voltage value
  x[0] = Ain * (3.3f / 4096.0f);  // Current input value

  // calc. the output
  arm_dot_prod_f32(B, x, 7, &int_res);  // Intermediate calculation
  y[0] = (-A[1] * y[1] - A[2] * y[2] - A[3] * y[3] - A[4] * y[4] - A[5] * y[5] - A[6] * y[6]
          + int_res);
  // Propagate inputs
  x[6] = x[5];
  x[5] = x[4];
  x[4] = x[3];
  x[3] = x[2];
  x[2] = x[1];
  x[1] = x[0];
  // Propagate outpus
  y[6] = y[5];
  y[5] = y[4];
  y[4] = y[3];
  y[3] = y[2];
  y[2] = y[1];
  y[1] = y[0];
}





float32_t cubicFit(float32_t x, float32_t params[]) {
  return params[0] * powf(x, 3) + params[1] * powf(x, 2) + params[2] * x + params[3];
}

float32_t vToTemp(float32_t v) {
  float32_t aux;
  aux = v + (1581 / 5050);
  aux *= (607 / 130680);
  aux = logf(aux);
  aux /= 3895.633;
  aux -= (20 / 5463);
  aux = (1 / aux);
  return aux + 273.15f;
}