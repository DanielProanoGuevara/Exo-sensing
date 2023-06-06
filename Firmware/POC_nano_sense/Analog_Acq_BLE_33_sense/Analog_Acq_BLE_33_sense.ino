#include "Arduino.h"
#include <nrfx.h>
#include <nrf.h>
#include "nrf_timer.h"
#include "arm_math.h"

#include <Arduino_HS300x.h>
#include <Arduino_LPS22HB.h>

#include "SPI.h"

// Pin  declarations
#define DEBUG 2
#define sync 5


// Variables 
uint8_t decimationCounter = 1;
float32_t FSR1_filt = 0.0f;
uint16_t DAC_o = 0;

float32_t int_res; // Auxiliar dot product result for IIR filter
uint32_t idx; //  Auxiliar min and max index (not used)

// Final pollable variables
//FSR
float32_t interfaceingForceMean = 0.0f;
float32_t interfaceingForceMin = 0.0f;
float32_t interfaceingForceMax = 0.0f;
//NTC
float32_t skinTemperatureMean = 0.0f;
float32_t skinTemperatureMin = 0.0f;
float32_t skinTemperatureMax = 0.0f;
//Fluids
float32_t precissionForceMean = 0.0f;
float32_t precissionForceMin = 0.0f;
float32_t precissionForceMax = 0.0f;
//Use time
uint32_t seconds = 0;
uint8_t click = 1;
//Environmental values
float32_t amb_baro_pressure = 0.0f;
float32_t amb_temperature = 0.0f;
float32_t amb_humidity = 0.0f;





// Filter variables
// *** IIR FSR coefficients
float32_t B_FSR[7] = {0.00034054f, 0.00204323f, 0.00510806f, 0.00681075f, 0.00510806f, 0.00204323f, 0.00034054f};
float32_t A_FSR[7] = {1.0f,        -3.5794348f, 5.65866717f,-4.96541523f, 2.52949491f,-0.70527411f, 0.08375648f};

// *** IIR NTC coefficients
float32_t B_NTC[7] = {1.41440730E-11f, 8.48644379E-11f, 2.12161095E-10f, 2.82881460E-10f, 2.12161095E-10f, 8.48644379E-11f, 1.41440730E-11f};
float32_t A_NTC[7] = {1.0f,           -5.87861916f,     14.40044053f,   -18.81528973f,    13.82942474f,   -5.42164649f,     0.88569011f};

float32_t FSR1_x[7] = {0.0f}; // Input memory buffer
float32_t FSR1_y[7] = {0.0f}; // Output memory buffer
float32_t FSR2_x[7] = {0.0f};
float32_t FSR2_y[7] = {0.0f};
float32_t FSR3_x[7] = {0.0f};
float32_t FSR3_y[7] = {0.0f};
float32_t FSR4_x[7] = {0.0f};
float32_t FSR4_y[7] = {0.0f};
float32_t NTC1_x[7] = {0.0f};
float32_t NTC1_y[7] = {0.0f};
float32_t Fluids1_x[7] = {0.0f};
float32_t Fluids1_y[7] = {0.0f};

// *** MA ring buffer and variables

const uint8_t M = 20; // Window size
float32_t average_FSR[4] = {0.0f};
float32_t min_FSR[4] = {0.0f};
float32_t max_FSR[4] = {0.0f};

float32_t FSR_mean;
float32_t FSR_min;
float32_t FSR_max;

float32_t MA_ring_buffer_FSR1 [M] = {0.0f}; // FSR1 moving average ring buffer
uint8_t RB_i_FSR1 = 0; // FSR1 Ring buffer iterator

float32_t MA_ring_buffer_FSR2 [M] = {0.0f};
uint8_t RB_i_FSR2 = 0;

float32_t MA_ring_buffer_FSR3 [M] = {0.0f};
uint8_t RB_i_FSR3 = 0;

float32_t MA_ring_buffer_FSR4 [M] = {0.0f};
uint8_t RB_i_FSR4 = 0;

float32_t MA_ring_buffer_NTC1 [M] = {0.0f};
float32_t average_NTC1 = 0.0f;
float32_t min_NTC1 = 0.0f;
float32_t max_NTC1 = 0.0f;
uint8_t RB_i_NTC1 = 0;

float32_t MA_ring_buffer_Fluids1 [M] = {0.0f};
float32_t average_Fluids1 = 0.0f;
float32_t min_Fluids1 = 0.0f;
float32_t max_Fluids1 = 0.0f;
uint8_t RB_i_Fluids1 = 0;



// Regression constants for the FSR in the form y = ax^3 + bx^2 + cx + d
float32_t cubic_params_FSR[4] = {4.6029917878732345f, -14.025875655473554f, 26.84085391232573f, -6.390769522330579};


// Analog acquisition
uint16_t FSR1_raw = 0;
uint16_t FSR2_raw = 0;
uint16_t FSR3_raw = 0;
uint16_t FSR4_raw = 0;
uint16_t NTC1_raw = 0;
uint16_t Fluids1_raw = 0;

// Timer interrupt flag
volatile bool timerFlag = false;


/***** Configuration Setup *****/
// Initialize the timer
void setupTimer(){
  // Uses timer 4, since timer 1,2,3 are used in the BLE transactions

  // Disable interrupts before configuring
  //NRF_TIMER4->INTENCLR = TIMER_INTENCLR_COMPARE0_Clear << TIMER_INTENCLR_COMPARE0_Pos;

  // Stop and clear the timer before using it
  NRF_TIMER4->TASKS_STOP = 1;
  NRF_TIMER4->TASKS_CLEAR = 1;


  // Set up the timer interrupt
  NRF_TIMER4->MODE = TIMER_MODE_MODE_Timer;  // Set timer mode
  NRF_TIMER4->BITMODE = TIMER_BITMODE_BITMODE_16Bit;  // Set timer to 32-bit mode
  
  // Set the timer prescaler to achieve desired interval
  NRF_TIMER4->PRESCALER = 7;
  
  // Set the compare value to trigger the interrupt
  // The compare value depends on the desired interval and timer resolution
  NRF_TIMER4->CC[0] = 0x270;
  
  // Enable the compare interrupt on CC[0]
  NRF_TIMER4->INTENSET = TIMER_INTENSET_COMPARE0_Set << TIMER_INTENSET_COMPARE0_Pos;
  // Create a shortcut for the timer clearing
  NRF_TIMER4->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;
  
  // Set interrupt priority
  NVIC_SetPriority(TIMER4_IRQn, 1ul);
  // Enable the interrupt in the NVIC
  NVIC_EnableIRQ(TIMER4_IRQn);
  
  // Start the timer
  NRF_TIMER4->TASKS_START = 1;
}
// SPI settings for pmod DA2
SPISettings mySettings(16000000, MSBFIRST, SPI_MODE0);
void setupDAC(){
  pinMode(sync, OUTPUT); // CS
  digitalWrite(sync, HIGH);
  SPI.begin();
  SPI.beginTransaction(mySettings);
  SPI.endTransaction();
}

// Timer interrupt handler
void timerInterruptHandler() {
  digitalWrite(DEBUG, HIGH);

  timerFlag = true;

  if(click == 200){
    seconds ++;
    click = 0;
  }
  click ++;
}




/******* DSP functions ********/
void movingAverage( float32_t ringBuffer[],
                    uint8_t win_size,
                    uint8_t &i,
                    float32_t acq,
                    float32_t *average,
                    float32_t *min,
                    float32_t *max){

  if (i == win_size) i = 0; // Resets the iterator
  ringBuffer[i] = acq; // Replaces the oldest element of the ring buffer with the new one
  arm_mean_f32(ringBuffer, win_size, average);
  arm_min_f32(ringBuffer, win_size, min, &idx);
  arm_max_no_idx_f32(ringBuffer, win_size, max);
  i++; // Move the iterator
}

void IIR( uint16_t Ain,
          float32_t x[],
          float32_t y[],
          float32_t A[],
          float32_t B[]){
  // Convert from uint16 to voltage value
  x[0] = Ain * (3.3f / 4096.0f); // Current input value
  
  // calc. the output
  arm_dot_prod_f32(B, x, 7, &int_res); // Intermediate calculation
  y[0] = (- A[1]*y[1] - A[2]*y[2] - A[3]*y[3] - A[4]*y[4] - A[5]*y[5] - A[6]*y[6]
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

/******** Regression **********/
float32_t cubicFit(float32_t x, float32_t params[]){
  return params[0] * powf(x,3) + params[1] * powf(x,2) + params[2] * x + params[3];
}

float32_t vToTemp(float32_t v){
  float32_t aux;
  aux = v + (1581/5050);
  aux *= (607/130680);
  aux = logf(aux);
  aux /= 3895.633;
  aux -= (20 / 5463);
  aux = (1/aux);
  return aux + 273.15f;
}



/******** Auxiliary *********/
void writeDAC(uint16_t val){
  // Enable DAC
  SPI.begin();
  digitalWrite(sync, LOW);
  // Send data
  SPI.transfer16(val);
  // De-assert DAC
  digitalWrite(sync, HIGH);
  SPI.endTransaction();
}

void readADC(){
  FSR1_raw = analogRead(A0);
  FSR2_raw = analogRead(A1);
  FSR3_raw = analogRead(A2);
  FSR4_raw = analogRead(A3);
  NTC1_raw = analogRead(A6);
  Fluids1_raw = analogRead(A7);
}

void filterIIRAll(){
  IIR(FSR1_raw, 
      (float32_t*) &FSR1_x,
      (float32_t*) &FSR1_y,
      A_FSR,
      B_FSR);

  IIR(FSR2_raw, (float32_t*) &FSR2_x, (float32_t*) &FSR2_y, A_FSR, B_FSR);
  IIR(FSR3_raw, (float32_t*) &FSR3_x, (float32_t*) &FSR3_y, A_FSR, B_FSR);
  IIR(FSR4_raw, (float32_t*) &FSR4_x, (float32_t*) &FSR4_y, A_FSR, B_FSR);
  IIR(NTC1_raw, (float32_t*) &NTC1_x, (float32_t*) &NTC1_y, A_NTC, B_NTC);
  IIR(Fluids1_raw, (float32_t*) &Fluids1_x, (float32_t*) &Fluids1_y, A_FSR, B_FSR);
}

void filterMAAll(){
  movingAverage(  (float32_t*) &MA_ring_buffer_FSR1,
                  M,
                  RB_i_FSR1,
                  FSR1_y[0],
                  &average_FSR[0],
                  &min_FSR[0],
                  &max_FSR[0]);
  
  movingAverage((float32_t*) &MA_ring_buffer_FSR2, M, RB_i_FSR2, FSR2_y[0], &average_FSR[1], &min_FSR[1], &max_FSR[1]);
  movingAverage((float32_t*) &MA_ring_buffer_FSR3, M, RB_i_FSR3, FSR3_y[0], &average_FSR[2], &min_FSR[2], &max_FSR[2]);
  movingAverage((float32_t*) &MA_ring_buffer_FSR4, M, RB_i_FSR4, FSR1_y[0], &average_FSR[3], &min_FSR[3], &max_FSR[3]);
  movingAverage((float32_t*) &MA_ring_buffer_NTC1, M, RB_i_NTC1, NTC1_y[0], &average_NTC1, &min_NTC1, &max_NTC1);
  movingAverage((float32_t*) &MA_ring_buffer_Fluids1, M, RB_i_Fluids1, FSR1_y[0], &average_Fluids1, &min_Fluids1, &max_Fluids1);

}

void updateFSR(){
  arm_mean_f32(average_FSR, 4, &FSR_mean);
  arm_min_f32(min_FSR, 4, &FSR_min, &idx);
  arm_max_no_idx_f32(max_FSR, 4, &FSR_max);

  // Voltage to force conversion
  interfaceingForceMean = cubicFit(FSR_mean, cubic_params_FSR);
  interfaceingForceMin = cubicFit(FSR_min, cubic_params_FSR);
  interfaceingForceMax = cubicFit(FSR_max, cubic_params_FSR);
}

void updateBodyTemperature(){
  skinTemperatureMean = vToTemp(average_NTC1);
  skinTemperatureMin = vToTemp(min_NTC1);
  skinTemperatureMax = vToTemp(max_NTC1);
}

void updatePrecisionPressure(){
  /***** IMPLEMENT REGRESSION OVER FLUIDS *****/
}

void updateComfort(){
  /***** IMPLEMENT FUZZY PREDICTOR FOR COMFORT *****/
}

void updateAmbTemperatureHumidity(){
  amb_temperature = HS300x.readTemperature();
  amb_humidity = HS300x.readTemperature();
}

void updateAmbPressure(){
  amb_baro_pressure = BARO.readPressure();
}

// Modify to publish the values of the sensors through BLE
void publishValues(){
  updateAmbTemperatureHumidity();
  updateAmbPressure();
  Serial.print("Mean force:");
  Serial.print(interfaceingForceMean);
  Serial.print(",");
  Serial.print("Amb temp:");
  Serial.print(amb_temperature);
  Serial.print(",");
  Serial.print("Amb humidity");
  Serial.println(amb_humidity);
}


void setup() {
  // analog configuration
  analogReadResolution(12);

  // Initializes the debug pin
  pinMode(DEBUG, OUTPUT);
  digitalWrite(DEBUG, HIGH);

  // Initializes the timer
  setupTimer();

  // Initialize serial
  Serial.begin(115200);
  // Initializes SPI for the DAC
  setupDAC();
  // Initialize humidity/temperature sensor
  HS300x.begin();
  // Initialize pressure sensor
  BARO.begin();
  
}

void loop() {
  // Process the interruption on runtime
  if(timerFlag){
    timerFlag = false;

    // Read all the ports
    readADC();

    //IIR_filter
    filterIIRAll();
        
    //Decimation
    if(decimationCounter < 5){
      decimationCounter ++;
    }
    else {
      // Restart decimator counter
      decimationCounter = 1; 
      
      
      //Moving_average // Compensates the operations
      filterMAAll();

      // Get FSR values **** temporal
      updateFSR();

      // Get temperature values **** temporal
      updateBodyTemperature();



      /***** DEBUG PLAY *****/
      FSR1_filt = cubicFit(average_FSR[0], cubic_params_FSR);

      DAC_o = FSR1_filt * (4095.0f / 80.0f);
      //DAC_o = FSR1_y[0] * (4095.0f / 3.3f);
      writeDAC(DAC_o);  
    }
  }

  /* BLE publishing section
  if(){
    publishValues();
  }
  */

  digitalWrite(DEBUG, LOW);
  __WFE(); // Wait for event instruction (sleeps waiting for event)
}

// Timer interrupt service routine
extern "C" void TIMER4_IRQHandler_v(void) {
  // Check if the interrupt was triggered by CC[0]
  if (NRF_TIMER4->EVENTS_COMPARE[0]) {
    NRF_TIMER4->EVENTS_COMPARE[0] = 0;  // Clear the event
    
    timerInterruptHandler();  // Call the interrupt handler function
  }
}