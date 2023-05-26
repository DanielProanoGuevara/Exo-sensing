#include "Arduino.h"
#include <nrfx.h>
#include <nrf.h>
#include "nrf_timer.h"
#include "nrf_delay.h"
#include "arm_math.h"

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

// Filter variables
// *** IIR FSR coefficients
float32_t B_FSR[7] = {0.00034054f, 0.00204323f, 0.00510806f, 0.00681075f, 0.00510806f, 0.00204323f, 0.00034054f};
float32_t A_FSR[7] = {1.0f,        -3.5794348f, 5.65866717f,-4.96541523f, 2.52949491f,-0.70527411f, 0.08375648f};

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



// Regression constants in the form y = ax^3 + bx^2 + cx + d


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

// Timer interrupt handler
void timerInterruptHandler() {
  digitalWrite(DEBUG, HIGH);
  timerFlag = true;
}

// SPI settings for pmod DA2
SPISettings mySettings(16000000, MSBFIRST, SPI_MODE0);


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



void setup() {
  // analog configuration
  analogReadResolution(12);

  // Initializes the debug pin
  pinMode(DEBUG, OUTPUT);
  digitalWrite(DEBUG, HIGH);

  // Initializes SPI for the DAC
  pinMode(sync, OUTPUT); // CS
  digitalWrite(sync, HIGH);
  SPI.begin();
  SPI.beginTransaction(mySettings);
  SPI.endTransaction();
  

  // Initializes the timer
  setupTimer();
}

void loop() {
  // Process the interruption on runtime
  if(timerFlag){
    timerFlag = false;

    // Read all the ports
    FSR1_raw = analogRead(A0);
    FSR2_raw = analogRead(A1);
    FSR3_raw = analogRead(A2);
    FSR4_raw = analogRead(A3);
    NTC1_raw = analogRead(A6);
    Fluids1_raw = analogRead(A7);

    //IIR_filter
    IIR(FSR1_raw, 
        (float32_t*) &FSR1_x,
        (float32_t*) &FSR1_y,
        A_FSR,
        B_FSR);

    IIR(FSR2_raw, (float32_t*) &FSR2_x, (float32_t*) &FSR2_y, A_FSR, B_FSR);
    IIR(FSR3_raw, (float32_t*) &FSR3_x, (float32_t*) &FSR3_y, A_FSR, B_FSR);
    IIR(FSR4_raw, (float32_t*) &FSR4_x, (float32_t*) &FSR4_y, A_FSR, B_FSR);
    IIR(NTC1_raw, (float32_t*) &NTC1_x, (float32_t*) &NTC1_y, A_FSR, B_FSR);
    IIR(Fluids1_raw, (float32_t*) &Fluids1_x, (float32_t*) &Fluids1_y, A_FSR, B_FSR);
        
    //Decimation
    if(decimationCounter < 5){
      decimationCounter ++;
      //skip 5 samples
      return;
    }
    // Restart decimator counter
    decimationCounter = 1; 
    
    
    //Moving_average // Compensates the operations
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


    // Get the absolute metrics of the 4 FSR channels
    arm_mean_f32(average_FSR, 4, &FSR_mean);
    arm_min_f32(min_FSR, 4, &FSR_min, &idx);
    arm_max_no_idx_f32(max_FSR, 4, &FSR_max);

    // Voltage to force convertion


    FSR1_filt = average_FSR[0];

    DAC_o = FSR1_filt * (4095.0f / 3.3f);
    //DAC_o = FSR1_y[0] * (4095.0f / 3.3f);

    // Enable DAC
    SPI.begin();
    digitalWrite(sync, LOW);
    // Send data
    SPI.transfer16(DAC_o);
    // De-assert DAC
    digitalWrite(sync, HIGH);
    SPI.endTransaction();
    
    digitalWrite(DEBUG, LOW);
  }

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