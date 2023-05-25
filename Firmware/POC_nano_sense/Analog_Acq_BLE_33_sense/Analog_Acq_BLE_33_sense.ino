#include "Arduino.h"
#include <nrfx.h>
#include <nrf.h>
#include "nrf_timer.h"
#include "nrf_delay.h"

#include "SPI.h"

// Pin  declarations
#define DEBUG 2
#define sync 5


// Variables 
uint8_t decimationCounter = 1;
float FSR1_filt = 0.0f;
uint16_t DAC_o = 0;

// Filter variables
// *** IIR FSR coefficients
float B_FSR[7] = {0.00034054f, 0.00204323f, 0.00510806f, 0.00681075f, 0.00510806f, 0.00204323f, 0.00034054f};
float A_FSR[7] = {1.0f,        -3.5794348f, 5.65866717f,-4.96541523f, 2.52949491f,-0.70527411f, 0.08375648f};

float FSR1_x[7] = {0.0f}; // Input memory buffer
float FSR1_y[7] = {0.0f}; // Output memory buffer
float FSR2_x[7] = {0.0f};
float FSR2_y[7] = {0.0f};
float FSR3_x[7] = {0.0f};
float FSR3_y[7] = {0.0f};
float FSR4_x[7] = {0.0f};
float FSR4_y[7] = {0.0f};
float NTC1_x[7] = {0.0f};
float NTC1_y[7] = {0.0f};
float Fluids1_x[7] = {0.0f};
float Fluids1_y[7] = {0.0f};

// *** MA ring buffer and variables
const uint8_t M = 20; // Window size
float MA_ring_buffer_FSR1 [M] = {0.0f}; // FSR1 moving average ring buffer
float acc_FSR1 = 0.0f; // FSR1 accumulator
float average_FSR1 = 0.0f;
uint8_t RB_i_FSR1 = 0; // FSR1 Ring buffer iterator

float MA_ring_buffer_FSR2 [M] = {0.0f};
float acc_FSR2 = 0.0f;
float average_FSR2 = 0.0f;
uint8_t RB_i_FSR2 = 0;

float MA_ring_buffer_FSR3 [M] = {0.0f};
float acc_FSR3 = 0.0f;
float average_FSR3 = 0.0f;
uint8_t RB_i_FSR3 = 0;

float MA_ring_buffer_FSR4 [M] = {0.0f};
float acc_FSR4 = 0.0f;
float average_FSR4 = 0.0f;
uint8_t RB_i_FSR4 = 0;

float MA_ring_buffer_NTC1 [M] = {0.0f};
float acc_NTC1 = 0.0f;
float average_NTC1 = 0.0f;
uint8_t RB_i_NTC1 = 0;

float MA_ring_buffer_Fluids1 [M] = {0.0f};
float acc_Fluids1 = 0.0f;
float average_Fluids1 = 0.0f;
uint8_t RB_i_Fluids1 = 0;

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
void movingAverage( float ringBuffer[],
                    uint8_t win_size,
                    uint8_t &i,
                    float &acc,
                    float acq,
                    float &average){

  if (i == win_size) i = 0; // Resets the iterator
  acc -= ringBuffer[i]; // Substracts the oldest element from the accumulator
  ringBuffer[i] = acq; // Replaces the oldest element of the ring buffer with the new one
  acc += acq; // Increses the accumulator with the new sample
  average = acc / win_size; // Get the buffer average
  i++; // Move the iterator
}

void IIR( uint16_t Ain,
          float x[],
          float y[],
          float A[],
          float B[]){
  // Convert from uint16 to voltage value
  x[0] = Ain * (3.3f / 4096.0f); // Current input value
  
  // calc. the output
  y[0] = (- A[1]*y[1] - A[2]*y[2] - A[3]*y[3] - A[4]*y[4] - A[5]*y[5] - A[6]*y[6]
          + B[0]*x[0] + B[1]*x[1] + B[2]*x[2] + B[3]*x[3] + B[4]*x[4] + B[5]*x[5] + B[6]*x[6]);
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
        (float*) &FSR1_x,
        (float*) &FSR1_y,
        A_FSR,
        B_FSR);

    IIR(FSR2_raw, (float*) &FSR2_x, (float*) &FSR2_y, A_FSR, B_FSR);
    IIR(FSR3_raw, (float*) &FSR3_x, (float*) &FSR3_y, A_FSR, B_FSR);
    IIR(FSR4_raw, (float*) &FSR4_x, (float*) &FSR4_y, A_FSR, B_FSR);
    IIR(NTC1_raw, (float*) &NTC1_x, (float*) &NTC1_y, A_FSR, B_FSR);
    IIR(Fluids1_raw, (float*) &Fluids1_x, (float*) &Fluids1_y, A_FSR, B_FSR);
        
    //Decimation
    if(decimationCounter < 5){
      decimationCounter ++;
      //skip 5 samples
      return;
    }
    // Restart decimator counter
    decimationCounter = 1;
    
    
    //Moving_average // Compensates the operations
    movingAverage(  (float*) &MA_ring_buffer_FSR1,
                    M,
                    RB_i_FSR1,
                    acc_FSR1,
                    FSR1_y[0],
                    average_FSR1);
    
    movingAverage((float*) &MA_ring_buffer_FSR2, M, RB_i_FSR2, acc_FSR2, FSR2_y[0] * 2, average_FSR2);
    movingAverage((float*) &MA_ring_buffer_FSR3, M, RB_i_FSR3, acc_FSR3, FSR3_y[0] * 2, average_FSR3);
    movingAverage((float*) &MA_ring_buffer_FSR4, M, RB_i_FSR4, acc_FSR4, FSR1_y[0] * 2, average_FSR4);
    movingAverage((float*) &MA_ring_buffer_NTC1, M, RB_i_NTC1, acc_NTC1, NTC1_y[0] * 2, average_NTC1);
    movingAverage((float*) &MA_ring_buffer_Fluids1, M, RB_i_Fluids1, acc_Fluids1, FSR1_y[0] * 2, average_Fluids1);

    FSR1_filt = average_FSR1;

    DAC_o = FSR1_filt * (4095.0f / 3.3f);
    //DAC_o = FSR1_filt;

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