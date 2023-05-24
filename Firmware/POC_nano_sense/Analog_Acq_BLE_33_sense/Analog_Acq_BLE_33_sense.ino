#include "Arduino.h"
#include <nrfx.h>
#include <nrf.h>
#include "nrf_timer.h"
#include "nrf_delay.h"

#include "SPI.h"



// Pin  declarations
#define Ain0 A0
#define DEBUG 2
#define sync 5




// Variables 
int decimationCounter = 1;
float FSR1_filt = 0.0;

volatile uint16_t DAC_o = 0;


SPISettings mySettings(4000000, MSBFIRST, SPI_MODE3);

// Filter variables
// *** FSR coefficients
float B_FSR[7] = {0.00034054, 0.00204323, 0.00510806, 0.00681075, 0.00510806, 0.00204323, 0.00034054};
float A_FSR[7] = {1.0,        -3.5794348, 5.65866717,-4.96541523, 2.52949491,-0.70527411, 0.08375648};
float FSR1_x[7] = {0.0};
float FSR1_y[7] = {0.0};



// Interrupt-based analog acquisition
volatile uint32_t Ain0_raw = 0;

// Timer interrupt flag
volatile bool timerFlag = false;

// Timer interrupt handler
void timerInterruptHandler() {
  digitalWrite(DEBUG, HIGH);
  timerFlag = true;
}


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


void setup() {
  // analog configuration
  analogReadResolution(12);

  // Initializes the debug pin
  pinMode(DEBUG, OUTPUT);
  digitalWrite(DEBUG, HIGH);

  // Initializes SPI for the DAC
  pinMode(3, OUTPUT); // CS
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
    Ain0_raw = analogRead(Ain0);
    // convert uint to float Ain1_raw
    FSR1_x[0] = Ain0_raw * (3.3 / 4095.0); // Current input value
    //FIR_filter()
    // calc. the output
    FSR1_y[0] = (-A_FSR[1]*FSR1_y[1] - A_FSR[2]*FSR1_y[2] - A_FSR[3]*FSR1_y[3] - A_FSR[4]*FSR1_y[4] - A_FSR[5]*FSR1_y[5] - A_FSR[6]*FSR1_y[6]
                + B_FSR[0]*FSR1_x[0] + B_FSR[1]*FSR1_x[1] + B_FSR[2]*FSR1_x[2] + B_FSR[3]*FSR1_x[3] + B_FSR[4]*FSR1_x[4] + B_FSR[5]*FSR1_x[5] + B_FSR[6]*FSR1_x[6]);
    // Propagate inputs
    FSR1_x[6] = FSR1_x[5];
    FSR1_x[5] = FSR1_x[4];
    FSR1_x[4] = FSR1_x[3];
    FSR1_x[3] = FSR1_x[2];
    FSR1_x[2] = FSR1_x[1];
    FSR1_x[1] = FSR1_x[0];
    // Propagate outpus
    FSR1_y[6] = FSR1_y[5];
    FSR1_y[5] = FSR1_y[4];
    FSR1_y[4] = FSR1_y[3];
    FSR1_y[3] = FSR1_y[2];
    FSR1_y[2] = FSR1_y[1];
    FSR1_y[1] = FSR1_y[0];
    

    // Set output
    FSR1_filt = FSR1_y[0];
    DAC_o = FSR1_filt * (4095.0 / 3.3);

    
    //Decimation?()
    if(decimationCounter < 5){
      decimationCounter ++;
      //skip 9 samples
      return;
    }
    // Restart decimator counter
    decimationCounter = 1;
    
    // Enable DAC
    SPI.begin();
    digitalWrite(sync, LOW);
    // Send data
    SPI.transfer16(DAC_o);
    // De-assert DAC
    digitalWrite(sync, HIGH);
    SPI.endTransaction();


    //Moving_average()
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