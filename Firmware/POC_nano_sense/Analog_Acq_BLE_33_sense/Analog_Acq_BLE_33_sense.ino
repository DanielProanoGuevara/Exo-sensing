#include "Arduino.h"
#include <nrfx.h>
#include <nrf.h>
#include "nrf_timer.h"


// Pin  declarations
#define Ain0 A0


// Variables 
float FSR1_raw = 0.0;

// Interrupt-based analog acquisition
volatile uint32_t Ain0_raw = 0;

// Timer interrupt flag
volatile bool timerFlag = false;

// Timer interrupt handler
void timerInterruptHandler() {
  digitalWrite(LED_BUILTIN, HIGH);
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
  // initialize serial communications at  bps:
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Serial active");

  // analog configuration
  analogReadResolution(12);
  Serial.println("Analog resolution modified");

  // Initializes the debug pin
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  // Initializes the timer
  setupTimer();
  Serial.println("Timer Active");
}

void loop() {
  // Process the interruption on runtime
  if(timerFlag){
    timerFlag = false;
    Ain0_raw = analogRead(Ain0);
    // convert uint to float Ain1_raw
    FSR1_raw = Ain0_raw * (3.3 / 4096.0);
    //FIR_filter()
    //Decimation?()
    //Moving_average()
    digitalWrite(LED_BUILTIN, LOW);
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