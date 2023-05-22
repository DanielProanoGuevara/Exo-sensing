#include <nrfx.h>
#include <nrf.h>
#include "nrf_timer.h"


// Timer settings
const unsigned long timerInterval = 1000;  // 1 second interval

// Flag to track LED state
volatile bool ledState = false;

// Timer interrupt handler
void timerInterruptHandler() {
  ledState = !ledState;  // Toggle LED state
}



// Initialize the timer
void setupTimer(){
  // Uses timer 4, since timer 1,2,3 are used in the BLE transactions

  // Disable interrupts before configuring
  NRF_TIMER4->INTENCLR = TIMER_INTENCLR_COMPARE0_Msk;

  // Stop and clear the timer before using it
  NRF_TIMER4->TASKS_STOP = 1;
  NRF_TIMER4->TASKS_CLEAR = 1;


  // Set up the timer interrupt
  NRF_TIMER4->MODE = TIMER_MODE_MODE_Timer;  // Set timer mode
  NRF_TIMER4->BITMODE = TIMER_BITMODE_BITMODE_32Bit;  // Set timer to 32-bit mode
  
  // Set the timer prescaler to achieve desired interval
  // For example, with a 16 MHz clock, prescaler 4 gives a 1 microsecond resolution
  NRF_TIMER4->PRESCALER = 4;
  
  // Set the compare value to trigger the interrupt
  // The compare value depends on the desired interval and timer resolution
  // For example, with a 1-second interval and 1 microsecond resolution,
  // the compare value is (timerInterval * 1000) - 1
  NRF_TIMER4->CC[0] = (timerInterval * 1000) - 1;
  
  // Enable the compare interrupt on CC[0]
  NRF_TIMER4->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
  // Create a shortcut for the timer clearing
  NRF_TIMER4->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
  
  // Enable the interrupt in the NVIC
  NVIC_EnableIRQ(TIMER4_IRQn);
  
  // Start the timer
  NRF_TIMER4->TASKS_START = 1;
}




void setup() {
  // Initializes the debug pin
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  // Initializes the timer
  setupTimer();
}

void loop() {
  // Update the LED state
  if (ledState) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  //__WFE(); // Wait for event instruction (sleeps waiting for event)
}

// Timer interrupt service routine
void TIMER4_IRQHandler() {
  // Check if the interrupt was triggered by CC[0]
  if (NRF_TIMER4->EVENTS_COMPARE[0]) {
    NRF_TIMER4->EVENTS_COMPARE[0] = 0;  // Clear the event
    
    timerInterruptHandler();  // Call the interrupt handler function
  }
}