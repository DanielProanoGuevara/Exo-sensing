#include <nrfx.h>
#include <nrf.h>
#include "nrf_timer.h"
#include "nrf_gpio.h"


volatile bool timer_triggered = false; // Flag to acces to the service function without overloading the ISR


// Initialize the timer
void setupTimer(){
  // Uses timer 4, since timer 1,2,3 are used in the BLE transactions

  // Disable interrupts before configuring
  //NRF_TIMER0->INTENCLR = 1UL << TIMER_INTENSET_COMPARE0_Pos;

  // Initialize and configure timer
  NRF_TIMER0->TASKS_STOP = 1UL; // Stop timer in case it was running
  NRF_TIMER0->TASKS_CLEAR = 1UL; // Clear timer to initialize it propperly 

  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos; //16-bit mode
  //NRF_TIMER4->MODE = 0UL; // Timer mode
  NRF_TIMER0->PRESCALER = 4 << TIMER_PRESCALER_PRESCALER_Pos; // Prescaler: timer_freq = 16MHz / 2^prescaler = 125KHz
  NRF_TIMER0->CC[0] = 1000; // Compare value = timer_freq * (1/PPS) -- PulsesPerSecond = 200

  // Enable interrupts
  NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
  // Create a shortcut with a clear timer whenever an event occurs
  NRF_TIMER0->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;

  // Point to the interrupts vector list
  //NVIC_EnableIRQ(TIMER0_IRQn);
  // Start the timer
  NRF_TIMER0->TASKS_START = 1;
}


void TIMER0_IRQHandler(void){
  volatile uint32_t dummy;
  if (NRF_TIMER0->EVENTS_COMPARE[0] == 1){ // Verifies that the interrupt is triggered on the event
    NRF_TIMER0->EVENTS_COMPARE[0] = 0; // Clears the value (in case the shortcut didn't do it)


    dummy = NRF_TIMER0->EVENTS_COMPARE[0];
    dummy;

    //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    //timer_triggered =  true;
  }
}


void toggleLed(){
  if (timer_triggered == true){
    // Whatever it needs to do
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // Recovers initial flag state
    timer_triggered = false;
  }
}




void setup() {
  // Initializes the timer
  setupTimer();
  // Initializes the debug pin
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  timer_triggered = false;

}

void loop() {
  toggleLed(); // Execute function served on event
  __WFE(); // Wait for event instruction (sleeps waiting for event)
}
