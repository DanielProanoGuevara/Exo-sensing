#include "nrf52840.h"
#include "nrf52840_bitfields.h"
#include "setup_peripheral.h"


// Initialize the timer
void setupTimer() {
  // Uses timer 4, since timer 1,2,3 are used in the BLE transactions

  // Disable interrupts before configuring
  NRF_TIMER4->INTENCLR = TIMER_INTENCLR_COMPARE0_Clear << TIMER_INTENCLR_COMPARE0_Pos;

  // Stop and clear the timer before using it
  NRF_TIMER4->TASKS_STOP = 1;
  NRF_TIMER4->TASKS_CLEAR = 1;


  // Set up the timer interrupt
  NRF_TIMER4->MODE = TIMER_MODE_MODE_Timer;           // Set timer mode
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
  NVIC_SetPriority(TIMER4_IRQn, 2ul);
  // Enable the interrupt in the NVIC
  NVIC_EnableIRQ(TIMER4_IRQn);

  // Start the timer
  NRF_TIMER4->TASKS_START = 1;
}


// Initialize the RTC
void setupRTC() {
  // Disable interrupts before configuring
  NRF_RTC2->INTENCLR = RTC_INTENCLR_COMPARE0_Clear << RTC_INTENCLR_COMPARE0_Pos;


  // Start LFCLK (32kHz) crystal oscillator.
  NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_RC << CLOCK_LFCLKSRC_SRC_Pos; // Select RC low power clock source
  NRF_CLOCK->LFRCMODE = 1; // Ultra-low power mode (ULP)
  NRF_CLOCK->TASKS_LFCLKSTART = 1; // Start the low power clock
  while(NRF_CLOCK->EVENTS_LFCLKSTARTED == 0); // Wait for the clock to initialize
  NRF_CLOCK->EVENTS_LFCLKSTARTED = 0; // Reset the events register

  // Stop and clear the timer before using it
  NRF_RTC2->TASKS_STOP = 1;
  NRF_RTC2->TASKS_CLEAR = 1;

  // 8 Hz timer period (Maximum)
  NRF_RTC2->PRESCALER = 0xFFF;
  // Compare value at 1 second, i.e. every 8 ticks
  NRF_RTC2->CC[0] = 0X8;
  // Enable events on tick 
  NRF_RTC2->EVTENSET = RTC_EVTENSET_COMPARE0_Set << RTC_EVTENSET_COMPARE0_Pos;
  // Enable IRQ on TICK
  NRF_RTC2->INTENSET = RTC_INTENSET_COMPARE0_Set << RTC_INTENSET_COMPARE0_Pos;

  // Set interrupt priority
  NVIC_SetPriority(RTC2_IRQn, 1ul);
  // Enable the interrupt in the NVIC
  NVIC_EnableIRQ(RTC2_IRQn);

  // Start the RTC
  NRF_RTC2->TASKS_START = 1;
}


// Initialize the I2C Bus Sensors
void setupIntegrated(){
  // Initialize humidity/temperature sensor
  HS300x.begin();
  // Initialize pressure sensor
  BARO.begin();
}


// SPI settings for pmod DA2
#ifdef DEBUG_DAC
  SPISettings mySettings(16000000, MSBFIRST, SPI_MODE0);
  void setupDAC() {
    pinMode(sync, OUTPUT);  // CS
    digitalWrite(sync, HIGH);
    SPI.begin();
    SPI.beginTransaction(mySettings);
    SPI.endTransaction();
  }

  void writeDAC(uint16_t val) {
    // Enable DAC
    SPI.begin();
    digitalWrite(sync, LOW);
    // Send data
    SPI.transfer16(val);
    // De-assert DAC
    digitalWrite(sync, HIGH);
    SPI.endTransaction();
  }
#endif

