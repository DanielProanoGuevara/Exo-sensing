//Include libraries

#include "dsp.h"
#include "sys.h"
#include "bt_low_energy.h"
#include "setup_peripheral.h"
#include "low_power.h"


// Interrupt flags
volatile bool timerFlag = false;
volatile bool rtcFlag = false;







void setup() {
  // Setup RTC
  setupRTC();
  // Initialize integrated sensors
  setupIntegrated();


  Serial.begin(9600);
  while(!Serial); // Wait until serial is availabe
  

  setupBluetoothLE();
  setupLowPower();

  Serial.println("BLE Demo Peripheral");
}

void loop() {
  poll_BLE();
  if (rtcFlag) {
    rtcFlag = false;
    #ifdef LOW_POWER_H
    digitalWrite(PIN_ENABLE_SENSORS_3V3, HIGH); //PIN_ENABLE_SENSORS_3V3 set to HIGH:
    digitalWrite(PIN_ENABLE_I2C_PULLUP, HIGH); // PIN_ENABLE_I2C_PULLUP set to HIGH:
    #endif
    updatePressure();
    updateTemperature();
    updateHumidity();
  }
  __SEV();
  __WFE();
  __WFE();  
}



// RTC interrupt service routine
extern "C" void RTC2_IRQHandler_v(void){
  // Check if the interrupt was triggered by a tick event
  if (NRF_RTC2->EVENTS_COMPARE[0]) {
    NRF_RTC2->EVENTS_COMPARE[0] = 0; // Clear event 
    NRF_RTC2->TASKS_CLEAR = 1; // Clear register value
    __SEV(); // to avoid race condition

    rtcFlag = true;
    // Use time control below this
  }
}