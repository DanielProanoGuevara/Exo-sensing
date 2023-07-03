//Include libraries
#include <ArduinoBLE.h>
#include <Arduino_HS300x.h>
#include <Arduino_LPS22HB.h>


#include <nrfx.h>
#include <nrf.h>
#include <nrf_timer.h>
#include <nrf_rtc.h>


volatile bool rtcFlag = false;





// Create BLE services
BLEService Environmental_Sensing_Service("181A");

// Create Characteristics
//  Environmental Sensing Temperature, read and notify
BLEByteCharacteristic Ambient_Temperature("2A6E", BLERead | BLENotify);
//  Environmental Sensing Pressure, read and notify
BLEByteCharacteristic Ambient_Pressure("2A6D", BLERead | BLENotify);
//  Environmental Sensing Humidity, read and notify
BLEByteCharacteristic Ambient_Humidity("2A6F", BLERead | BLENotify);

// Create descriptors
BLEDescriptor TemperatureDescriptor("2901", "Temperature Value");
BLEDescriptor PressureDescriptor("2901", "Pressure Value");
BLEDescriptor HumidityDescriptor("2901", "Humidity Value");









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


void setupLowPower(){
  digitalWrite(LED_PWR, LOW); // "ON" LED turned off:
  digitalWrite(PIN_ENABLE_SENSORS_3V3, LOW); //PIN_ENABLE_SENSORS_3V3 set to LOW:
  digitalWrite(PIN_ENABLE_I2C_PULLUP, LOW); // PIN_ENABLE_I2C_PULLUP set to LOW:
}










void setup() {
  setupRTC();
  Serial.begin(9600);
  while(!Serial); // Wait until serial is availabe
  
  // Block if temperature sensor is not recognized
  if (!HS300x.begin()){
    Serial.println("Failed to initialize humidity-temperature sensor!");
    while(1);
  }
  // Block if barometric sensor is not recognized
  if (!BARO.begin()){
    Serial.println("Failed to initialize barometric sensor!");
    while(1);
  }

  // Block if BLE cannot be initalized
  if (!BLE.begin()){
    Serial.println("Failed to initialize BLE");
    while(1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("ArduinoBLE_Test");

  BLE.setAdvertisedService(Environmental_Sensing_Service);

  // add the characteristic to the service:
  Environmental_Sensing_Service.addCharacteristic(Ambient_Temperature);
  Environmental_Sensing_Service.addCharacteristic(Ambient_Pressure);
  Environmental_Sensing_Service.addCharacteristic(Ambient_Humidity);

  // add the descriptors to the characteristics:
  Ambient_Temperature.addDescriptor(TemperatureDescriptor);
  Ambient_Pressure.addDescriptor(PressureDescriptor);
  Ambient_Humidity.addDescriptor(HumidityDescriptor);

  // add service
  BLE.addService(Environmental_Sensing_Service);

  // set the initial value for the characteristic
  Ambient_Temperature.writeValue(HS300x.readTemperature());
  Ambient_Pressure.writeValue(BARO.readPressure());
  Ambient_Humidity.writeValue(HS300x.readHumidity());

  setupLowPower();

  // start advertising
  BLE.advertise();

  Serial.println("BLE Demo Peripheral");
}

void loop() {
  BLE.poll();
  if (rtcFlag) {
    rtcFlag = false;
    digitalWrite(PIN_ENABLE_SENSORS_3V3, HIGH); //PIN_ENABLE_SENSORS_3V3 set to HIGH:
    digitalWrite(PIN_ENABLE_I2C_PULLUP, HIGH); // PIN_ENABLE_I2C_PULLUP set to HIGH:
    updatePressure();
    updateTemperature();
    updateHumidity();
    
  }
  __SEV();
  __WFE();
  __WFE();
}


void updateTemperature(){
  int temperature = HS300x.readTemperature();
  Serial.print("Current temperature: ");
  Serial.print(temperature);
  Serial.println(" ºC");
  Ambient_Temperature.writeValue(temperature);
}

void updatePressure(){
  int pressure = BARO.readPressure();
  Serial.print("Current pressure: ");
  Serial.print(pressure);
  Serial.println(" kPa");
  Ambient_Pressure.writeValue(pressure);
}

void updateHumidity(){
  int humidity = HS300x.readHumidity();
  Serial.print("Current humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  Ambient_Humidity.writeValue(humidity);
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