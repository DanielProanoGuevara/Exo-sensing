//Include libraries
#include <ArduinoBLE.h>

#include "dsp.h"
#include "sys.h"
#include "setup_peripheral.h"
#include "lowPower.h"

// Interrupt flags
volatile bool timerFlag = false;
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




void setup() {
  // Setup RTC
  setupRTC();
  // Initialize integrated sensors
  setupIntegrated();


  Serial.begin(9600);
  while(!Serial); // Wait until serial is availabe
  

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
  // Check for BLE central
  BLEDevice central = BLE.central();

  // If central is connected to peripheral
  if (central){
    #if UART_EN
      Serial.print("Connected to central: ");
      // Print central's MAC address
      Serial.println(central.address());
    #endif
    // Transmit data only when connected
    while(central.connected()){

      if (rtcFlag) {
        rtcFlag = false;
        #ifdef LOWPOWER_INCLUDED
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
  }  
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