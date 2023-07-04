//Include libraries
#include <ArduinoBLE.h>

#include "dsp.h"
#include "sys.h"
#include "setup_peripheral.h"
#include "lowPower.h"

// Interrupt flags
volatile bool timerFlag = false;
volatile bool rtcFlag = false;



/*********************** Create BLE services ********************************/
BLEService Environmental_Sensing_Service("181A");
BLEService Device_Interfacing_Service("00000000-0000-0000-0001-000000000000");
BLEService Comfort_Service("00000000-0000-0000-0002-000000000000");

/*********************** Create Characteristics ****************************/
//  Environmental Sensing Temperature, read and notify
BLEByteCharacteristic Ambient_Temperature("2A6E", BLERead | BLENotify);
//  Environmental Sensing Pressure, read and notify
BLEByteCharacteristic Ambient_Pressure("2A6D", BLERead | BLENotify);
//  Environmental Sensing Humidity, read and notify
BLEByteCharacteristic Ambient_Humidity("2A6F", BLERead | BLENotify);

// Interfacing Temperature, read and notify
BLEByteCharacteristic Body_Temp_Min("00000000-0000-0000-0001-100000000001", BLERead | BLENotify);
BLEByteCharacteristic Body_Temp_Avg("00000000-0000-0000-0001-100000000002", BLERead | BLENotify);
BLEByteCharacteristic Body_Temp_Max("00000000-0000-0000-0001-100000000003", BLERead | BLENotify);
// Interfacing Force, read and notify
BLEByteCharacteristic Interface_Force_Min("00000000-0000-0000-0001-200000000001", BLERead | BLENotify);
BLEByteCharacteristic Interface_Force_Avg("00000000-0000-0000-0001-200000000002", BLERead | BLENotify);
BLEByteCharacteristic Interface_Force_Max("00000000-0000-0000-0001-200000000003", BLERead | BLENotify);
// Device ON time, read and notify
BLEByteCharacteristic ON_time("00000000-0000-0000-0001-300000000001", BLERead | BLENotify);

// Predicted Comfort Level, read and notify
BLEByteCharacteristic Comfort("00000000-0000-0000-0002-100000000001", BLERead | BLENotify);


/*************************************** Create descriptors **********************************************/
BLEDescriptor AmbTemperatureDescriptor("2901", "Amb_temperature_C");
BLEDescriptor AmbPressureDescriptor("2901", "Amb_pressure_Pa");
BLEDescriptor AmbHumidityDescriptor("2901", "Amb_Humidity_%");

//

BLEDescriptor MinBodyTemperatureDescriptor("2901", "Min_Body_Temp_C");
BLEDescriptor AvgBodyTemperatureDescriptor("2901", "Avg_Body_Temp_C");
BLEDescriptor MaxBodyTemperatureDescriptor("2901", "Max_Body_Temp_C");

BLEDescriptor MinInterfaceForceDescriptor("2901", "Min_Interface_Force_N");
BLEDescriptor AvgInterfaceForceDescriptor("2901", "Avg_Interface_Force_N");
BLEDescriptor MaxInterfaceForceDescriptor("2901", "Max_Interface_Force_N");

BLEDescriptor ONTimeDescriptor("2901", "Device_Active_Time_s");

//

BLEDescriptor ComfortDescriptor("2901", "Fuzzy_Comfort");



void setup() {
  // Setup RTC
  setupRTC();
  // Initialize integrated sensors
  setupIntegrated();


  Serial.begin(9600);
  while(!Serial); // Wait until serial is availabe
  
  /******************* Initialize BLE *****************/
  // Block if BLE cannot be initalized
  if (!BLE.begin()){
    Serial.println("Failed to initialize BLE");
    while(1);
  }

  // set advertised local name and services UUID:
  BLE.setLocalName("ExoSensing");

  /************************ Declare the services **********************/
  BLE.setAdvertisedService(Environmental_Sensing_Service);
  BLE.setAdvertisedService(Device_Interfacing_Service);
  BLE.setAdvertisedService(Comfort_Service);

  /********************** Declare the characteristics *******************/
  // add the characteristic to the Environmental_Sensing_Service:
  Environmental_Sensing_Service.addCharacteristic(Ambient_Temperature);
  Environmental_Sensing_Service.addCharacteristic(Ambient_Pressure);
  Environmental_Sensing_Service.addCharacteristic(Ambient_Humidity);

  // add the characteristic to the Device_Interfacing_Service:
  Device_Interfacing_Service.addCharacteristic(Body_Temp_Min);
  Device_Interfacing_Service.addCharacteristic(Body_Temp_Avg);
  Device_Interfacing_Service.addCharacteristic(Body_Temp_Max);
  Device_Interfacing_Service.addCharacteristic(Interface_Force_Min);
  Device_Interfacing_Service.addCharacteristic(Interface_Force_Avg);
  Device_Interfacing_Service.addCharacteristic(Interface_Force_Max);
  Device_Interfacing_Service.addCharacteristic(ON_time);

  // add the characteristic to the Comfort_Service:
  Comfort_Service.addCharacteristic(Comfort);

  /******************** Declare the Descriptors *************************/

  // add the descriptors to the characteristics:
  Ambient_Temperature.addDescriptor(AmbTemperatureDescriptor);
  Ambient_Pressure.addDescriptor(AmbPressureDescriptor);
  Ambient_Humidity.addDescriptor(AmbHumidityDescriptor);
  Body_Temp_Min.addDescriptor(MinBodyTemperatureDescriptor);
  Body_Temp_Avg.addDescriptor(AvgBodyTemperatureDescriptor);
  Body_Temp_Max.addDescriptor(MaxBodyTemperatureDescriptor);
  Interface_Force_Min.addDescriptor(MinInterfaceForceDescriptor);
  Interface_Force_Avg.addDescriptor(AvgInterfaceForceDescriptor);
  Interface_Force_Max.addDescriptor(MaxInterfaceForceDescriptor);
  ON_time.addDescriptor(ONTimeDescriptor);
  Comfort.addDescriptor(ComfortDescriptor);

  /********************** Initialize the Services ********************************/
  BLE.addService(Environmental_Sensing_Service);
  BLE.addService(Device_Interfacing_Service);
  BLE.addService(Comfort_Service);

  // set the initial value for the characteristics
  Ambient_Temperature.writeValue(0x0000);
  Ambient_Pressure.writeValue(0x00000000);
  Ambient_Humidity.writeValue(0x0000);
  Body_Temp_Min.writeValue(0x0000);
  Body_Temp_Avg.writeValue(0x0000);
  Body_Temp_Max.writeValue(0x0000);
  Interface_Force_Min.writeValue(0x0000);
  Interface_Force_Avg.writeValue(0x0000);
  Interface_Force_Max.writeValue(0x0000);
  ON_time.writeValue(0x00000000);
  Comfort.writeValue(0x00);
  

  // start advertising
  BLE.advertise();


  setupLowPower();

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