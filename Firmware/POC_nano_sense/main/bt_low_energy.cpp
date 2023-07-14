#include "bt_low_energy.h"



/*********************** Create BLE services ********************************/
BLEService Environmental_Sensing_Service("181A");
BLEService Device_Interfacing_Service("00000000-0000-0000-0001-000000000000");
BLEService Comfort_Service("00000000-0000-0000-0002-000000000000");

/*********************** Create Characteristics ****************************/
//  Environmental Sensing Temperature, read and notify
BLECharacteristic Ambient_Temperature("2A6E", BLERead | BLENotify, 2);
//  Environmental Sensing Pressure, read and notify
BLECharacteristic Ambient_Pressure("2A6D", BLERead | BLENotify, 4);
//  Environmental Sensing Humidity, read and notify
BLECharacteristic Ambient_Humidity("2A6F", BLERead | BLENotify, 2);

// Interfacing Temperature, read and notify
BLECharacteristic Body_Temp_Min("00000000-0000-0000-0001-100000000001", BLERead | BLENotify, 2);
BLECharacteristic Body_Temp_Avg("00000000-0000-0000-0001-100000000002", BLERead | BLENotify, 2);
BLECharacteristic Body_Temp_Max("00000000-0000-0000-0001-100000000003", BLERead | BLENotify, 2);
// Interfacing Force, read and notify
BLECharacteristic Interface_Force_Min("00000000-0000-0000-0001-200000000001", BLERead | BLENotify, 2);
BLECharacteristic Interface_Force_Avg("00000000-0000-0000-0001-200000000002", BLERead | BLENotify, 2);
BLECharacteristic Interface_Force_Max("00000000-0000-0000-0001-200000000003", BLERead | BLENotify, 2);
// Device ON time, read and notify
BLECharacteristic ON_time("00000000-0000-0000-0001-300000000001", BLERead | BLENotify, 4);

// Predicted Comfort Level, read and notify
BLECharacteristic Comfort("00000000-0000-0000-0002-100000000001", BLERead | BLENotify, 1);


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





void setupBluetoothLE(){
  /******************* Initialize BLE *****************/
  // Block if BLE cannot be initalized
  BLE.begin();
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
  Ambient_Temperature.writeValue((int16_t)0);
  Ambient_Pressure.writeValue((uint32_t)0);
  Ambient_Humidity.writeValue((uint16_t)0);
  Body_Temp_Min.writeValue((int16_t)0);
  Body_Temp_Avg.writeValue((int16_t)0);
  Body_Temp_Max.writeValue((int16_t)0);
  Interface_Force_Min.writeValue((uint16_t)0);
  Interface_Force_Avg.writeValue((uint16_t)0);
  Interface_Force_Max.writeValue((uint16_t)0);
  ON_time.writeValue((uint32_t)0);
  Comfort.writeValue((uint8_t)0);
  
  // start advertising
  BLE.advertise();
}


void poll_BLE(){
  BLE.poll();
}

void updateAmbTemperature(float32_t AmbTemp){
  int16_t temperature = AmbTemp * 100;
  Ambient_Temperature.writeValue(temperature);
}
void updateAmbPressure(float32_t AmbPres){
  uint32_t pressure = AmbPres * 10000;
  Ambient_Pressure.writeValue(pressure);
}
void updateAmbHumidity(float32_t AmbHum){
  uint16_t humidity = AmbHum * 100;
  Ambient_Humidity.writeValue(humidity);
}

void updateMinBodyTemp(float32_t T){
  int16_t minT = T * 100;
  Body_Temp_Min.writeValue(minT);
}
void updateAvgBodyTemp(float32_t T){
  int16_t avgT = T * 100;
  Body_Temp_Avg.writeValue(avgT);
}
void updateMaxBodyTemp(float32_t T){
  int16_t maxT = T * 100;
  Body_Temp_Max.writeValue(maxT);
}

void updateMinForce(float32_t F){
  uint16_t minF = F * 100;
  Interface_Force_Min.writeValue(minF);
}
void updateAvgForce(float32_t F){
  uint16_t avgF = F * 100;
  Interface_Force_Avg.writeValue(avgF);
}
void updateMaxForce(float32_t F){
  uint16_t maxF = F * 100;
  Interface_Force_Max.writeValue(maxF);
}

void updateONTime(uint32_t time){
  ON_time.writeValue(time);
}

void updateComfort(float comf){
  uint8_t comfort = comf * 10;
  Comfort.writeValue(comfort);
}

