//Include libraries
#include <ArduinoBLE.h>
#include <Arduino_HS300x.h>
#include <Arduino_LPS22HB.h>


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

// Global variables
long previousMillis = 0;


void setup() {
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

  // start advertising
  BLE.advertise();

  Serial.println("BLE Demo Peripheral");
}

void loop() {


  // check for BLE central
  BLEDevice central = BLE.central();

  // if central is connected to peripheral:
  if (central){
    Serial.print("Connected to central: ");
    // Print central's MAC address:
    Serial.println(central.address());

    // Poll/transmit only when connected:
    while(central.connected()){
        long currentMillis = millis();
        // if 5s has passed, check again the sensors
        if (currentMillis - previousMillis >= 5000){
          previousMillis = currentMillis;
          updatePressure();
          updateTemperature();
          updateHumidity();
        }
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