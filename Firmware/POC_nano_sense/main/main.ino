//Include libraries
#include <ArduinoBLE.h>
#include <Arduino_HS300x.h>
#include <Arduino_LPS22HB.h>

// Create BLE services
BLEService RGB_Color_Service("cfa3772c-4442-48c2-846f-3824c3abb5f9");
BLEService Environmental_Sensing_Service("181A");

// Create Characteristics
//  RGB Red Value characteristic, read and writable
BLEByteCharacteristic Red("482fb2d1-33ed-4373-812e-f860c8899d08", BLERead | BLEWrite);
//  RGB Green Value characteristic, read and writable
BLEByteCharacteristic Green("1c8daadc-c76e-465e-8779-a9f4981c2715", BLERead | BLEWrite);
//  RGB Blue Value characteristic, read and writable
BLEByteCharacteristic Blue("7276b11e-4e7c-49b5-823f-59c2bee3dfde", BLERead | BLEWrite);
//  Environmental Sensing Temperature, read and notify
BLEByteCharacteristic Ambient_Temperature("2A6E", BLERead | BLENotify);
//  Environmental Sensing Pressure, read and notify
BLEByteCharacteristic Ambient_Pressure("2A6D", BLERead | BLENotify);
//  Environmental Sensing Humidity, read and notify
BLEByteCharacteristic Ambient_Humidity("2A6F", BLERead | BLENotify);

// Create descriptors
BLEDescriptor RedDescriptor("2901", "8-bit Red Value");
BLEDescriptor GreenDescriptor("2901", "8-bit Green Value");
BLEDescriptor BlueDescriptor("2901", "8-bit Blue Value");
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

  BLE.setAdvertisedService(RGB_Color_Service);
  BLE.setAdvertisedService(Environmental_Sensing_Service);

  // add the characteristic to the service:
  RGB_Color_Service.addCharacteristic(Red);
  RGB_Color_Service.addCharacteristic(Green);
  RGB_Color_Service.addCharacteristic(Blue);

  Environmental_Sensing_Service.addCharacteristic(Ambient_Temperature);
  Environmental_Sensing_Service.addCharacteristic(Ambient_Pressure);
  Environmental_Sensing_Service.addCharacteristic(Ambient_Humidity);

  // add the descriptors to the characteristics:
  Red.addDescriptor(RedDescriptor);
  Green.addDescriptor(GreenDescriptor);
  Blue.addDescriptor(BlueDescriptor);

  Ambient_Temperature.addDescriptor(TemperatureDescriptor);
  Ambient_Pressure.addDescriptor(PressureDescriptor);
  Ambient_Humidity.addDescriptor(HumidityDescriptor);

  // add service
  BLE.addService(RGB_Color_Service);
  BLE.addService(Environmental_Sensing_Service);

  // set the initial value for the characteristic
  Red.writeValue(0);
  Green.writeValue(0);
  Blue.writeValue(0);

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

        if (Red.written()){
          updateRed();
        }

        if (Green.written()){
          updateGreen();
        }

        if (Blue.written()){
          updateBlue();
        }
    }
  }
}



void updateRed(){
  int RGBRed = Red.value();;
  Serial.print("Got value on Red of: ");
  Serial.println(RGBRed);
  analogWrite(LEDR, 0xFF - RGBRed);
}

void updateGreen(){
  int RGBGreen = Green.value();;
  Serial.print("Got value on Green of: ");
  Serial.println(RGBGreen);
  analogWrite(LEDG, 0xFF - RGBGreen);
}

void updateBlue(){
  int RGBBlue = Blue.value();
  Serial.print("Got value on Blue of: ");
  Serial.println(RGBBlue);
  analogWrite(LEDB, 0xFF - RGBBlue);
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