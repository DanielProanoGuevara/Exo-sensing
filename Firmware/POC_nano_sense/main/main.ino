//Include libraries
#include <ArduinoBLE.h>
#include <Arduino_HS300x.h>
#include <Arduino_LPS22HB.h>

// Create BLE services
BLEService RGB_Color_Service("cfa3772c-4442-48c2-846f-3824c3abb5f9");

// Create Characteristics
//  RGB Red Switch characteristic, read and writable
BLEByteCharacteristic Red("482fb2d1-33ed-4373-812e-f860c8899d08", BLERead | BLEWrite);
//  RGB Green Switch characteristic, read and writable
BLEByteCharacteristic Green("1c8daadc-c76e-465e-8779-a9f4981c2715", BLERead | BLEWrite);
//  RGB Blue Switch characteristic, read and writable
BLEByteCharacteristic Blue("7276b11e-4e7c-49b5-823f-59c2bee3dfde", BLERead | BLEWrite);

// Create descriptors
BLEDescriptor RedDescriptor("2901", "8-bit Red Value");
BLEDescriptor GreenDescriptor("2901", "8-bit Green Value");
BLEDescriptor BlueDescriptor("2901", "8-bit Blue Value");


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

  // add the characteristic to the service:
  RGB_Color_Service.addCharacteristic(Red);
  RGB_Color_Service.addCharacteristic(Green);
  RGB_Color_Service.addCharacteristic(Blue);

  // add the descriptors to the characteristics:
  Red.addDescriptor(RedDescriptor);
  Green.addDescriptor(GreenDescriptor);
  Blue.addDescriptor(BlueDescriptor);
  

  // add service
  BLE.addService(RGB_Color_Service);

  // set the initial value for the characteristic
  Red.writeValue(0);
  Green.writeValue(0);
  Blue.writeValue(0);

  // start advertising
  BLE.advertise();

  Serial.println("BLE RGB Peripheral");
}

void loop() {
  /*
  // read all the sensor values
  float temperature = HS300x.readTemperature();
  float humidity = HS300x.readHumidity();
  float pressure = BARO.readPressure();

  // print each of the sensor values
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" ºC");

  Serial.print("Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" kPa");

  // print empty line
  Serial.println();

  // wait 1 s (soft) to print again
  delay(1000);
  */

  //Hold read values
  int RGBRed = 0;
  int RGBGreen = 0;
  int RGBBlue = 0;

  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // while the centras is still connected to peripheral:
    while (central.connected()){
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (Red.written()){
        RGBRed = Red.value();
        Serial.print("Got value on Red of: ");
        Serial.println(RGBRed);
        analogWrite(LEDR, 0xFF - RGBRed);
      }

      if (Green.written()){
        RGBGreen = Green.value();
        Serial.print("Got value on Green of: ");
        Serial.println(RGBGreen);
        analogWrite(LEDG, 0xFF - RGBGreen);
      }

      if (Blue.written()){
        RGBBlue = Blue.value();
        Serial.print("Got value on Blue of: ");
        Serial.println(RGBBlue);
        analogWrite(LEDB, 0xFF - RGBBlue);
      }
    }

    // when the centra disconnects, print it out:
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
