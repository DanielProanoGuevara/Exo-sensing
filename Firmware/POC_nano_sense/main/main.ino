//Include libraries
#include <Arduino_HS300x.h>

void setup() {
  Serial.begin(9600);
  while(!Serial); // Wait until serial is availabe
  
  // Block if sensor is not reckognized
  if (!HS300x.begin()){
    Serial.println("Failed to initialize humidity-temperature sensor!");
    while(1);
  }

}

void loop() {
  // read all the sensor values
  float temperature = HS300x.readTemperature();
  float humidity = HS300x.readHumidity();

  // print each of the sensor values
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" ºC");

  Serial.print("Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");

  // print empty line
  Serial.println();

  // wait 1 s (soft) to print again
  delay(1000);
}
