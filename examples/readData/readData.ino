#include <Wire.h>
#include <Bonezegei_BME280.h>

// Initialize BME280 with default I2C address (0x76)
Bonezegei_BME280 bme(0x76);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  Serial.println("Bonezegei BME280 Sensor Initialized!");
}

void loop() {
  if (bme.read()) {
    Serial.print("Temperature: ");
    Serial.print(bme.getTemperature());
    Serial.println(" °C");

    Serial.print("Pressure: ");
    Serial.print(bme.getPressure() / 100.0f); // Convert Pa to hPa
    Serial.println(" hPa");

    Serial.print("Humidity: ");
    Serial.print(bme.getHumidity());
    Serial.println(" %");

    Serial.print("Approx Altitude: ");
    Serial.print(bme.getAltitude());
    Serial.println(" m");

    Serial.println("------------------------------------");
  } else {
    Serial.println("Failed to read from BME280 sensor!");
  }

  delay(2000);
}