#include <Wire.h>
#include <BH1750.h>
#include <DHT.h>

// ===== DHT22 CONFIG =====
#define DHTPIN 2        // Digital pin
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// ===== SOIL SENSOR =====
#define SOIL_PIN A0     // Analog pin for Uno

// ===== BH1750 =====
BH1750 lightMeter;

void setup() {
  Serial.begin(9600);

  // I2C for BH1750
  Wire.begin();
  lightMeter.begin();

  // DHT22
  dht.begin();

  Serial.println("System Ready...");
}

void loop() {
  // ===== READ BH1750 =====
  float lux = lightMeter.readLightLevel();

  // ===== READ DHT22 =====
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // ===== READ SOIL =====
  int soilRaw = analogRead(SOIL_PIN);

  // Convert soil to percentage (adjust after calibration)
  int soilPercent = map(soilRaw, 1023, 300, 0, 100); 
  soilPercent = constrain(soilPercent, 0, 100);

  // ===== ERROR CHECK =====
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("DHT read error!");
  }

  // ===== PRINT =====
  Serial.println("----- DATA -----");

  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lux");

  Serial.print("Temp: ");
  Serial.print(temperature);
  Serial.println(" C");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Soil Raw: ");
  Serial.println(soilRaw);

  Serial.print("Soil Moisture: ");
  Serial.print(soilPercent);
  Serial.println(" %");

  Serial.println("----------------\n");

  delay(2000);
}