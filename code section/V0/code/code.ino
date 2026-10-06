#include <Wire.h>
#include <DHT11.h>
#include <BH1750.h>
#include <SimpleTimer.h>
#include <WiFiManager.h>
#include <WiFi.h>

SimpleTimer timer;

// Outputs (relays) - verify these are valid GPIOs for your ESP32 board
#define PIN_HEATER_FAN_CTRL   2
#define PIN_HEATER_FAN_SWITCH 3
#define PIN_WATER_PUMP        4
#define PIN_HUMIDIFIERS       5
#define PIN_GROW_LIGHTS       6

// Inputs (ESP32-safe choices)
#define PIN_DHT11_SENSOR      16     // avoid 6-11
#define PIN_SOIL_MOIST_SENSOR 34     // ADC1 pin (works with Wi-Fi)

DHT11 dht11(PIN_DHT11_SENSOR);
BH1750 lightMeter;

// Thresholds
float min_temp  = 18.0;
float max_temp  = 28.0;
float min_humid = 40.0;
float min_moist = 35.0;
float min_lux   = 200.0;

// Pump timing
unsigned long pump_duration_ms = 5000;
unsigned long pump_start_ms = 0;

// Readings
float tempC = NAN;
float hum   = NAN;
float moist = NAN;
float lux   = NAN;

// Relay helpers (active-low relay modules)
void relayOn(int pin)  { digitalWrite(pin, LOW); }
void relayOff(int pin) { digitalWrite(pin, HIGH); }
bool relayIsOn(int pin){ return digitalRead(pin) == LOW; }

float readSoilMoistPct() {
  int raw = analogRead(PIN_SOIL_MOIST_SENSOR);
  return 100.0f - (100.0f * (float)raw / 4095.0f); // ESP32 ADC is 12-bit by default
}

void readSensors() {
  float t = dht11.readTemperature();
  float h = dht11.readHumidity();

  // If your DHT library returns invalid values, guard here.
  // (Some libs return 0 on error; some return NAN; adapt as needed.)
  if (!isnan(t)) tempC = t;
  if (!isnan(h)) hum   = h;

  moist = readSoilMoistPct();
  lux   = lightMeter.readLightLevel();

  Serial.print("TempC=");  Serial.print(tempC, 1);
  Serial.print(" Hum%=");  Serial.print(hum, 0);
  Serial.print(" Moist%=");Serial.print(moist, 0);
  Serial.print(" Lux=");   Serial.println(lux, 0);
}

void controlTemperature() {
  if (isnan(tempC)) return;

  if (tempC < min_temp) {
    relayOn(PIN_HEATER_FAN_CTRL);
    digitalWrite(PIN_HEATER_FAN_SWITCH, HIGH); // HEAT direction
    Serial.println("Heater ON");
  } else if (tempC > max_temp) {
    relayOn(PIN_HEATER_FAN_CTRL);
    digitalWrite(PIN_HEATER_FAN_SWITCH, LOW);  // FAN direction
    Serial.println("Fan ON");
  } else {
    relayOff(PIN_HEATER_FAN_CTRL);
    Serial.println("Temp OK - System OFF");
  }
}

void controlHumidity() {
  if (isnan(hum)) return;

  if (hum < min_humid) {
    relayOn(PIN_HUMIDIFIERS);
    Serial.println("Humidifier ON");
  } else {
    relayOff(PIN_HUMIDIFIERS);
    Serial.println("Humidifier OFF");
  }
}

void controlIrrigation() {
  if (isnan(moist)) return;

  if (!relayIsOn(PIN_WATER_PUMP)) {
    if (moist < min_moist) {
      relayOn(PIN_WATER_PUMP);
      pump_start_ms = millis();
      Serial.println("Irrigation ON");
    }
  } else {
    if ((millis() - pump_start_ms >= pump_duration_ms) || moist >= min_moist) {
      relayOff(PIN_WATER_PUMP);
      Serial.println("Irrigation OFF");
    }
  }
}

void controlLight() {
  if (isnan(lux)) return;

  if (lux < min_lux) {
    relayOn(PIN_GROW_LIGHTS);
    Serial.println("Grow Light ON");
  } else {
    relayOff(PIN_GROW_LIGHTS);
    Serial.println("Grow Light OFF");
  }
}

void setup() {
  Serial.begin(115200);

  // WiFiManager
  WiFi.mode(WIFI_STA);
  WiFiManager wm;

  // Keep this commented for normal operation; use only when you want to wipe creds.
  wm.resetSettings();

  bool res;
  res = wm.autoConnect("AutoConnectAP", "password");
  if (!res) {
    Serial.println("Failed to connect");
  } 
  else {
    Serial.println("connected... :)");
  }

  pinMode(PIN_HEATER_FAN_CTRL, OUTPUT);
  pinMode(PIN_HEATER_FAN_SWITCH, OUTPUT);
  pinMode(PIN_WATER_PUMP, OUTPUT);
  pinMode(PIN_HUMIDIFIERS, OUTPUT);
  pinMode(PIN_GROW_LIGHTS, OUTPUT);

  relayOff(PIN_HEATER_FAN_CTRL);
  digitalWrite(PIN_HEATER_FAN_SWITCH, LOW); // explicit default
  relayOff(PIN_WATER_PUMP);
  relayOff(PIN_HUMIDIFIERS);
  relayOff(PIN_GROW_LIGHTS);

  // ESP32 ADC config (optional but recommended)
  analogReadResolution(12); // 0..4095

  Wire.begin();       // If using custom SDA/SCL, pass them here: Wire.begin(SDA, SCL)
  lightMeter.begin();

  timer.setInterval(2000, readSensors);
  timer.setInterval(3000, controlTemperature);
  timer.setInterval(5000, controlHumidity);
  timer.setInterval(1000, controlIrrigation);
  timer.setInterval(5000, controlLight);
}

void loop() {
  timer.run();
}