#include <Wire.h>
#include <DHT11.h>
#include <BH1750.h>
#include <SimpleTimer.h>
#include <WiFiManager.h>
#include <WiFi.h>

#include <WebServer.h>
#include <GreenhouseDashboard.h>

SimpleTimer timer;

// Web dashboard
WebServer server(80);
GreenhouseDashboard dashboard(server);

// Outputs (relays) - active LOW
#define HEATER_FAN        26   // relay: heater + fan together
#define FAN               27   // relay: fan only
#define PIN_WATER_PUMP    4
#define PIN_HUMIDIFIERS   5
#define PIN_GROW_LIGHTS   32

// Inputs
#define PIN_DHT11_SENSOR      16
#define PIN_SOIL_MOIST_SENSOR 34

DHT11 dht11(PIN_DHT11_SENSOR);
BH1750 lightMeter;

// ===== Thresholds =====
float min_temp  = 18.0;
float max_temp  = 28.0;
float min_humid = 40.0;
float min_moist = 35.0;
float min_lux   = 200.0;

// Pump timing
unsigned long pump_duration_ms = 5000;
unsigned long pump_start_ms = 0;

// Global sensor readings
float tempC = 0;
float hum   = 0;
float moist = 0;
float lux   = 0;

// ================= Relay Helpers =================
void relayOn(int pin)  { digitalWrite(pin, LOW); }
void relayOff(int pin) { digitalWrite(pin, HIGH); }
bool relayIsOn(int pin){ return digitalRead(pin) == LOW; }

// ================= Sensor Reading =================
float readSoilMoistPct() {
  int raw = analogRead(PIN_SOIL_MOIST_SENSOR);  // ESP32 ADC: 0..4095
  float pct = 100.0f - (100.0f * (float)raw / 4095.0f);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return pct;
}

void readSensors() {
  tempC = dht11.readTemperature();
  hum   = dht11.readHumidity();
  moist = readSoilMoistPct();
  lux   = lightMeter.readLightLevel();

  Serial.print("TempC=");   Serial.print(tempC, 1);
  Serial.print(" Hum%=");   Serial.print(hum, 0);
  Serial.print(" Moist%="); Serial.print(moist, 0);
  Serial.print(" Lux=");    Serial.println(lux, 0);
}

// ================= Temperature Control =================
// Two-switch heater logic:
// - temp < min_temp  -> HEATER_FAN ON (heater+fan), FAN OFF
// - temp > max_temp  -> FAN ON (fan only), HEATER_FAN OFF
// - else             -> both OFF (mid temp => OFF)
void controlTemperature() {
  if (tempC < min_temp) {
    relayOff(FAN);
    relayOn(HEATER_FAN);
    Serial.println("Temp LOW -> HEATER+FAN ON");
  }
  else if (tempC > max_temp) {
    relayOff(HEATER_FAN);
    relayOn(FAN);
    Serial.println("Temp HIGH -> FAN ONLY ON");
  }
  else {
    relayOff(HEATER_FAN);
    relayOff(FAN);
    Serial.println("Temp OK -> OFF");
  }
}

// ================= Humidity Control =================
void controlHumidity() {
  if (hum < min_humid) {
    relayOn(PIN_HUMIDIFIERS);
    Serial.println("Humidifier ON");
  } else {
    relayOff(PIN_HUMIDIFIERS);
    Serial.println("Humidifier OFF");
  }
}

// ================= Irrigation Control =================
void controlIrrigation() {
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

// ================= Light Control =================
void controlLight() {
  if (lux < min_lux) {
    relayOn(PIN_GROW_LIGHTS);
    Serial.println("Grow Light ON");
  } else {
    relayOff(PIN_GROW_LIGHTS);
    Serial.println("Grow Light OFF");
  }
}

// ================= Setup =================
void setup() {
  Serial.begin(115200);

  // ---- WiFiManager ----
  WiFi.mode(WIFI_STA);
  WiFiManager wm;

  // If you want to erase saved WiFi each boot, uncomment:
  // wm.resetSettings();

  bool res = wm.autoConnect("AutoConnectAP", "password");
  if (!res) {
    Serial.println("Failed to connect (WiFiManager)");
    // ESP.restart();
  } else {
    Serial.print("WiFi connected. IP: ");
    Serial.println(WiFi.localIP());
  }

  // ---- Pins ----
  pinMode(HEATER_FAN, OUTPUT);
  pinMode(FAN, OUTPUT);
  pinMode(PIN_WATER_PUMP, OUTPUT);
  pinMode(PIN_HUMIDIFIERS, OUTPUT);
  pinMode(PIN_GROW_LIGHTS, OUTPUT);

  relayOff(HEATER_FAN);
  relayOff(FAN);
  relayOff(PIN_WATER_PUMP);
  relayOff(PIN_HUMIDIFIERS);
  relayOff(PIN_GROW_LIGHTS);

  // ---- Sensors ----
  // Default ESP32 I2C pins: SDA=21, SCL=22
  Wire.begin();
  lightMeter.begin();

  // ---- Timers ----
  timer.setInterval(2000, readSensors);        // every 2 s
  timer.setInterval(3000, controlTemperature); // every 3 s
  timer.setInterval(5000, controlHumidity);    // every 5 s
  timer.setInterval(1000, controlIrrigation);  // every 1 s
  timer.setInterval(5000, controlLight);       // every 5 s

  // ---- Dashboard wiring ----
  dashboard.setSensors(&tempC, &hum, &moist, &lux);
  dashboard.setTemperatureSystemPins(HEATER_FAN, FAN);

  static const GreenhouseDashboard::Actuator ACTS[] = {
    {"heater_fan", "Heater+Fan", HEATER_FAN, true},
    {"fan",        "Fan Only",   FAN,        true},
    {"water_pump", "Water Pump", PIN_WATER_PUMP, true},
    {"humidifiers","Humidifier", PIN_HUMIDIFIERS, true},
    {"grow_lights","Grow Lights", PIN_GROW_LIGHTS, true},
  };
  dashboard.setActuators(ACTS, sizeof(ACTS) / sizeof(ACTS[0]));
  dashboard.begin("Greenhouse Dashboard");

  server.begin();
  Serial.println("Dashboard ready. Open: http://<ESP32-IP>/");
}

// ================= Loop =================
void loop() {
  timer.run();
  server.handleClient();
}