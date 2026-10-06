#include <DHT.h>
#include <Wire.h>
#include <BH1750.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <ArduinoOTA.h>

// --- NEW RTC LIBRARIES (DS1302) ---
#include <ThreeWire.h>  
#include <RtcDS1302.h>


#include <mohamed_said-project-1_inferencing.h>

// =============================================================
// WIFI + MQTT (LOCAL BROKER)
// =============================================================
const char* WIFI_SSID   = "GreenhouseNet";
const char* WIFI_PASS   = "12345678";
const char* MQTT_SERVER = "192.168.4.1";
const int   MQTT_PORT   = 1883;

const char* TOPIC_DATA  = "greenhouse/data";
const char* TOPIC_CMD   = "greenhouse/commands";

// =============================================================
// PINS
// =============================================================
#define PIN_SDA      8   // I2C for BH1750
#define PIN_SCL      9   // I2C for BH1750
#define PIN_DHT      4
#define PIN_SOIL     14
#define PIN_WATER    10
#define PIN_HEATER   5
#define PIN_FAN      6
#define PIN_PUMP     7
#define PIN_HUMID    15
#define PIN_LIGHT    16
#define PIN_LED_RED  42
#define PIN_VOLTAGE  3   

// DS1302 RTC Pins
#define PIN_RTC_CLK  11
#define PIN_RTC_DAT  12
#define PIN_RTC_RST  13

// =============================================================
// DRIVERS & GLOBALS
// =============================================================
#define DHTTYPE DHT22
DHT dht(PIN_DHT, DHTTYPE);
BH1750 lightMeter;
WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;

// Initialize DS1302
ThreeWire myWire(PIN_RTC_DAT, PIN_RTC_CLK, PIN_RTC_RST); // DAT, CLK, RST
RtcDS1302<ThreeWire> rtc(myWire);

// =============================================================
// SETTINGS
// =============================================================
const float ANOMALY_THRESHOLD = 3.0f; 

volatile bool manualMode = false;

// Setpoints (Overwritten by Flash Memory)
float sp_temp_low  = 18.0f;
float sp_temp_high = 26.0f;
float sp_hum_low   = 50.0f;
float sp_lux_low   = 50.0f;
float sp_moist_low = 30.0f;

volatile int cmd_heater = 0;
volatile int cmd_fan    = 0;
volatile int cmd_pump   = 0;
volatile int cmd_humid  = 0;
volatile int cmd_light  = 0;

// =============================================================
// SHARED STATE
// =============================================================
struct SharedData {
  float temp, humid, lux, moist, water_flow, mains_voltage;
  int heater_state, fan_state, pump_state, light_state, humid_state;
  float anomaly_score;
  String diagnosis;
  int current_hour;
};

SharedData systemState = {
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
  0, 0, 0, 0, 0,
  0.0f, "Booting...", 12
};

SemaphoreHandle_t xMutex = NULL;
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

// =============================================================
// HELPERS
// =============================================================
void relayWrite(int pin, bool turnOn) {
  if (pin == PIN_HUMID) digitalWrite(pin, turnOn ? HIGH : LOW);
  else digitalWrite(pin, turnOn ? LOW : HIGH);
}

int relayRead(int pin) {
  if (pin == PIN_HUMID) return (digitalRead(pin) == HIGH) ? 1 : 0;
  else return (digitalRead(pin) == LOW) ? 1 : 0;
}

float readSoilMoisture() {
  const int raw_dry = 3200, raw_wet = 1100;
  int raw = analogRead(PIN_SOIL);
  float pct = (float)(raw_dry - raw) / (float)(raw_dry - raw_wet) * 100.0f;
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return pct;
}

volatile uint32_t flowPulseCount = 0;
void IRAM_ATTR flowPulseISR() { flowPulseCount++; }
float calculateFlowRate(uint32_t elapsedMs) {
  uint32_t pulses = flowPulseCount; flowPulseCount = 0;
  if (elapsedMs == 0) return 0.0f;
  return ((pulses * 1000.0f) / elapsedMs) / 7.5f; 
}

float readACVoltage() {
  uint32_t start_time = millis();
  int max_val = 0, min_val = 4095;
  while((millis() - start_time) < 40) { // 40ms = 2 full AC cycles
    int val = analogRead(PIN_VOLTAGE);
    if(val > max_val) max_val = val;
    if(val < min_val) min_val = val;
  }
  float v_pp = ((max_val - min_val) * 3.3f) / 4095.0f;
  float v_rms = v_pp / 2.828f;
  float CALIBRATION_FACTOR = 237.0f; // Calibrated for 240V
  float actual_voltage = v_rms * CALIBRATION_FACTOR;
  if (actual_voltage < 10.0f) actual_voltage = 0.0f; 
  return actual_voltage;
}

// =============================================================
// MQTT & WIFI
// =============================================================
void connectWiFi() {
  WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) delay(500);
}

void reconnectMQTT() {
  if (WiFi.status() == WL_CONNECTED && client.connect("ESP32_GH")) {
    client.subscribe(TOPIC_CMD);
  }
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, payload, length)) return;

  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (doc.containsKey("mode")) manualMode = (doc["mode"].as<String>() == "MANUAL");

    bool configChanged = false;
    if (doc.containsKey("temp_low"))  { sp_temp_low  = doc["temp_low"]; configChanged = true; }
    if (doc.containsKey("temp_high")) { sp_temp_high = doc["temp_high"]; configChanged = true; }
    if (doc.containsKey("hum_low"))   { sp_hum_low   = doc["hum_low"]; configChanged = true; }
    if (doc.containsKey("lux_low"))   { sp_lux_low   = doc["lux_low"]; configChanged = true; }
    if (doc.containsKey("moist_low")) { sp_moist_low = doc["moist_low"]; configChanged = true; }
    if (sp_temp_high < sp_temp_low) { float t = sp_temp_low; sp_temp_low = sp_temp_high; sp_temp_high = t; }

    if (configChanged) {
      preferences.putFloat("sp_t_l", sp_temp_low); preferences.putFloat("sp_t_h", sp_temp_high);
      preferences.putFloat("sp_h_l", sp_hum_low); preferences.putFloat("sp_l_l", sp_lux_low);
      preferences.putFloat("sp_m_l", sp_moist_low);
    }

    if (doc.containsKey("heater")) cmd_heater = doc["heater"] ? 1 : 0;
    if (doc.containsKey("fan"))    cmd_fan    = doc["fan"] ? 1 : 0;
    if (doc.containsKey("pump"))   cmd_pump   = doc["pump"] ? 1 : 0;
    if (doc.containsKey("humid"))  cmd_humid  = doc["humid"] ? 1 : 0;
    if (doc.containsKey("light"))  cmd_light  = doc["light"] ? 1 : 0;

    xSemaphoreGive(xMutex);
  }
}

// =============================================================
// TASKS
// =============================================================
void TaskSensors(void* pvParameters) {
  uint32_t lastFlowCheck = millis();
  for (;;) {
    float t = dht.readTemperature(); float h = dht.readHumidity();
    float l = lightMeter.readLightLevel(); float m = readSoilMoisture();
    uint32_t now = millis(); float w = calculateFlowRate(now - lastFlowCheck); lastFlowCheck = now;
    float v_ac = readACVoltage();
    
    // Get Time from DS1302
    RtcDateTime now_time = rtc.GetDateTime();

    if (isnan(t) || t < -40 || t > 80) t = -999.0f;
    if (isnan(h) || h < 0 || h > 100) h = -999.0f;
    if (l < 0) l = 0;

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      systemState.temp = t; systemState.humid = h; systemState.lux = l;
      systemState.moist = m; systemState.water_flow = w; systemState.mains_voltage = v_ac;
      systemState.heater_state = relayRead(PIN_HEATER); systemState.fan_state = relayRead(PIN_FAN);
      systemState.pump_state = relayRead(PIN_PUMP); systemState.light_state = relayRead(PIN_LIGHT);
      systemState.humid_state = relayRead(PIN_HUMID); systemState.current_hour = now_time.Hour(); 
      xSemaphoreGive(xMutex);
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void TaskAI(void* pvParameters) {
  for (;;) {
    float t, h, l, m, w; int pump_state, heater_state, humid_state, light_state;
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      t = systemState.temp; h = systemState.humid; l = systemState.lux; m = systemState.moist; w = systemState.water_flow;
      pump_state = systemState.pump_state; heater_state = systemState.heater_state; 
      humid_state = systemState.humid_state; light_state = systemState.light_state;
      xSemaphoreGive(xMutex);
    }

    for (size_t i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; i += 5) {
      features[i] = t; if (i+1 < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) features[i+1] = h;
      if (i+2 < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) features[i+2] = m;
      if (i+3 < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) features[i+3] = l;
      if (i+4 < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) features[i+4] = w;
    }

    signal_t signal; numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    ei_impulse_result_t result = { 0 }; run_classifier(&signal, &result, false);

    float score = result.anomaly; String diag = "Normal";

    if (t == -999.0f) diag = "CRITICAL: Temperature sensor failure";
    else if (h == -999.0f) diag = "CRITICAL: Humidity sensor failure";
    else if (pump_state == 1 && w < 0.1f) diag = "CRITICAL: Pipe blocked (pump ON, no flow)";
    else if (pump_state == 0 && w > 0.1f) diag = "CRITICAL: Water leak detected";
    else if (heater_state == 1 && t < 15.0f) diag = "CRITICAL: Heater failure";
    else if (humid_state == 1 && h < 40.0f) diag = "CRITICAL: Humidifier failure";
    else if (score > ANOMALY_THRESHOLD) diag = "WARNING: AI detected abnormal environment";

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      systemState.anomaly_score = score; systemState.diagnosis = diag;
      xSemaphoreGive(xMutex);
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void TaskControl(void* pvParameters) {
  for (;;) {
    float t, h, l, m, v_ac; String diag; bool modeManual; int hr;
    float tLow, tHigh, hLow, luxLow, moistLow; int cHeater, cFan, cPump, cHumid, cLight;

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      t = systemState.temp; h = systemState.humid; l = systemState.lux; m = systemState.moist; 
      v_ac = systemState.mains_voltage; diag = systemState.diagnosis; hr = systemState.current_hour;
      modeManual = manualMode; tLow = sp_temp_low; tHigh = sp_temp_high; hLow = sp_hum_low; 
      luxLow = sp_lux_low; moistLow = sp_moist_low;
      cHeater = cmd_heater; cFan = cmd_fan; cPump = cmd_pump; cHumid = cmd_humid; cLight = cmd_light;
      xSemaphoreGive(xMutex);
    }

    bool safety_lockout = false;
    
    // BROWNOUT PROTECTION
    if (v_ac > 10.0f && v_ac < 190.0f) { 
      relayWrite(PIN_HEATER, false); relayWrite(PIN_PUMP, false); relayWrite(PIN_FAN, false);
      digitalWrite(PIN_LED_RED, HIGH); safety_lockout = true; 
    } 
    else if (diag.indexOf("CRITICAL") != -1) {
      relayWrite(PIN_HEATER, false); relayWrite(PIN_PUMP, false); 
      digitalWrite(PIN_LED_RED, HIGH); safety_lockout = true;
    } else {
      digitalWrite(PIN_LED_RED, LOW);
    }

    if (!safety_lockout) {
      if (modeManual) {
        relayWrite(PIN_HEATER, cHeater); relayWrite(PIN_FAN, cFan); relayWrite(PIN_PUMP, cPump);
        relayWrite(PIN_HUMID, cHumid); relayWrite(PIN_LIGHT, cLight);
      } else {
        
        // 1. THERMAL & MOISTURE
        if (t != -999.0f) {
          if (t < tLow) { relayWrite(PIN_HEATER, true); relayWrite(PIN_FAN, false); }
          else if (t > tHigh) { relayWrite(PIN_HEATER, false); relayWrite(PIN_FAN, true); }
          else { relayWrite(PIN_HEATER, false); relayWrite(PIN_FAN, false); }
        }
        if (h != -999.0f) relayWrite(PIN_HUMID, (h < hLow));
        relayWrite(PIN_PUMP,  (m < moistLow));
        
        bool isDaytime = true ; // 6 AM to 8 PM
        bool current_light = (relayRead(PIN_LIGHT) == 1);
        
        if (isDaytime) {
          if (!current_light && (l < luxLow)) {
            relayWrite(PIN_LIGHT, true);
          } 
          else if (current_light && (l > (luxLow + 150.0f))) {
            relayWrite(PIN_LIGHT, false);
          }
        } else {
          // It is Nighttime -> Force light OFF
          relayWrite(PIN_LIGHT, false);
        }
      }
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void TaskMQTT(void* pvParameters) {
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) connectWiFi();
    if (!client.connected()) reconnectMQTT();
    client.loop();

    StaticJsonDocument<768> doc;
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      
      if (systemState.temp == -999.0f) doc["temp"] = nullptr;
      else doc["temp"] = systemState.temp;

      if (systemState.humid == -999.0f) doc["humid"] = nullptr;
      else doc["humid"] = systemState.humid;

      doc["lux"] = systemState.lux; doc["moist"] = systemState.moist; 
      doc["water_flow"] = systemState.water_flow; doc["mains_voltage"] = systemState.mains_voltage;
      
      doc["heater_state"] = relayRead(PIN_HEATER); doc["fan_state"] = relayRead(PIN_FAN);
      doc["pump_state"] = relayRead(PIN_PUMP); doc["light_state"] = relayRead(PIN_LIGHT);
      doc["humid_state"] = relayRead(PIN_HUMID);
      
      doc["anomaly_score"] = systemState.anomaly_score; doc["diagnosis"] = systemState.diagnosis;
      doc["mode"] = manualMode ? "MANUAL" : "AUTO";
      doc["hour"] = systemState.current_hour; 
      
      doc["temp_low"] = sp_temp_low; doc["temp_high"] = sp_temp_high; doc["hum_low"] = sp_hum_low;
      doc["lux_low"] = sp_lux_low; doc["moist_low"] = sp_moist_low;

      doc["cmd_heater"] = cmd_heater; doc["cmd_fan"] = cmd_fan; doc["cmd_pump"] = cmd_pump;
      doc["cmd_humid"] = cmd_humid; doc["cmd_light"] = cmd_light;
      xSemaphoreGive(xMutex);
    }

    char buffer[1024]; size_t n = serializeJson(doc, buffer, sizeof(buffer));
    if (n > 0) client.publish(TOPIC_DATA, buffer);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// =============================================================
// SETUP & LOOP
// =============================================================
void setup() {
  Serial.begin(115200);
  preferences.begin("ghouse", false);
  sp_temp_low  = preferences.getFloat("sp_t_l", 18.0f);
  sp_temp_high = preferences.getFloat("sp_t_h", 26.0f);
  sp_hum_low   = preferences.getFloat("sp_h_l", 50.0f);
  sp_lux_low   = preferences.getFloat("sp_l_l", 50.0f);
  sp_moist_low = preferences.getFloat("sp_m_l", 30.0f);

  // Init Sensors
  Wire.begin(PIN_SDA, PIN_SCL);
  dht.begin();
  
  // Init DS1302
  rtc.Begin();
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  if (!rtc.IsDateTimeValid()) rtc.SetDateTime(compiled);
  if (!rtc.GetIsRunning()) rtc.SetIsRunning(true);
  
  Wire.beginTransmission(0x23);
  if (Wire.endTransmission() == 0) lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);

  // Init Pins
  pinMode(PIN_HEATER, OUTPUT); pinMode(PIN_FAN, OUTPUT); pinMode(PIN_PUMP, OUTPUT);
  pinMode(PIN_HUMID, OUTPUT); pinMode(PIN_LIGHT, OUTPUT); pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_WATER, INPUT_PULLUP);
  relayWrite(PIN_HEATER, false); relayWrite(PIN_FAN, false); relayWrite(PIN_PUMP, false);
  relayWrite(PIN_HUMID, false); relayWrite(PIN_LIGHT, false); digitalWrite(PIN_LED_RED, LOW);

  attachInterrupt(digitalPinToInterrupt(PIN_WATER), flowPulseISR, RISING);

  xMutex = xSemaphoreCreateMutex();
  connectWiFi();
  
  ArduinoOTA.setHostname("Smart-Greenhouse"); ArduinoOTA.begin();
  client.setServer(MQTT_SERVER, MQTT_PORT); client.setCallback(onMqttMessage);
  client.setBufferSize(1024); client.setKeepAlive(20); client.setSocketTimeout(3);

  // Start FreeRTOS Tasks
  xTaskCreatePinnedToCore(TaskSensors, "Sensors", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskControl, "Control", 6144, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskMQTT,    "MQTT",    6144, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskAI,      "AI",      8192, NULL, 1, NULL, 0);
}

void loop() {
  ArduinoOTA.handle();
  vTaskDelay(10 / portTICK_PERIOD_MS);
}