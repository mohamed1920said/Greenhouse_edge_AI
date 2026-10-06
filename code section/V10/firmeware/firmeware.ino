#include <DHT.h>
#include <Wire.h>
#include <BH1750.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
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
#define PIN_SDA      8
#define PIN_SCL      9
#define PIN_DHT      4
#define PIN_SOIL     14
#define PIN_WATER    10
#define PIN_HEATER   5
#define PIN_FAN      6
#define PIN_PUMP     7
#define PIN_HUMID    15
#define PIN_LIGHT    16
#define PIN_LED_RED  42

// =============================================================
// DRIVERS
// =============================================================
#define DHTTYPE DHT11
DHT dht(PIN_DHT, DHTTYPE);
BH1750 lightMeter;
WiFiClient espClient;
PubSubClient client(espClient);

// =============================================================
// SETTINGS
// =============================================================
const float ANOMALY_THRESHOLD = 1.0f;

// Mode: false=AUTO, true=MANUAL
volatile bool manualMode = false;

// Setpoints
float sp_temp_low  = 18.0f;
float sp_temp_high = 26.0f;
float sp_hum_low   = 50.0f;
float sp_lux_low   = 200.0f;
float sp_moist_low = 30.0f;

// Manual desired actuator commands (single source of truth)
volatile int cmd_heater = 0;
volatile int cmd_fan    = 0;
volatile int cmd_pump   = 0;
volatile int cmd_humid  = 0;
volatile int cmd_light  = 0;

// =============================================================
// WATER FLOW
// =============================================================
volatile uint32_t flowPulseCount = 0;
void IRAM_ATTR flowPulseISR() { flowPulseCount++; }

float calculateFlowRate(uint32_t elapsedMs) {
  uint32_t pulses = flowPulseCount;
  flowPulseCount = 0;
  if (elapsedMs == 0) return 0.0f;
  float freq = (pulses * 1000.0f) / elapsedMs; // pulses/sec
  return freq / 7.5f; // L/min
}

// =============================================================
// SHARED STATE
// =============================================================
struct SharedData {
  float temp, humid, lux, moist, water_flow;
  int heater_state, fan_state, pump_state, light_state, humid_state;
  float anomaly_score;
  String diagnosis;
};

SharedData systemState = {
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
  0, 0, 0, 0, 0,
  0.0f, "Booting..."
};

SemaphoreHandle_t xMutex = NULL;
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

// =============================================================
// HELPERS
// =============================================================
void relayWrite(int pin, bool turnOn) {
  // Active-low relay: ON->LOW, OFF->HIGH
  digitalWrite(pin, turnOn ? LOW : HIGH);
}

int relayRead(int pin) {
  return (digitalRead(pin) == LOW) ? 1 : 0;
}

float readSoilMoisture() {
  const int raw_dry = 3200;
  const int raw_wet = 1100;
  int raw = analogRead(PIN_SOIL);
  float pct = (float)(raw_dry - raw) / (float)(raw_dry - raw_wet) * 100.0f;
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return pct;
}

// =============================================================
// WIFI
// =============================================================
void connectWiFi() {
  Serial.println("Connecting WiFi...");
  WiFi.disconnect(true, true);
  delay(300);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 50) {
    delay(300);
    Serial.print(".");
    tries++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");
    Serial.print("SSID    : "); Serial.println(WiFi.SSID());
    Serial.print("Local IP: "); Serial.println(WiFi.localIP());
    Serial.print("Gateway : "); Serial.println(WiFi.gatewayIP());
    Serial.print("RSSI    : "); Serial.println(WiFi.RSSI());
  } else {
    Serial.println("WiFi failed -> restart");
    ESP.restart();
  }
}

// =============================================================
// MQTT CALLBACK
// =============================================================
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.print("MQTT JSON parse error: ");
    Serial.println(err.c_str());
    return;
  }

  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    // Mode
    if (doc.containsKey("mode")) {
      String m = doc["mode"].as<String>();
      m.toUpperCase();
      if (m == "MANUAL") manualMode = true;
      else if (m == "AUTO") manualMode = false;
    }

    // Setpoints
    if (doc.containsKey("temp_low"))  sp_temp_low  = doc["temp_low"];
    if (doc.containsKey("temp_high")) sp_temp_high = doc["temp_high"];
    if (doc.containsKey("hum_low"))   sp_hum_low   = doc["hum_low"];
    if (doc.containsKey("lux_low"))   sp_lux_low   = doc["lux_low"];
    if (doc.containsKey("moist_low")) sp_moist_low = doc["moist_low"];

    // Sanity
    if (sp_temp_high < sp_temp_low) {
      float t = sp_temp_low;
      sp_temp_low = sp_temp_high;
      sp_temp_high = t;
    }

    // Manual desired commands (store only, do not drive relay here)
    if (doc.containsKey("heater")) cmd_heater = ((int)doc["heater"]) ? 1 : 0;
    if (doc.containsKey("fan"))    cmd_fan    = ((int)doc["fan"]) ? 1 : 0;
    if (doc.containsKey("pump"))   cmd_pump   = ((int)doc["pump"]) ? 1 : 0;
    if (doc.containsKey("humid"))  cmd_humid  = ((int)doc["humid"]) ? 1 : 0;
    if (doc.containsKey("light"))  cmd_light  = ((int)doc["light"]) ? 1 : 0;

    xSemaphoreGive(xMutex);
  }

  Serial.print("MQTT cmd received | mode=");
  Serial.println(manualMode ? "MANUAL" : "AUTO");
}

// =============================================================
// MQTT RECONNECT
// =============================================================
void reconnectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;

  Serial.print("Connecting to MQTT ");
  Serial.print(MQTT_SERVER);
  Serial.print(":");
  Serial.print(MQTT_PORT);
  Serial.print(" ... ");

  String cid = "ESP32S3_GH_" + String((uint32_t)ESP.getEfuseMac(), HEX);

  if (client.connect(cid.c_str())) {
    Serial.println("OK");
    client.subscribe(TOPIC_CMD);
    Serial.print("Subscribed: ");
    Serial.println(TOPIC_CMD);
  } else {
    Serial.print("FAIL rc=");
    Serial.println(client.state());
  }
}

// =============================================================
// TASK: SENSORS
// =============================================================
void TaskSensors(void* pvParameters) {
  uint32_t lastFlowCheck = millis();

  for (;;) {
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    float l = lightMeter.readLightLevel();
    float m = readSoilMoisture();

    uint32_t now = millis();
    float w = calculateFlowRate(now - lastFlowCheck);
    lastFlowCheck = now;

    if (isnan(t) || t < -40 || t > 80) t = -999.0f;
    if (isnan(h) || h < 0 || h > 100) h = -999.0f;
    if (l < 0) l = 0;

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      systemState.temp = t;
      systemState.humid = h;
      systemState.lux = l;
      systemState.moist = m;
      systemState.water_flow = w;

      systemState.heater_state = relayRead(PIN_HEATER);
      systemState.fan_state    = relayRead(PIN_FAN);
      systemState.pump_state   = relayRead(PIN_PUMP);
      systemState.light_state  = relayRead(PIN_LIGHT);
      systemState.humid_state  = relayRead(PIN_HUMID);

      xSemaphoreGive(xMutex);
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// =============================================================
// TASK: AI
// =============================================================
void TaskAI(void* pvParameters) {
  for (;;) {
    float t, h, l, m, w;
    int heater_state, fan_state, pump_state, light_state, humid_state;

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      t = systemState.temp;
      h = systemState.humid;
      l = systemState.lux;
      m = systemState.moist;
      w = systemState.water_flow;

      heater_state = systemState.heater_state;
      fan_state    = systemState.fan_state;
      pump_state   = systemState.pump_state;
      light_state  = systemState.light_state;
      humid_state  = systemState.humid_state;
      xSemaphoreGive(xMutex);
    }

    for (size_t i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; i += 3) {
      features[i] = t;
      if (i + 1 < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) features[i + 1] = l;
      if (i + 2 < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) features[i + 2] = (float)heater_state;
    }

    signal_t signal;
    int err = numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
      vTaskDelay(5000 / portTICK_PERIOD_MS);
      continue;
    }

    ei_impulse_result_t result = { 0 };
    err = run_classifier(&signal, &result, false);
    if (err != 0) {
      vTaskDelay(5000 / portTICK_PERIOD_MS);
      continue;
    }

    float score = result.anomaly;
    String diag = "Normal";

    if (score > ANOMALY_THRESHOLD) {
      if (t == -999.0f) diag = "CRITICAL: Temperature sensor failure";
      else if (h == -999.0f) diag = "CRITICAL: Humidity sensor failure";
      else if (heater_state == 1 && t < 15) diag = "CRITICAL: Heater failure";
      else if (humid_state == 1 && h < 40) diag = "CRITICAL: Humidifier failure";
      else if (pump_state == 1 && m > 80 && w == 0.0f) diag = "CRITICAL: Pipe blocked (pump running, no flow)";
      else if (pump_state == 0 && w > 0.1f) diag = "CRITICAL: Water leak detected";
      else if (light_state == 1 && l > 5000.0f) diag = "WARNING: Light sensor may be saturated or broken";
      else diag = "WARNING: Unknown anomaly";
    }

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      systemState.anomaly_score = score;
      systemState.diagnosis = diag;
      xSemaphoreGive(xMutex);
    }

    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

// =============================================================
// TASK: CONTROL (single writer to relays)
// =============================================================
void TaskControl(void* pvParameters) {
  for (;;) {
    // Snapshot shared vars under mutex
    float t, h, l, m, w, score;
    String diag;
    bool modeManual;
    float tLow, tHigh, hLow, luxLow, moistLow;
    int cHeater, cFan, cPump, cHumid, cLight;

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      t = systemState.temp;
      h = systemState.humid;
      l = systemState.lux;
      m = systemState.moist;
      w = systemState.water_flow;
      score = systemState.anomaly_score;
      diag = systemState.diagnosis;

      modeManual = manualMode;

      tLow = sp_temp_low;
      tHigh = sp_temp_high;
      hLow = sp_hum_low;
      luxLow = sp_lux_low;
      moistLow = sp_moist_low;

      cHeater = cmd_heater;
      cFan    = cmd_fan;
      cPump   = cmd_pump;
      cHumid  = cmd_humid;
      cLight  = cmd_light;

      xSemaphoreGive(xMutex);
    }

    bool safety_lockout = false;
    String system_status = "NORMAL";

    // Safety lockout has highest priority
    if (diag.indexOf("Heater failure") != -1) {
      relayWrite(PIN_HEATER, false);
      relayWrite(PIN_FAN, true);
      digitalWrite(PIN_LED_RED, HIGH);
      safety_lockout = true;
      system_status = "SAFETY STOP: HEATER BROKEN";
    } else if (diag.indexOf("sensor failure") != -1 || diag.indexOf("sensor may be") != -1) {
      relayWrite(PIN_HEATER, false);
      relayWrite(PIN_FAN, true);
      digitalWrite(PIN_LED_RED, HIGH);
      safety_lockout = true;
      system_status = "FAILSAFE: SENSOR ERROR";
    } else if (diag.indexOf("Water leak") != -1) {
      relayWrite(PIN_PUMP, false);
      digitalWrite(PIN_LED_RED, HIGH);
      safety_lockout = true;
      system_status = "SAFETY STOP: WATER LEAK";
    } else if (score > ANOMALY_THRESHOLD) {
      digitalWrite(PIN_LED_RED, HIGH);
      system_status = "WARNING: ANOMALY";
    } else {
      digitalWrite(PIN_LED_RED, LOW);
    }

    // Normal control (only if not safety lockout)
    if (!safety_lockout) {
      if (modeManual) {
        // MANUAL: apply command states exactly
        relayWrite(PIN_HEATER, cHeater);
        relayWrite(PIN_FAN,    cFan);
        relayWrite(PIN_PUMP,   cPump);
        relayWrite(PIN_HUMID,  cHumid);
        relayWrite(PIN_LIGHT,  cLight);
      } else {
        // AUTO
        if (t != -999.0f) {
          if (t < tLow) {
            relayWrite(PIN_HEATER, true);
            relayWrite(PIN_FAN, false);
          } else if (t > tHigh) {
            relayWrite(PIN_HEATER, false);
            relayWrite(PIN_FAN, true);
          } else {
            relayWrite(PIN_HEATER, false);
            relayWrite(PIN_FAN, false);
          }
        }

        if (h != -999.0f) relayWrite(PIN_HUMID, (h < hLow));
        relayWrite(PIN_LIGHT, (l < luxLow));
        relayWrite(PIN_PUMP,  (m < moistLow));
      }
    }

    // Console
    Serial.println("\n================================================");
    Serial.println("SMART GREENHOUSE");
    Serial.println("================================================");
    Serial.printf("Mode: %s\n", modeManual ? "MANUAL" : "AUTO");
    Serial.printf("Setpoints: Tlow=%.1f Thigh=%.1f Hlow=%.1f LuxLow=%.1f MoistLow=%.1f\n",
                  tLow, tHigh, hLow, luxLow, moistLow);
    Serial.printf("ManualCmd: heater=%d fan=%d pump=%d humid=%d light=%d\n",
                  cHeater, cFan, cPump, cHumid, cLight);

    if (t == -999.0f) Serial.println("TEMP: SENSOR ERROR");
    else Serial.printf("TEMP: %.1f C | HUM: %.1f %%\n", t, h);

    Serial.printf("LUX: %.1f | MOIST: %.1f %% | FLOW: %.2f L/min\n", l, m, w);
    Serial.printf("AI score: %.2f | %s\n", score, diag.c_str());
    Serial.printf("Status: %s\n", system_status.c_str());
    Serial.printf("Actuators: Htr=%d Fan=%d Pump=%d Humid=%d Light=%d\n",
                  relayRead(PIN_HEATER), relayRead(PIN_FAN), relayRead(PIN_PUMP),
                  relayRead(PIN_HUMID), relayRead(PIN_LIGHT));
    Serial.println("================================================");

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// =============================================================
// TASK: MQTT
// =============================================================
void TaskMQTT(void* pvParameters) {
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) connectWiFi();
    if (!client.connected()) reconnectMQTT();
    client.loop();

    StaticJsonDocument<768> doc;

    bool modeManual;
    float tLow, tHigh, hLow, luxLow, moistLow;
    int cHeater, cFan, cPump, cHumid, cLight;

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      // sensors
      if (systemState.temp == -999.0f) doc["temp"] = nullptr;
      else                             doc["temp"] = systemState.temp;

      if (systemState.humid == -999.0f) doc["humid"] = nullptr;
      else                              doc["humid"] = systemState.humid;

      doc["lux"]        = systemState.lux;
      doc["moist"]      = systemState.moist;
      doc["water_flow"] = systemState.water_flow;

      // real relay states
      doc["heater_state"] = relayRead(PIN_HEATER);
      doc["fan_state"]    = relayRead(PIN_FAN);
      doc["pump_state"]   = relayRead(PIN_PUMP);
      doc["light_state"]  = relayRead(PIN_LIGHT);
      doc["humid_state"]  = relayRead(PIN_HUMID);

      // AI
      doc["anomaly_score"] = systemState.anomaly_score;
      doc["diagnosis"]     = systemState.diagnosis;

      // config snapshot
      modeManual = manualMode;
      tLow = sp_temp_low; tHigh = sp_temp_high; hLow = sp_hum_low;
      luxLow = sp_lux_low; moistLow = sp_moist_low;

      cHeater = cmd_heater; cFan = cmd_fan; cPump = cmd_pump; cHumid = cmd_humid; cLight = cmd_light;

      xSemaphoreGive(xMutex);
    }

    doc["mode"]      = modeManual ? "MANUAL" : "AUTO";
    doc["temp_low"]  = tLow;
    doc["temp_high"] = tHigh;
    doc["hum_low"]   = hLow;
    doc["lux_low"]   = luxLow;
    doc["moist_low"] = moistLow;

    // expose desired manual commands too
    doc["cmd_heater"] = cHeater;
    doc["cmd_fan"]    = cFan;
    doc["cmd_pump"]   = cPump;
    doc["cmd_humid"]  = cHumid;
    doc["cmd_light"]  = cLight;

    char buffer[1024];
    size_t n = serializeJson(doc, buffer, sizeof(buffer));

    if (n > 0) {
      bool ok = client.publish(TOPIC_DATA, buffer);
      Serial.printf("Publish [%s] bytes=%u -> %s\n",
                    TOPIC_DATA, (unsigned)n, ok ? "OK" : "FAIL");
    } else {
      Serial.println("MQTT serialize failed");
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// =============================================================
// SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  delay(1200);
  Serial.println("\n=== GREENHOUSE BOOT ===");

  Wire.begin(PIN_SDA, PIN_SCL);
  dht.begin();

  Wire.beginTransmission(0x23);
  if (Wire.endTransmission() == 0) {
    Serial.println("BH1750 found");
    lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  } else {
    Serial.println("BH1750 not found (check wiring)");
  }

  pinMode(PIN_HEATER, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_PUMP, OUTPUT);
  pinMode(PIN_HUMID, OUTPUT);
  pinMode(PIN_LIGHT, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_WATER, INPUT_PULLUP);

  // all OFF
  relayWrite(PIN_HEATER, false);
  relayWrite(PIN_FAN, false);
  relayWrite(PIN_PUMP, false);
  relayWrite(PIN_HUMID, false);
  relayWrite(PIN_LIGHT, false);
  digitalWrite(PIN_LED_RED, LOW);

  attachInterrupt(digitalPinToInterrupt(PIN_WATER), flowPulseISR, RISING);

  xMutex = xSemaphoreCreateMutex();
  if (xMutex == NULL) {
    Serial.println("Mutex creation failed");
    while (1) delay(1000);
  }

  connectWiFi();

  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(onMqttMessage);
  client.setBufferSize(1024);
  client.setKeepAlive(20);
  client.setSocketTimeout(3);

  xTaskCreatePinnedToCore(TaskSensors, "Sensors", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskControl, "Control", 6144, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskMQTT,    "MQTT",    6144, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskAI,      "AI",      8192, NULL, 1, NULL, 0);

  Serial.println("All tasks started.");
}

void loop() {
  vTaskDelete(NULL);
}