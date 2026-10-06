
#include <DHT.h>
#include <Wire.h>
#include <BH1750.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// *** CHANGE THIS TO YOUR EXPORTED EDGE IMPULSE HEADER ***
#include <mohamed_said-project-1_inferencing.h>

// =============================================================
//  WIFI + MQTT
// =============================================================
const char* WIFI_SSID   = "msi";
const char* WIFI_PASS   = "12345678";

const char* MQTT_SERVER = "da5e9c9f3af6499b92bdd5819d675d8c.s1.eu.hivemq.cloud";
const int   MQTT_PORT   = 8883;
const char* MQTT_USER   = "mohamedsaid";
const char* MQTT_PASS   = "13025651aB";

const char* TOPIC_DATA  = "greenhouse/data";      // ESP32 -> Node-RED
const char* TOPIC_CMD   = "greenhouse/commands";  // Node-RED -> ESP32

// =============================================================
//  PINS
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
//  DRIVERS
// =============================================================
#define DHTTYPE DHT11
DHT dht(PIN_DHT, DHTTYPE);
BH1750 lightMeter;

WiFiClientSecure espClient;
PubSubClient client(espClient);

// =============================================================
//  SETTINGS
// =============================================================
const float ANOMALY_THRESHOLD = 1.0f;

// Mode: false=AUTO, true=MANUAL
volatile bool manualMode = false;

// User setpoints (from dashboard)
float sp_temp_low  = 18.0f;
float sp_temp_high = 26.0f;
float sp_hum_low   = 50.0f;
float sp_lux_low   = 200.0f;
float sp_moist_low = 30.0f;

// =============================================================
//  WATER FLOW
// =============================================================
volatile uint32_t flowPulseCount = 0;

void IRAM_ATTR flowPulseISR() {
  flowPulseCount++;
}

float calculateFlowRate(uint32_t elapsedMs) {
  uint32_t pulses = flowPulseCount;
  flowPulseCount = 0;
  if (elapsedMs == 0) return 0.0f;
  float freq = (pulses * 1000.0f) / elapsedMs; // pulses/sec
  return freq / 7.5f; // L/min
}

// =============================================================
//  SHARED STATE
// =============================================================
struct SharedData {
  float temp;
  float humid;
  float lux;
  float moist;
  float water_flow;

  int heater_state;
  int fan_state;
  int pump_state;
  int light_state;
  int humid_state;

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
//  HELPERS
// =============================================================
void relayWrite(int pin, bool turnOn) {
  // active-low relay: ON -> LOW, OFF -> HIGH
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
//  WIFI
// =============================================================
void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 50) {
    delay(400);
    Serial.print(".");
    tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("WiFi connected: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi failed -> restart");
    ESP.restart();
  }
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<512> doc;  //Converts raw MQTT bytes to JSON object (doc)
  DeserializationError err = deserializeJson(doc, payload, length); // analyze JSON
  if (err) {
    Serial.print("MQTT JSON parse error: ");
    Serial.println(err.c_str());// Outputs something like "InvalidInput" or "NoMemory"
    return;
  }

  // 1) mode
  if (doc.containsKey("mode")) {
    String m = doc["mode"].as<String>();
    m.toUpperCase();
    if (m == "MANUAL") manualMode = true;
    else if (m == "AUTO") manualMode = false;
    Serial.print("Mode set to: ");
    Serial.println(manualMode ? "MANUAL" : "AUTO");
  }

  // 2) setpoints
  if (doc.containsKey("temp_low"))  sp_temp_low  = doc["temp_low"];
  if (doc.containsKey("temp_high")) sp_temp_high = doc["temp_high"];
  if (doc.containsKey("hum_low"))   sp_hum_low   = doc["hum_low"];
  if (doc.containsKey("lux_low"))   sp_lux_low   = doc["lux_low"];
  if (doc.containsKey("moist_low")) sp_moist_low = doc["moist_low"];

  // sanity
  if (sp_temp_high < sp_temp_low) {
    float t = sp_temp_low;
    sp_temp_low = sp_temp_high;
    sp_temp_high = t;
  }

  // 3) manual actuator commands
  if (manualMode) {
    if (doc.containsKey("heater")) relayWrite(PIN_HEATER, (int)doc["heater"]);
    if (doc.containsKey("fan"))    relayWrite(PIN_FAN,    (int)doc["fan"]);
    if (doc.containsKey("pump"))   relayWrite(PIN_PUMP,   (int)doc["pump"]);
    if (doc.containsKey("humid"))  relayWrite(PIN_HUMID,  (int)doc["humid"]);
    if (doc.containsKey("light"))  relayWrite(PIN_LIGHT,  (int)doc["light"]);
    Serial.println("Manual command applied.");
  } else {
    if (doc.containsKey("heater") || doc.containsKey("fan") || doc.containsKey("pump") ||
        doc.containsKey("humid")  || doc.containsKey("light")) {
      Serial.println("AUTO mode active: manual command ignored.");
    }
  }
}

// =============================================================
//  MQTT RECONNECT
// =============================================================
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to HiveMQ...");
    String cid = "ESP32S3_GH_" + String((uint32_t)ESP.getEfuseMac(), HEX); // unique client ID using MAC

    if (client.connect(cid.c_str(), MQTT_USER, MQTT_PASS)) {
      Serial.println("Connected");
      client.subscribe(TOPIC_CMD);
      Serial.print("Subscribed: ");
      Serial.println(TOPIC_CMD);
    } else {
      Serial.print("Failed rc=");
      Serial.print(client.state());
      Serial.println(" retry 5s");
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
  }
}

// =============================================================
//  TASK: SENSORS
// =============================================================
void TaskSensors(void* pvParameters) {
  uint32_t lastFlowCheck = millis();

  for (;;) {
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    float l = lightMeter.readLightLevel();
    float m = readSoilMoisture();

    uint32_t now = millis();
    uint32_t elapsed = now - lastFlowCheck;
    float w = calculateFlowRate(elapsed);
    lastFlowCheck = now;

    if (isnan(t) || t < -40 || t > 80) t = -999.0f; // unrealistic temp -> sensor error
    if (isnan(h) || h < 0 || h > 100) h = -999.0f;
    if (l < 0) l = 0;

    int heater = relayRead(PIN_HEATER);
    int fan    = relayRead(PIN_FAN);
    int pump   = relayRead(PIN_PUMP);
    int light  = relayRead(PIN_LIGHT);
    int humid  = relayRead(PIN_HUMID);

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      systemState.temp         = t;
      systemState.humid        = h;
      systemState.lux          = l;
      systemState.moist        = m;
      systemState.water_flow   = w;
      systemState.heater_state = heater;
      systemState.fan_state    = fan;
      systemState.pump_state   = pump;
      systemState.light_state  = light;
      systemState.humid_state  = humid;
      xSemaphoreGive(xMutex);
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// =============================================================
//  TASK: AI
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
      if (t == -999.0f)                     diag = "CRITICAL: Temperature sensor failure";
      else if (h == -999.0f)                diag = "CRITICAL: Humidity sensor failure";
      else if (heater_state == 1 && t < 15) diag = "CRITICAL: Heater failure";
      else if (humid_state == 1 && h < 40)  diag = "CRITICAL: Humidifier failure";
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
//  TASK: CONTROL
// =============================================================
void TaskControl(void* pvParameters) {
  for (;;) {
    float t, h, l, m, w, score;
    String diag;

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      t     = systemState.temp;
      h     = systemState.humid;
      l     = systemState.lux;
      m     = systemState.moist;
      w     = systemState.water_flow;
      score = systemState.anomaly_score;
      diag  = systemState.diagnosis;
      xSemaphoreGive(xMutex);
    }

    bool safety_lockout = false;
    String system_status = "NORMAL";

    // Safety lockout
    if (diag.indexOf("Heater failure") != -1) {
      relayWrite(PIN_HEATER, false);
      relayWrite(PIN_FAN, true);    // force ON
      digitalWrite(PIN_LED_RED, HIGH);
      safety_lockout = true;
      system_status = "SAFETY STOP: HEATER BROKEN";
    } else if (diag.indexOf("sensor failure") != -1 || diag.indexOf("sensor may be") != -1) {
      relayWrite(PIN_HEATER, false);
      relayWrite(PIN_FAN, true);    // failsafe ventilation
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

    // Automatic control only when AUTO
    if (!safety_lockout && !manualMode) {
      // Temperature
      if (t != -999.0f) {
        if (t < sp_temp_low) {
          relayWrite(PIN_HEATER, true);
          relayWrite(PIN_FAN, false);
        } else if (t > sp_temp_high) {
          relayWrite(PIN_HEATER, false);
          relayWrite(PIN_FAN, true);
        } else {
          relayWrite(PIN_HEATER, false);
          relayWrite(PIN_FAN, false);
        }
      }

      // Humidity
      if (h != -999.0f) {
        if (h < sp_hum_low) relayWrite(PIN_HUMID, true);
        else                relayWrite(PIN_HUMID, false);
      }

      // Light
      if (l < sp_lux_low) relayWrite(PIN_LIGHT, true);
      else                relayWrite(PIN_LIGHT, false);

      // Pump
      if (m < sp_moist_low) relayWrite(PIN_PUMP, true);
      else                  relayWrite(PIN_PUMP, false);
    }

    Serial.println("\n================================================");
    Serial.println("SMART GREENHOUSE");
    Serial.println("================================================");
    Serial.printf("Mode: %s\n", manualMode ? "MANUAL" : "AUTO");
    Serial.printf("Setpoints: Tlow=%.1f Thigh=%.1f Hlow=%.1f LuxLow=%.1f MoistLow=%.1f\n",
                  sp_temp_low, sp_temp_high, sp_hum_low, sp_lux_low, sp_moist_low);

    if (t == -999.0f) Serial.println("TEMP: SENSOR ERROR");
    else Serial.printf("TEMP: %.1f C | HUM: %.1f %%\n", t, h);

    Serial.printf("LUX: %.1f | MOIST: %.1f %% | FLOW: %.2f L/min\n", l, m, w);
    Serial.printf("AI score: %.2f | %s\n", score, diag.c_str());
    Serial.printf("Status: %s\n", system_status.c_str());
    Serial.println("================================================");

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// =============================================================
//  TASK: MQTT
// =============================================================
void TaskMQTT(void* pvParameters) {
  for (;;) {
    if (!client.connected()) reconnectMQTT();
    client.loop();

    StaticJsonDocument<768> doc;

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      // sensors
      if (systemState.temp == -999.0f) doc["temp"] = nullptr;
      else                             doc["temp"] = systemState.temp;

      if (systemState.humid == -999.0f) doc["humid"] = nullptr;
      else                              doc["humid"] = systemState.humid;

      doc["lux"]           = systemState.lux;
      doc["moist"]         = systemState.moist;
      doc["water_flow"]    = systemState.water_flow;

      // actuator states
      doc["heater_state"]  = systemState.heater_state;
      doc["fan_state"]     = systemState.fan_state;
      doc["pump_state"]    = systemState.pump_state;
      doc["light_state"]   = systemState.light_state;
      doc["humid_state"]   = systemState.humid_state;

      // AI
      doc["anomaly_score"] = systemState.anomaly_score;
      doc["diagnosis"]     = systemState.diagnosis;

      xSemaphoreGive(xMutex);
    }

    // mode + setpoints
    doc["mode"]      = manualMode ? "MANUAL" : "AUTO";
    doc["temp_low"]  = sp_temp_low;
    doc["temp_high"] = sp_temp_high;
    doc["hum_low"]   = sp_hum_low;
    doc["lux_low"]   = sp_lux_low;
    doc["moist_low"] = sp_moist_low;

    char buffer[1024];
    size_t n = serializeJson(doc, buffer, sizeof(buffer));

    if (n == 0) {
      Serial.println("MQTT serialize failed");
    } else {
      bool ok = client.publish(TOPIC_DATA, buffer);
      Serial.printf("Publish [%s] bytes=%u -> %s\n",
                    TOPIC_DATA, (unsigned)n, ok ? "OK" : "FAIL");
      if (!ok) {
        Serial.printf("MQTT state=%d\n", client.state());
      }
    }

    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== GREENHOUSE BOOT ===");

  Serial.printf("Free heap: %u\n", (unsigned)ESP.getFreeHeap()); // dinamique memory check

  Wire.begin(PIN_SDA, PIN_SCL);
  dht.begin();

  // BH1750 check
  Wire.beginTransmission(0x23);
  if (Wire.endTransmission() == 0) {
    Serial.println("BH1750 found");
    lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  } else {
    Serial.println("BH1750 not found (check wiring)");
  }

  // Pins
  pinMode(PIN_HEATER, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_PUMP, OUTPUT);
  pinMode(PIN_HUMID, OUTPUT);
  pinMode(PIN_LIGHT, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_WATER, INPUT_PULLUP);

  // All OFF
  relayWrite(PIN_HEATER, false);
  relayWrite(PIN_FAN, false);
  relayWrite(PIN_PUMP, false);
  relayWrite(PIN_HUMID, false);
  relayWrite(PIN_LIGHT, false);
  digitalWrite(PIN_LED_RED, LOW);

  attachInterrupt(digitalPinToInterrupt(PIN_WATER), flowPulseISR, RISING);

  connectWiFi();

  // MQTT/TLS
  espClient.setInsecure(); // for quick setup. Production -> load CA cert.
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(onMqttMessage);
  client.setBufferSize(1024); // important for bigger JSON

  xMutex = xSemaphoreCreateMutex();
  if (xMutex == NULL) {
    Serial.println("Mutex creation failed");
    while (1) delay(1000);
  }

  // Tasks
  xTaskCreatePinnedToCore(TaskSensors, "Sensors", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskControl, "Control", 6144, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskMQTT,    "MQTT",    6144, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskAI,      "AI",      8192, NULL, 1, NULL, 0);

  Serial.println("All tasks started.");
}

void loop() {
  vTaskDelete(NULL);
}