#include <mohamed_said-project-1_inferencing.h>

/* 
 * ARCHITECTURE:
 * CORE 1 (App Core): Handles Sensors, Relays, MQTT (Real-time operations)
 * CORE 0 (Pro Core): Handles Edge Impulse AI Inference (Heavy Computation)
 *
 * Publishes ALL status fields over MQTT:
 * water_flow, temp, humid, lux, moist, heater_state, fan_state, pump_state,
 * anomaly_score, diagnosis
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <Wire.h>
#include <BH1750.h>

// *** EDGE IMPULSE INCLUDE ***

// ================= WIFI + MQTT CONFIG =================
// WiFi (PC mobile hotspot)
const char* ssid     = "msi";
const char* password = "12345678";

// Cloud MQTT (example: HiveMQ Cloud)
const char* mqtt_server = "da5e9c9f3af6499b92bdd5819d675d8c.s1.eu.hivemq.cloud";   // e.g. "xxxx.s1.eu.hivemq.cloud"
const int   mqtt_port   = 8883;                // TLS port for cloud brokers
const char* mqtt_user   = "mohamedsaid";
const char* mqtt_pass   = "13025651aB";

// MQTT topics
const char* TOPIC_DATA = "greenhouse/data";
const char* TOPIC_CMD  = "greenhouse/commands";

// ================= PINS =================
#define PIN_Flow     2
#define PIN_SDA      8
#define PIN_SCL      9
#define PIN_DHT      4
#define PIN_SOIL     10

// NOTE: your relays appear to be active-LOW (LOW=ON, HIGH=OFF)
#define PIN_HEATER   5
#define PIN_FAN      6
#define PIN_PUMP     7

#define PIN_LED_RED  42

// ================= OBJECTS =================
DHT dht(PIN_DHT, DHT11);
BH1750 lightMeter;

WiFiClientSecure espClient;      // TLS
PubSubClient client(espClient);

// ================= SHARED DATA (MUTEX PROTECTED) =================
struct SharedData {
  float water_flow;
  float temp;
  float humid;
  float lux;
  float moist;

  int heater_state; // 1=ON, 0=OFF
  int fan_state;    // 1=ON, 0=OFF
  int pump_state;   // 1=ON, 0=OFF

  float anomaly_score;
  String diagnosis;
};

SharedData systemState;
SemaphoreHandle_t xMutex;

// AI Buffer
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

// ================= HELPERS =================
static inline int relayStateTo01(int pin) {
  // active-LOW relay: LOW means ON
  return (digitalRead(pin) == LOW) ? 1 : 0;
}

static inline void relayWrite01(int pin, int on01) {
  // active-LOW relay: ON->LOW, OFF->HIGH
  digitalWrite(pin, on01 ? LOW : HIGH);
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  // Optional: accept commands like:
  // {"heater":1,"fan":0,"pump":1}
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) return;

  if (doc.containsKey("heater")) relayWrite01(PIN_HEATER, (int)doc["heater"]);
  if (doc.containsKey("fan"))    relayWrite01(PIN_FAN,    (int)doc["fan"]);
  if (doc.containsKey("pump"))   relayWrite01(PIN_PUMP,   (int)doc["pump"]);
}

void reconnectMQTT() {
  while (!client.connected()) {
    // unique-ish client id
    String cid = "ESP32S3_GH_" + String((uint32_t)ESP.getEfuseMac(), HEX);

    if (client.connect(cid.c_str(), mqtt_user, mqtt_pass)) {
      client.subscribe(TOPIC_CMD);
    } else {
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
  }
}

// ================= TASKS =================

/**
 * TASK: SENSORS (Runs on CORE 1)
 */
void TaskSensors(void *pvParameters) {
  for (;;) {
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    float l = lightMeter.readLightLevel();

    // Soil moisture (calibration may be needed)
    int raw = analogRead(PIN_SOIL);
    float m = map(raw, 4095, 0, 0, 100);
    if (m < 0) m = 0;
    if (m > 100) m = 100;

    // Placeholder: real water flow typically uses pulse counting with interrupt
    float water_flow = (float)digitalRead(PIN_Flow);

    // Safety for DHT
    if (isnan(t)) t = 0;
    if (isnan(h)) h = 0;
    if (isnan(l)) l = 0;

    int heater_state = relayStateTo01(PIN_HEATER);
    int fan_state    = relayStateTo01(PIN_FAN);
    int pump_state   = relayStateTo01(PIN_PUMP);

    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      systemState.temp = t;
      systemState.humid = h;
      systemState.lux = l;
      systemState.moist = m;
      systemState.water_flow = water_flow;

      systemState.heater_state = heater_state;
      systemState.fan_state = fan_state;
      systemState.pump_state = pump_state;

      xSemaphoreGive(xMutex);
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

/**
 * TASK: AI INFERENCE (Runs on CORE 0)
 */
void TaskAI(void *pvParameters) {
  for (;;) {
    float t, l;
    int h_state;

    // snapshot
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      t = systemState.temp;
      l = systemState.lux;
      h_state = systemState.heater_state;
      xSemaphoreGive(xMutex);
    }

    // features: [t, l, heater_state, t, l, heater_state, ...]
    for (int i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; i += 3) {
      features[i + 0] = t;
      features[i + 1] = l;
      features[i + 2] = (float)h_state;
    }

    signal_t signal;
    numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);

    ei_impulse_result_t result = { 0 };
    run_classifier(&signal, &result, false);

    float score = result.anomaly;
    String diag = "Normal";

    if (score > 1.0) {
      if (h_state == 1 && t < 15.0) diag = "Heater Failure";
      else if (l > 1000 && t < 5.0) diag = "Sensor Failure";
      else diag = "Unknown Anomaly";

      Serial.println("CORE 0 AI ALERT: " + diag);
    }

    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      systemState.anomaly_score = score;
      systemState.diagnosis = diag;
      xSemaphoreGive(xMutex);
    }

    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

/**
 * TASK: CONTROL (Runs on CORE 1)
 */
void TaskControl(void *pvParameters) {
  for (;;) {
    float t;
    String diag;

    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      t = systemState.temp;
      diag = systemState.diagnosis;
      xSemaphoreGive(xMutex);
    }

    // AI override
    if (diag.indexOf("Heater Failure") != -1) {
      relayWrite01(PIN_HEATER, 0);      // Force OFF
      digitalWrite(PIN_LED_RED, HIGH);  // LED ON
    } else {
      digitalWrite(PIN_LED_RED, LOW);

      // Normal thermostat (active-LOW relay)
      if (t < 18.0) relayWrite01(PIN_HEATER, 1);
      else if (t > 25.0) relayWrite01(PIN_HEATER, 0);
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

/**
 * TASK: MQTT (Runs on CORE 1)
 */
void TaskMQTT(void *pvParameters) {
  for (;;) {
    if (!client.connected()) reconnectMQTT();
    client.loop();

    StaticJsonDocument<512> doc;

    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      doc["water_flow"] = systemState.water_flow;
      doc["temp"] = systemState.temp;
      doc["humid"] = systemState.humid;
      doc["lux"] = systemState.lux;
      doc["moist"] = systemState.moist;

      doc["heater_state"] = systemState.heater_state;
      doc["fan_state"] = systemState.fan_state;
      doc["pump_state"] = systemState.pump_state;

      doc["anomaly_score"] = systemState.anomaly_score;
      doc["diagnosis"] = systemState.diagnosis;

      xSemaphoreGive(xMutex);
    }

    char buffer[512];
    size_t n = serializeJson(doc, buffer, sizeof(buffer));
    if (n > 0) client.publish(TOPIC_DATA, buffer);

    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Hardware init
  Wire.begin(PIN_SDA, PIN_SCL);
  dht.begin();
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);

  pinMode(PIN_Flow, INPUT);
  pinMode(PIN_SOIL, INPUT);

  pinMode(PIN_HEATER, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_PUMP, OUTPUT);

  pinMode(PIN_LED_RED, OUTPUT);

  // default OFF for active-LOW relays
  relayWrite01(PIN_HEATER, 0);
  relayWrite01(PIN_FAN, 0);
  relayWrite01(PIN_PUMP, 0);
  digitalWrite(PIN_LED_RED, LOW);

  connectWiFi();

  // TLS quick start (works, but not best security)
  // For production: set a CA certificate instead of setInsecure().
  espClient.setInsecure();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(onMqttMessage);

  xMutex = xSemaphoreCreateMutex();

  // Initialize shared data defaults
  if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    systemState = {};
    systemState.diagnosis = "Booting";
    xSemaphoreGive(xMutex);
  }

  Serial.println("Starting Dual-Core Tasks...");

  // --- CORE 1 TASKS (Real-World IO) ---
  xTaskCreatePinnedToCore(TaskSensors, "Sensors", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskControl, "Control", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskMQTT,    "MQTT",    4096, NULL, 1, NULL, 1);

  // --- CORE 0 TASKS (Heavy Math / AI) ---
  xTaskCreatePinnedToCore(TaskAI,      "AI_Core", 8192, NULL, 1, NULL, 0);

  Serial.println("System Running.");
}

void loop() {
  vTaskDelete(NULL);
}