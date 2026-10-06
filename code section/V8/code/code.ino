// =============================================================
//   SMART GREENHOUSE — ESP32-S3  |  Full Corrected Firmware
//   Sensors : DHT11, BH1750, Soil Moisture, Water Flow
//   Actuators: Heater, Fan, Pump, Humidifier, Grow-Light
//   AI      : Edge Impulse anomaly detection
//   Cloud   : HiveMQ (TLS port 8883) → Node-RED
// =============================================================

#include <DHT.h>
#include <Wire.h>
#include <BH1750.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// *** EDGE IMPULSE — change to your exported library name ***
#include <mohamed_said-project-1_inferencing.h>

// =============================================================
//  WIFI + MQTT CREDENTIALS
// =============================================================
const char* WIFI_SSID     = "msi";
const char* WIFI_PASS     = "12345678";

const char* MQTT_SERVER   = "da5e9c9f3af6499b92bdd5819d675d8c.s1.eu.hivemq.cloud";
const int   MQTT_PORT     = 8883;          // TLS port
const char* MQTT_USER     = "mohamedsaid";
const char* MQTT_PASS     = "13025651aB";

// MQTT Topics
const char* TOPIC_DATA    = "greenhouse/data";      // ESP32 → Node-RED  (publish)
const char* TOPIC_CMD     = "greenhouse/commands";  // Node-RED → ESP32  (subscribe)

// =============================================================
//  HARDWARE PINS
// =============================================================
#define PIN_SDA      8
#define PIN_SCL      9

// Sensors
#define PIN_DHT      4
#define PIN_SOIL     14
#define PIN_WATER    10   // Water flow sensor (pulse input)

// Actuators  (Active-LOW relay board)
#define PIN_HEATER   5
#define PIN_FAN      6
#define PIN_PUMP     7
#define PIN_HUMID    15
#define PIN_LIGHT    16
#define PIN_LED_RED  42

// =============================================================
//  DRIVER OBJECTS
// =============================================================
#define DHTTYPE DHT11
DHT dht(PIN_DHT, DHTTYPE);
BH1750 lightMeter;

WiFiClientSecure espClient;
PubSubClient     client(espClient);

// =============================================================
//  AI THRESHOLD
// =============================================================
const float ANOMALY_THRESHOLD = 1.0f;

// =============================================================
//  WATER FLOW — pulse counter (ISR-safe)
// =============================================================
volatile uint32_t flowPulseCount = 0;

void IRAM_ATTR flowPulseISR() {
    flowPulseCount++;
}

// Returns litres/minute based on pulses counted since last call.
// Calibration factor 7.5 pulses/sec per L/min (common YF-S201).
// Call this once per measurement window and pass the elapsed ms.
float calculateFlowRate(uint32_t elapsedMs) {
    uint32_t pulses = flowPulseCount;
    flowPulseCount  = 0;                        // Reset counter
    if (elapsedMs == 0) return 0.0f;
    float freq = (pulses * 1000.0f) / elapsedMs; // pulses/sec
    return freq / 7.5f;                           // L/min
}

// =============================================================
//  SHARED STATE  (protected by xMutex)
// =============================================================
struct SharedData {
    float  temp;
    float  humid;
    float  lux;
    float  moist;
    float  water_flow;

    int    heater_state;
    int    fan_state;
    int    pump_state;
    int    light_state;
    int    humid_state;

    float  anomaly_score;
    String diagnosis;
};

SharedData systemState = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f,   // sensor readings
    0, 0, 0, 0, 0,                    // actuator states
    0.0f, "Booting..."                // AI output
};

SemaphoreHandle_t xMutex = NULL;

// AI feature buffer
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

// =============================================================
//  HELPERS
// =============================================================

// Relay write — ON = LOW (active-low board), OFF = HIGH
void relayWrite(int pin, bool turnOn) {
    digitalWrite(pin, turnOn ? LOW : HIGH);
}

// Read actuator state: returns 1 if relay is ON (pin LOW)
int relayRead(int pin) {
    return (digitalRead(pin) == LOW) ? 1 : 0;
}

// Soil moisture: map 12-bit ADC to 0-100 %
// Adjust raw_dry / raw_wet for your sensor if needed
float readSoilMoisture() {
    const int raw_dry = 3200;   // raw value in dry air
    const int raw_wet = 1100;   // raw value submerged in water
    int raw = analogRead(PIN_SOIL);
    float pct = (float)(raw_dry - raw) / (float)(raw_dry - raw_wet) * 100.0f;
    if (pct < 0.0f)   pct = 0.0f;
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
    while (WiFi.status() != WL_CONNECTED && tries < 40) {
        delay(500);
        Serial.print(".");
        tries++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ WiFi connected: " + WiFi.localIP().toString());
    } else {
        Serial.println("\n❌ WiFi FAILED — restarting");
        ESP.restart();
    }
}

// =============================================================
//  MQTT CALLBACK  (Node-RED → ESP32 commands)
//  Expected JSON:  {"heater":1,"fan":0,"pump":1,"humid":0,"light":1}
// =============================================================
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, payload, length);
    if (err) {
        Serial.println("MQTT JSON parse error: " + String(err.c_str()));
        return;
    }
    if (doc.containsKey("heater")) relayWrite(PIN_HEATER, (int)doc["heater"]);
    if (doc.containsKey("fan"))    relayWrite(PIN_FAN,    (int)doc["fan"]);
    if (doc.containsKey("pump"))   relayWrite(PIN_PUMP,   (int)doc["pump"]);
    if (doc.containsKey("humid"))  relayWrite(PIN_HUMID,  (int)doc["humid"]);
    if (doc.containsKey("light"))  relayWrite(PIN_LIGHT,  (int)doc["light"]);
    Serial.println("📥 Remote command received and applied.");
}

// =============================================================
//  MQTT RECONNECT
// =============================================================
void reconnectMQTT() {
    while (!client.connected()) {
        Serial.print("Connecting to HiveMQ...");
        String cid = "ESP32S3_GH_" + String((uint32_t)ESP.getEfuseMac(), HEX);
        if (client.connect(cid.c_str(), MQTT_USER, MQTT_PASS)) {
            Serial.println(" ✅ Connected");
            client.subscribe(TOPIC_CMD);
        } else {
            Serial.printf(" ❌ Failed (rc=%d) — retry in 5s\n", client.state());
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
    }
}

// =============================================================
//  TASK 1 — READ SENSORS  (Core 1)
// =============================================================
void TaskSensors(void* pvParameters) {
    uint32_t lastFlowCheck = millis();

    for (;;) {
        // --- Read sensors ---
        float t = dht.readTemperature();
        float h = dht.readHumidity();
        float l = lightMeter.readLightLevel();
        float m = readSoilMoisture();

        // Water flow rate over last interval
        uint32_t now      = millis();
        uint32_t elapsed  = now - lastFlowCheck;
        float    w        = calculateFlowRate(elapsed);
        lastFlowCheck     = now;

        // Sanity clamps
        if (isnan(t) || t < -40.0f || t > 80.0f) t = -999.0f; // sentinel for broken sensor
        if (isnan(h) || h < 0.0f   || h > 100.0f) h = -999.0f;
        if (l < 0.0f) l = 0.0f;

        // --- Read actuator states ---
        int h_state     = relayRead(PIN_HEATER);
        int fan_state   = relayRead(PIN_FAN);
        int pump_state  = relayRead(PIN_PUMP);
        int light_state = relayRead(PIN_LIGHT);
        int humid_state = relayRead(PIN_HUMID);

        // --- Update shared state ---
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            systemState.temp         = t;
            systemState.humid        = h;
            systemState.lux          = l;
            systemState.moist        = m;
            systemState.water_flow   = w;
            systemState.heater_state = h_state;
            systemState.fan_state    = fan_state;
            systemState.pump_state   = pump_state;
            systemState.light_state  = light_state;
            systemState.humid_state  = humid_state;
            xSemaphoreGive(xMutex);
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

// =============================================================
//  TASK 2 — AI ANOMALY DETECTION  (Core 0)
// =============================================================
void TaskAI(void* pvParameters) {
    for (;;) {
        // --- Snapshot all needed values ---
        float t, h, l, m, w;
        int h_state, fan_state, pump_state, light_state, humid_state;

        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            t           = systemState.temp;
            h           = systemState.humid;
            l           = systemState.lux;
            m           = systemState.moist;
            w           = systemState.water_flow;
            h_state     = systemState.heater_state;
            fan_state   = systemState.fan_state;
            pump_state  = systemState.pump_state;
            light_state = systemState.light_state;
            humid_state = systemState.humid_state;
            xSemaphoreGive(xMutex);
        }

        // --- Fill AI feature buffer ---
        // Features used: temp, lux, heater_state
        // Extend this to match exactly what your Edge Impulse model was trained on
        for (size_t i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; i += 3) {
            features[i]     = t;
            if (i + 1 < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) features[i + 1] = l;
            if (i + 2 < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) features[i + 2] = (float)h_state;
        }

        // --- Run inference ---
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

        float  score = result.anomaly;
        String diag  = "Normal";

        // --- Diagnosis logic (only runs when anomaly score is high) ---
        if (score > ANOMALY_THRESHOLD) {

            // Temperature sensor broken (sentinel value set in TaskSensors)
            if (t == -999.0f) {
                diag = "CRITICAL: Temperature sensor failure";
            }
            // Humidity sensor broken
            else if (h == -999.0f) {
                diag = "CRITICAL: Humidity sensor failure";
            }
            // Heater ON but temperature not rising — heater broken
            else if (h_state == 1 && t < 15.0f) {
                diag = "CRITICAL: Heater failure";
            }
            // Humidifier ON but humidity still very low — humidifier broken
            else if (humid_state == 1 && h < 40.0f) {
                diag = "CRITICAL: Humidifier failure";
            }
            // Pump ON, soil already saturated, no water flow — pipe blocked
            else if (pump_state == 1 && m > 80.0f && w == 0.0f) {
                diag = "CRITICAL: Pipe blocked (pump running, no flow)";
            }
            // Pump OFF but water is still flowing — water leak
            else if (pump_state == 0 && w > 0.1f) {
                diag = "CRITICAL: Water leak detected";
            }
            // Light ON but lux is still very high — grow light sensor broken
            // (If light is ON and sensor reads > 5000 lux, it may be shorted / stuck)
            else if (light_state == 1 && l > 5000.0f) {
                diag = "WARNING: Light sensor may be saturated or broken";
            }
            else {
                diag = "WARNING: Unknown anomaly";
            }
        }

        // --- Write result back ---
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            systemState.anomaly_score = score;
            systemState.diagnosis     = diag;
            xSemaphoreGive(xMutex);
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

// =============================================================
//  TASK 3 — CONTROL + SERIAL DASHBOARD  (Core 1)
// =============================================================
void TaskControl(void* pvParameters) {
    for (;;) {
        float  t, h, l, m, w, score;
        int    pump_state, light_state, humid_state;
        String diag;

        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            t           = systemState.temp;
            h           = systemState.humid;
            l           = systemState.lux;
            m           = systemState.moist;
            w           = systemState.water_flow;
            pump_state  = systemState.pump_state;
            light_state = systemState.light_state;
            humid_state = systemState.humid_state;
            score       = systemState.anomaly_score;
            diag        = systemState.diagnosis;
            xSemaphoreGive(xMutex);
        }

        // ---- SAFETY LOCKOUT ----
        bool   safety_lockout = false;
        String system_status  = "NORMAL";

        if (diag.indexOf("Heater failure") != -1) {
            relayWrite(PIN_HEATER, false);
            relayWrite(PIN_FAN,    false);   // Force fan ON to cool down
            digitalWrite(PIN_LED_RED, HIGH);
            safety_lockout = true;
            system_status  = "SAFETY STOP: HEATER BROKEN";
        }
        else if (diag.indexOf("sensor failure") != -1 ||
                 diag.indexOf("sensor may be") != -1) {
            relayWrite(PIN_HEATER, false);
            relayWrite(PIN_FAN,    false);   // Failsafe: ventilate
            digitalWrite(PIN_LED_RED, HIGH);
            safety_lockout = true;
            system_status  = "FAILSAFE: SENSOR ERROR";
        }
        else if (diag.indexOf("Water leak") != -1) {
            relayWrite(PIN_PUMP, false);     // Emergency: stop pump
            digitalWrite(PIN_LED_RED, HIGH);
            safety_lockout = true;
            system_status  = "SAFETY STOP: WATER LEAK";
        }
        else if (score > ANOMALY_THRESHOLD) {
            digitalWrite(PIN_LED_RED, HIGH);
            system_status = "WARNING: Anomaly detected";
        }
        else {
            digitalWrite(PIN_LED_RED, LOW);
        }

        // ---- NORMAL CONTROL ----
        String heater_s = "OFF", fan_s = "OFF", pump_s = "OFF";
        String light_s  = "OFF", humid_s = "OFF";

        if (!safety_lockout) {

            // Temperature control
            if (t != -999.0f) {
                if (t < 18.0f) {
                    relayWrite(PIN_HEATER, true);
                    relayWrite(PIN_FAN,    false);
                    heater_s = "ON";
                } else if (t > 26.0f) {
                    relayWrite(PIN_HEATER, false);
                    relayWrite(PIN_FAN,    true);
                    fan_s = "ON";
                } else {
                    relayWrite(PIN_HEATER, false);
                    relayWrite(PIN_FAN,    false);
                }
            }

            // Humidity control
            if (h != -999.0f) {
                if (h < 50.0f) { relayWrite(PIN_HUMID, true);  humid_s = "ON"; }
                else            { relayWrite(PIN_HUMID, false); }
            }

            // Light control
            if (l < 200.0f) { relayWrite(PIN_LIGHT, true);  light_s = "ON"; }
            else             { relayWrite(PIN_LIGHT, false); }

            // Pump / irrigation control
            if (m < 30.0f) { relayWrite(PIN_PUMP, true);  pump_s = "ON"; }
            else            { relayWrite(PIN_PUMP, false); }
        }

        // ---- SERIAL DASHBOARD ----
        Serial.println("\n================================================");
        Serial.println("        SMART GREENHOUSE MONITOR               ");
        Serial.println("================================================");
        if (t == -999.0f) Serial.println(" TEMP:  [SENSOR ERROR]");
        else               Serial.printf(" TEMP:  %5.1f C   |  HUMID: %4.0f %%\n", t, h);
        Serial.printf(" LIGHT: %5.0f Lux  |  MOIST: %4.0f %%\n", l, m);
        Serial.printf(" FLOW:  %.2f L/min\n", w);
        Serial.println("------------------------------------------------");
        Serial.printf(" [ACTUATORS]  Heater:%-4s  Fan:%-4s  Pump:%-4s\n",
                      heater_s.c_str(), fan_s.c_str(), pump_s.c_str());
        Serial.printf("              Light:%-4s   Humid:%-4s\n",
                      light_s.c_str(), humid_s.c_str());
        Serial.println("------------------------------------------------");
        Serial.printf(" [AI]  Score: %.2f  |  %s\n", score, diag.c_str());
        Serial.println(" STATUS: " + system_status);
        Serial.println("================================================");

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

// =============================================================
//  TASK 4 — MQTT PUBLISH + RECEIVE  (Core 1)
// =============================================================
void TaskMQTT(void* pvParameters) {
    for (;;) {
        // Reconnect if needed
        if (!client.connected()) {
            reconnectMQTT();
        }
        client.loop();   // Process incoming messages

        // --- Build JSON payload ---
    StaticJsonDocument<512> doc;

if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
    // Handle sensor error sentinel values (-999.0f) by setting null
    if (systemState.temp == -999.0f) {
        doc["temp"] = nullptr;
    } else {
        doc["temp"] = systemState.temp;
    }
    
    if (systemState.humid == -999.0f) {
        doc["humid"] = nullptr;
    } else {
        doc["humid"] = systemState.humid;
    }
    
    // Other values assign directly
    doc["lux"]           = systemState.lux;
    doc["moist"]         = systemState.moist;
    doc["water_flow"]    = systemState.water_flow;
    doc["heater_state"]  = systemState.heater_state;
    doc["fan_state"]     = systemState.fan_state;
    doc["pump_state"]    = systemState.pump_state;
    doc["light_state"]   = systemState.light_state;
    doc["humid_state"]   = systemState.humid_state;
    doc["anomaly_score"] = systemState.anomaly_score;
    doc["diagnosis"]     = systemState.diagnosis;
    xSemaphoreGive(xMutex);
}

        char buffer[512];
        size_t n = serializeJson(doc, buffer, sizeof(buffer));
        if (n > 0) {
            client.publish(TOPIC_DATA, buffer);
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

// =============================================================
//  SETUP
// =============================================================
void setup() {
    Serial.begin(115200);
    delay(3000);
    Serial.println("\n=== GREENHOUSE SYSTEM BOOT ===");

    // --- Memory check ---
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
    if (ESP.getFreeHeap() < 20000) {
        Serial.println("WARNING: Low memory — AI may be unstable.");
    }

    // --- I2C + sensors ---
    Wire.begin(PIN_SDA, PIN_SCL);
    dht.begin();

    Serial.print("Scanning I2C...");
    Wire.beginTransmission(0x23);
    if (Wire.endTransmission() == 0) {
        Serial.println(" BH1750 found");
        lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
    } else {
        Serial.println(" BH1750 NOT FOUND — check SDA=8, SCL=9");
    }

    // --- Pin modes ---
    pinMode(PIN_HEATER,  OUTPUT);
    pinMode(PIN_FAN,     OUTPUT);
    pinMode(PIN_PUMP,    OUTPUT);
    pinMode(PIN_HUMID,   OUTPUT);
    pinMode(PIN_LIGHT,   OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_WATER,   INPUT_PULLUP);  // Flow sensor pulse input

    // Default: all actuators OFF (active-low → HIGH)
    relayWrite(PIN_HEATER, false);
    relayWrite(PIN_FAN,    false);
    relayWrite(PIN_PUMP,   false);
    relayWrite(PIN_HUMID,  false);
    relayWrite(PIN_LIGHT,  false);
    digitalWrite(PIN_LED_RED, LOW);

    // --- Water flow ISR ---
    attachInterrupt(digitalPinToInterrupt(PIN_WATER), flowPulseISR, RISING);

    // --- WiFi ---
    connectWiFi();

    // --- MQTT (TLS, no certificate verification — upgrade to CA cert in production) ---
    espClient.setInsecure();
    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setCallback(onMqttMessage);

    // --- FreeRTOS mutex ---
    xMutex = xSemaphoreCreateMutex();
    if (xMutex == NULL) {
        Serial.println("CRITICAL: Mutex creation failed — halting.");
        while (1);
    }

    // --- Create tasks ---
    Serial.println("Starting FreeRTOS tasks...");
    //                            name          stack   param  prio  handle  core
    xTaskCreatePinnedToCore(TaskSensors, "Sensors",  4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(TaskControl, "Control",  6144, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(TaskMQTT,    "MQTT",     6144, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(TaskAI,      "AI",       8192, NULL, 1, NULL, 0);

    Serial.println("All tasks started. System running.");
}

// loop() is unused — FreeRTOS scheduler takes over
void loop() {
    vTaskDelete(NULL);
}

// =============================================================
//  NODE-RED INTEGRATION NOTES
//  ─────────────────────────────────────────────────────────
//  Subscribe to:  greenhouse/data
//  JSON fields published every 5 s:
//    temp, humid, lux, moist, water_flow
//    heater_state, fan_state, pump_state, light_state, humid_state
//    anomaly_score, diagnosis
//
//  Publish to:    greenhouse/commands
//  JSON format:
//    {"heater":1,"fan":0,"pump":1,"humid":0,"light":1}
//    Any subset of keys is accepted — omitted keys are ignored.
// =============================================================
