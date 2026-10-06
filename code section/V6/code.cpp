#include <mohamed_said-project-1_inferencing.h>


 

#include <DHT.h>
#include <Wire.h>
#include <BH1750.h>

// *** EDGE IMPULSE LIBRARY ***
// Verify this includes your specific library name

// ================= HARDWARE PINS =================
#define PIN_SDA      8
#define PIN_SCL      9

// Sensors
#define PIN_DHT      4   
#define PIN_SOIL     14  
#define PIN_WATER    10 // water flow sensor

// Actuators (Active LOW Relays)
#define PIN_HEATER   5
#define PIN_FAN      6
#define PIN_PUMP     7
#define PIN_HUMID    15
#define PIN_LIGHT    16
#define PIN_LED_RED  42
// ================= OBJECTS =================
#define DHTTYPE DHT11
DHT dht(PIN_DHT, DHTTYPE);
BH1750 lightMeter;
// ================= AI THRESHOLD =================
const float anomaly_threshold = 1.0; // Adjust based on model performance and testing

// ================= SHARED DATA =================
struct SharedData {
    float temp;
    float humid;
    float lux;
    float moist;
    int heater_state;
    float anomaly_score;
    float water_flow;
    String diagnosis;
};

// Initialize with safe default values
SharedData systemState = {0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, "Booting..."};
SemaphoreHandle_t xMutex = NULL; 

// AI Feature Buffer
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

// ================= HELPER FUNCTIONS =================
float readWaterFlow() {
    // Placeholder for water flow sensor reading logic
    // This should be replaced with actual code to read from the water flow sensor
    return 0.0; // Return dummy value for now
}
float readSoilMoisture() {
    int raw = analogRead(PIN_SOIL);
    // Map 12-bit ADC (0-4095) to Percentage
    // 4095 = Dry (Air), 1500 = Wet (Water) - Adjust these if needed
    float pct = map(raw, 4095, 0, 0, 100);
    if(pct < 0) pct = 0; 
    if(pct > 100) pct = 100;
    return pct;
}

// ================= FREERTOS TASKS =================

/* 
 * TASK 1: READ SENSORS (Core 1)
 */
void TaskSensors(void *pvParameters) {
    for (;;) {
        // Read Hardware
        float t = dht.readTemperature();
        float h = dht.readHumidity();
        float l = lightMeter.readLightLevel();
        float m = readSoilMoisture();
        float w = readWaterFlow();
        // Safety checks
        if (isnan(t)) t = 0.0;
        if (l < 0) l = 0.0;

        // Check Actuator (LOW = ON)
        int h_state = (digitalRead(PIN_HEATER) == LOW) ? 1 : 0;
        int light_state = (digitalRead(PIN_LIGHT) == LOW) ? 1 : 0;
        int humid_state = (digitalRead(PIN_HUMID) == LOW) ? 1 : 0;
        int pump_state = (digitalRead(PIN_PUMP) == LOW) ? 1 : 0;
    
        // Update Global State safely
        if (xMutex != NULL) {
            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
                systemState.temp = t;
                systemState.humid = h;
                systemState.lux = l;
                systemState.moist = m;
                systemState.heater_state = h_state;
                systemState.water_flow = w; 
                xSemaphoreGive(xMutex);
            }
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS); 
    }
}

/* 
 * TASK 2: AI INFERENCE (Core 0)
 */
void TaskAI(void *pvParameters) {
    for (;;) {
        float t = 0, l = 0;
        int h_state = 0;

        // 1. Get Snapshot
        if (xMutex != NULL) {
            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
                t = systemState.temp;
                l = systemState.lux;
                h_state = systemState.heater_state;
                xSemaphoreGive(xMutex);
            }
        }

        // 2. Prepare AI Buffer (CRITICAL FIX: Safe filling)
        // We use a loop that strictly respects the array size to prevent crashing
        for (size_t i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; i += 3) {
            features[i] = t;
            if (i + 1 < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) features[i + 1] = l;
            if (i + 2 < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) features[i + 2] = (float)h_state;
        }

        // 3. Run Inference
        signal_t signal;
        int err = numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        
        if (err == 0) {
            ei_impulse_result_t result = { 0 };
            err = run_classifier(&signal, &result, false);

            if (err == 0) {
                float score = result.anomaly;
                String diag = "Normal";

                // Logic Analysis
                if (score > anomaly_threshold) {
                    if (h_state == 1 && t < 15.0) diag = "CRITICAL: Heater Failure"; // if heater run and temp < 15 the heater is broken 
                    else if (l > 1000 && light_state == 1) diag = "CRITICAL: Light sensor is broken";   // if lux > 1000 and light is ON, the light sensor is broken
                    else if (humid_state == 1 && h < 40.0) diag = "CRITICAL: Humidifier Failure"; // if humidifier is ON and humidity < 30, the humidifier is broken
                    else if (pump_state == 1 && m >  80.0) diag = "CRITICAL: Pump Failure"; // if pump is ON and soil moisture > 80, the pump is broken
                    else if (pump_state == 1 && w != 0) diag = "CRITICAL : pipe is blocked "// if pump is ON and water flow is not 0, the pipe is blocked
                    else if (pump_state == 0 && w != 0) diag = "CRITICAL : water leaks "   // if pump is OFF and water flow is not 0, there is a water leak
                    else if (l< 0 ) diag = "CRITICAL: Light sensor is broken"; // if lux < 0, the light sensor is broken
                    else if (t < 0 || t > 50) diag = "CRITICAL: Temperature sensor is broken"; // if temp < 0 or temp > 50, the temperature sensor is broken
                    else diag = "WARNING: Unknown Anomaly";
                }

                // Update Result
                if (xMutex != NULL) {
                    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
                        systemState.anomaly_score = score;
                        systemState.diagnosis = diag;
                        xSemaphoreGive(xMutex);
                    }
                }
            }
        }

        // Yield to prevent watchdog timeout on heavy math
        vTaskDelay(5000 / portTICK_PERIOD_MS); 
    }
}

/* 
 * TASK 3: CONTROL & DASHBOARD (Core 1)
 */
void TaskControl(void *pvParameters) {
    for (;;) {
        float t, h, l, m, score;
        String diag;

        // 1. Read Data
        if (xMutex != NULL) {
            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
                t = systemState.temp;
                h = systemState.humid;
                l = systemState.lux;
                m = systemState.moist;
                score = systemState.anomaly_score;
                diag = systemState.diagnosis;
                xSemaphoreGive(xMutex);
            }
        }

        // ================= SAFETY LOGIC =================
        bool safety_lockout = false;
        String system_status = "NORMAL ✅";

        // Check Diagnosis Strings
        if (diag.indexOf("Heater Failure") != -1) {
            digitalWrite(PIN_HEATER, HIGH);
            digitalWrite(PIN_FAN, HIGH);
            digitalWrite(PIN_LED_RED, HIGH);
            safety_lockout = true;
            system_status = "🛑 SAFETY STOP: HEATER BROKEN";
        } 
        else if (diag.indexOf("Sensor Error") != -1) {
            digitalWrite(PIN_HEATER, HIGH);
            digitalWrite(PIN_FAN, LOW); // Force Fan ON
            digitalWrite(PIN_LED_RED, HIGH);
            safety_lockout = true;
            system_status = "⚠️ FAILSAFE: SENSOR ERROR";
        }
        else if (score > anomaly_threshold) {
             system_status = "⚠️ WARNING: High Anomaly Score";
             digitalWrite(PIN_LED_RED, HIGH);
        }
        else {
            digitalWrite(PIN_LED_RED, LOW);
        }

        // ================= ACTUATOR STRINGS =================
        String heater_s="OFF", fan_s="OFF", pump_s="OFF", light_s="OFF", humid_s="OFF";

        // ================= NORMAL CONTROL =================
        if (!safety_lockout) {
            // Heater/Fan
            if (t < 18.0) { digitalWrite(PIN_HEATER, LOW); heater_s="ON 🔥"; }
            else if (t > 26.0) { digitalWrite(PIN_HEATER, HIGH); 
                digitalWrite(PIN_FAN, LOW); fan_s="ON 💨"; }
            else { digitalWrite(PIN_HEATER, HIGH); digitalWrite(PIN_FAN, HIGH); }

            // Humidifier
            if (h < 50.0) { digitalWrite(PIN_HUMID, LOW); humid_s="ON 💧"; }
            else digitalWrite(PIN_HUMID, HIGH);

            // Light
            if (l < 200.0) { digitalWrite(PIN_LIGHT, LOW); light_s="ON 💡"; }
            else digitalWrite(PIN_LIGHT, HIGH);

            // Pump
            if (m < 30.0) { digitalWrite(PIN_PUMP, LOW); pump_s="ON 🚿"; }
            else digitalWrite(PIN_PUMP, HIGH);
        }

        // ================= DASHBOARD PRINT =================
        Serial.println("\n================================================");
        Serial.println("         🌱 SMART GREENHOUSE MONITOR 🌱         ");
        Serial.println("================================================");
        Serial.printf(" 🌡️ TEMP:  %5.1f °C   |  💧 HUMID: %4.0f %%\n", t, h);
        Serial.printf(" ☀️ LIGHT: %5.0f Lux  |  🌱 MOIST: %4.0f %%\n", l, m);
        Serial.println("------------------------------------------------");
        Serial.printf(" [ACTUATORS] H: %-6s F: %-6s P: %-6s\n", heater_s.c_str(), fan_s.c_str(), pump_s.c_str());
        Serial.printf("             L: %-6s Hum: %-6s\n", light_s.c_str(), humid_s.c_str());
        Serial.println("------------------------------------------------");
        Serial.println(" [AI WATCHDOG]");
        Serial.printf(" Score: %.2f  |  Status: %s\n", score, diag.c_str());
        Serial.println(" SYSTEM: " + system_status);
        Serial.println("================================================");

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

// ================= SETUP =================
void setup() {
    Serial.begin(115200);
    delay(3000); // Allow USB to catch up
    Serial.println("\n\n=== SYSTEM BOOT ===");

    // 1. Memory Check
    Serial.printf("Total Heap: %d bytes\n", ESP.getHeapSize());
    Serial.printf("Free Heap:  %d bytes\n", ESP.getFreeHeap());

    if (ESP.getFreeHeap() < 20000) {
        Serial.println("❌ WARNING: Low Memory! AI model might cause instability.");
    }

    // 2. Hardware Init
    Wire.begin(PIN_SDA, PIN_SCL); // Explicitly set pins 8 and 9
    dht.begin();

    // I2C Debug Scan
    Serial.println("Scanning I2C Bus...");
    Wire.beginTransmission(0x23);
    if (Wire.endTransmission() == 0) {
        Serial.println("✅ BH1750 Found at 0x23");
        if(lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire)) {
             Serial.println("BH1750 Initialized.");
        }
    } else {
        Serial.println("❌ BH1750 NOT FOUND! Check wiring (SDA=8, SCL=9).");
    }

    // 3. Pin Modes
    pinMode(PIN_HEATER, OUTPUT);
    pinMode(PIN_FAN, OUTPUT);
    pinMode(PIN_PUMP, OUTPUT);
    pinMode(PIN_HUMID, OUTPUT);
    pinMode(PIN_LIGHT, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_WATER, INPUT);


    // Default States (All OFF)
    digitalWrite(PIN_HEATER, HIGH);
    digitalWrite(PIN_FAN, HIGH);
    digitalWrite(PIN_PUMP, HIGH);
    digitalWrite(PIN_HUMID, HIGH);
    digitalWrite(PIN_LIGHT, HIGH);
    digitalWrite(PIN_LED_RED, LOW);

    // 4. Create Mutex
    xMutex = xSemaphoreCreateMutex();
    if (xMutex == NULL) {
        Serial.println("❌ CRITICAL: Failed to create Mutex.");
        while(1);
    }

    // 5. Create Tasks
    Serial.println("Creating Tasks...");
    
    // Core 1 (Sensors/Control)
    xTaskCreatePinnedToCore(TaskSensors, "Sensors", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(TaskControl, "Control", 4096, NULL, 1, NULL, 1);

    // Core 0 (AI) - Increased stack to 8192 bytes
    xTaskCreatePinnedToCore(TaskAI,      "AI",      8192, NULL, 1, NULL, 0);

    Serial.println("✅ Tasks Started. Monitor will appear shortly...");
}

void loop() {
    vTaskDelete(NULL);
}