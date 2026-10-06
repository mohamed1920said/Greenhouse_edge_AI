/*
 * GREENHOUSE AI WATCHDOG - ESP32-S3
 * 
 * Logic:
 * 1. Read Temp, Light, and Heater Status.
 * 2. Send data to Edge Impulse AI.
 * 3. AI calculates "Anomaly Score".
 * 4. If Score > 1.0 -> Something is BROKEN (Heater/Sensor).
 * 5. Trigger Emergency Safety Mode.
 */

// CHANGE THIS to your specific library name!
// Look in: File -> Examples -> [Your Project Name] -> static_buffer
#include <mohamed_said_project_1_inferencing.h> 

#include <DHT.h>
#include <Wire.h>
#include <BH1750.h>

// --- PIN DEFINITIONS ---
#define PIN_DHT      16  // DHT11 Data Pin
#define PIN_HEATER   2   // Relay for Heater
#define PIN_FAN      3   // Relay for Fan
#define PIN_LED_RED  42  // Status LED (Optional)

// --- SETTINGS ---
#define DHTTYPE      DHT11
#define ANOMALY_THRESHOLD 1.0  // Scores above this are alerts

// --- OBJECTS ---
DHT dht(PIN_DHT, DHTTYPE);
BH1750 lightMeter;

// Buffer for the AI (Window size is huge, but we only feed 1 frame of averages)
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE]; 

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("Starting AI Greenhouse Watchdog...");

    // Initialize Sensors
    dht.begin();
    Wire.begin();
    if (lightMeter.begin()) {
        Serial.println("Light sensor ready");
    } else {
        Serial.println("Error initiating Light sensor");
    }

    // Initialize Relays
    pinMode(PIN_HEATER, OUTPUT);
    pinMode(PIN_FAN, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);
    
    // Default State: OFF (assuming Active LOW relays)
    digitalWrite(PIN_HEATER, HIGH); 
    digitalWrite(PIN_FAN, HIGH);
}

void loop() {
    // ==========================================================
    // 1. READ PHYSICAL WORLD
    // ==========================================================
    float temp = dht.readTemperature();
    float lux = lightMeter.readLightLevel();
    
    // Check Heater State (Are we TRYING to heat?)
    // Inverted logic: LOW means ON for many relays
    int is_heater_on = (digitalRead(PIN_HEATER) == LOW) ? 1 : 0; 

    // Safety check for sensor wiring
    if (isnan(temp)) {
        Serial.println("Failed to read from DHT sensor!");
        return;
    }

    // ==========================================================
    // 2. PREPARE DATA FOR AI
    // ==========================================================
    // IMPORTANT: The order MUST match your Edge Impulse "Impulse Design"!
    // We selected: tempC, lux, heater_fan_on
    // Note: We fill the whole buffer with the SAME values because 
    // the model expects a "window", but we are giving it a "snapshot".
    
    for (int i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; i += 3) {
        features[i + 0] = temp;          // tempC
        features[i + 1] = lux;           // lux
        features[i + 2] = (float)is_heater_on; // heater_fan_on
    }

    // ==========================================================
    // 3. RUN AI INFERENCE (THE BRAIN)
    // ==========================================================
    signal_t signal;
    int err = numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        Serial.printf("Error creating signal: %d\n", err);
        return;
    }

    ei_impulse_result_t result = { 0 };
    err = run_classifier(&signal, &result, false);
    if (err != 0) {
        Serial.printf("Error running classifier: %d\n", err);
        return;
    }

    // ==========================================================
    // 4. CHECK RESULTS (THE DECISION)
    // ==========================================================
    float anomaly_score = result.anomaly;

    Serial.print("Sensors -> Temp: "); Serial.print(temp);
    Serial.print(" | Lux: "); Serial.print(lux);
    Serial.print(" | Heater: "); Serial.println(is_heater_on);
    
    Serial.print("AI Analysis -> Anomaly Score: "); 
    Serial.println(anomaly_score);

    // ==========================================================
    // 5. ACT ON ANOMALY
    // ==========================================================
    if (anomaly_score > ANOMALY_THRESHOLD) {
        Serial.println("\n>>> ⚠️ DANGER: ANOMALY DETECTED! ⚠️ <<<");
        digitalWrite(PIN_LED_RED, HIGH); // Turn on Warning Light
        
        // ---------------------------------------------------
        // DIAGNOSIS LOGIC (Explain WHY the AI is mad)
        // ---------------------------------------------------
        
        // CASE A: Broken Heater?
        // Logic: Heater is ON (1), but Temp is Cold (< 18) and falling?
        if (is_heater_on == 1 && temp < 18.0) {
             Serial.println("Diagnosis: HEATER FAILURE. Relay is ON but room is cold.");
             Serial.println("Action: Shutting down heater to prevent electrical fire.");
             digitalWrite(PIN_HEATER, HIGH); // Force OFF
        }
        
        // CASE B: Broken Sensor?
        // Logic: Lux is High (Sun is out) but Temp is near Zero?
        else if (lux > 1000 && temp < 5.0) {
             Serial.println("Diagnosis: SENSOR FAILURE. It's sunny but sensor reads 0C.");
             Serial.println("Action: Ignoring sensor data. Running failsafe fan.");
             digitalWrite(PIN_FAN, LOW); // Turn Fan ON to be safe
        }
        
        else {
             Serial.println("Diagnosis: Unknown Anomaly. System behaving weirdly.");
        }

    } else {
        // ==========================================================
        // 6. NORMAL OPERATION (Standard Control Loop)
        // ==========================================================
        Serial.println("System Status: NORMAL ✅");
        digitalWrite(PIN_LED_RED, LOW);

        // Standard Bang-Bang Control
        if (temp < 18.0) {
            digitalWrite(PIN_HEATER, LOW); // Turn ON
        } else if (temp > 20.0) {
            digitalWrite(PIN_HEATER, HIGH); // Turn OFF
        }
    }
    
    Serial.println("----------------------------------------");
    delay(5000); // Check every 5 seconds
}