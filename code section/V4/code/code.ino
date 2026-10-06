#include <mohamed_said-project-1_inferencing.h>

#include <Arduino.h>

// Edge Impulse model header (must be installed / available in Arduino)

// =============== Shared State ===============
struct SharedData {
    // injected sensor values (same order as Python CSV)
    float temp;
    float humid;
    float soil;
    float light;
    float flow;

    // model output
    float anomaly_score;
    String diagnosis;
    bool is_anomaly;

    // injection freshness
    bool inject_enabled;
    uint32_t last_inject_ms;
};

static SharedData systemState = {
    0, 0, 0, 0, 0,
    0.0f, "Booting...", false,
    false, 0
};

static SemaphoreHandle_t xMutex = NULL;

// EI features buffer
static float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

// =============== CSV Parser ===============
// expected line: temp,humid,soil,light,flow
static bool parseCsv5(const String &line, float &a, float &b, float &c, float &d, float &e) {
    int p1 = line.indexOf(',');
    if (p1 < 0) return false;
    int p2 = line.indexOf(',', p1 + 1);
    if (p2 < 0) return false;
    int p3 = line.indexOf(',', p2 + 1);
    if (p3 < 0) return false;
    int p4 = line.indexOf(',', p3 + 1);
    if (p4 < 0) return false;

    String s1 = line.substring(0, p1);
    String s2 = line.substring(p1 + 1, p2);
    String s3 = line.substring(p2 + 1, p3);
    String s4 = line.substring(p3 + 1, p4);
    String s5 = line.substring(p4 + 1);

    s1.trim(); s2.trim(); s3.trim(); s4.trim(); s5.trim();

    a = s1.toFloat();
    b = s2.toFloat();
    c = s3.toFloat();
    d = s4.toFloat();
    e = s5.toFloat();
    return true;
}

// =============== TASK 1: Serial RX (Core 1) ===============
void TaskSerialRx(void *pvParameters) {
    String line;

    for (;;) {
        while (Serial.available()) {
            char c = (char)Serial.read();

            if (c == '\n') {
                line.trim();
                if (line.length() > 0) {
                    if (line.equalsIgnoreCase("INJECT=0")) {
                        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
                            systemState.inject_enabled = false;
                            systemState.diagnosis = "Injection disabled";
                            xSemaphoreGive(xMutex);
                        }
                        Serial.println("OK=inject_off");
                    } else {
                        float t, h, s, l, f;
                        if (!parseCsv5(line, t, h, s, l, f)) {
                            Serial.println("ERR=bad_csv");
                        } else {
                            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
                                systemState.temp = t;
                                systemState.humid = h;
                                systemState.soil = s;
                                systemState.light = l;
                                systemState.flow = f;

                                systemState.inject_enabled = true;
                                systemState.last_inject_ms = millis();
                                xSemaphoreGive(xMutex);
                            }
                            Serial.println("OK=inject");
                        }
                    }
                }
                line = "";
            } else if (c != '\r') {
                line += c;
                if (line.length() > 120) line = ""; // guard
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// =============== TASK 2: AI Inference (Core 0) ===============
void TaskAI(void *pvParameters) {
    for (;;) {
        float t, h, s, l, f;
        bool inj;
        uint32_t age;

        // snapshot
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            t = systemState.temp;
            h = systemState.humid;
            s = systemState.soil;
            l = systemState.light;
            f = systemState.flow;

            inj = systemState.inject_enabled;
            age = millis() - systemState.last_inject_ms;
            xSemaphoreGive(xMutex);
        } else {
            vTaskDelay(200 / portTICK_PERIOD_MS);
            continue;
        }

        // if no data or stale, don’t run model
        if (!inj || age > 3000) {
            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
                systemState.anomaly_score = 0.0f;
                systemState.is_anomaly = false;
                systemState.diagnosis = "Waiting injection...";
                xSemaphoreGive(xMutex);
            }
            vTaskDelay(200 / portTICK_PERIOD_MS);
            continue;
        }

        // Fill features with your 5 inputs repeatedly until frame is full.
        // Order MUST match training signal order in Edge Impulse.
        for (size_t i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; i += 5) {
            features[i] = t;
            if (i + 1 < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) features[i + 1] = h;
            if (i + 2 < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) features[i + 2] = s;
            if (i + 3 < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) features[i + 3] = l;
            if (i + 4 < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) features[i + 4] = f;
        }

        signal_t signal;
        int err = numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        if (err != 0) {
            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
                systemState.diagnosis = "signal_from_buffer error";
                xSemaphoreGive(xMutex);
            }
            vTaskDelay(200 / portTICK_PERIOD_MS);
            continue;
        }

        ei_impulse_result_t result = { 0 };
        err = run_classifier(&signal, &result, false);
        if (err != 0) {
            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
                systemState.diagnosis = "run_classifier error";
                xSemaphoreGive(xMutex);
            }
            vTaskDelay(200 / portTICK_PERIOD_MS);
            continue;
        }

        float score = result.anomaly;

        // Decide anomaly: you can tune threshold to your model
        // Many EI anomaly models use threshold around 1.0, but it depends.
        bool is_anom = (score > 1.0f);

        String diag = is_anom ? "ANOMALY" : "Normal";

        // Update shared
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            systemState.anomaly_score = score;
            systemState.is_anomaly = is_anom;
            systemState.diagnosis = diag;
            xSemaphoreGive(xMutex);
        }

        vTaskDelay(300 / portTICK_PERIOD_MS); // fast enough for tests
    }
}

// =============== TASK 3: Dashboard (Core 1) ===============
void TaskDashboard(void *pvParameters) {
    for (;;) {
        float t, h, s, l, f, score;
        bool inj, anom;
        uint32_t age;
        String diag;

        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            t = systemState.temp;
            h = systemState.humid;
            s = systemState.soil;
            l = systemState.light;
            f = systemState.flow;

            score = systemState.anomaly_score;
            anom = systemState.is_anomaly;
            diag = systemState.diagnosis;

            inj = systemState.inject_enabled;
            age = millis() - systemState.last_inject_ms;

            xSemaphoreGive(xMutex);
        }

        // Human readable
        Serial.println("================================================");
        Serial.printf("TEMP=%.2f HUM=%.2f SOIL=%.2f LIGHT=%.2f FLOW=%.2f\n", t, h, s, l, f);
        Serial.printf("MODE=%s age_ms=%lu\n", inj ? "INJECT" : "OFF", (unsigned long)age);
        Serial.printf("EI_FRAME=%d\n", (int)EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
        Serial.println("------------------------------------------------");

        // Machine-readable for Python assertions
        Serial.printf("ANOMALY=%d SCORE=%.4f DIAG=%s\n", anom ? 1 : 0, score, diag.c_str());
        Serial.println("================================================");

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

// =============== Setup/Loop ===============
void setup() {
    Serial.begin(115200);
    delay(1500);

    Serial.println("\n=== EI Serial Injection Test (FreeRTOS) ===");
    Serial.println("Send CSV: temp,humid,soil,light,flow");
    Serial.println("Example: 24.0,60.0,60.0,1200.0,0.0");
    Serial.println("Optional: send INJECT=0 to stop.");

    xMutex = xSemaphoreCreateMutex();
    if (!xMutex) {
        Serial.println("CRITICAL: mutex create failed");
        while (1) delay(1000);
    }

    // Start tasks
    xTaskCreatePinnedToCore(TaskSerialRx,   "SerialRx",  4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(TaskDashboard,  "Dashboard", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(TaskAI,         "AI",        8192, NULL, 1, NULL, 0);

    Serial.println("Tasks started.");
}

void loop() {
    vTaskDelete(NULL);
}