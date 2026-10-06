#include <WiFi.h>
#include <PicoMQTT.h>

const char* ssid = "GreenhouseNet";
const char* password = "12345678";

IPAddress apIP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

PicoMQTT::Server mqtt(1883);

void setup() {
  Serial.begin(115200);
  delay(800);

  WiFi.mode(WIFI_AP);
  WiFi.softAPdisconnect(true);
  delay(200);

  bool cfgOk = WiFi.softAPConfig(apIP, gateway, subnet);
  bool apOk  = WiFi.softAP(ssid, password, 6, 0, 8); // channel=6, visible, max clients=8

  Serial.printf("softAPConfig: %s\n", cfgOk ? "OK" : "FAIL");
  Serial.printf("softAPStart : %s\n", apOk ? "OK" : "FAIL");
  Serial.print("AP IP       : ");
  Serial.println(WiFi.softAPIP());

  mqtt.begin();
  Serial.println("MQTT broker started on port 1883");

  mqtt.subscribe("greenhouse/commands", [](const char* topic, const char* payload) {
    Serial.printf("[BROKER RX] %s -> %s\n", topic, payload);
  });
}

void loop() {
  mqtt.loop();
  delay(2);
}