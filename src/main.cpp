#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// --- Константы ---
#define MODULE_TYPE "simple-conveyor"
#define MOTOR_IN1 3  // Подключение к L298N
#define MOTOR_IN2 2

#define WIFI_SSID "Beeline_MF"
#define WIFI_PASS "$@ndr0nix"
#define MQTT_SERVER "192.168.8.100"

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_VL53L0X sensor = Adafruit_VL53L0X();
String mac_id = "";

// --- Сеть и MQTT ---
void setup_wifi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  mac_id = WiFi.macAddress();
  Serial.println("\nWiFi connected, MAC: " + mac_id);
}

void sendIdentity() {
  String ip = WiFi.localIP().toString();
  client.publish(("module/" + mac_id + "/identity/type").c_str(), MODULE_TYPE);
  client.publish(("module/" + mac_id + "/identity/ip").c_str(), ip.c_str());
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect(mac_id.c_str())) {
      Serial.println("Connected to MQTT");
      sendIdentity();
      client.subscribe(("module/" + mac_id + "/command").c_str());
    } else {
      delay(1000);
    }
  }
}

// --- Управление мотором ---
void startMotor() {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
}

void endMotor() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

// --- Обработка команд ---
void onMessage(char* topic, byte* payload, unsigned int length) {
  String cmd;
  for (unsigned int i = 0; i < length; i++) cmd += (char)payload[i];

  if (cmd == "start") {
    startMotor();
  } else if (cmd == "end") {
    endMotor();
  }
}

// --- Основной цикл ---
void sendDistance() {
  VL53L0X_RangingMeasurementData_t measure;
  sensor.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {
    char topic[64];
    snprintf(topic, sizeof(topic), "module/%s/sensor/vl53l0x", mac_id.c_str());
    char payload[16];
    snprintf(payload, sizeof(payload), "%d", measure.RangeMilliMeter);
    client.publish(topic, payload);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);

  setup_wifi();
  client.setServer(MQTT_SERVER, 1883);
  client.setCallback(onMessage);

  if (!sensor.begin()) {
    Serial.println("VL53L0X not found");
    while (1);
  } else {
    Serial.println("VL53L0X ready");
  }
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  sendDistance();
  delay(1000);
}
