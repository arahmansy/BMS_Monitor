#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ======== WIFI CONFIG ========
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// ======== MQTT CONFIG ========
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_user = "user1";
const char* mqtt_pass = "Admin123";
const char* mqtt_topic = "adnansy/bms/status/info";

// ======== SERIAL CONFIG ========
#define RX_PIN 16   // Connect to BMS TX
#define TX_PIN 17   // Connect to BMS RX
#define RS485 Serial2
#define BAUD 9600
#define INTERVAL 10000UL   // 10 seconds
uint8_t CMD_BMS[] = {0x7E, 0x32, 0x35, 0x30, 0x31, 0x34, 0x36, 0x34, 0x32, 0x45, 0x30, 0x30, 0x32, 0x30, 0x31, 0x46, 0x44, 0x33, 0x30, 0x0D};

// ======== GLOBALS ========
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastRead = 0;

// ======== WIFI + MQTT ========
void setup_wifi() {
  Serial.println("\nConnecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[WiFi] Connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void reconnect_mqtt() {
  while (!client.connected()) {
    Serial.print("[MQTT] Connecting...");
    if (client.connect("ESP32_BMS", mqtt_user, mqtt_pass)) {
      Serial.println(" connected.");
    } else {
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      Serial.println(" retry in 5s...");
      delay(5000);
    }
  }
}

// ======== FRAME HELPERS ========
bool readFrame(uint8_t *buffer, size_t &len) {
  len = 0;
  bool inFrame = false;
  unsigned long start = millis();

  while (millis() - start < 2000) { // 2s timeout
    if (RS485.available()) {
      uint8_t b = RS485.read();
      if (b == 0x7E && !inFrame) {
        len = 0;
        buffer[len++] = b;
        inFrame = true;
      } else if (inFrame) {
        buffer[len++] = b;
        if (b == 0x0D) return true;
        if (len >= 512) break;
      }
    }
  }
  return false;
}

bool decodeAsciiHex(const uint8_t *raw, size_t len, uint8_t *out, size_t &outlen) {
  if (len < 3 || raw[0] != 0x7E || raw[len - 1] != 0x0D) return false;
  outlen = 0;
  for (size_t i = 1; i < len - 1; i += 2) {
    char hi = raw[i];
    char lo = raw[i + 1];
    uint8_t val = (uint8_t)((strtol((String(hi) + String(lo)).c_str(), NULL, 16)) & 0xFF);
    out[outlen++] = val;
  }
  return true;
}

// ======== PARSER ========
void parseAndPublish(uint8_t *p, size_t len) {
  if (len < 20) return;
  size_t idx = 0;

  uint8_t VER = p[idx++], ADR = p[idx++], CID1 = p[idx++], RTN = p[idx++];
  uint16_t LEN = (p[idx++] << 8) | p[idx++];
  uint8_t infoflag = p[idx++];
  uint8_t pack_no = p[idx++];
  uint8_t cell_count = p[idx++];

  float cells[24];
  for (int i = 0; i < cell_count && idx + 1 < len; i++) {
    uint16_t mv = (p[idx++] << 8) | p[idx++];
    cells[i] = mv / 1000.0;
  }

  uint8_t temp_count = p[idx++];
  float temps[8];
  for (int i = 0; i < temp_count && idx + 1 < len; i++) {
    uint16_t rawt = (p[idx++] << 8) | p[idx++];
    temps[i] = (rawt / 10.0) - 273.15;
  }

  int16_t raw_current = (int16_t)((p[idx++] << 8) | p[idx++]);
  float pack_current = raw_current * 0.01;

  float pack_voltage = ((p[idx++] << 8) | p[idx++]) / 1000.0;
  float remain_Ah = ((p[idx++] << 8) | p[idx++]) * 0.01;
  idx++; // user define
  float full_Ah = ((p[idx++] << 8) | p[idx++]) * 0.01;
  uint16_t cycle = (p[idx++] << 8) | p[idx++];
  float design_Ah = ((p[idx++] << 8) | p[idx++]) * 0.01;

  float SOC = (full_Ah > 0) ? (remain_Ah / full_Ah * 100) : 0;
  float remain_hours = (pack_current != 0) ? (remain_Ah / abs(pack_current)) : 0;

  // ======== JSON CREATION ========
  StaticJsonDocument<512> doc;
  doc["timestamp"] = millis();
  doc["pack_voltage"] = pack_voltage;
  doc["pack_current"] = pack_current;
  doc["remain_Ah"] = remain_Ah;
  doc["full_Ah"] = full_Ah;
  doc["SOC"] = SOC;
  doc["remaining_hours"] = remain_hours;
  doc["cycles"] = cycle;
  doc["design_Ah"] = design_Ah;

  JsonArray arrCells = doc.createNestedArray("cells");
  for (int i = 0; i < cell_count; i++) arrCells.add(cells[i]);

  JsonArray arrTemps = doc.createNestedArray("temps");
  for (int i = 0; i < temp_count; i++) arrTemps.add(temps[i]);

  char jsonBuffer[512];
  size_t n = serializeJson(doc, jsonBuffer);

  Serial.println("[DATA]");
  Serial.println(jsonBuffer);

  client.publish(mqtt_topic, jsonBuffer, n);
  Serial.println("[MQTT] Published");
}

// ======== SETUP ========
void setup() {
  Serial.begin(115200);
  RS485.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
}

// ======== LOOP ========
void loop() {
  if (!client.connected()) reconnect_mqtt();
  client.loop();

  if (millis() - lastRead > INTERVAL) {
    lastRead = millis();
    RS485.write(CMD_BMS, sizeof(CMD_BMS));
    RS485.flush();

    uint8_t frame[512];
    size_t framelen;
    if (readFrame(frame, framelen)) {
      uint8_t payload[256];
      size_t paylen;
      if (decodeAsciiHex(frame, framelen, payload, paylen)) {
        parseAndPublish(payload, paylen);
      } else {
        Serial.println("[WARN] Invalid ASCII frame.");
      }
    } else {
      Serial.println("[WARN] No response from BMS.");
    }
  }
}
