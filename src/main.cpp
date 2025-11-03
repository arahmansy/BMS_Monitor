#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ElegantOTA.h>
#include "time.h"
#include <HTTPClient.h>

#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
// ======== WIFI CONFIG ========
#define TRIGGER_PIN 0
const char *ssid = "Test";
const char *password = "123456789";
WiFiManager wifiManager;
const char *firmwareUrl = "https://github.com/arahmansy/BMS_Monitor/releases/download/v1.0.1/firmware.bin";


// JSON file hosted on GitHub (raw URL)
const char* versionURL = "https://raw.githubusercontent.com/arahmansy/BMS_Monitor/main/version.json";
const char* currentVersion = "1.0.3";   // <-- change this each build



// ==== NTP Server and Timezone ====
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 2 * 3600; // GMT+2 (Lebanon)
const int daylightOffset_sec = 0;    // no daylight saving
unsigned long lastNtpSync = 0;
const unsigned long RESYNC_INTERVAL = 6UL * 3600UL * 1000UL; // 6 hours in ms

// ======== MQTT CONFIG ========
const char *mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char *mqtt_client_id = "ESP32_BMS";
const char *mqtt_topic = "adnansy/bms/status/info";
const char *mqtt_topic_state = "adnansy/bms/status/state"; // online/offline

WiFiClient espClient;
PubSubClient client(espClient);

void updateFirmware()
{
  Serial.println("Start Update frimware....");
  HTTPClient http;
  http.begin(firmwareUrl);
  int httpCode = http.GET();
  printf("HTTP Code : {0}",httpCode);
  if (httpCode == HTTP_CODE_OK)
  {
    int len = http.getSize();
    WiFiClient *stream = http.getStreamPtr();
    if (Update.begin(len))
    {
      Update.writeStream(*stream);
      if (Update.end() && Update.isFinished())
      {
        Serial.println("✅ OTA Success! Rebooting...");
        ESP.restart();
      }
    }
  }
  http.end();
}
String getDateTimeString()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    return "NTP Error";
  }

  char buffer[25];
  strftime(buffer, sizeof(buffer), "%Y/%m/%d %H:%M:%S", &timeinfo);
  return String(buffer);
}

void mqtt_connect()
{
  while (!client.connected())
  {
    Serial.print("[MQTT] Connecting...");
    if (client.connect(mqtt_client_id, nullptr, nullptr,
                       mqtt_topic_state, 0, true, "offline"))
    {
      Serial.println(" connected!");
      client.publish(mqtt_topic_state, "online", true);
    }
    else
    {
      Serial.print(" failed (rc=");
      Serial.print(client.state());
      Serial.println(") retrying in 3s");
      delay(3000);
    }
  }
}

// ======== SERIAL CONFIG ========
#define RX_PIN 16 // Connect to BMS TX
#define TX_PIN 17 // Connect to BMS RX
#define RS485 Serial2
#define BAUD 9600
#define INTERVAL 10000UL // 10 seconds
uint8_t CMD_BMS[] = {0x7E, 0x32, 0x35, 0x30, 0x31, 0x34, 0x36, 0x34, 0x32, 0x45, 0x30, 0x30, 0x32, 0x30, 0x31, 0x46, 0x44, 0x33, 0x30, 0x0D};

WebServer server(80);

// ======== GLOBALS ========

unsigned long lastRead = 0;

void toggleSRV()
{
  digitalWrite(5, LOW);
  delay(100);
  digitalWrite(5, HIGH);
}

void toggleSerialLED()
{
  digitalWrite(4, LOW);
  delay(100);
  digitalWrite(4, HIGH);
}

void pinSetup()
{
  pinMode(TRIGGER_PIN, INPUT);
  pinMode(4, OUTPUT); // ext
  pinMode(5, OUTPUT); // srv
  pinMode(2, OUTPUT); // net
  // pinMode(15, OUTPUT); // RS enable
  pinMode(14, OUTPUT);   // reset relay
  pinMode(13, OUTPUT);   // set relay
  digitalWrite(4, HIGH); // ext
  digitalWrite(5, HIGH); // srv
  digitalWrite(2, HIGH); // net
  digitalWrite(14, LOW);
  digitalWrite(13, HIGH); //
}

void checkButton()
{
  // check for button press
  if (digitalRead(TRIGGER_PIN) == LOW)
  {
    // poor mans debounce/press-hold, code not ideal for production
    delay(50);
    if (digitalRead(TRIGGER_PIN) == LOW)
    {
      Serial.println("Button Pressed");
      // still holding button for 3000 ms, reset settings, code not ideaa for production
      delay(3000); // reset delay hold
      if (digitalRead(TRIGGER_PIN) == LOW)
      {
        Serial.println("Button Held");
        Serial.println("Erasing Config, restarting");
        wifiManager.resetSettings();
        ESP.restart();
      }

      // start portal w delay
      Serial.println("Starting config portal");
      wifiManager.setConfigPortalTimeout(120);

      if (!wifiManager.startConfigPortal("OnDemandAP", "password"))
      {
        Serial.println("failed to connect or hit timeout");
        delay(3000);
        // ESP.restart();
      }
      else
      {
        // if you get here you have connected to the WiFi
        Serial.println("connected...yeey :)");
      }
    }
  }
}

// ======== FRAME HELPERS ========
bool readFrame(uint8_t *buffer, size_t &len)
{
  len = 0;
  bool inFrame = false;
  unsigned long start = millis();

  while (millis() - start < 2000)
  { // 2s timeout
    if (Serial.available())
    {
      uint8_t b = Serial.read();
      if (b == 0x7E && !inFrame)
      {
        len = 0;
        buffer[len++] = b;
        inFrame = true;
      }
      else if (inFrame)
      {
        buffer[len++] = b;
        if (b == 0x0D)
          return true;
        if (len >= 512)
          break;
      }
    }
  }
  return false;
}

bool decodeAsciiHex(const uint8_t *raw, size_t len, uint8_t *out, size_t &outlen)
{
  if (len < 3 || raw[0] != 0x7E || raw[len - 1] != 0x0D)
    return false;
  outlen = 0;
  for (size_t i = 1; i < len - 1; i += 2)
  {
    char hi = raw[i];
    char lo = raw[i + 1];
    uint8_t val = (uint8_t)((strtol((String(hi) + String(lo)).c_str(), NULL, 16)) & 0xFF);
    out[outlen++] = val;
  }
  return true;
}

// ======== PARSER ========
void parseAndPublish(uint8_t *p, size_t len)
{
  if (len < 20)
    return;
  size_t idx = 0;

  uint8_t VER = p[idx++], ADR = p[idx++], CID1 = p[idx++], RTN = p[idx++];
  uint16_t LEN = (p[idx++] << 8) | p[idx++];
  uint8_t infoflag = p[idx++];
  uint8_t pack_no = p[idx++];
  uint8_t cell_count = p[idx++];

  float cells[24];
  for (int i = 0; i < cell_count && idx + 1 < len; i++)
  {
    uint16_t mv = (p[idx++] << 8) | p[idx++];
    cells[i] = mv / 1000.0;
  }

  uint8_t temp_count = p[idx++];
  float temps[8];
  for (int i = 0; i < temp_count && idx + 1 < len; i++)
  {
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
  doc["timestamp"] = getDateTimeString();
  doc["pack_voltage"] = pack_voltage;
  doc["pack_current"] = pack_current;
  doc["remain_Ah"] = remain_Ah;
  doc["full_Ah"] = full_Ah;
  doc["SOC"] = SOC;
  doc["remaining_hours"] = remain_hours;
  doc["cycles"] = cycle;
  doc["design_Ah"] = design_Ah;

  JsonArray arrCells = doc.createNestedArray("cells");
  for (int i = 0; i < cell_count; i++)
    arrCells.add(cells[i]);

  JsonArray arrTemps = doc.createNestedArray("temps");
  for (int i = 0; i < temp_count; i++)
    arrTemps.add(temps[i]);

  char jsonBuffer[512];
  size_t n = serializeJson(doc, jsonBuffer);

  // Serial.println("[DATA]");
  // Serial.println(jsonBuffer);

  mqtt_connect();
  // Serial.println("[MQTT] Publishing...");

  bool ok = client.publish(mqtt_topic, jsonBuffer, n);
  // Serial.println(ok ? "[MQTT] Sent OK ✅" : "[MQTT] Send failed ❌");
  //  if (client.publish(mqtt_topic, jsonBuffer, n))
  //  {
  //    Serial.println("[MQTT] Sent successfully.");
  //  }
  //  else
  //  {
  //    Serial.println("[MQTT] Send failed!");
  //  }

  // Serial.print("Topic: ");
  // Serial.println(mqtt_topic);
  // Serial.println(jsonBuffer);
}

// ======== SETUP ========

void configModeCallback(WiFiManager *myWiFiManager)
{
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());

  Serial.println(myWiFiManager->getConfigPortalSSID());
}

// flag for saving data
bool shouldSaveConfig = false;

// callback notifying us of the need to save config
void saveConfigCallback()
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

// ==== Function: Sync time with NTP ====
void syncTime()
{
  Serial.println("⏳ Syncing time with NTP...");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  int retries = 0;
  while (!getLocalTime(&timeinfo) && retries < 10)
  {
    Serial.print(".");
    delay(1000);
    retries++;
  }
  if (retries < 10)
  {
    Serial.println("\n✅ Time synced successfully!");
    lastNtpSync = millis();
  }
  else
  {
    Serial.println("\n❌ Failed to sync time!");
  }
}


void performOTA(String url) {
  WiFiClientSecure client;
  client.setInsecure(); // skip certificate validation (or use your own root CA)

  HTTPClient https;
  https.begin(client, url);
  int httpCode = https.GET();
  if (httpCode == HTTP_CODE_OK) {
    int len = https.getSize();
    WiFiClient * stream = https.getStreamPtr();

    if (Update.begin(len)) {
      size_t written = Update.writeStream(*stream);
      if (written == len) {
        Serial.println("Firmware written successfully!");
      } else {
        Serial.printf("Written %d/%d bytes\n", written, len);
      }
      if (Update.end() && Update.isFinished()) {
        Serial.println("✅ OTA complete! Rebooting...");
        ESP.restart();
      } else {
        Serial.printf("❌ OTA error: %s\n", Update.errorString());
      }
    } else {
      Serial.println("❌ Not enough space for OTA");
    }
  } else {
    Serial.printf("❌ HTTP error: %d\n", httpCode);
  }
  https.end();
}


void checkForUpdates() {
  HTTPClient http;
  http.begin(versionURL);
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
      Serial.println("❌ JSON parse failed");
      return;
    }
    String latestVersion = doc["version"];
    String firmwareUrl = doc["url"];
    Serial.printf("Current version: %s | Latest: %s\n", currentVersion, latestVersion.c_str());
    if (latestVersion != currentVersion) {
      Serial.println("🟢 New version found! Starting OTA...");
      performOTA(firmwareUrl);
    } else {
      Serial.println("✅ Firmware is up to date.");
    }
  } else {
    Serial.printf("❌ Failed to fetch version.json, code: %d\n", httpCode);
  }
  http.end();
}


void setup()
{
  Serial.println("BMS Monitor V 1.2");
  pinSetup();
  Serial.begin(9600);
  RS485.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  // setup_wifi();

  client.setServer(mqtt_server, mqtt_port);
  client.setBufferSize(1024);
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setConfigPortalTimeout(180);
  bool res = wifiManager.autoConnect("ESP-Config", "123456789");
  if (!res)
  {
    // Serial.println("Failed to connect");
    //  ESP.restart();
  }
  else
  {
    // if you get here you have connected to the WiFi
    // Serial.println("connected...yeey :)");
  }

  mqtt_connect();
  server.on("/", []()
            { server.send(200, "text/plain", "Hi! This is ElegantOTA Demo."); });

  ElegantOTA.begin(&server); // Start ElegantOTA
  server.begin();
  // Serial.println("HTTP server started");
  mqtt_connect();

  // Serial.println("[MQTT] Publishing...");
  if (client.publish(mqtt_topic, "Started"))
  {
    /// Serial.println("[MQTT] Sent successfully.");
  }
  else
  {
    // Serial.println("[MQTT] Send failed!");
  }

  syncTime();
  Serial.println(getDateTimeString());
  ///updateFirmware();

  checkForUpdates();
}

// ======== LOOP ========
void loop()
{
  if (millis() - lastNtpSync > RESYNC_INTERVAL)
  {
    syncTime();
  }

  checkButton();

  if (WiFi.isConnected())
  {
    digitalWrite(2, LOW);
  }
  else
  {
    digitalWrite(2, HIGH);
  }

  if (!client.connected())
  {
    mqtt_connect();
    digitalWrite(5, HIGH);
  }
  else
  {
    digitalWrite(5, LOW);
  }

  client.loop();

  if (millis() - lastRead > INTERVAL)
  {
    lastRead = millis();
    Serial.write(CMD_BMS, sizeof(CMD_BMS));

    Serial.flush();

    uint8_t frame[512];
    size_t framelen;
    if (readFrame(frame, framelen))
    {
      uint8_t payload[256];
      size_t paylen;
      if (decodeAsciiHex(frame, framelen, payload, paylen))
      {
        parseAndPublish(payload, paylen);
        toggleSerialLED();
        toggleSRV();
      }
      else
      {
        // Serial.println("[WARN] Invalid ASCII frame.");
      }
    }
    else
    {
      // Serial.println("[WARN] No response from BMS.");
    }
  }

  server.handleClient();
  ElegantOTA.loop();
}
