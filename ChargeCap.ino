/*
 * ESP32-C3 ChargeCap
 * - OTA updates enabled
 * - mDNS hostname: chargecap.local
 *
 * Boot strategy: Core functions (OLED, button, ADC, output pin) initialize
 * immediately. WiFi, mDNS, OTA, and WebServer connect in the background via
 * a state machine in loop(). The display is usable within ~200ms of power-on.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Secrets.h>  // Contains MYSSID and MYPSK

// ============ CONFIGURABLE PARAMETERS ============
const char* MDNS_HOSTNAME = "chargecap";

const int ADC_PIN = 0;
const int BUTTON_PIN = 8;
const int SDA_PIN = 7;
const int SCL_PIN = 6;

// OUTPUT_PIN and LONG_PRESS_TIME are now runtime-configurable via the web UI
// and persisted in NVS. These are the compile-time defaults used on first boot.
const int DEFAULT_OUTPUT_PIN      = 10;
const unsigned long DEFAULT_LONG_PRESS_MS = 1000;

const float SHUNT_RESISTANCE = 0.027;
const float ADC_REFERENCE_VOLTAGE = 0.95;
const int ADC_RESOLUTION = 4095;
const float VOLTAGE_GAIN = 1.0;
const float CURRENT_OFFSET = -0.15;

Preferences preferences;

int ADC_OVERSAMPLING = 1;
float ADC_SMOOTHING = 1.0;
float filteredCurrent = 0.0;

// Runtime-settable via web UI
int OUTPUT_PIN = DEFAULT_OUTPUT_PIN;
unsigned long LONG_PRESS_TIME = DEFAULT_LONG_PRESS_MS;

const float CUTOFF_PERCENTAGE = 0.50;
const int SAMPLE_INTERVAL = 100;
const int WEB_UPDATE_INTERVAL = 1000;
const int MAX_DATA_POINTS = 500;

unsigned long lastSampleTime = 0;
unsigned long lastWebUpdate = 0;
unsigned long lastMediumStore = 0;
unsigned long lastLongStore = 0;

const int SCREEN_WIDTH = 128;
const int SCREEN_HEIGHT = 32;
const int OLED_RESET = -1;
const int OLED_ADDRESS = 0x3C;
// ============ END CONFIGURABLE PARAMETERS ============

// ============ WIFI BACKGROUND STATE MACHINE ============
enum WifiState {
  WIFI_STATE_IDLE,
  WIFI_STATE_CONNECTING,
  WIFI_STATE_CONNECTED,
  WIFI_STATE_READY
};

WifiState wifiState = WIFI_STATE_IDLE;
unsigned long wifiStartTime = 0;
const unsigned long WIFI_TIMEOUT_MS = 20000;
unsigned long wifiRetryTime = 0;
const unsigned long WIFI_RETRY_INTERVAL = 30000;
bool wifiServicesStarted = false;
// ============ END WIFI STATE MACHINE ============

// Global objects — declared before wifiBackgroundTick() uses them
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
WebServer server(80);

float currentCurrent = 0.0;
float savedCurrent = 0.0;
float cutoffThreshold = 0.0;
unsigned long chargeStartTime = 0;
unsigned long chargeElapsedTime = 0;
bool monitoring = false;
bool outputEnabled = true;
bool chargingComplete = false;

bool lastButtonState = LOW;
bool buttonState = LOW;
unsigned long lastDebounceTime = 0;
unsigned long buttonPressStartTime = 0;
bool buttonPressHandled = false;
const unsigned long DEBOUNCE_DELAY = 50;

struct DataPoint {
  unsigned long timestamp;
  float current;
};

#define RECENT_POINTS 100
#define MEDIUM_POINTS 60
#define LONG_POINTS 100

DataPoint recentData[RECENT_POINTS];
DataPoint mediumData[MEDIUM_POINTS];
DataPoint longData[LONG_POINTS];

int recentIndex = 0;
int mediumIndex = 0;
int longIndex = 0;

// Forward declarations for web handlers used in wifiBackgroundTick
void handleRoot();
void handleCurrent();
void handleHistory();
void handleGetSettings();
void handleSetSettings();
void handleToggleOutput();
String getIndexHTML();

// ============ WIFI BACKGROUND TICK ============
void wifiBackgroundTick() {
  switch (wifiState) {

    case WIFI_STATE_IDLE:
      WiFi.mode(WIFI_STA);
      WiFi.begin(MYSSID, MYPSK);
      wifiStartTime = millis();
      wifiState = WIFI_STATE_CONNECTING;
      Serial.println("WiFi: connecting in background...");
      break;

    case WIFI_STATE_CONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        wifiState = WIFI_STATE_CONNECTED;
      } else if (millis() - wifiStartTime > WIFI_TIMEOUT_MS) {
        Serial.println("\nWiFi: timed out, will retry later");
        WiFi.disconnect(true);
        wifiRetryTime = millis();
        wifiState = WIFI_STATE_IDLE;
      }
      break;

    case WIFI_STATE_CONNECTED:
      if (!MDNS.begin(MDNS_HOSTNAME)) {
        Serial.println("Error setting up mDNS responder!");
      } else {
        Serial.print("mDNS responder started: http://");
        Serial.print(MDNS_HOSTNAME);
        Serial.println(".local");
        MDNS.addService("http", "tcp", 80);
      }

      ArduinoOTA.setHostname(MDNS_HOSTNAME);

      ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        Serial.println("Start updating " + type);
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("OTA Update");
        display.println("Starting...");
        display.display();
      });

      ArduinoOTA.onEnd([]() {
        Serial.println("\nEnd");
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("OTA Update");
        display.println("Complete!");
        display.display();
      });

      ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        unsigned int percent = (progress / (total / 100));
        Serial.printf("Progress: %u%%\r", percent);
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("OTA Update");
        display.setCursor(0, 16);
        display.print("Progress: ");
        display.print(percent);
        display.println("%");
        display.display();
      });

      ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)         Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)     Serial.println("End Failed");
      });

      ArduinoOTA.begin();

      server.on("/", HTTP_GET, handleRoot);
      server.on("/current", HTTP_GET, handleCurrent);
      server.on("/history", HTTP_GET, handleHistory);
      server.on("/settings", HTTP_GET, handleGetSettings);
      server.on("/settings", HTTP_POST, handleSetSettings);
      server.on("/toggle", HTTP_POST, handleToggleOutput);

      server.on("/debug", HTTP_GET, [&](){
        String html = "<html><body><h2>Data Buffer Status</h2>";
        html += "<p>Uptime: " + String(millis() / 1000) + " seconds</p>";
        html += "<p>Recent buffer: " + String(recentIndex) + " / " + String(RECENT_POINTS) + "</p>";
        html += "<p>Medium buffer: " + String(mediumIndex) + " / " + String(MEDIUM_POINTS) + "</p>";
        html += "<p>Long buffer: " + String(longIndex) + " / " + String(LONG_POINTS) + "</p>";
        html += "<h3>Recent Data (last 10):</h3><ul>";
        for(int i = 0; i < 10; i++) {
          int idx = (recentIndex - 10 + i + RECENT_POINTS) % RECENT_POINTS;
          if(recentData[idx].timestamp > 0) {
            html += "<li>Time: " + String(recentData[idx].timestamp/1000) + "s, Current: " + String(recentData[idx].current, 2) + "A</li>";
          }
        }
        html += "</ul></body></html>";
        server.send(200, "text/html", html);
      });

      server.begin();
      wifiServicesStarted = true;
      wifiState = WIFI_STATE_READY;
      Serial.println("WiFi services ready.");
      break;

    case WIFI_STATE_READY:
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi: lost connection, will reconnect...");
        wifiServicesStarted = false;
        wifiRetryTime = millis();
        wifiState = WIFI_STATE_IDLE;
      }
      break;
  }
}
// ============ END WIFI BACKGROUND TICK ============

void loadSettings() {
  preferences.begin("chargecap", false);
  ADC_OVERSAMPLING = preferences.getInt("oversampling", 32);
  ADC_SMOOTHING    = preferences.getFloat("smoothing", 0.1);
  OUTPUT_PIN       = preferences.getInt("output_pin", DEFAULT_OUTPUT_PIN);
  LONG_PRESS_TIME  = (unsigned long)preferences.getInt("long_press_ms", (int)DEFAULT_LONG_PRESS_MS);
  preferences.end();
  Serial.println("=== Settings Loaded ===");
  Serial.print("ADC Oversampling: "); Serial.println(ADC_OVERSAMPLING);
  Serial.print("ADC Smoothing: ");    Serial.println(ADC_SMOOTHING, 3);
  Serial.print("Output PIN: ");       Serial.println(OUTPUT_PIN);
  Serial.print("Long Press ms: ");    Serial.println(LONG_PRESS_TIME);
}

void saveSettings() {
  preferences.begin("chargecap", false);
  preferences.putInt("oversampling", ADC_OVERSAMPLING);
  preferences.putFloat("smoothing", ADC_SMOOTHING);
  preferences.putInt("output_pin", OUTPUT_PIN);
  preferences.putInt("long_press_ms", (int)LONG_PRESS_TIME);
  preferences.end();
  Serial.println("=== Settings Saved ===");
  Serial.print("ADC Oversampling: "); Serial.println(ADC_OVERSAMPLING);
  Serial.print("ADC Smoothing: ");    Serial.println(ADC_SMOOTHING, 3);
  Serial.print("Output PIN: ");       Serial.println(OUTPUT_PIN);
  Serial.print("Long Press ms: ");    Serial.println(LONG_PRESS_TIME);
}

void setup() {
  Serial.begin(115200);

  loadSettings();

  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(OUTPUT_PIN, HIGH);

  monitoring = false;
  savedCurrent = 0.0;
  cutoffThreshold = 0.0;
  chargingComplete = false;

  analogReadResolution(12);
  analogSetAttenuation(ADC_0db);

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    while(1);
  }

  display.clearDisplay();
  display.display();
  delay(100);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("ChargeCap"));
  display.println(F("WiFi: connecting..."));
  display.display();

  // WiFi connects in the background via wifiBackgroundTick() in loop()
  chargeStartTime = millis();
}

void loop() {
  static unsigned long lastWifiTick = 0;
  unsigned long now = millis();
  bool shouldTickWifi = false;

  if (wifiState == WIFI_STATE_IDLE) {
    if (wifiRetryTime == 0 || (now - wifiRetryTime >= WIFI_RETRY_INTERVAL)) {
      shouldTickWifi = true;
    }
  } else if (wifiState == WIFI_STATE_CONNECTING) {
    if (now - lastWifiTick >= 200) shouldTickWifi = true;
  } else if (wifiState == WIFI_STATE_CONNECTED) {
    shouldTickWifi = true;
  } else if (wifiState == WIFI_STATE_READY) {
    if (now - lastWifiTick >= 5000) shouldTickWifi = true;
  }

  if (shouldTickWifi) {
    lastWifiTick = now;
    wifiBackgroundTick();
  }

  if (wifiServicesStarted) {
    ArduinoOTA.handle();
    server.handleClient();
  }

  if (millis() - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = millis();
    currentCurrent = readCurrent();
    if (millis() - lastWebUpdate >= WEB_UPDATE_INTERVAL) {
      lastWebUpdate = millis();
      storeDataPoint(currentCurrent);
    }
  }

  handleButton();

  if (monitoring && !chargingComplete) {
    if (currentCurrent <= cutoffThreshold) {
      digitalWrite(OUTPUT_PIN, LOW);
      outputEnabled = false;
      chargingComplete = true;
      chargeElapsedTime = millis() - chargeStartTime;
      display.invertDisplay(true);
    } else {
      chargeElapsedTime = millis() - chargeStartTime;
    }
  }

  updateDisplay();
}

float readCurrent() {
  long adcSum = 0;
  for(int i = 0; i < ADC_OVERSAMPLING; i++) {
    adcSum += analogRead(ADC_PIN);
    delayMicroseconds(50);
  }
  int adcValue = adcSum / ADC_OVERSAMPLING;
  float voltage = (adcValue / (float)ADC_RESOLUTION) * ADC_REFERENCE_VOLTAGE;
  voltage = voltage / VOLTAGE_GAIN;
  float current = (voltage / SHUNT_RESISTANCE) + CURRENT_OFFSET;
  if (current < 0) current = 0;
  if (filteredCurrent == 0.0) {
    filteredCurrent = current;
  } else {
    filteredCurrent = (ADC_SMOOTHING * current) + ((1.0 - ADC_SMOOTHING) * filteredCurrent);
  }
  return filteredCurrent;
}

void handleButton() {
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == HIGH) {
        buttonPressStartTime = millis();
        buttonPressHandled = false;
      } else {
        unsigned long pressDuration = millis() - buttonPressStartTime;
        if (pressDuration < LONG_PRESS_TIME && !buttonPressHandled) {
          outputEnabled = !outputEnabled;
          digitalWrite(OUTPUT_PIN, outputEnabled ? HIGH : LOW);
          display.invertDisplay(!outputEnabled);
          if (outputEnabled) {
            monitoring = false;
            savedCurrent = 0.0;
            cutoffThreshold = 0.0;
            chargingComplete = false;
          }
          Serial.print("Output toggled: ");
          Serial.println(outputEnabled ? "ON" : "OFF");
        }
        buttonPressHandled = false;
      }
    }
    if (buttonState == HIGH && !buttonPressHandled) {
      if (millis() - buttonPressStartTime >= LONG_PRESS_TIME) {
        savedCurrent = currentCurrent;
        cutoffThreshold = savedCurrent * CUTOFF_PERCENTAGE;
        monitoring = true;
        chargingComplete = false;
        chargeStartTime = millis();
        chargeElapsedTime = 0;
        if (!outputEnabled) {
          outputEnabled = true;
          digitalWrite(OUTPUT_PIN, HIGH);
          display.invertDisplay(false);
        }
        buttonPressHandled = true;
        Serial.println("=== Long Press - Monitoring Set ===");
        Serial.print("Saved current: "); Serial.print(savedCurrent, 2);
        Serial.print("A, Cutoff at: ");  Serial.print(cutoffThreshold, 2); Serial.println("A");
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(0, 0);
        display.println("SAVED!");
        display.setTextSize(1);
        display.setCursor(0, 20);
        display.print(savedCurrent, 2);
        display.print("A -> ");
        display.print(cutoffThreshold, 2);
        display.print("A");
        display.display();
        delay(800);
      }
    }
  }
  lastButtonState = reading;
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Current");
  display.setCursor(SCREEN_WIDTH - 36, 0);
  display.print("Cutoff");
  display.setTextSize(2);
  display.setCursor(0, 8);
  display.print(currentCurrent, 2);
  display.print("A");
  char cutoffStr[8];
  if (monitoring) {
    dtostrf(cutoffThreshold, 1, 2, cutoffStr);
    strcat(cutoffStr, "A");
  } else {
    strcpy(cutoffStr, "--");
  }
  int cutoffWidth = strlen(cutoffStr) * 12;
  display.setCursor(SCREEN_WIDTH - cutoffWidth, 8);
  display.print(cutoffStr);
  display.setTextSize(1);
  unsigned long displayTime = chargingComplete ? chargeElapsedTime : (millis() - chargeStartTime);
  unsigned long seconds = displayTime / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  char timeStr[12];
  sprintf(timeStr, "%02lu:%02lu:%02lu", hours, minutes % 60, seconds % 60);
  int timeWidth = strlen(timeStr) * 6;
  int xPos = (SCREEN_WIDTH - timeWidth) / 2;
  display.setCursor(xPos, 24);
  display.print(timeStr);
  if (chargingComplete) {
    display.setCursor(104, 24);
    display.print("DONE");
  } else if (!outputEnabled) {
    display.setCursor(110, 24);
    display.print("OFF");
  }
  // Small WiFi status indicator, bottom-left, disappears once ready
  if (wifiState != WIFI_STATE_READY) {
    display.setCursor(0, 24);
    display.print(wifiState == WIFI_STATE_CONNECTING ? "W.." : "W?");
  }
  display.display();
}

void handleGetSettings() {
  String json = "{";
  json += "\"oversampling\":" + String(ADC_OVERSAMPLING) + ",";
  json += "\"smoothing\":" + String(ADC_SMOOTHING, 3) + ",";
  json += "\"output_pin\":" + String(OUTPUT_PIN) + ",";
  json += "\"long_press_ms\":" + String((int)LONG_PRESS_TIME);
  json += "}";
  server.send(200, "application/json", json);
}

void handleSetSettings() {
  if (server.hasArg("oversampling")) {
    int v = server.arg("oversampling").toInt();
    if (v >= 1 && v <= 64) ADC_OVERSAMPLING = v;
  }
  if (server.hasArg("smoothing")) {
    float v = server.arg("smoothing").toFloat();
    if (v >= 0.0 && v <= 1.0) {
      ADC_SMOOTHING = v;
      filteredCurrent = 0.0;
    }
  }
  if (server.hasArg("output_pin")) {
    int v = server.arg("output_pin").toInt();
    // Accept any valid ESP32-C3 GPIO (0-10, 18-21)
    if (v >= 0 && v <= 21) {
      // Release old pin first
      pinMode(OUTPUT_PIN, INPUT);
      OUTPUT_PIN = v;
      pinMode(OUTPUT_PIN, OUTPUT);
      digitalWrite(OUTPUT_PIN, outputEnabled ? HIGH : LOW);
    }
  }
  if (server.hasArg("long_press_ms")) {
    int v = server.arg("long_press_ms").toInt();
    if (v >= 200 && v <= 5000) LONG_PRESS_TIME = (unsigned long)v;
  }
  saveSettings();
  handleGetSettings();
}

void handleToggleOutput() {
  outputEnabled = !outputEnabled;
  digitalWrite(OUTPUT_PIN, outputEnabled ? HIGH : LOW);
  display.invertDisplay(!outputEnabled);
  if (outputEnabled) {
    monitoring = false;
    savedCurrent = 0.0;
    cutoffThreshold = 0.0;
    chargingComplete = false;
  }
  Serial.print("Web toggle - Output: ");
  Serial.println(outputEnabled ? "ON" : "OFF");
  handleCurrent();
}

void storeDataPoint(float current) {
  unsigned long now = millis();
  recentData[recentIndex].timestamp = now;
  recentData[recentIndex].current = current;
  recentIndex = (recentIndex + 1) % RECENT_POINTS;
  if (now - lastMediumStore >= 60000) {
    mediumData[mediumIndex].timestamp = now;
    mediumData[mediumIndex].current = current;
    mediumIndex = (mediumIndex + 1) % MEDIUM_POINTS;
    lastMediumStore = now;
  }
  unsigned long elapsed = now - chargeStartTime;
  unsigned long longInterval = max(120000UL, elapsed / LONG_POINTS);
  if (now - lastLongStore >= longInterval) {
    longData[longIndex].timestamp = now;
    longData[longIndex].current = current;
    longIndex = (longIndex + 1) % LONG_POINTS;
    lastLongStore = now;
  }
}

void handleRoot() {
  server.send(200, "text/html", getIndexHTML());
}

void handleCurrent() {
  String json = "{";
  json += "\"current\":" + String(currentCurrent, 3) + ",";
  json += "\"time\":" + String(millis()) + ",";
  json += "\"output\":" + String(outputEnabled ? "true" : "false") + ",";
  json += "\"monitoring\":" + String(monitoring ? "true" : "false") + ",";
  json += "\"cutoff\":" + String(cutoffThreshold, 3) + ",";
  json += "\"elapsed\":" + String(chargingComplete ? chargeElapsedTime : (millis() - chargeStartTime));
  json += "}";
  server.send(200, "application/json", json);
}

void handleHistory() {
  unsigned long now = millis();
  unsigned long recentCutoff = now - 100000;
  unsigned long mediumCutoff = now - 3700000;
  String json = "[";
  bool first = true;
  for (int i = 0; i < LONG_POINTS; i++) {
    int idx = (longIndex + i) % LONG_POINTS;
    if (longData[idx].timestamp > 0 && longData[idx].timestamp < mediumCutoff) {
      if (!first) json += ",";
      first = false;
      json += "{\"t\":" + String(longData[idx].timestamp) + ",\"c\":" + String(longData[idx].current, 3) + "}";
    }
  }
  for (int i = 0; i < MEDIUM_POINTS; i++) {
    int idx = (mediumIndex + i) % MEDIUM_POINTS;
    if (mediumData[idx].timestamp > 0 &&
        mediumData[idx].timestamp >= mediumCutoff &&
        mediumData[idx].timestamp < recentCutoff) {
      if (!first) json += ",";
      first = false;
      json += "{\"t\":" + String(mediumData[idx].timestamp) + ",\"c\":" + String(mediumData[idx].current, 3) + "}";
    }
  }
  for (int i = 0; i < RECENT_POINTS; i++) {
    int idx = (recentIndex + i) % RECENT_POINTS;
    if (recentData[idx].timestamp > 0 && recentData[idx].timestamp >= recentCutoff) {
      if (!first) json += ",";
      first = false;
      json += "{\"t\":" + String(recentData[idx].timestamp) + ",\"c\":" + String(recentData[idx].current, 3) + "}";
    }
  }
  json += "]";
  server.send(200, "application/json", json);
}

void dummyTest() { }

String getIndexHTML() {
  String html = R"rawliteral(<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script src="https://code.highcharts.com/highcharts.js"></script>
  <style>
    body { font-family: Arial; text-align: center; margin: 0 auto; max-width: 900px; padding: 20px; }
    h1 { font-size: 2.5rem; }
    h2 { font-size: 1.5rem; margin-top: 30px; }
    .stats { display: flex; justify-content: space-around; margin: 20px 0; }
    .stat-box { background: #f0f0f0; padding: 15px; border-radius: 5px; min-width: 150px; }
    #statusBox:hover { background: #e0e0e0; }
    .stat-value { font-size: 2rem; font-weight: bold; color: #059e8a; }
    .stat-label { font-size: 0.9rem; color: #666; }
    .settings-box { background: #f9f9f9; border: 1px solid #ddd; border-radius: 5px; padding: 20px; margin: 20px 0; text-align: left; }
    .setting-row { margin: 15px 0; display: flex; align-items: center; justify-content: space-between; }
    .setting-label { font-weight: bold; flex: 1; }
    .setting-input { width: 80px; padding: 5px; font-size: 1rem; }
    .setting-desc { font-size: 0.85rem; color: #666; margin-top: 5px; }
    button { background: #059e8a; color: white; border: none; padding: 10px 20px; font-size: 1rem; border-radius: 5px; cursor: pointer; margin-top: 10px; }
    button:hover { background: #047a6a; }
    .success-msg { color: #059e8a; font-weight: bold; display: none; margin-top: 10px; }
    .section-divider { border: none; border-top: 1px solid #ddd; margin: 20px 0; }
  </style>
</head>
<body>
  <h1>ChargeCap</h1>
  <div class="stats">
    <div class="stat-box">
      <div class="stat-value" id="currentValue">0.00</div>
      <div class="stat-label">Current (A)</div>
    </div>
    <div class="stat-box" id="statusBox" style="cursor: pointer;" onclick="toggleOutput()">
      <div class="stat-value" id="statusValue">ON</div>
      <div class="stat-label">Output Status (click to toggle)</div>
    </div>
    <div class="stat-box">
      <div class="stat-value" id="cutoffValue">--</div>
      <div class="stat-label">Cutoff (A)</div>
    </div>
    <div class="stat-box">
      <div class="stat-value" id="timeValue">00:00:00</div>
      <div class="stat-label">Charge Time</div>
    </div>
  </div>
  <div id="chart-current" style="width:100%; height:400px;"></div>

  <h2>Settings</h2>
  <div class="settings-box">

    <div class="setting-row">
      <div style="flex: 2;">
        <div class="setting-label">Output GPIO Pin</div>
        <div class="setting-desc">GPIO pin that drives the MOSFET/relay cutoff (0&ndash;21). Takes effect immediately &mdash; old pin is released. Default: 10.</div>
      </div>
      <input type="number" id="output_pin" class="setting-input" min="0" max="21" value="10">
    </div>

    <div class="setting-row">
      <div style="flex: 2;">
        <div class="setting-label">Trigger Hold Duration (ms)</div>
        <div class="setting-desc">How long to hold the button to set the cutoff threshold (200&ndash;5000 ms). Default: 1000.</div>
      </div>
      <input type="number" id="long_press_ms" class="setting-input" min="200" max="5000" step="50" value="1000">
    </div>

    <hr class="section-divider">

    <div class="setting-row">
      <div style="flex: 2;">
        <div class="setting-label">ADC Oversampling</div>
        <div class="setting-desc">Number of ADC samples to average (1&ndash;64). Higher = smoother but slower.</div>
      </div>
      <input type="number" id="oversampling" class="setting-input" min="1" max="64" value="32">
    </div>

    <div class="setting-row">
      <div style="flex: 2;">
        <div class="setting-label">ADC Smoothing Factor</div>
        <div class="setting-desc">Exponential moving average (0.0&ndash;1.0). Lower = smoother.</div>
      </div>
      <input type="number" id="smoothing" class="setting-input" min="0" max="1" step="0.01" value="0.10">
    </div>

    <button onclick="saveSettings()">Save Settings</button>
    <div class="success-msg" id="successMsg">Settings saved! Changes applied immediately.</div>
  </div>
</body>
<script>
var latestTime = 0;
var chart = new Highcharts.Chart({
  chart: { renderTo: 'chart-current', zoomType: 'x' },
  title: { text: 'Charge Current Over Time' },
  series: [{ name: 'Current', showInLegend: false, data: [], turboThreshold: 0 }],
  plotOptions: {
    line: { animation: false, dataLabels: { enabled: false }, marker: { enabled: false } },
    series: { color: '#059e8a' }
  },
  xAxis: {
    type: 'linear',
    title: { text: 'Time Ago' },
    labels: {
      formatter: function() {
        if (latestTime === 0) return "";
        const diffMs = latestTime - this.value;
        const secondsAgo = Math.floor(diffMs / 1000);
        if (secondsAgo < 5) return "Now";
        if (secondsAgo < 60) return "-" + secondsAgo + "s";
        if (secondsAgo < 3600) return "-" + Math.floor(secondsAgo/60) + "m";
        return "-" + (secondsAgo/3600).toFixed(1) + "h";
      }
    }
  },
  yAxis: { title: { text: 'Current (A)' }, min: 0, minRange: 0.3 },
  credits: { enabled: false },
  tooltip: {
    formatter: function() {
      const diffMs = latestTime - this.x;
      const secondsAgo = Math.floor(diffMs / 1000);
      let timeLabel = "Now";
      if (secondsAgo >= 60) {
        timeLabel = "-" + Math.floor(secondsAgo/60) + "m " + (secondsAgo%60) + "s";
      } else if (secondsAgo > 0) {
        timeLabel = "-" + secondsAgo + "s";
      }
      return '<b>' + timeLabel + '</b><br/>Current: ' + this.y.toFixed(2) + ' A';
    }
  }
});

function loadHistory() {
  fetch('/history').then(r => r.json()).then(data => {
    if (data.length > 0) {
      data.sort((a, b) => a.t - b.t);
      latestTime = data[data.length - 1].t;
      chart.series[0].setData(data.map(p => [p.t, p.c]), true, false, false);
    }
  });
}
loadHistory();

setInterval(function() {
  fetch('/current').then(r => r.json()).then(data => {
    document.getElementById('currentValue').innerText = data.current.toFixed(2);
    if (data.monitoring && data.cutoff > 0) {
      document.getElementById('cutoffValue').innerText = data.cutoff.toFixed(2);
      document.getElementById('cutoffValue').style.color = '#059e8a';
    } else {
      document.getElementById('cutoffValue').innerText = '--';
      document.getElementById('cutoffValue').style.color = '#ccc';
    }
    let s = Math.floor(data.elapsed/1000), m = Math.floor(s/60), h = Math.floor(m/60);
    document.getElementById('timeValue').innerText =
      String(h).padStart(2,'0')+':'+String(m%60).padStart(2,'0')+':'+String(s%60).padStart(2,'0');
    document.getElementById('statusValue').innerText = data.output ? 'ON' : 'OFF';
    latestTime = data.time;
    const series = chart.series[0];
    series.addPoint([data.time, data.current], true, series.data.length > 1000, false);
  });
}, 1000);

setInterval(loadHistory, 60000);

function toggleOutput() {
  fetch('/toggle', {method:'POST'}).then(r => r.json()).then(data => {
    document.getElementById('statusValue').innerText = data.output ? 'ON' : 'OFF';
  });
}

function saveSettings() {
  const params = new URLSearchParams();
  params.append('oversampling',  document.getElementById('oversampling').value);
  params.append('smoothing',     document.getElementById('smoothing').value);
  params.append('output_pin',    document.getElementById('output_pin').value);
  params.append('long_press_ms', document.getElementById('long_press_ms').value);
  fetch('/settings', {method:'POST', body:params}).then(r => r.json()).then(() => {
    document.getElementById('successMsg').style.display = 'block';
    setTimeout(() => { document.getElementById('successMsg').style.display = 'none'; }, 3000);
  });
}

fetch('/settings').then(r => r.json()).then(data => {
  document.getElementById('oversampling').value  = data.oversampling;
  document.getElementById('smoothing').value     = data.smoothing;
  document.getElementById('output_pin').value    = data.output_pin;
  document.getElementById('long_press_ms').value = data.long_press_ms;
});
</script>
</html>
)rawliteral";
  return html;
}
