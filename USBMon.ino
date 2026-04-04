/*
 * ESP32-C3 USB Charge Monitor
 * - OTA updates enabled
 * - mDNS hostname: usbmon.local
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
// WiFi credentials now in Secrets.h as MYSSID and MYPSK
const char* MDNS_HOSTNAME = "usbmon";  // Access via usbmon.local

// Pin definitions
const int ADC_PIN = 0;              // GPIO 0 for current measurement
const int OUTPUT_PIN = 10;          // GPIO 10 for output control
const int BUTTON_PIN = 8;           // GPIO 8 for button input
const int SDA_PIN = 7;              // GPIO 6 for OLED SDA
const int SCL_PIN = 6;              // GPIO 7 for OLED SCL

// Current measurement calibration
const float SHUNT_RESISTANCE = 0.027;  // 50 milliohms
const float ADC_REFERENCE_VOLTAGE = 0.95;  // 950mV for 0dB attenuation
const int ADC_RESOLUTION = 4095;    // 12-bit ADC
const float VOLTAGE_GAIN = 1.0;     // Amplifier gain (1.0 if direct connection)
const float CURRENT_OFFSET = -0.15;   // Calibration offset in amps

// Add after other globals
Preferences preferences;

// ADC Configuration (adjustable via web interface)
int ADC_OVERSAMPLING = 1;           // Number of samples to average (1-64)
float ADC_SMOOTHING = 1.0;           // Exponential smoothing factor (0.0-1.0)
float filteredCurrent = 0.0;

// Measurement settings
const float CUTOFF_PERCENTAGE = 0.50;  // 50% of saved current
const int SAMPLE_INTERVAL = 100;    // ADC sampling interval (ms)
const int WEB_UPDATE_INTERVAL = 1000;  // Web chart update interval (ms)
const int MAX_DATA_POINTS = 500;    // Maximum data points stored

// Timing variables
unsigned long lastSampleTime = 0;
unsigned long lastWebUpdate = 0;
unsigned long lastMediumStore = 0;
unsigned long lastLongStore = 0;

// OLED settings
const int SCREEN_WIDTH = 128;
const int SCREEN_HEIGHT = 32;
const int OLED_RESET = -1;          // Reset pin (-1 if sharing Arduino reset)
const int OLED_ADDRESS = 0x3C;  // Try 0x3D instead of 0x3C

// ============ END CONFIGURABLE PARAMETERS ============

// Global variables
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

// Button debouncing and long press detection
bool lastButtonState = LOW;
bool buttonState = LOW;
unsigned long lastDebounceTime = 0;
unsigned long buttonPressStartTime = 0;
bool buttonPressHandled = false;
const unsigned long DEBOUNCE_DELAY = 50;
const unsigned long LONG_PRESS_TIME = 1000;  // 1 seconds

// Multi-resolution data storage
struct DataPoint {
  unsigned long timestamp;
  float current;
};

// Three buffers with different resolutions
#define RECENT_POINTS 100      // Last 100 seconds at 1s resolution
#define MEDIUM_POINTS 60       // Last hour at 1min resolution (after first 100s)
#define LONG_POINTS 100        // Remaining time at smart intervals

DataPoint recentData[RECENT_POINTS];
DataPoint mediumData[MEDIUM_POINTS];
DataPoint longData[LONG_POINTS];

int recentIndex = 0;
int mediumIndex = 0;
int longIndex = 0;

// Remove old globals: dataPoints, dataIndex, MAX_DATA_POINTS

void loadSettings() {
  preferences.begin("usbmon", false);
  ADC_OVERSAMPLING = preferences.getInt("oversampling", 32);
  ADC_SMOOTHING = preferences.getFloat("smoothing", 0.1);
  preferences.end();
  
  Serial.println("=== Settings Loaded ===");
  Serial.print("ADC Oversampling: ");
  Serial.println(ADC_OVERSAMPLING);
  Serial.print("ADC Smoothing: ");
  Serial.println(ADC_SMOOTHING, 3);
}

void saveSettings() {
  preferences.begin("usbmon", false);
  preferences.putInt("oversampling", ADC_OVERSAMPLING);
  preferences.putFloat("smoothing", ADC_SMOOTHING);
  preferences.end();
  
  Serial.println("=== Settings Saved ===");
  Serial.print("ADC Oversampling: ");
  Serial.println(ADC_OVERSAMPLING);
  Serial.print("ADC Smoothing: ");
  Serial.println(ADC_SMOOTHING, 3);
}

void setup() {
  Serial.begin(115200);
  
  // Load saved settings
  loadSettings();

  // Configure pins
  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(OUTPUT_PIN, HIGH);  // Output ON by default

  // Initialize monitoring state
  monitoring = false;
  savedCurrent = 0.0;
  cutoffThreshold = 0.0;
  chargingComplete = false;
  
  // Configure ADC
  analogReadResolution(12);  // 12-bit resolution (0-4095)
  analogSetAttenuation(ADC_0db);   // 0-950mV range, better resolution
  
  // Initialize I2C for OLED
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    while(1);
  }
  
  // IMPORTANT: Clear the buffer immediately
  display.clearDisplay();
  display.display();
  delay(100);  // Let it settle
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("USB Monitor"));
  display.println(F("Connecting WiFi..."));
  display.display();
  
  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(MYSSID, MYPSK);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  // Setup mDNS
  if (!MDNS.begin(MDNS_HOSTNAME)) {
    Serial.println("Error setting up mDNS responder!");
  } else {
    Serial.print("mDNS responder started: http://");
    Serial.print(MDNS_HOSTNAME);
    Serial.println(".local");
    MDNS.addService("http", "tcp", 80);
  }
  
  // Setup OTA
  ArduinoOTA.setHostname(MDNS_HOSTNAME);
  
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_SPIFFS
      type = "filesystem";
    }
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
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  
  ArduinoOTA.begin();
  
  // Display IP and hostname on OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("WiFi Connected"));
  display.print(MDNS_HOSTNAME);
  display.println(".local");
  display.print(F("IP: "));
  display.println(WiFi.localIP());
  display.display();
  delay(2000);
  
  // Setup web server routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/current", HTTP_GET, handleCurrent);
  server.on("/history", HTTP_GET, handleHistory);
  server.on("/settings", HTTP_GET, handleGetSettings);
  server.on("/settings", HTTP_POST, handleSetSettings);
  server.on("/toggle", HTTP_POST, handleToggleOutput);
  
  server.on("/debug", HTTP_GET, [](){
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
  
  chargeStartTime = millis();
}

void loop() {
  // Handle OTA updates
  ArduinoOTA.handle();
  
  // Handle web server requests
  server.handleClient();
  
  // Read current from ADC
  if (millis() - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = millis();
    currentCurrent = readCurrent();
    
    // Store data point for web graph
    if (millis() - lastWebUpdate >= WEB_UPDATE_INTERVAL) {
      lastWebUpdate = millis();
      storeDataPoint(currentCurrent);
    }
  }
  
  // Handle button press
  handleButton();
  
  // Check for cutoff condition
  if (monitoring && !chargingComplete) {
    if (currentCurrent <= cutoffThreshold) {
      digitalWrite(OUTPUT_PIN, LOW);
      outputEnabled = false;
      chargingComplete = true;
      chargeElapsedTime = millis() - chargeStartTime;
      display.invertDisplay(true);  // Invert screen when done
    } else {
      chargeElapsedTime = millis() - chargeStartTime;
    }
  }
  
  // Update OLED display
  updateDisplay();
}

float readCurrent() {
  // Oversample: take multiple readings and average them
  long adcSum = 0;
  
  for(int i = 0; i < ADC_OVERSAMPLING; i++) {
    adcSum += analogRead(ADC_PIN);
    delayMicroseconds(50);  // Small delay between samples
  }
  
  int adcValue = adcSum / ADC_OVERSAMPLING;
  
  float voltage = (adcValue / (float)ADC_RESOLUTION) * ADC_REFERENCE_VOLTAGE;
  voltage = voltage / VOLTAGE_GAIN;
  float current = (voltage / SHUNT_RESISTANCE) + CURRENT_OFFSET;
  
  if (current < 0) current = 0;
  
  // Apply exponential moving average for additional smoothing
  if (filteredCurrent == 0.0) {
    // First reading - initialize
    filteredCurrent = current;
  } else {
    filteredCurrent = (ADC_SMOOTHING * current) + ((1.0 - ADC_SMOOTHING) * filteredCurrent);
  }
  
  return filteredCurrent;
}

void handleButton() {
  int reading = digitalRead(BUTTON_PIN);
  
  // Debounce
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != buttonState) {
      buttonState = reading;
      
      if (buttonState == HIGH) {  
        // Button just pressed - start timing
        buttonPressStartTime = millis();
        buttonPressHandled = false;
      } else {
        // Button released
        unsigned long pressDuration = millis() - buttonPressStartTime;
        
        // SHORT PRESS: Toggle output on/off
        if (pressDuration < LONG_PRESS_TIME && !buttonPressHandled) {
          outputEnabled = !outputEnabled;
          digitalWrite(OUTPUT_PIN, outputEnabled ? HIGH : LOW);
          display.invertDisplay(!outputEnabled);  // Invert when off
          
          // Reset monitoring when turning output back ON
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
    
    // Check for LONG PRESS (2 seconds) while button is still held
    if (buttonState == HIGH && !buttonPressHandled) {
      if (millis() - buttonPressStartTime >= LONG_PRESS_TIME) {
        // Long press detected - set or reset monitoring
        savedCurrent = currentCurrent;
        cutoffThreshold = savedCurrent * CUTOFF_PERCENTAGE;
        monitoring = true;
        chargingComplete = false;  // Reset if starting over
        chargeStartTime = millis();
        chargeElapsedTime = 0;
        
        // Ensure output is ON when starting monitoring
        if (!outputEnabled) {
          outputEnabled = true;
          digitalWrite(OUTPUT_PIN, HIGH);
          display.invertDisplay(false);  // Un-invert screen
        }
        
        buttonPressHandled = true;  // Prevent multiple triggers and short press on release
        
        Serial.println("=== Long Press - Monitoring Set ===");
        Serial.print("Saved current: ");
        Serial.print(savedCurrent, 2);
        Serial.print("A, Cutoff at: ");
        Serial.print(cutoffThreshold, 2);
        Serial.println("A");
        
        // Brief OLED feedback
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
  
  // Top labels (small text)
  display.setTextSize(1);
  
  // Left label - left justified
  display.setCursor(0, 0);
  display.print("Current");
  
  // Right label - right justified
  // "Cutoff" is 6 chars * 6 pixels = 36 pixels wide
  display.setCursor(SCREEN_WIDTH - 36, 0);
  display.print("Cutoff");
  
  // Current readings (large text, side by side)
  display.setTextSize(2);
  
  // Left side - Current reading (left justified)
  display.setCursor(0, 8);
  display.print(currentCurrent, 2);
  display.print("A");
  
  // Right side - Cutoff value (right justified)
  char cutoffStr[8];
  if (monitoring) {
    // Format the cutoff string
    dtostrf(cutoffThreshold, 1, 2, cutoffStr);
    strcat(cutoffStr, "A");
  } else {
    strcpy(cutoffStr, "--");
  }
  // Each size-2 char is 12 pixels wide
  int cutoffWidth = strlen(cutoffStr) * 12;
  display.setCursor(SCREEN_WIDTH - cutoffWidth, 8);
  display.print(cutoffStr);
  
  // Bottom - Time (centered)
  display.setTextSize(1);
  unsigned long displayTime = chargingComplete ? chargeElapsedTime : (millis() - chargeStartTime);
  unsigned long seconds = displayTime / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  
  char timeStr[12];
  sprintf(timeStr, "%02lu:%02lu:%02lu", hours, minutes % 60, seconds % 60);
  
  // Center the time string (each char is 6 pixels wide in size 1)
  int timeWidth = strlen(timeStr) * 6;
  int xPos = (SCREEN_WIDTH - timeWidth) / 2;
  
  display.setCursor(xPos, 24);
  display.print(timeStr);
  
  // Status indicator on far right of time line
  if (chargingComplete) {
    display.setCursor(104, 24);
    display.print("DONE");
  } else if (!outputEnabled) {
    display.setCursor(110, 24);
    display.print("OFF");
  }
  
  display.display();
}

void handleGetSettings() {
  String json = "{";
  json += "\"oversampling\":" + String(ADC_OVERSAMPLING) + ",";
  json += "\"smoothing\":" + String(ADC_SMOOTHING, 3);
  json += "}";
  server.send(200, "application/json", json);
}

void handleSetSettings() {
  if (server.hasArg("oversampling")) {
    int newOversampling = server.arg("oversampling").toInt();
    if (newOversampling >= 1 && newOversampling <= 64) {
      ADC_OVERSAMPLING = newOversampling;
    }
  }
  
  if (server.hasArg("smoothing")) {
    float newSmoothing = server.arg("smoothing").toFloat();
    if (newSmoothing >= 0.0 && newSmoothing <= 1.0) {
      ADC_SMOOTHING = newSmoothing;
      // Reset filtered value when smoothing changes
      filteredCurrent = 0.0;
    }
  }
  
  saveSettings();
  handleGetSettings();  // Return updated settings
}

void handleToggleOutput() {
  outputEnabled = !outputEnabled;
  digitalWrite(OUTPUT_PIN, outputEnabled ? HIGH : LOW);
  display.invertDisplay(!outputEnabled);
  
  // Reset monitoring when turning output back ON
  if (outputEnabled) {
    monitoring = false;
    savedCurrent = 0.0;
    cutoffThreshold = 0.0;
    chargingComplete = false;
  }
  
  Serial.print("Web toggle - Output: ");
  Serial.println(outputEnabled ? "ON" : "OFF");
  
  handleCurrent();  // Return current status
}

void storeDataPoint(float current) {
  unsigned long now = millis();
  
  // Always store in recent buffer (1 second resolution)
  recentData[recentIndex].timestamp = now;
  recentData[recentIndex].current = current;
  recentIndex = (recentIndex + 1) % RECENT_POINTS;
  
  // Store in medium buffer every 60 seconds (1 minute resolution)
  if (now - lastMediumStore >= 60000) {
    mediumData[mediumIndex].timestamp = now;
    mediumData[mediumIndex].current = current;
    mediumIndex = (mediumIndex + 1) % MEDIUM_POINTS;
    lastMediumStore = now;
  }
  
  // Store in long buffer at smart intervals based on elapsed time
  // This spreads data across the entire time period
  unsigned long elapsed = now - chargeStartTime;
  unsigned long longInterval = max(120000UL, elapsed / LONG_POINTS);  // At least 2 minutes
  
  if (now - lastLongStore >= longInterval) {
    longData[longIndex].timestamp = now;
    longData[longIndex].current = current;
    longIndex = (longIndex + 1) % LONG_POINTS;
    lastLongStore = now;
  }
}

// Web server handlers
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
  unsigned long recentCutoff = now - 100000;    // Last 100 seconds
  unsigned long mediumCutoff = now - 3700000;   // Last ~1 hour
  
  String json = "[";
  bool first = true;
  
  // Add long-term data (oldest, sparsest)
  for (int i = 0; i < LONG_POINTS; i++) {
    int idx = (longIndex + i) % LONG_POINTS;
    if (longData[idx].timestamp > 0 && longData[idx].timestamp < mediumCutoff) {
      if (!first) json += ",";
      first = false;
      json += "{\"t\":" + String(longData[idx].timestamp) + ",";
      json += "\"c\":" + String(longData[idx].current, 3) + "}";
    }
  }
  
  // Add medium resolution data (last hour, 1min intervals)
  for (int i = 0; i < MEDIUM_POINTS; i++) {
    int idx = (mediumIndex + i) % MEDIUM_POINTS;
    if (mediumData[idx].timestamp > 0 && 
        mediumData[idx].timestamp >= mediumCutoff && 
        mediumData[idx].timestamp < recentCutoff) {
      if (!first) json += ",";
      first = false;
      json += "{\"t\":" + String(mediumData[idx].timestamp) + ",";
      json += "\"c\":" + String(mediumData[idx].current, 3) + "}";
    }
  }
  
  // Add recent high-resolution data (last 100 seconds, 1s intervals)
  for (int i = 0; i < RECENT_POINTS; i++) {
    int idx = (recentIndex + i) % RECENT_POINTS;
    if (recentData[idx].timestamp > 0 && recentData[idx].timestamp >= recentCutoff) {
      if (!first) json += ",";
      first = false;
      json += "{\"t\":" + String(recentData[idx].timestamp) + ",";
      json += "\"c\":" + String(recentData[idx].current, 3) + "}";
    }
  }
  
  json += "]";
  server.send(200, "application/json", json);
}

void dummyTest() { }  // Test function

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
  </style>
</head>
<body>
  <h1>USB Charge Monitor</h1>
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
  <h2>ADC Settings</h2>
  <div class="settings-box">
    <div class="setting-row">
      <div style="flex: 2;">
        <div class="setting-label">Oversampling</div>
        <div class="setting-desc">Number of ADC samples to average (1-64). Higher = smoother but slower. Set to 1 to disable.</div>
      </div>
      <input type="number" id="oversampling" class="setting-input" min="1" max="64" value="32">
    </div>
    <div class="setting-row">
      <div style="flex: 2;">
        <div class="setting-label">Smoothing Factor</div>
        <div class="setting-desc">Exponential moving average (0.0-1.0). Lower = smoother. 0.0 disables, 1.0 = no smoothing.</div>
      </div>
      <input type="number" id="smoothing" class="setting-input" min="0" max="1" step="0.01" value="0.10">
    </div>
    <button onclick="saveSettings()">Save Settings</button>
    <div class="success-msg" id="successMsg">Settings saved! Changes applied immediately.</div>
  </div>
</body>
<script>
// Global variable to track the latest timestamp from the device
var latestTime = 0;

var chart = new Highcharts.Chart({
  chart: { 
    renderTo: 'chart-current',
    zoomType: 'x'
  },
  title: { text: 'USB Current Over Time' },
  series: [{
    name: 'Current',
    showInLegend: false,
    data: [],
    turboThreshold: 0
  }],
  plotOptions: {
    line: { 
      animation: false,
      dataLabels: { enabled: false },
      marker: { enabled: false }
    },
    series: { color: '#059e8a' }
  },
  xAxis: { 
    type: 'linear',  // Linear axis for raw milliseconds
    title: { text: 'Time Ago' },
    reversed: false, // Standard left-to-right time
    labels: {
      formatter: function() {
        if (latestTime === 0) return "";
        
        // Calculate difference from latest known time
        const diffMs = latestTime - this.value;
        const secondsAgo = Math.floor(diffMs / 1000);
        
        if (secondsAgo < 5) return "Now";
        if (secondsAgo < 60) return "-" + secondsAgo + "s";
        if (secondsAgo < 3600) return "-" + Math.floor(secondsAgo/60) + "m";
        return "-" + (secondsAgo/3600).toFixed(1) + "h";
      }
    }
  },
  yAxis: {
    title: { text: 'Current (A)' },
    min: 0,
    minRange: 0.3
  },
  credits: { enabled: false },
  tooltip: {
    formatter: function() {
      const diffMs = latestTime - this.x;
      const secondsAgo = Math.floor(diffMs / 1000);
      let timeLabel = "Now";
      
      if (secondsAgo >= 60) {
        const mins = Math.floor(secondsAgo / 60);
        const secs = secondsAgo % 60;
        timeLabel = "-" + mins + "m " + secs + "s";
      } else if (secondsAgo > 0) {
        timeLabel = "-" + secondsAgo + "s";
      }
      
      return '<b>' + timeLabel + '</b><br/>' +
             'Current: ' + this.y.toFixed(2) + ' A';
    }
  }
});

// Function to load full history
function loadHistory() {
  fetch('/history')
    .then(response => response.json())
    .then(data => {
      if (data.length > 0) {
        // Sort data by timestamp
        data.sort((a, b) => a.t - b.t);
        
        // Update latestTime from the newest data point
        latestTime = data[data.length - 1].t;
        
        // Convert to Highcharts format [timestamp, value]
        const chartData = data.map(point => [point.t, point.c]);
        
        chart.series[0].setData(chartData, true, false, false);
      }
    });
}

// Initial load
loadHistory();

// Update loop
setInterval(function() {
  fetch('/current')
    .then(response => response.json())
    .then(data => {
      // Update text stats
      document.getElementById('currentValue').innerText = data.current.toFixed(2);
      
      if(data.monitoring && data.cutoff > 0) {
        document.getElementById('cutoffValue').innerText = data.cutoff.toFixed(2);
        document.getElementById('cutoffValue').style.color = '#059e8a';
      } else {
        document.getElementById('cutoffValue').innerText = '--';
        document.getElementById('cutoffValue').style.color = '#ccc';
      }
      
      let seconds = Math.floor(data.elapsed / 1000);
      let minutes = Math.floor(seconds / 60);
      let hours = Math.floor(minutes / 60);
      let timeStr = String(hours).padStart(2, '0') + ':' + 
                    String(minutes % 60).padStart(2, '0') + ':' + 
                    String(seconds % 60).padStart(2, '0');
      document.getElementById('timeValue').innerText = timeStr;
      document.getElementById('statusValue').innerText = data.output ? 'ON' : 'OFF';
      
      // Update global latestTime
      latestTime = data.time;
      
      // Update Chart
      const series = chart.series[0];
      const shift = series.data.length > 1000; 
      series.addPoint([data.time, data.current], true, shift, false);
    });
}, 1000);

// Refresh full history every minute
setInterval(loadHistory, 60000);
</script>
</html>
)rawliteral";

  return html;
}