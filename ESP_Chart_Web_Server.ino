#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <U8g2lib.h>
#include <math.h>

// ---------------------------
// OLED SETUP (U8G2)
// ---------------------------
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ---------------------------
// Wi-Fi & Server
// ---------------------------
const char* ssid = "Redmi c";
const char* password = "1234567890";
AsyncWebServer server(80);

// ---------------------------
// MPU6050 & Motion Variables
// ---------------------------
Adafruit_MPU6050 mpu;
float angleX = 0;
float alpha = 0.90;
unsigned long prevTime = 0;
unsigned long stableStartTime = 0;
bool isStable = false;
bool ledState = false;

// ---------------------------
// Thresholds
// ---------------------------
#define STABLE_ACCEL_LOW   -1.5
#define STABLE_ACCEL_HIGH   2
#define STABLE_TIME_REQUIRED 1500
#define ANGLE_LOW   -5
#define ANGLE_HIGH   5
const unsigned long blinkInterval = 300;
unsigned long previousBlink = 0;

// ---------------------------
// LED Indicators
// ---------------------------


// ---------------------------
// Button (Start/Stop)
// ---------------------------
#define BTN_PIN D7
String currentState = "STOP";
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// ---------------------------
// Grip Sensors
// ---------------------------
const int numReadings = 10;
int readings1[numReadings];
int readings2[numReadings];
int readings3[numReadings];
int readIndex = 0;
int total1 = 0, total2 = 0, total3 = 0;
int average1 = 0, average2 = 0, average3 = 0;
int overallAverage = 0;

int sensor1Pin = A0;
int sensor2Pin = A1;
int sensor3Pin = A2;

// ---------------------------
// OLED Timer
// ---------------------------
unsigned long startTime = 0;
const unsigned long displayDuration = 20000;

// ---------------------------
// Helper Functions
// ---------------------------
String readGrip() {
  if (overallAverage > 1000) return "TIGHT";
  else if (overallAverage > 800) return "NORMAL";
  else if (overallAverage > 450) return "Loose";
  else return "Null";
}

int getGripLevelValue() {
  int val = map(overallAverage, 300, 1100, 0, 100);
  return constrain(val, 0, 100);
}

String readTotalAccel() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float total = sqrt(pow(a.acceleration.x, 2) +
                     pow(a.acceleration.y, 2) +
                     pow(a.acceleration.z, 2));
  return String(total - 9.78, 3);
}

// ---------------------------
// SETUP
// ---------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Initializing System...");

  // OLED Init
  u8g2.begin();
  u8g2.setFlipMode(1);

  // Button
  pinMode(BTN_PIN, INPUT_PULLUP);

  // MPU Setup
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050. Check wiring!");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
  Serial.println("MPU6050 connected!");

  // LEDs


  // Filesystem
  if (!LittleFS.begin()) {
    Serial.println("LittleFS Mount Failed");
    return;
  }

  // WiFi Setup
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.print("Connected! IP: ");
  Serial.println(WiFi.localIP());
  startTime = millis();

  // Initialize grip arrays
  for (int i = 0; i < numReadings; i++) {
    readings1[i] = readings2[i] = readings3[i] = 0;
  }

  // ---------------------------
  // Web Server Endpoints
  // ---------------------------
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request) {
    String response = readTotalAccel() + "," + readGrip();
    request->send(200, "text/plain", response);
  });

  server.on("/state", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("set")) {
      currentState = request->getParam("set")->value();
      Serial.println("State changed to: " + currentState);
    }
    request->send(200, "text/plain", currentState);
  });

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  server.begin();
  Serial.println("Server started!");

  prevTime = millis();
}

// ---------------------------
// LOOP
// ---------------------------
void loop() {
  // --- Button Toggle Logic ---
  int reading = digitalRead(BTN_PIN);
if (reading != lastButtonState) {
  lastDebounceTime = millis();
}

if ((millis() - lastDebounceTime) > debounceDelay) {
  static bool buttonState = HIGH; // remember last stable state

  if (reading != buttonState) {
    buttonState = reading;
    if (buttonState == LOW) { // Button pressed (active low)
      if (currentState == "STOP") currentState = "START";
      else currentState = "STOP";
      Serial.println("Button toggled: " + currentState);
    }
  }
}

lastButtonState = reading;


  // --- Sensor Readings ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accelMag = sqrt(a.acceleration.x * a.acceleration.x +
                        a.acceleration.y * a.acceleration.y +
                        a.acceleration.z * a.acceleration.z) - 10.8;

  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  float accelAngleX = atan2(a.acceleration.y,
                            sqrt(a.acceleration.x * a.acceleration.x +
                                 a.acceleration.z * a.acceleration.z)) * (180.0 / PI);
  angleX = alpha * (angleX + g.gyro.x * dt) + (1 - alpha) * accelAngleX;

  // --- Grip Sensor Smoothing ---
  total1 -= readings1[readIndex];
  total2 -= readings2[readIndex];
  total3 -= readings3[readIndex];

  readings1[readIndex] = analogRead(sensor1Pin);
  readings2[readIndex] = analogRead(sensor2Pin);
  readings3[readIndex] = analogRead(sensor3Pin);

  total1 += readings1[readIndex];
  total2 += readings2[readIndex];
  total3 += readings3[readIndex];
  readIndex = (readIndex + 1) % numReadings;

  average1 = total1 / numReadings;
  average2 = total2 / numReadings;
  average3 = total3 / numReadings;
  overallAverage = (average1 + average2 + average3) / 3;

  // --- OLED Display ---
  u8g2.clearBuffer();

  if (millis() - startTime <= displayDuration) {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(5, 20, "WiFi Connected");
    u8g2.drawStr(5, 40, "IP:");
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(30, 40, WiFi.localIP().toString().c_str());
  } else {
    u8g2.drawLine(64, 0, 64, 64);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(5, 10, "Bat Angle");
    char angleStr[10];
    sprintf(angleStr, "%.1f", angleX);
    u8g2.setFont(u8g2_font_fub14_tf);
    u8g2.drawStr(10, 35, angleStr);

    int barCenter = 32;
    float limitedAngle = constrain(angleX, -40, 40);
    int barLength = map((int)limitedAngle, -40, 40, -25, 25);
    u8g2.drawLine(barCenter, 55, barCenter + barLength, 55);
    u8g2.drawDisc(barCenter, 55, 2);

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(75, 10, "Strength");
    int gripVal = getGripLevelValue();
    int barHeight = map(gripVal, 0, 100, 0, 40);
    int barY = 60 - barHeight;
    u8g2.drawFrame(110, 20, 10, 40);
    u8g2.drawBox(111, barY, 8, barHeight);
    u8g2.drawStr(75, 55, readGrip().c_str());
  }

  u8g2.sendBuffer();

  // --- Debug Info ---
  Serial.print("State: ");
  Serial.print(currentState);
  Serial.print(" | Accel: ");
  Serial.print(readTotalAccel());
  Serial.print(" | Grip: ");
  Serial.println(readGrip());

  delay(20);
}
