#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

Adafruit_MPU6050 mpu;

// Wi-Fi Credentials
const char* ssid = "Redmi c";
const char* password = "1234567890";

// Web Server
AsyncWebServer server(80);

// LED Pins
#define LED_D2 4
#define LED_D3 5

// Motion & Filter Variables
float angleX = 0;
float alpha = 0.90;
unsigned long prevTime = 0;
unsigned long stableStartTime = 0;
bool isStable = false;
bool ledState = false;

// Thresholds
#define STABLE_ACCEL_LOW   -1.5
#define STABLE_ACCEL_HIGH 2
#define STABLE_TIME_REQUIRED 1500  // ms
#define ANGLE_LOW  -5
#define ANGLE_HIGH  5
const unsigned long blinkInterval = 300;
unsigned long previousBlink = 0;

// --- Read Total Acceleration (for web) ---
String readTotalAccel() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float total = sqrt(pow(a.acceleration.x, 2) +
                     pow(a.acceleration.y, 2) +
                     pow(a.acceleration.z, 2));
  return String(total - 10.8, 3);
}

// =======================
//   SENSOR CODE SECTION
// =======================
const int numReadings = 10;

// Arrays for three sensors
int readings1[numReadings];
int readings2[numReadings];
int readings3[numReadings];

int readIndex = 0;
int total1 = 0, total2 = 0, total3 = 0;
int average1 = 0, average2 = 0, average3 = 0;
int overallAverage = 0; // <-- make this global so readGrip() can access latest value

// Define input pins for three sensors
int sensor1Pin = A0;
int sensor2Pin = A1;
int sensor3Pin = A2;

// =======================
//       Grip helper
// =======================
String readGrip() {
  // Use same thresholds as in loop prints so strings match
  if (overallAverage > 1000) {
    return String("TIGHT");
  } else if (overallAverage > 800) {
    return String("NORMAL");
  } else if (overallAverage > 450) {
    return String("Loose");
  } else {
    return String("Null");
  }
}

// =======================
//        SETUP
// =======================
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Initializing MPU6050...");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050. Check wiring!");
    while (1) delay(10);
  }
  Serial.println("MPU6050 connected!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  pinMode(LED_D2, OUTPUT);
  pinMode(LED_D3, OUTPUT);

  if (!LittleFS.begin()) {
    Serial.println("LittleFS Mount Failed");
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.print("Connected! IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });

  server.on("/accel", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", readTotalAccel());
  });

  // NEW: grip endpoint
  server.on("/grip", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", readGrip());
  });

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  server.begin();
  Serial.println("Server started!");

  prevTime = millis();

  // Initialize arrays with 0
  for (int i = 0; i < numReadings; i++) {
    readings1[i] = 0;
    readings2[i] = 0;
    readings3[i] = 0;
  }
}

// =======================
//         LOOP
// =======================
void loop() {
  // ----------------------------
  // MPU + LED + WEB SERVER PART
  // ----------------------------
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accelMag = sqrt(
    a.acceleration.x * a.acceleration.x +
    a.acceleration.y * a.acceleration.y +
    a.acceleration.z * a.acceleration.z
  ) - 10.8;

  if (accelMag >= STABLE_ACCEL_LOW && accelMag <= STABLE_ACCEL_HIGH) {
    if (stableStartTime == 0) stableStartTime = millis();
    if (millis() - stableStartTime >= STABLE_TIME_REQUIRED) {
      isStable = true;
    }
  } else {
    stableStartTime = 0;
    isStable = false;
  }

  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  float accelAngleX = atan2(a.acceleration.y,
                            sqrt(a.acceleration.x * a.acceleration.x +
                                 a.acceleration.z * a.acceleration.z)) * (180.0 / PI);

  angleX = alpha * (angleX + g.gyro.x * dt) + (1 - alpha) * accelAngleX;

  if (isStable) {
    if (angleX >= ANGLE_LOW && angleX <= ANGLE_HIGH) {
      digitalWrite(LED_D3, HIGH);
      digitalWrite(LED_D2, LOW);
    } 
    else if (angleX < ANGLE_LOW) {
      if (millis() - previousBlink >= blinkInterval) {
        previousBlink = millis();
        ledState = !ledState;
        digitalWrite(LED_D2, ledState);
      }
      digitalWrite(LED_D3, LOW);
    } 
    else if (angleX > ANGLE_HIGH) {
      digitalWrite(LED_D2, HIGH);
      digitalWrite(LED_D3, LOW);
    }
    Serial.print("Mode: BAT ANGLE | ");
  } 
  else {
    digitalWrite(LED_D2, LOW);
    digitalWrite(LED_D3, LOW);
    Serial.print("Mode: SWING | ");
  }

  Serial.print("AccelMag: ");
  Serial.print(accelMag, 2);
  Serial.print(" m/s² | Stable: ");
  Serial.print(isStable ? "YES" : "NO");
  Serial.print(" | AngleX: ");
  Serial.print(angleX, 2);
  Serial.print("° | ");

  if (isStable) {
    if (angleX >= ANGLE_LOW && angleX <= ANGLE_HIGH)
      Serial.println("CENTER");
    else if (angleX < ANGLE_LOW)
      Serial.println("LEFT");
    else if (angleX > ANGLE_HIGH)
      Serial.println("RIGHT");
  } else {
    Serial.println("SWINGING");
  }

  // ----------------------------
  // SENSOR SMOOTHING PART
  // ----------------------------
  total1 -= readings1[readIndex];
  total2 -= readings2[readIndex];
  total3 -= readings3[readIndex];

  readings1[readIndex] = analogRead(sensor1Pin);
  readings2[readIndex] = analogRead(sensor2Pin);
  readings3[readIndex] = analogRead(sensor3Pin);

  total1 += readings1[readIndex];
  total2 += readings2[readIndex];
  total3 += readings3[readIndex];

  readIndex++;
  if (readIndex >= numReadings) readIndex = 0;

  average1 = total1 / numReadings;
  average2 = total2 / numReadings;
  average3 = total3 / numReadings;

  overallAverage = (average1 + average2 + average3) / 3; // <-- update global variable

  Serial.print("Sensor Average: ");
  Serial.println(overallAverage);

  if (overallAverage > 1000) {
    Serial.println("TIGHT");
  } else if (overallAverage > 800) {
    Serial.println("NORMAL");
  } else if (overallAverage > 450) {
    Serial.println("Loose");
  } else {
    Serial.println("Null");
  }

  delay(200);
}