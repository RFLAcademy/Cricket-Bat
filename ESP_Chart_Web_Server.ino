#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

Adafruit_MPU6050 mpu;

// Replace with your Wi-Fi details
const char* ssid = "Redmi c";     // Replace with your phone hotspot name
const char* password = "1234567890";

AsyncWebServer server(80);

String readTotalAccel() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float total = sqrt(pow(a.acceleration.x, 2) +
                     pow(a.acceleration.y, 2) +
                     pow(a.acceleration.z, 2));
  return String((total-10.7), 3); // round to 3 decimal places
}

void setup() {
  Serial.begin(115200);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050. Check wiring!");
    while (1);
  }

  // Initialize FileSystem
  if (!LittleFS.begin()) {
    Serial.println("LittleFS Mount Failed");
    return;
  }

  // Wi-Fi Connection
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.print("Connected! IP Address: ");
  Serial.println(WiFi.localIP());

  // Serve HTML page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });

  // Serve total acceleration
  server.on("/accel", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", readTotalAccel());
  });

  // CORS for browser
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");

  server.begin();
  Serial.println("Server started!");
}

void loop() {}
