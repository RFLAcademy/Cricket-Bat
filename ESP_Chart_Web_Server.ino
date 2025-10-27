#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Replace with your Wi-Fi details
const char* ssid = "Redmi c";     // Replace with your phone hotspot name
const char* password = "1234567890";

// Web Server on port 80
AsyncWebServer server(80);

String readAccelX() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return String(a.acceleration.x);
}

String readAccelY() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return String(a.acceleration.y);
}

String readAccelZ() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return String(a.acceleration.z);
}

void setup() {
  Serial.begin(115200);

  if(!mpu.begin()){
    Serial.println("Failed to find MPU6050. Check wiring!");
    while(1);
  }

  // Initialize FileSystem
  if(!LittleFS.begin()){
    Serial.println("LittleFS Mount Failed");
    return;
  }

  // Wi-Fi Connection
 WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while(WiFi.status() != WL_CONNECTED){
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println(WiFi.localIP());

  // Web Routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest
   *request){
    request->send(LittleFS, "/index.html");
  });
  server.on("/ax", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", readAccelX());
  });
  server.on("/ay", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", readAccelY());
  });
  server.on("/az", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", readAccelZ());
  });
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");


  server.begin();
}

void loop(){
}
