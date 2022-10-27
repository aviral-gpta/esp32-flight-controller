#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>

const char* ssid = "Galaxy M52 5G";
const char* password = "euvq9457";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
// Create a WebSocket object
AsyncWebSocket ws("/ws");

IPAddress local_IP(192, 168, 98, 101);
IPAddress gateway(192, 168, 1, 1);

IPAddress subnet(255, 255, 0, 0);
// IPAddress primaryDNS(8, 8, 8, 8);   //optional
// IPAddress secondaryDNS(8, 8, 4, 4); //optional

String message = "";
float sliderValues[3][3] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
int max_thrust = 1000;

//Json Variable to Hold Slider Values
JSONVar sliderValuesJSON;

//Get Slider Values
String createJSONPacket(int ind){
  sliderValuesJSON["sliderValue1"] = String(sliderValues[ind][0]);
  sliderValuesJSON["sliderValue2"] = String(sliderValues[ind][1]);
  sliderValuesJSON["sliderValue3"] = String(sliderValues[ind][2]);
  sliderValuesJSON["thrustSliderValue"] = String(max_thrust);

  String jsonString = JSON.stringify(sliderValuesJSON);
  return jsonString;
}

// Initialize SPIFFS
void initFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  else{
   Serial.println("SPIFFS mounted successfully");
  }
}

// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }  
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.println(WiFi.localIP());
}

void notifyClients(String sliderValues) {
  ws.textAll(sliderValues);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  Serial.print("Start: ");
  Serial.println(millis());
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    message = (char*)data;
    Serial.println(message);
    if (message.indexOf("c") == 1 && message.indexOf("s") == 3) {
      int c = message.charAt(0) - '1';
      int s = message.charAt(2) - '1';
      Serial.println(String(c)+"/"+String(s));
      sliderValues[c][s] = message.substring(4).toFloat();
      Serial.println(createJSONPacket(c));
      notifyClients(createJSONPacket(c));
    }
    if (message.indexOf("t") == 1) {
      int c = message.charAt(0) - '1';
      max_thrust = message.substring(2).toInt();
      Serial.println(createJSONPacket(c));
      notifyClients(createJSONPacket(c));
    }
    if (message.indexOf("getValues") >= 0) {
      notifyClients(createJSONPacket(message.substring(9).toInt() - 1));
    }
  }
  Serial.print("End: ");
  Serial.println(millis());
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void setup() {
  Serial.begin(115200);
  initFS();
  initWiFi();

  initWebSocket();
  
  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });
  
  server.serveStatic("/", SPIFFS, "/");

  // Start server
  server.begin();

}

void loop() {
  ws.cleanupClients();
}
