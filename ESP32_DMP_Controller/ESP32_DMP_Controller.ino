#include <ESP32Servo.h>       //for motor
#include "I2Cdev.h"      //for IMU
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include <Arduino.h>


// Replace with your network credentials
const char* ssid = "Galaxy M52 5G";
const char* password = "euvq9457";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
// Create a WebSocket object

// Set your Static IP address
IPAddress local_IP(192, 168, 1, 184);
// Set your Gateway IP address
IPAddress gateway(192, 168, 1, 1);

IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional

AsyncWebSocket ws("/ws");

String message = "";
float sliderValues[3][3] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
int thrust = 1000;

//Json Variable to Hold Slider Values
JSONVar sliderValuesJSON;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//ESC parameters
Servo motor1;
Servo motor2; 
Servo motor3;
Servo motor4;

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN1 15
#define MOTOR_PIN2 2
#define MOTOR_PIN3 4
#define MOTOR_PIN4 5

// int thrust;   //take this value as input from radio
float P1, P2, P3, P4;

float K_P[3] = {3, 3, 1};
float K_I[3] = {0.0, 0.0, 0.0};
int sat = 1600;

int err[3];

MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

String getSliderValues(int ind){
  sliderValuesJSON["sliderValue1"] = String(sliderValues[ind][0]);
  sliderValuesJSON["sliderValue2"] = String(sliderValues[ind][1]);
  sliderValuesJSON["sliderValue3"] = String(sliderValues[ind][2]);
  sliderValuesJSON["thrustSliderValue"] = String(thrust);

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
//  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
//    Serial.println("STA Failed to configure");
//  }  
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

void notifyClients(String sliderValues) {
  ws.textAll(sliderValues);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  // Serial.println(millis());
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
      Serial.println(getSliderValues(c));
      notifyClients(getSliderValues(c));
    }
    if (message.indexOf("t") == 1) {
      int c = message.charAt(0) - '1';
      thrust = message.substring(2).toInt();
      Serial.println(getSliderValues(c));
      notifyClients(getSliderValues(c));
      updateMotorSpeed(thrust);
    }
    if (message.indexOf("getValues") >= 0) {
      notifyClients(getSliderValues(message.substring(9).toInt() - 1));
    }
  }
  // Serial.println(millis());
  
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

void updateMotorSpeed(int thrust) {
      P1 = 1000 + thrust/4;
      P2 = 1000 + thrust/4;
      P3 = 1000 + thrust/4;
      P4 = 1000 + thrust/4;
      motor1.writeMicroseconds(P1);
      motor2.writeMicroseconds(P2);
      motor3.writeMicroseconds(P3);
      motor4.writeMicroseconds(P4);
      err[0] = 0; err[1] = 0; err[2] = 0; 
}

void setup() {
  Serial.begin(115200); // initialize serial communication
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

  motor1.attach(MOTOR_PIN1);
  motor2.attach(MOTOR_PIN2);
  motor3.attach(MOTOR_PIN3);
  motor4.attach(MOTOR_PIN4);
  
  motor1.writeMicroseconds(MIN_SIGNAL);
  motor2.writeMicroseconds(MIN_SIGNAL);
  motor3.writeMicroseconds(MIN_SIGNAL);
  motor4.writeMicroseconds(MIN_SIGNAL);

  Serial.println("IMU initialisation...Hold steady");
  delay(1000);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
 
  mpu.initialize();  // initialize device

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }

  thrust = 0;
  updateMotorSpeed(thrust);
}

void loop() {
  Serial.print("Millis init: ");
  Serial.println(millis());
  // delay(200);

  // if (Serial.available()) {
  //   int tmp = Serial.parseInt();
  //   Serial.parseInt();
  //   {
  //     if(tmp >= 60000) {
  //       tmp = tmp % 10000;
  //       sliderValues[2][1] = tmp/10;
  //     }
  //     else if(tmp >= 50000) {
  //       tmp = tmp % 10000;
  //       sliderValues[1][1] = tmp/10;
  //     }
  //     else if(tmp >= 40000) {
  //       tmp = tmp % 10000;
  //       sliderValues[0][1] = tmp/10;
  //     }
  //     else if(tmp >= 30000) {
  //       tmp = tmp % 10000;
  //       sliderValues[2][0] = tmp/10;
  //     }
  //     else if(tmp >= 20000) {
  //       tmp = tmp % 10000;
  //       sliderValues[1][0] = tmp/10;
  //     }
  //     else if(tmp >= 10000) {
  //       tmp = tmp % 10000;
  //       sliderValues[0][0] = tmp/10;
  //     }
  //     else {
  //       if(tmp>2000) tmp=2000;
  //       thrust = tmp;
  //       updateMotorSpeed(thrust);
  //     }
  //   }
  // }
   // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI); 
    }

    err[0] += int(ypr[0] * 180/M_PI);
    err[1] += int(ypr[1] * 180/M_PI);
    err[2] += int(ypr[2] * 180/M_PI);

    if (err[0] > 400)
      err[0] = 400;
    if (err[0] < -400)
      err[0] = -400;
    if (err[0] > 400)
      err[1] = 400;
    if (err[1] < -400)
      err[1] = -400;
    if (err[2] > 400)
      err[2] = 400;
    if (err[2] < -400)
      err[2] = -400;

    int delta[3];

    delta[0] = int(ypr[0] * 180/M_PI) * sliderValues[0][0] + err[0] * sliderValues[0][1];
    delta[1] = int(ypr[1] * 180/M_PI) * sliderValues[1][0] + err[1] * sliderValues[1][1];
    delta[2] = int(ypr[2] * 180/M_PI) * sliderValues[2][0] + err[2] * sliderValues[2][1];
    // delta[0] = 0;
    // delta[1] = 0;
    // delta[2] = 0;

    P1 = 1000 + thrust/4 - delta[0] + delta[1] - delta[2];
    if (P1 > sat)
      P1 = sat;
    P2 = 1000 + thrust/4 + delta[0] - delta[1] - delta[2];
    if (P2 > sat)
      P2 = sat;
    P3 = 1000 + thrust/4 - delta[0] - delta[1] + delta[2];
    if (P3 > sat)
      P3 = sat;
    P4 = 1000 + thrust/4 + delta[0] + delta[1] + delta[2];
    if (P4 > sat)
      P4 = sat;

    motor1.writeMicroseconds(P1);
    motor2.writeMicroseconds(P2);
    motor3.writeMicroseconds(P3);
    motor4.writeMicroseconds(P4);

    Serial.print(delta[1]);
    Serial.print("\t");
    Serial.print(delta[2]);
    Serial.print("\t");
    Serial.print(P1);
    Serial.print("\t");
    Serial.print(P2); 
    Serial.print("\t");
    Serial.print(P3); 
    Serial.print("\t");
    Serial.println(P4); 
    Serial.println(""); 
  Serial.print("Millis end: ");
  Serial.println(millis());
  delay(10);
}
