#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include <ESP32Servo.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include <SPIMemory.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// SERVER PARAMS //

const char* ssid = "Galaxy M52 5G";
const char* password = "euvq9457";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

IPAddress local_IP(192, 168, 98, 101);
IPAddress gateway(192, 168, 1, 1);

IPAddress subnet(255, 255, 0, 0);

String message = "";
JSONVar sliderValuesJSON;

// FLASH PARAMS

SPIFlash flash(SS, &SPI);
uint32_t memAddr;

// TUNING PARAMS //

float control_coeff[3][3] = {4.00, 0, 0.10, 0, 0, 0, 0, 0, 0};
int min_thrust = 1000;
int max_thrust = 1500;
int err_sat = 10*3600*100;

// FLIGHT PARAMS

bool running = false;
int thrust;
float P1, P2, P3, P4;
int delta[3];
int err[3] = {0, 0, 0};
int cumm_err[3] = {0, 0, 0};

// ESC PARAMS //

#define MOTOR_PIN1 33
#define MOTOR_PIN2 25
#define MOTOR_PIN3 26
#define MOTOR_PIN4 27

Servo motor1;
Servo motor2; 
Servo motor3;
Servo motor4;

// IMU PARAMS

#define IMU_PWR 32
#define IMU_LED 2

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

// TIMING

unsigned long loop_duration;

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

String createJSONPacket(int ind){
  sliderValuesJSON["sliderKp"] = String(control_coeff[ind][0]);
  sliderValuesJSON["sliderKi"] = String(control_coeff[ind][1]);
  sliderValuesJSON["sliderKd"] = String(control_coeff[ind][2]);
  sliderValuesJSON["sliderMaxThrust"] = String(max_thrust);
  sliderValuesJSON["sliderThrust"] = String(thrust);
  sliderValuesJSON["running"] = String(running);

  String jsonString = JSON.stringify(sliderValuesJSON);
  // Serial.println(jsonString);
  return jsonString;
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  // Serial.println(millis());
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    message = (char*)data;
    // Serial.println(message);
    if (message.indexOf("stop") == 0) {
      running = false;
      thrust = 1000;
      setMotorSpeed();
      notifyClients(createJSONPacket(message.charAt(7) - '1'));
    }
    else if (message.indexOf("start") == 0) {
      running = true;
      thrust = 1000;
      setMotorSpeed();
      notifyClients(createJSONPacket(message.charAt(8) - '1'));
    }
    else if (message.indexOf("res") == 0) {
      digitalWrite(IMU_LED, LOW);
      mpu.setDMPEnabled(false);

      Serial.println("Initializing DMP.");
      mpu.dmpInitialize();

      Serial.println("Calibrating DMP.");
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();

      Serial.println("Enabling DMP.");
      mpu.setDMPEnabled(true);
      digitalWrite(IMU_LED, HIGH);

      running = false;
      thrust = 1000;
      setMotorSpeed();
      notifyClients(createJSONPacket(message.charAt(6) - '1'));
    }
    else if (message.indexOf("update") == 0) {
      int c = message.charAt(9) - '1';
      int s = message.charAt(12) - '1';
      float v = message.substring(15).toFloat();
      if (s <= 2)
        control_coeff[c][s] = v;
      else if (s == 3)
        max_thrust = v;
      else if (s == 4) {
        thrust = v;
        setMotorSpeed();
      }
      notifyClients(createJSONPacket(c));
    }
    else if (message.indexOf("getValues") == 0) {
      notifyClients(createJSONPacket(message.charAt(12) - '1'));
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

void setMotorSpeed() {
  if (thrust > max_thrust)
    thrust = max_thrust;
  P1 = thrust;
  P2 = thrust;
  P3 = thrust;
  P4 = thrust;
  if (running) {
    motor1.writeMicroseconds(P1);
    motor2.writeMicroseconds(P2);
    motor3.writeMicroseconds(P3);
    motor4.writeMicroseconds(P4);
  }
  err[0] = 0; err[1] = 0; err[2] = 0; 
  cumm_err[0] = 0; cumm_err[1] = 0; cumm_err[2] = 0; 
}

void updateMotorSpeed() {
  int new_err[3] = {int(ypr[0] * 180/M_PI), int(ypr[1] * 180/M_PI), int(ypr[2] * 180/M_PI)};

  cumm_err[0] += new_err[0];
  cumm_err[1] += new_err[1];
  cumm_err[2] += new_err[2];

  if (cumm_err[0] > err_sat || cumm_err[0] < -err_sat)
    cumm_err[0] = 0;
  if (cumm_err[1] > err_sat || cumm_err[1] < -err_sat)
    cumm_err[1] = 0;
  if (cumm_err[2] > err_sat || cumm_err[2] < -err_sat)
    cumm_err[2] = 0;

  delta[0] = new_err[0] * control_coeff[0][0] + cumm_err[0] * control_coeff[0][1] + (new_err[0] - err[0]) * control_coeff[0][2];
  delta[1] = new_err[1] * control_coeff[2][0] + cumm_err[1] * control_coeff[2][1] + (new_err[1] - err[1]) * control_coeff[2][2];
  delta[2] = new_err[2] * control_coeff[1][0] + cumm_err[2] * control_coeff[1][1] + (new_err[2] - err[2]) * control_coeff[1][2];
  // delta[0] = 0;
  // delta[1] = 0;
  // delta[2] = 0;

  err[0] = new_err[0];
  err[1] = new_err[1];
  err[2] = new_err[2];

  P1 = thrust - delta[0] - delta[2] - delta[1];
  P2 = thrust + delta[0] + delta[2] - delta[1];
  P3 = thrust - delta[0] + delta[2] + delta[1];
  P4 = thrust + delta[0] - delta[2] + delta[1];

  if (P1 > max_thrust)
    P1 = max_thrust;
  if (P1 < min_thrust)
    P1 = min_thrust;

  if (P2 > max_thrust)
    P2 = max_thrust;
  if (P2 < min_thrust)
    P2 = min_thrust;

  if (P3 > max_thrust)
    P3 = max_thrust;
  if (P3 < min_thrust)
    P3 = min_thrust;

  if (P4 > max_thrust)
    P4 = max_thrust;
  if (P4 < min_thrust)
    P4 = min_thrust;

  if (running) {
    motor1.writeMicroseconds(P1);
    motor2.writeMicroseconds(P2);
    motor3.writeMicroseconds(P3);
    motor4.writeMicroseconds(P4);
  }
}

void setup() {
  Serial.begin(115200); // Initialize serial communication

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

  // Start flash memory
  // flash.begin();

  // Serial.print(F("Total capacity: "));
  // Serial.println(flash.getCapacity());

  // Initiate motors
  motor1.attach(MOTOR_PIN1);
  motor2.attach(MOTOR_PIN2);
  motor3.attach(MOTOR_PIN3);
  motor4.attach(MOTOR_PIN4);
  
  thrust = min_thrust;
  setMotorSpeed();
  delay(1000);

  // Join I2C bus
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  pinMode(IMU_PWR, OUTPUT);
  pinMode(IMU_LED, OUTPUT);

  digitalWrite(IMU_PWR, HIGH);

  Serial.println("IMU initialisation. Hold steady.");
  mpu.initialize(); 

  Serial.println("Initializing DMP.");
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
      Serial.println("Calibrating DMP.");
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();

      Serial.println("Enabling DMP.");
      mpu.setDMPEnabled(true);
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
      digitalWrite(IMU_LED, HIGH);
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
}

void loop() {

  unsigned long loop_start = micros();

  // Serial.print("Millis init: ");
  // Serial.println(loop_start);

  // Get the Latest packet 
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetEuler(euler, &q);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI); 
  }

  updateMotorSpeed();

  Serial.print(P1);
  Serial.print("\t");
  Serial.print(P2); 
  Serial.print("\t");
  Serial.print(P3); 
  Serial.print("\t");
  Serial.println(P4); 
  Serial.println(""); 
  // Serial.print("Millis end: ");
  // Serial.println(millis());

  loop_duration = micros() - loop_start;
  Serial.println(loop_duration);
  if(loop_duration<9990) 
    _delay_us(9990 - loop_duration);

}
