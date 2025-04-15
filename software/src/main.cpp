#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "secrets.h"
#include <PID_v1.h>

#include <MPU6050_light.h>
MPU6050 mpu(Wire);

const unsigned int UDP_PORT = 1234;

const char *ssid = "drone-0001";
const char *password = "0987654321";

IPAddress local_ip(192, 168, 0, 1);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiUDP udp;
char incomingPacket[512];

bool buttonC, buttonZ = false;
int joyX, joyY, rollAngle, pitchAngle, _accX, _accY, _accZ = 0;

bool buttonCDebounce = false;
bool buttonZDebounce = false;

int MAX_MOTOR = 60;

int16_t accX, accY, accZ, gyroX, gyroY, gyroZ;
float pitch, roll, yaw;

// PID
double setpointRoll = 0, inputRoll, outputRoll;
double setpointPitch = 0, inputPitch, outputPitch;

double Kp = 1.3, Ki = 0.05, Kd = 0.2;  // Tune these!
PID pidRoll(&inputRoll, &outputRoll, &setpointRoll, Kp, Ki, Kd, DIRECT);
PID pidPitch(&inputPitch, &outputPitch, &setpointPitch, Kp, Ki, Kd, DIRECT);

float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
float accX_offset = 0, accY_offset = 0, accZ_offset = 0;

int baseThrottle = 0;

#define MOTOR1_PIN 26 // Front left
#define MOTOR2_PIN 25 // front right
#define MOTOR3_PIN 27 // rear right
#define MOTOR4_PIN 32 // rear left

#define PWM_FREQ 1000 
#define PWM_RES 8

#define M1_CH 0
#define M2_CH 1
#define M3_CH 2
#define M4_CH 3

void setupMotors()
{
  ledcSetup(M1_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR1_PIN, M1_CH);

  ledcSetup(M2_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR2_PIN, M2_CH);

  ledcSetup(M3_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR3_PIN, M3_CH);

  ledcSetup(M4_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR4_PIN, M4_CH);

  ledcWrite(M1_CH, 0);
  ledcWrite(M2_CH, 0);
  ledcWrite(M3_CH, 0);
  ledcWrite(M4_CH, 0);

}

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP(ssid, password);
  Serial.print("[+] AP Created with IP Gateway ");
  Serial.println(WiFi.softAPIP());

  udp.begin(UDP_PORT);

  delay(5000);

  byte status = mpu.begin();
  while(status!=0){ } // stop everything if could not connect to MPU6050
  mpu.calcOffsets(true,true); // gyro and accelero

  setupMotors();

  delay(10000);

  pidRoll.SetMode(AUTOMATIC);
  pidPitch.SetMode(AUTOMATIC);
}

void handleUDP();

void loop() {
  mpu.update();

  inputRoll = mpu.getAngleX();
  inputPitch = mpu.getAngleY();

  pidRoll.Compute();
  pidPitch.Compute();

  int motorFL = constrain(baseThrottle - outputRoll + outputPitch, 0, MAX_MOTOR);
  int motorFR = constrain(baseThrottle + outputRoll + outputPitch, 0, MAX_MOTOR);
  int motorBL = constrain(baseThrottle - outputRoll - outputPitch, 0, MAX_MOTOR);
  int motorBR = constrain(baseThrottle + outputRoll - outputPitch, 0, MAX_MOTOR);

  ledcWrite(M1_CH, motorFL);
  ledcWrite(M2_CH, motorFR);
  ledcWrite(M3_CH, motorBL);
  ledcWrite(M4_CH, motorBR);

  Serial.print("Roll: "); Serial.print(roll);
  Serial.print(" Pitch: "); Serial.print(pitch);
  Serial.print(" | PID Roll: "); Serial.print(outputRoll);
  Serial.print(" Pitch: "); Serial.println(outputPitch);

  delay(8);

  handleUDP();
}

void handleUDP()
{
  int packetSize = udp.parsePacket();
  if (packetSize)
  {
    memset(incomingPacket, 0, sizeof(incomingPacket));

    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0)
    {
      incomingPacket[len] = 0; // Null-terminate
    }

    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, incomingPacket);
    if (!error)
    {
      // parse JSON sended by client with nunchuck
      JsonObject data = doc["data"];
      if (data.containsKey("joyX"))
        joyX = data["joyX"].as<int>();
      if (data.containsKey("joyY"))
        joyY = data["joyY"].as<int>();
      if (data.containsKey("rollAngle"))
        rollAngle = data["rollAngle"].as<int>();
      if (data.containsKey("pitchAngle"))
        pitchAngle = data["pitchAngle"].as<int>();
      if (data.containsKey("accX"))
        _accX = data["accX"].as<int>();
      if (data.containsKey("accY"))
        _accY = data["accY"].as<int>();
      if (data.containsKey("accZ"))
        _accY = data["accZ"].as<int>();
      if (data.containsKey("buttonC"))
        buttonC = data["buttonC"].as<bool>();
      if (data.containsKey("buttonZ"))
        buttonZ = data["buttonZ"].as<bool>();

      setpointRoll = map(joyX, 0, 1023, -10, 10);
      setpointPitch = map(joyY, 0, 1023, -10, 10);

      if (buttonC) {
        if (baseThrottle < MAX_MOTOR && !buttonCDebounce)
        {
          baseThrottle += 1;
          buttonCDebounce = true;
        }
      } else {
        buttonCDebounce = false;
      }
      if (buttonZ) {
        if (baseThrottle > 0 && !buttonZDebounce)
        {
          baseThrottle -= 1;
          buttonZDebounce = true;
        }
      } else {
        buttonZDebounce = false;
      }
    }
    else
    {
      Serial.println("[UDP] JSON parse failed.");
    }
  }
}