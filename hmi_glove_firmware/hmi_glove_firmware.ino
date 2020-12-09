#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Firmata.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const char* deviceName = "hmi_right";
//const char* deviceName = "hmi_left";

bool hasHandshake = false;

unsigned long prevRecTime, prevSendTime, currentTime;
int sendIntervalMs = 30;
int disconnectionTimeoutMs = 2000;

void setup()
{
  Serial.begin(115200);
  Print("-------------- START -------------\n");

  SetupImu();
  SetupMotors();
  SetupBle();

  Print("Connect using Python app");

  StartAdvertisingBle();
}

void loop()
{
  currentTime = millis();

  if (Bluefruit.connected() && hasHandshake 
      && currentTime - prevSendTime > sendIntervalMs)
  {
    SendImuData();
    prevSendTime = currentTime;
  }

  bool isMsgRecieved = ProcessBleUartData();
  if (Bluefruit.connected() && isMsgRecieved)
  {
    prevRecTime = currentTime;
  }
  if (Bluefruit.connected() && hasHandshake && prevRecTime > 0 && 
      currentTime - prevRecTime > disconnectionTimeoutMs)
  {
    DisconnectDueTimeout();
  }

  //PrintParameter("Time", currentTime); 
}
