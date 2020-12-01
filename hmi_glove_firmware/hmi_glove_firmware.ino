#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Firmata.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

using namespace Adafruit_LittleFS_Namespace;

//const char* deviceName = "hmi-glove-right";
const char* deviceName = "hmi-glove-left";

BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

#define CALIBRATION_FILENAME "/offsets.txt"
File file(InternalFS);

int ledPin = 17;
float ledSensitivity = 3.5f;
bool isBleLedActive = true;
int vibrationMinSpeed = 50;
const byte BUF_LENGTH = 192;
uint8_t buf[BUF_LENGTH]; 
short digits = 2;

const byte NUM_MOTORS = 6;
// motors X, Y, Z, -X, -Y, -Z
int motorPins[NUM_MOTORS] = {PIN_A3,PIN_A5,PIN_A1,PIN_A4,PIN_A2,PIN_A0};
// TODO if pins are different in left and right versions -> make simple if(deviceName contains "left"/ right)

int motorVals[NUM_MOTORS];
int stop_m[NUM_MOTORS] = {0,0,0,0,0,0};

unsigned long prevRecTime, prevSendTime, currentTime;
int disconnectionTimeoutMs = 1000;
int sendIntervalMs = 30;

const byte NUM_OFFSETS = NUM_BNO055_OFFSET_REGISTERS; 
uint8_t offsets [NUM_OFFSETS]; // = {8,2,3,4,5,6,7,8,9,10,11,12,13,14,45,16,78,18,19,20,21,22};

Adafruit_BNO055 bno;

bool hasHandshake = false;
bool canRecalibrate = false;
String handshake = "handshake:";

void SetupBle()
{
  Bluefruit.autoConnLed(isBleLedActive);
  
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName(deviceName);

  bledfu.begin();// To be consistent OTA DFU should be added first if it exists

  //Bluefruit.Periph.setConnectCallback(connect_callback);

  bledis.setManufacturer("VSB-TUO 354");
  bledis.setModel(deviceName);
  bledis.begin();

  bleuart.begin();

  blebas.begin();   // Start BLE Battery Service
  blebas.write(100);

  Serial.println("BLE OK");
}

// CHECK IT OUT !!!!!!
//void connect_callback(uint16_t conn_handle)
//{
//  BLEConnection* conn = Bluefruit.Connection(conn_handle);
//  Serial.println("Connected");
//
//  // request PHY changed to 2MB
//  Serial.println("Request to change PHY");
//  conn->requestPHY();
//
//  // request to update data length
//  Serial.println("Request to change Data Length");
//  conn->requestDataLengthUpdate();
//    
//  // request mtu exchange
//  Serial.println("Request to change MTU");
//  conn->requestMtuExchange(247);
//
//  // request connection interval of 7.5 ms
//  //conn->requestConnectionParameter(6); // in unit of 1.25
//
//  // delay a bit for all the request to complete
//  delay(1000);
////}
//
//void disconnect_callback(uint16_t conn_handle, uint8_t reason)
//{
//  (void) conn_handle;
//  (void) reason;
//
//  Serial.println();
//  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
//}

void SetupMotors()
{
  for (byte i = 0; i < NUM_MOTORS; i++) 
  {
    SetupPwmPin(motorPins[i]);
  }
  StopMotors();
  Serial.println("Motors OK");
}

void StopMotors()
{
  SetAllMotorsSpeed(stop_m);
}

void SetupPwmPin(int pin)
{
  pinMode(pin, OUTPUT);
  Firmata.setPinMode(pin, PIN_MODE_PWM);
}

void SetAllMotorsSpeed(int *speedComps)
{
  analogWrite(ledPin, CalculateLedIntensity(GetMax(speedComps, NUM_MOTORS)));

  for (byte i = 0; i < NUM_MOTORS; i++) 
  {
    analogWrite(motorPins[i], speedComps[i]);
  }
}

byte GetMax(int *vals, byte length)
{
  byte maxv=0;
  for (byte i=0; i < length; i++)
    if (vals[i] > maxv) 
       maxv = vals[i];
  return maxv;
}

void StartAdvertisingBle(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(9999);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
  Serial.println("Waiting to connect...");
}

byte CalculateLedIntensity(int speed)
{
  return constrain(speed - vibrationMinSpeed, 0, 255);
}

// Arduino does not allow to return arrays :(
void ParseSpeedValues(String &message, int *fill_array)
{
  for (byte i = 0; i < NUM_MOTORS; i++)  
  {
    byte start = i * 3 + i + 1;
    if (i > 2)
    {
      start = i * 4 + i - 1;
    }
    fill_array[i] = ParseValue(message, start , 3); 
  }
}


int ParseValue(String &message, byte pos_byte, byte num_length)
{
  char speed_chars[num_length];
  for (int i = 0; i < num_length; i++)
  {
    speed_chars[i] = message.charAt(pos_byte + i);
  }
  return atoi(speed_chars);
}

bool ProcessBleUartData()
{
  bool isMsgRecieved = false;
  String inputString = "";
  int len = 0;
  while ( bleuart.available() )
  {
    len++;
    char inChar = (char) bleuart.read();
    inputString += inChar;
    if (inChar == '\n')
    {
      hasHandshake = true; // any msg, not only handshake
      if (inputString.charAt(0) == 'X' || inputString.startsWith(handshake))
      {        
        isMsgRecieved = true; 
        if (inputString.charAt(0) == 'X')
        {
          ParseSpeedValues(inputString, motorVals);
          SetAllMotorsSpeed(motorVals);
          Serial.print("Speed changed: ");
          for (byte i = 0; i< NUM_MOTORS; i = i +1)
          {
            Serial.print(motorVals[i]);
            Serial.print("; ");
          }
          Serial.println();
        }
        if (inputString.startsWith(handshake))
        {
          if (inputString[handshake.length() + 1] == 1)
            canRecalibrate = true;
          else
            canRecalibrate = false;
        }
      }
      inputString = "";
    }
  }
  return isMsgRecieved;
}

//////////////////////////////////////////////////////////////////////// IMU


void SetupImu()
{
  bno = Adafruit_BNO055(55, 0x28);
  if(!bno.begin())
  {
    Serial.println("No IMU detected!");
    while(true);
  }

  bno.setExtCrystalUse(true);  
  Serial.println("IMU OK");
}

void SendImuData()
{
  // BNO055 incorrectly calculates Euler angles
  // use only Quaternions!
  imu::Quaternion quat = bno.getQuat();

  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  // for BLE default max packet size is 20 bytes
  // so the packets will be divided 
  // packets over 20 bytes cause errors in long-term (some bug in Bluefruit Firmware)
  String msg = "Q"+String(quat.x(),digits)+String(quat.y(),digits)+String(quat.z(),digits)+String(quat.w(),digits);
  
  msg.replace(".", "");
  msg = msg + String(sys) + String(gyro) + String(accel) + String(mag);
  
//  
//  if (bno.isFullyCalibrated())
//  {
//    bno.getSensorOffsets(offsets);
//    //msg = msg + "-O["+FormatOffsets(offsets)+"]";
//  }
  
  SendString(msg);
}

bool GetStoredOffsets() 
{
  if(file.open(CALIBRATION_FILENAME, FILE_O_READ)) 
  {
    file.read(offsets, sizeof(offsets));
    Serial.print("Got data from file: ");
    for (byte i = i; i < sizeof(offsets); i++)
      {Serial.print(offsets[i]); Serial.print(";");}
    Serial.println(); 
    file.close();
    return true;
  }
  return false;
}

void StoreOffsets() 
{
  if (file.open(CALIBRATION_FILENAME, FILE_O_WRITE)) 
  {
    InternalFS.format();
    file.write(offsets, sizeof(offsets));
    file.close();
    Serial.println("Saved offsets to file");    
    for (byte i = i; i < sizeof(offsets); i++)
      {Serial.print(offsets[i]); Serial.print(";");}
    Serial.println();
  }
}


void SendString(String &str)
{
  // TODO write can send even char[]
  str = str + "\n";
  int strLen = str.length();
  if (strLen > BUF_LENGTH)
  {
    Serial.println("Message cannot fit into the buffer, extend the buffer!");
    while(1);
  }
  str.getBytes(buf, strLen);
  bleuart.write(buf, strLen);
  
  delay(5);
}

void SetImuCalibration(const adafruit_bno055_offsets_t &calibData)
{
    bno.setSensorOffsets(calibData);
    Serial.println("\n\nCalibration data set");
 }

String FormatCalibration()
{
    adafruit_bno055_offsets_t currentCalib;
    bno.getSensorOffsets(currentCalib);
}

String FormatOffsets(const uint8_t *calibData)
{
  String offsetMsg = "";
  for (byte i=0; i < NUM_OFFSETS; i++)
  {
    offsetMsg += String(calibData[i]) + ';';
   // offsetMsg += (char)calibData[i] + ';';
  }
  Serial.println(offsetMsg);
  return offsetMsg;
}

void setup()
{
  Serial.begin(115200);
  Serial.println("-------------- START -------------\n");

  InternalFS.begin();
  
  //StoreOffsets();
  //GetStoredOffsets();
  
  SetupImu();
  SetupMotors();
  SetupPwmPin(ledPin);
  SetupBle();

  Serial.println("Connect using Python app");

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
    Serial.println("Disconnected due to inactivity on the PC side");
    Serial.print("Prev: "); Serial.print(prevRecTime); Serial.print("Current: "); Serial.println(currentTime);
    StopMotors();
    Bluefruit.disconnect(0);
    prevRecTime = 0;
    hasHandshake = false;
  }

 // Serial.print("Time:"); Serial.println(currentTime);


//  if(Bluefruit.connected() && prevRecTime > 0)
//    Serial.println(millis() - prevRecTime);
//  else
//    Serial.println("Not connected yet");
}
