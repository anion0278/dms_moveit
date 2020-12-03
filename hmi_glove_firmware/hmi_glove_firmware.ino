#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Firmata.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//const char* deviceName = "hmi-glove-right";
const char* deviceName = "hmi-glove-left";

BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

int ledPin = 17;
float ledSensitivity = 3.5f;
bool isBleLedActive = true;
int vibrationMinSpeed = 50;
const byte BUF_LENGTH = 192;
uint8_t buf[BUF_LENGTH]; 
short digits = 2;

const byte NUM_MOTORS = 6;
int motorPins[NUM_MOTORS];

unsigned long prevRecTime, prevSendTime, currentTime;
int disconnectionTimeoutMs = 1000;
int sendIntervalMs = 30;

const byte NUM_OFFSETS = NUM_BNO055_OFFSET_REGISTERS; 
uint8_t offsets [NUM_OFFSETS]; 

Adafruit_BNO055 bno;

bool hasHandshake = false;
bool canRecalibrate = false;

String offsetsRequest = "request-offsets";
String offsetsUpdate = "offsets:";

void SetupBle()
{
  Bluefruit.autoConnLed(isBleLedActive);
  
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName(deviceName);

  bledfu.begin();// To be consistent OTA DFU should be added first if it exists

  bledis.setManufacturer("VSB-TUO 354");
  bledis.setModel(deviceName);
  bledis.begin();

  bleuart.begin();

  blebas.begin();   // Start BLE Battery Service
  blebas.write(100);

  Serial.println("BLE OK");
}

void SetupMotors()
{
  if (String(deviceName).indexOf("right") != -1)
  {
    // motors X, Y, Z, -X, -Y, -Z
    int motorPins[NUM_MOTORS] = {PIN_A3,PIN_A5,PIN_A1,PIN_A4,PIN_A2,PIN_A0};
  }
  if (String(deviceName).indexOf("left") != -1)
  {
    // motors X, Y, Z, -X, -Y, -Z
    int motorPins[NUM_MOTORS] = {PIN_A3,PIN_A5,PIN_A1,PIN_A4,PIN_A2,PIN_A0};
  }
  // exception on incorrect name is by design
  for (byte i = 0; i < NUM_MOTORS; i++) 
  {
    SetupPwmPin(motorPins[i]);
  }
  StopMotors();
  Serial.println("Motors OK");
}

void StopMotors()
{
  int stop_m[NUM_MOTORS] = {0,0,0,0,0,0};
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
      if (inputString.charAt(0) == 'X' || inputString.startsWith(offsetsRequest) 
        || inputString.startsWith(offsetsUpdate))
      {        
        isMsgRecieved = true; 
        if (inputString.charAt(0) == 'X')
        {
          ProcesMotorData(inputString);
        }
        if (inputString.startsWith(offsetsRequest))
        {  
          ProcessImuOffsetsRequest(inputString);
        }
        if (inputString.startsWith(offsetsUpdate))
        { 
          ProcessOffsetsUpdate(inputString);
        }
      }
      inputString = "";
    }
  }
  return isMsgRecieved;
}

//////////////////////////////////////////////////////////////////////// IMU

void ProcesMotorData(String inputString)
{
  int motorVals[NUM_MOTORS];
  ParseSpeedValues(inputString, motorVals);
  SetAllMotorsSpeed(motorVals);
  Serial.print("Speed changed: ");
  for (byte i = 0; i< NUM_MOTORS; i = i +1) // TODO into method
  {
    Serial.print(motorVals[i]);
    Serial.print("; ");
  }
  Serial.println();
}

void ProcessImuOffsetsRequest(String inputString)
{
  String msg = FormatCalibrationStatus() + "-" + FormatOffsets();
  SendString(msg);
  Serial.println(msg);
  delay(10); // 2x message packets!
}

void ProcessOffsetsUpdate(String inputString)
{
  inputString.replace(offsetsUpdate, "");
  for (byte i = 0; i < NUM_OFFSETS; i++)
  {
    offsets[i] = (int8_t)inputString[i];
    Serial.print(offsets[i]);
    Serial.print("; ");
  }
  bno.setSensorOffsets(offsets);
  Serial.println("Restored offsets succesfully");
}


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
  msg = msg + FormatCalibrationStatus();
  
  SendString(msg);
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

String FormatCalibrationStatus()
{
  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  return String(sys) + String(gyro) + String(accel) + String(mag);
}


String FormatCalibration()
{
    adafruit_bno055_offsets_t currentCalib;
    bno.getSensorOffsets(currentCalib);
}

String FormatOffsets()
{
  bno.getSensorOffsets(offsets);
  String offsetMsg = "";
  for (byte i=0; i < NUM_OFFSETS; i++)
  {
    offsetMsg += (char)offsets[i];
  }
  Serial.println(offsetMsg);
  return offsetMsg;
}

void setup()
{
  Serial.begin(115200);
  Serial.println("-------------- START -------------\n");

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
}
