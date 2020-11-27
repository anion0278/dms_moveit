#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Firmata.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const char* deviceName = "hmi-glove-right";
//const char* deviceName = "hmi-glove-left";

BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery


int ledPin = 17;
float ledSensitivity = 3.5f;
bool isBleLedActive = true;
int vibrationMinSpeed = 50;
uint8_t buf[64];
short digits = 3;

const byte num_motors = 6;
// motors                     X, Y, Z, -X, -Y, -Z
int motorPins[num_motors] = {PIN_A3,PIN_A5,PIN_A1,PIN_A4,PIN_A2,PIN_A0};
// TODO if pins are different in left and right versions -> make simple if(deviceName contains "left"/ right)

int motorVals[num_motors];
int stop_m[num_motors] = {0,0,0,0,0,0};

unsigned long prevRecTime, prevSendTime, currentTime;
int disconnectionTimeoutMs = 500;
int sendIntervalMs = 65;

Adafruit_BNO055 bno;


void SetupBle()
{
  Bluefruit.autoConnLed(isBleLedActive);
  
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName(deviceName);

  bledfu.begin();// To be consistent OTA DFU should be added first if it exists

  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  Serial.println("BLE OK");
}

void SetupMotors()
{
  for (byte i = 0; i < num_motors; i++) 
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

void SetAllMotorsSpeed(int speed_comps[])
{
  analogWrite(ledPin, GetMax(speed_comps));

  for (byte i = 0; i < num_motors; i++) 
  {
    analogWrite(motorPins[i], speed_comps[i]);
  }
}

byte GetMax(int speed_comps[])
{
  byte maxv=0;
  for (byte i=0; i < num_motors; i++)
  {
    if (speed_comps[i] > maxv) 
    {
       maxv = speed_comps[i];
    }
  }
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
void ParseSpeedValues(String message, int fill_array[num_motors])
{
  for (byte i = 0; i < num_motors; i++)  
  {
    byte start = i * 3 + i + 1;
    if (i > 2)
    {
      start = i * 4 + i - 1;
    }
    fill_array[i] = ParseValue(message, start , 3); 
  }
}


int ParseValue(String message, byte pos_byte, byte num_length)
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
  bool flag = false;
  String inputString = "";
  int len = 0;
  while ( bleuart.available() )
  {
    len++;
    char inChar = (char) bleuart.read();
    inputString += inChar;
    if (inChar == '\n')
    {
      if ( inputString.charAt(0) == 'X' )
      {
        flag = true; // got a msg
        if (inputString == "CALIB")
        {
          SaveCalibration();
        }
        else 
        {
          ParseSpeedValues(inputString, motorVals);
          SetAllMotorsSpeed(motorVals);
          Serial.print("Speed changed: ");
          for (byte i = 0; i< num_motors; i = i +1)
          {
            Serial.print(motorVals[i]);
            Serial.print("; ");
          }
          Serial.println();
        }
      }
      inputString = "";
    }
  }
  return flag;
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
  sensors_event_t event;
  bno.getEvent(&event);

  //ShowOrientation(event);

  imu::Quaternion quat = bno.getQuat();

  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  String msg = "Q[x"+String(quat.x(),digits)+"; y"+String(quat.y(),digits)+"; z"+String(quat.z(),digits)+"; w"+String(quat.w(),digits)+"]";
  msg = msg + "-C[s"+sys+"; g"+gyro+"; a"+accel+"; m"+mag+"]";

  SendString(msg);
}

void ShowEuler(sensors_event_t event)
{
  Serial.print(F("Orientation: "));
  Serial.print(360 - (float)event.orientation.x);
  Serial.print(F(", "));
  Serial.print((float)event.orientation.y);
  Serial.print(F(", "));
  Serial.print((float)event.orientation.z);
  Serial.println(F("")); 
}

void SendString(String str)
{
  str = str + '\n';
  str.getBytes(buf, str.length());
  bleuart.write(buf, str.length());
}

void RestoreImuCalibration()
{
    int eeAddress = 0;
    long bnoID;
    bool foundCalib = false;

    //EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;
    
    bno.getSensor(&sensor);
    if (bnoID != sensor.sensor_id)
    {
        Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
    }
    else
    {
        Serial.println("\nFound Calibration for this sensor in EEPROM.");
        eeAddress += sizeof(long);
        //EEPROM.get(eeAddress, calibrationData);

        DisplayImuOffsets(calibrationData);

        Serial.println("\n\nRestoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calibrationData);

        Serial.println("\n\nCalibration data loaded into BNO055");
        foundCalib = true;
    }
 }

void SaveCalibration()
{
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    DisplayImuOffsets(newCalib);

    Serial.println("\n\nStoring calibration data to EEPROM...");

    int eeAddress = 0;
    //bno.getSensor(&sensor);
    //bnoID = sensor.sensor_id;

    //EEPROM.put(eeAddress, bnoID);

    eeAddress += sizeof(long);
    //EEPROM.put(eeAddress, newCalib);
    Serial.println("Data stored to EEPROM.");
}

void DisplayImuOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
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
  
  if (currentTime - prevSendTime > sendIntervalMs)
  {
    SendImuData();
    prevSendTime = currentTime;
  }

  bool isMsgRecieved = ProcessBleUartData();
  if (Bluefruit.connected() && isMsgRecieved)
  {
    prevRecTime = currentTime;
  }
  if (Bluefruit.connected() && prevRecTime > 0 && currentTime - prevRecTime > disconnectionTimeoutMs)
  {
    Serial.println("Disconnected due to inactivity on the PC side");
    StopMotors();
    Bluefruit.disconnect(0);
    prevRecTime = 0;
  }

// clientUart.discovered() pouzit???

//  if(Bluefruit.connected() && prevRecTime > 0)
//    Serial.println(millis() - prevRecTime);
//  else
//    Serial.println("Not connected yet");
}
