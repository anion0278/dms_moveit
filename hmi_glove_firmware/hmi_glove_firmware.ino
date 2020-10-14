#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Firmata.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

//const char* deviceName = "hmi-glove-right";
const char* deviceName = "hmi-glove-left";

int ledPin = 17;
float ledSensitivity = 3.5f;
bool isBleLedActive = true;
int vibrationMinSpeed = 50;

uint8_t buf[64];
short digits = 3;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void SetupBle()
{
  Bluefruit.autoConnLed(isBleLedActive);
  
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName(deviceName);
  
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  
  // do not define this callback!
  // it causes timeout auto-disconnection after 30sec
  //Bluefruit.Periph.setConnectCallback(connect_callback); 

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
  SetupPwmPin(PIN_A0);
  SetupPwmPin(PIN_A1);
  SetupPwmPin(PIN_A2);
  SetupPwmPin(PIN_A3);
  
  SetAllMotorsSpeed(0);
}

void SetupPwmPin(int pin)
{
  pinMode(pin, OUTPUT);
  Firmata.setPinMode(pin, PIN_MODE_PWM);
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

void SetAllMotorsSpeed(int speed)
{
  // when speeds values will be different use MAX(vals...)
  analogWrite(ledPin, CalculateLedIntensity(speed));

  int pwmValue = 255 - speed;
  analogWrite(PIN_A0, pwmValue);
  analogWrite(PIN_A1, pwmValue);
  analogWrite(PIN_A2, pwmValue);
  analogWrite(PIN_A3, pwmValue);
}

byte CalculateLedIntensity(int speed)
{
  return constrain(speed - vibrationMinSpeed, 0, 255);
}

int ParseSpeed(String message, byte pos_byte)
{
  char speed_chars[3];
  for (int i = 0; i < 3; i++)
  {
    speed_chars[i] = message.charAt(pos_byte + i);
  }
  return atoi(speed_chars);
}

void connect_callback(uint16_t conn_handle)
{
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  SetAllMotorsSpeed(0);
  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to PC: ");
  Serial.println(central_name);
}

// @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  SetAllMotorsSpeed(0);

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); 
  Serial.println(reason, HEX);
  //Bluefruit.disconnect(conn_handle);
}

void ProcessBleUartData()
{
  String inputString = "";
  int len = 0;
  while ( bleuart.available() )
  {
    len++;
    char inChar = (char) bleuart.read();
    inputString += inChar;
    if (inChar == '\n')
    {
      if ( inputString.charAt(0) == 'C' )
      {
        if (inputString == "CALIB")
        {
          SaveCalibration();
        }
        else 
        {
          int currentSpeed = ParseSpeed(inputString, 1);
          SetAllMotorsSpeed(currentSpeed);
          Serial.print("Speed changed: ");
          Serial.println(currentSpeed);
        }
      }
      inputString = "";
    }
  }
}

//////////////////////////////////////////////////////////////////////// IMU


void SetupImu()
{
  if(!bno.begin())
  {
    Serial.println("No IMU detected!");
    while(1);
  }
  delay(1000);

  bno.setExtCrystalUse(true);  
  
  //RestoreImuCalibration();

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

  String msg = "Q["+String(quat.w(),digits)+"; "+String(quat.x(),digits)+"; "+String(quat.y(),digits)+"; "+String(quat.z(),digits)+"]";
  msg = msg + "-C["+sys+"; "+gyro+"; "+accel+"; "+mag+"]";

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
        delay(500);
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
 
  ProcessBleUartData();
  SendImuData();
}
