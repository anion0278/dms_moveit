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

//                                  I2C id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

//const char* deviceName = "hmi-glove-right";
const char* deviceName = "hmi-glove-left";

int ledPin = 17;
float ledSensitivity = 3.5f;
bool isBleLedActive = true;
int vibrationMinSpeed = 50;

uint8_t buf[64];
short digits = 3;

void setup()
{
  Serial.begin(115200);
  Serial.println("-------------- START -------------\n");
  
  SetupMotors();
  SetupPwmPin(ledPin);
  SetupBle();

  StartAdvertisingBle();

  ConnectIMU();

  Serial.println("Connect using Python app");
}

void ConnectIMU()
{
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("No BNO055 detected!");
    while(1);
  }
  delay(1000);
  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);  
  DisplaySensorDetails();
}

void DisplaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  //delay(500);
}

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
        int currentSpeed = ParseSpeed(inputString, 1);
        SetAllMotorsSpeed(currentSpeed);
        Serial.print("Speed changed: ");
        Serial.println(currentSpeed);
      
      }
      inputString = "";
    }
  }
}

void SendImuData()
{
  sensors_event_t event;
  bno.getEvent(&event);

  Serial.print(F("Orientation: "));
  Serial.print(360 - (float)event.orientation.x);
  Serial.print(F(", "));
  Serial.print((float)event.orientation.y);
  Serial.print(F(", "));
  Serial.print((float)event.orientation.z);
  Serial.println(F("")); 

  imu::Quaternion quat = bno.getQuat();

  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  String msg = "Q["+String(quat.w(),digits)+"; "+String(quat.x(),digits)+";"+String(quat.y(),digits)+";"+String(quat.z(),digits)+"]";
  msg = msg + "-C["+sys+"; "+gyro+";"+accel+";"+mag+"]";

  SendString(msg);
}

void SendString(String str)
{
  str = str + '\n';
  str.getBytes(buf, str.length());
  bleuart.write(buf, str.length());
}



void loop()
{
 
  ProcessBleUartData();
  SendImuData();
}
