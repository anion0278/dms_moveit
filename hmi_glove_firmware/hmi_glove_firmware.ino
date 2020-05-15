#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Firmata.h>

BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

String inputString = "";
String recieved = "recieved";
int currentSpeed = 0;

void setup()
{
  SetupMotors();
  SetAllMotorsSpeed(0);
  
  Serial.begin(115200);
  Serial.println("-------------- START -------------\n");

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("HMI Glove Left");
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  startAdv();

  Serial.println("Connect using Python app");
}

void SetupMotors()
{
  pinMode(PIN_A0, OUTPUT);
  pinMode(PIN_A1, OUTPUT);
  pinMode(PIN_A2, OUTPUT);
  pinMode(PIN_A3, OUTPUT);
  Firmata.setPinMode(PIN_A0, PIN_MODE_PWM);
  Firmata.setPinMode(PIN_A1, PIN_MODE_PWM);
  Firmata.setPinMode(PIN_A2, PIN_MODE_PWM);
  Firmata.setPinMode(PIN_A3, PIN_MODE_PWM);
}

void startAdv(void)
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
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(9999);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void loop()
{
  inputString = "";
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
        currentSpeed = ParseSpeed(inputString, 1);
        Serial.print("Speed changed: ");
        Serial.println(currentSpeed);
        SetAllMotorsSpeed(currentSpeed);
        //uint8_t buf[64];
        //recieved.getBytes(buf, 8);
        //bleuart.write( buf, 8 );
      }
      inputString = "";
    }
  }
}

void SetAllMotorsSpeed(int speed)
{
  analogWrite(PIN_A0, 255 - speed);
  analogWrite(PIN_A1, 255 - speed);
  analogWrite(PIN_A2, 255 - speed);
  analogWrite(PIN_A3, 255 - speed);
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
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  SetAllMotorsSpeed(0);
  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
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
}
