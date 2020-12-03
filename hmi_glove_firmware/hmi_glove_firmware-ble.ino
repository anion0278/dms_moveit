
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

bool canRecalibrate = false;

String offsetsRequest = "request-offsets";
String offsetsUpdate = "offsets:";

const byte BUF_LENGTH = 192;
uint8_t buf[BUF_LENGTH]; 

bool isBleLedActive = true;

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

  Print("BLE OK");
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
  Print("Waiting to connect...");
}

void SendString(String &str)
{
  // TODO write can send even char[]
  str = str + "\n";
  int strLen = str.length();
  if (strLen > BUF_LENGTH)
  {
    FatalError("Message cannot fit into the buffer, extend the buffer!");
  }
  str.getBytes(buf, strLen);
  bleuart.write(buf, strLen);
  
  delay(5);
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
      Print(inputString);
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

void DisconnectDueTimeout()
{
  Print("Disconnected due to inactivity on the PC side");
  PrintParameter("Previous msg time", prevRecTime);
  PrintParameter("Current time", currentTime);
  StopMotors();
  Bluefruit.disconnect(0);
  prevRecTime = 0;
  hasHandshake = false;
}
