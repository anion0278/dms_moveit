
Adafruit_BNO055 bno;
const byte NUM_OFFSETS = NUM_BNO055_OFFSET_REGISTERS; 
uint8_t offsets [NUM_OFFSETS]; 

byte digits = 2;

void SetupImu()
{
  bno = Adafruit_BNO055(55, 0x28);
  if(!bno.begin())
  {
    FatalError("No IMU detected!");
  }
  bno.setExtCrystalUse(true);  
  Print("IMU OK");
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

void SetImuCalibration(const adafruit_bno055_offsets_t &calibData)
{
  bno.setSensorOffsets(calibData);
  Print("Offsets were set");
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
  PrintParameter("Offsets msg", offsetMsg);
  return offsetMsg;
}

void ProcessImuOffsetsRequest(String inputString)
{
  String msg = FormatCalibrationStatus() + "-" + FormatOffsets();
  SendString(msg);
  delay(10); // 2x message packets!
}

void ProcessOffsetsUpdate(String inputString)
{
  inputString.replace(offsetsUpdate, "");
  for (byte i = 0; i < NUM_OFFSETS; i++)
  {
    offsets[i] = (uint8_t)inputString[i];
  }
  bno.setSensorOffsets(offsets);
  PrintArray("Restored offsets succesfully", offsets, NUM_OFFSETS);
}
