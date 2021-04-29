
const byte NUM_MOTORS = 6;
int motorPins[NUM_MOTORS];
int motorPrevSpeed[NUM_MOTORS] = {0,0,0,0,0,0};

const int MOTORS_STARTUP_TIME_MS = 5;
const byte MOTOR_STARTUP_PWM = 150;

float ledSensitivity = 3.5f;
int ledPin = 17;
int vibrationMinSpeed = 50;


byte CalculateLedIntensity(int speed)
{
  return constrain(speed - vibrationMinSpeed, 0, 255);
}

void SetupMotors()
{
  SetupPwmPin(ledPin);
  
  if (String(deviceName).indexOf("right") != -1)
  {
    Print("Right Configuration");
    // it is not possible to re-assign array
    motorPins[0] = PIN_A5; // X
    motorPins[1] = PIN_A4; // Y
    motorPins[2] = PIN_A1; // Z
    motorPins[3] = PIN_A2; // -X
    motorPins[4] = PIN_A3; // -Y
    motorPins[5] = PIN_A0; // -Z
  }
  if (String(deviceName).indexOf("left") != -1)
  {
    Print("Left Configuration");
    motorPins[0] = PIN_A1; // X
    motorPins[1] = PIN_A5; // Y
    motorPins[2] = PIN_A3; // Z
    motorPins[3] = PIN_A4; // -X
    motorPins[4] = PIN_A0; // -Y
    motorPins[5] = PIN_A2; // -Z
  }
  
  // exception on incorrect name is intensional
  for (byte i = 0; i < NUM_MOTORS; i++) 
  {
    SetupPwmPin(motorPins[i]);
  }
  StopMotors();
  Print("Motors OK");
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

  bool isStartupNeeded = false; // TODO use decreasing manual timer array
  for (byte i = 0; i < NUM_MOTORS; i++) 
  {
    if (motorPrevSpeed[i] == 0 && speedComps[i] > 0)
    {
      analogWrite(motorPins[i], MOTOR_STARTUP_PWM);
      isStartupNeeded = true;
    }
  }
  if (isStartupNeeded)  
  {
    delay(MOTORS_STARTUP_TIME_MS);
  }
  
  for (byte i = 0; i < NUM_MOTORS; i++) 
  {
    analogWrite(motorPins[i], speedComps[i]);
    motorPrevSpeed[i] = speedComps[i];
    //PrintParameter("Pin:", motorPins[i]);
    //PrintParameter("Value:", speedComps[i]);
  }
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
    fill_array[i] = ParseInt(message, start , 3); 
  }
}

void ProcesMotorData(String inputString)
{
  int motorVals[NUM_MOTORS];
  ParseSpeedValues(inputString, motorVals);
  SetAllMotorsSpeed(motorVals);
  
  PrintArray("Speed changed", motorVals, NUM_MOTORS);
}
