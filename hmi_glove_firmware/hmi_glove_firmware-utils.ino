int ParseInt(String &message, byte pos_byte, byte num_length)
{
  char chars[num_length];
  for (int i = 0; i < num_length; i++)
  {
    chars[i] = message.charAt(pos_byte + i);
  }
  return atoi(chars);
}

byte GetMax(int *vals, byte length)
{
  byte maxv=0;
  for (byte i=0; i < length; i++)
    if (vals[i] > maxv) 
       maxv = vals[i];
  return maxv;
}

void Print(char *str)
{
  Serial.println(str);
}

void Print(String str)
{
  Serial.println(str);
}

void PrintParameter(char *paramName, int value)
{
  Serial.print(paramName); Serial.print(": "); Serial.println(value);
}

void PrintParameter(char *paramName, String value)
{
  Serial.print(paramName); Serial.print(": "); Serial.println(value);
}

void PrintArray(char *array_name, int * array_vals, byte length)
{
  Serial.print(array_name); Serial.print(": "); 
  for (byte i = 0; i< length; i = i +1) 
  {
    Serial.print(array_vals[i]);
    Serial.print("; ");
  }
  Serial.println();
}

void PrintArray(char *array_name, uint8_t * array_vals, byte length)
{
  Serial.print(array_name); Serial.print(": "); 
  for (byte i = 0; i< length; i = i +1) 
  {
    Serial.print(array_vals[i]);
    Serial.print("; ");
  }
  Serial.println();
}

void FatalError(char *str)
{
  Print(str);
  while(1);
}
