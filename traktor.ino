#include "Wire.h"
#include "Math.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <EEPROM.h>
#include <LiquidCrystal.h>

#define NUMBER_BUFFER_SIZE 8
#define PRESSURE_OFFSET 99 //teoretically 102
#define BUTTONS_PIN 7 //A7
#define PRESSURE_PIN 0 //A0
#define LED_PIN 6 //D6


MPU6050 accelerometr;
int MPUOffsets[6] = { -707, -3158, 952, 167, 54,  -258 }; //source: https://forum.arduino.cc/index.php?action=dlattach;topic=446713.0;attach=193816
LiquidCrystal lcd(12, 10, 5, 4, 3, 2);
#define ROWS 2
#define COLUMNS 16

int mayorMode = 0;
#define MAX_MAYOR_MODE 1

int zirafaMode = 0;
bool pressed = false;

float lastPressure = 0;
int lastXAngle = 0;
int lastYAngle = 0;

struct Eeprom
{
  int16_t xDegreeComp; //x
  int16_t yDegreeComp; //y
  int16_t pressureOffset; //o
  int16_t pressureDivider; //d
  int16_t angleSensitivity; //s
  int16_t pressureSensitivity; //S  
  int16_t maxValue0; //1
  int16_t maxValue1; //1
  int16_t maxValue2; //2
  int16_t maxValue3; //3
  int16_t maxValue4; //4
  int16_t maxValue5; //5
  int16_t maxValue6; //6
  int16_t maxValue7; //7
  int16_t maxValue8; //8
  int16_t limit;
} eeprom;

void fillAngles(int &xAngle, int &yAngle)
{
  const int maxCount = 100;
  
  int16_t ax, ay, az;
  long ax_p = 0;
  long ay_p = 0;
  long az_p = 0;
  float x, y, z;
  
  int count = 0;  
  while (count++ < maxCount)
  {
    accelerometr.getAcceleration(&ax, &ay, &az);
    ax_p = ax_p + ax;
    ay_p = ay_p + ay;
    az_p = az_p + az;
  }
    
  x = ax_p / maxCount;
  y = ay_p / maxCount;
  z = az_p / maxCount;

    
  xAngle = (int)(atan2(x, sqrt(square(y) + square(z)))/(M_PI/180)) + eeprom.xDegreeComp;
  yAngle = (int)(atan2(y, sqrt(square(x) + square(z)))/(M_PI/180)) + eeprom.yDegreeComp;  
}

void ResetEeprom()
{
  eeprom.xDegreeComp = 0;
  eeprom.yDegreeComp = 0;
  eeprom.pressureOffset = 0;
  eeprom.pressureDivider = 0;
  EEPROM.put(0, eeprom);
}

void setup() {
    Wire.begin();
    accelerometr.initialize();
    accelerometr.setXAccelOffset(MPUOffsets[0]);
    accelerometr.setYAccelOffset(MPUOffsets[1]);
    accelerometr.setZAccelOffset(MPUOffsets[2]);
    accelerometr.setXGyroOffset(MPUOffsets[3]);
    accelerometr.setYGyroOffset(MPUOffsets[4]);
    accelerometr.setZGyroOffset(MPUOffsets[5]);
      
    lcd.begin(COLUMNS, ROWS);

    EEPROM.get(0, eeprom);
    Serial.begin(115200);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB
    }

    pinMode(LED_PIN, OUTPUT);
}

float getPressure()
{
  /* 5V/1024bit = 4.883 mV/bit
   * offset_bit = 0 MPa  ... 0.5V  ... 102 bit  \
   * range_bit =                                 > 820 bit 
   * max_bit =    16MPa ... 4.5V  ... 922 bit   /
   * 
   * p(Mpa) / range_MPa = (analog_bit - offset_bit) / range_bit;
   * p(Mpa) = (analog_bit - offset_bit) * range_MPa / range_bit; 
   */
  return ((float)analogRead(PRESSURE_PIN) - PRESSURE_OFFSET + eeprom.pressureOffset) * 16 / (820 + eeprom.pressureDivider);
} 

void displayValue(float value, int row, int right, int maxDigit, int decimalCount)
{
  bool negative = false;
  if (value < 0)
  {
    negative = true;
    value = -value;
  }
  for (int i = 0; i < decimalCount; i++)
    value *= 10;

  int intValue = value;
  
  char buffer[NUMBER_BUFFER_SIZE];
  for (int i = 0; i < NUMBER_BUFFER_SIZE; i++)
    buffer[i] = ' ';

  int pos = 0;
  do
  {
    buffer[NUMBER_BUFFER_SIZE - 1 - pos] = (intValue % 10) + '0';
    intValue /= 10;  
    pos++;
    if (pos == decimalCount)
    {
      buffer[NUMBER_BUFFER_SIZE - 1 - pos] = '.';
      pos++;
    }
  }
  while (intValue != 0 || (0 != decimalCount && pos < decimalCount+2)); //one for point one for first 0
  if (negative)
    buffer[NUMBER_BUFFER_SIZE - 1 - pos] = '-';

  lcd.setCursor(right - maxDigit + 1, row);
  for (int i = 0; i < maxDigit; i++)
    lcd.print(buffer[i + NUMBER_BUFFER_SIZE - maxDigit]);
}

void SendValue(char variable, int value, bool valid = true)
{
  char buffer[10];
  if (valid)
    sprintf(buffer, "%c=%d\n", variable, value);
  else
    sprintf(buffer, "%c=?\n", variable);
  Serial.write(buffer);
}

int FillCommandValue()
{
  int value = 0;
  bool negative = false;
  while (true)
  {
    if (!Serial.available())
    {
      if (negative)
        return -value;
      else
        return value;
    } 
    char digit = Serial.read();
    
    if ('-' == digit)
      negative = true;
    else
    {  
      value *= 10;
      value += digit-'0';
    }
  } 
}
void _ProcessCommand(char command, bool setValue, int16_t & eepromVariable)
{
  if (setValue)
  {
    eepromVariable = FillCommandValue();
    EEPROM.put(0, eeprom);
  }
  SendValue(command, eepromVariable);
}
void ProcessCommand()
{
  if (Serial.available()) 
  {
    char command = Serial.read();
    if (command == 'r')
    {
      ResetEeprom();
      Serial.write("EEPROM reseted\n");
      return;
    }

    bool setValue = true;
    if (command == '?')
    {
      if (!Serial.available())
      {
         Serial.write("Příkazy:\n");
         Serial.write("x (xDegreeComp) - kompenzace predo-zadni osy\n");
         Serial.write("y (yDegreeComp) - kompenzace levo-prave osy\n");
         Serial.write("s (angleSensitivity) - senzitivita uhlu\n");
         Serial.write("S (pressureSensitivity) - tlakova senzitivita * 10\n");   
         Serial.write("o (pressureOffset) - ofset tlakoveho senzoru\n");
         Serial.write("d (pressureDivider) - delitel tlakového senzoru\n");
         Serial.write("0 (maxValue0) - maximalni hodnota pro bod1 a pozici1 \n");
         Serial.write("1 (maxValue1) - maximalni hodnota pro bod1 a pozici2 \n");
         Serial.write("2 (maxValue2) - maximalni hodnota pro bod1 a pozici3 \n");
         Serial.write("3 (maxValue3) - maximalni hodnota pro bod2 a pozici1 \n");
         Serial.write("4 (maxValue4) - maximalni hodnota pro bod2 a pozici2 \n");
         Serial.write("5 (maxValue5) - maximalni hodnota pro bod2 a pozici4 \n");
         Serial.write("6 (maxValue6) - maximalni hodnota pro bod3 a pozici1 \n");
         Serial.write("7 (maxValue7) - maximalni hodnota pro bod3 a pozici2 \n");
         Serial.write("8 (maxValue8) - maximalni hodnota pro bod3 a pozici3 \n");
         Serial.write("? - tato napoveda\n");
         Serial.write("?X - aktualni hodnota, kde X je znak pro pozadovanou hodnotu\n");
         return;
      }
      command = Serial.read();
      setValue = false;
    }
    Serial.read(); //expected '='
    switch (command)
    {   
      case 'x': _ProcessCommand(command, setValue, eeprom.xDegreeComp); break;
      case 'y': _ProcessCommand(command, setValue, eeprom.yDegreeComp); break;
      case 'o': _ProcessCommand(command, setValue, eeprom.pressureOffset); break;
      case 'd': _ProcessCommand(command, setValue, eeprom.pressureDivider); break;
      case 's': _ProcessCommand(command, setValue, eeprom.angleSensitivity); break;
      case 'S': _ProcessCommand(command, setValue, eeprom.pressureSensitivity); break;
      case '0': _ProcessCommand(command, setValue, eeprom.maxValue0); break;
      case '1': _ProcessCommand(command, setValue, eeprom.maxValue1); break;
      case '2': _ProcessCommand(command, setValue, eeprom.maxValue2); break;
      case '3': _ProcessCommand(command, setValue, eeprom.maxValue3); break;
      case '4': _ProcessCommand(command, setValue, eeprom.maxValue4); break;
      case '5': _ProcessCommand(command, setValue, eeprom.maxValue5); break;
      case '6': _ProcessCommand(command, setValue, eeprom.maxValue6); break;
      case '7': _ProcessCommand(command, setValue, eeprom.maxValue7); break;
      case '8': _ProcessCommand(command, setValue, eeprom.maxValue8); break;
      default:
        Serial.write("Unknown command");
        while (Serial.available())
        {
          Serial.read(); //I have to clear all followed letters after unknown command 
        }
      break;
    }     
  }
}
int getZirafaPercentage(double pressure)
{
  double numerator = pressure * 100 * 10;
  switch (zirafaMode)
  {
    case 0: return numerator / eeprom.maxValue0;
    case 1: return numerator / eeprom.maxValue1;
    case 2: return numerator / eeprom.maxValue2;
    case 3: return numerator / eeprom.maxValue3;
    case 4: return numerator / eeprom.maxValue4;
    case 5: return numerator / eeprom.maxValue5;
    case 6: return numerator / eeprom.maxValue6;
    case 7: return numerator / eeprom.maxValue7;
    case 8: return numerator / eeprom.maxValue8;
  }
}

void printZirafaMode()
{
  lcd.setCursor(7, 1);
  switch (zirafaMode)
  {
    case 0:
      lcd.print("b1p1 ");
      break;
    case 1:
      lcd.print("b1p2 ");
      break;
    case 2:
      lcd.print("b1p3 ");
      break;
    case 3:
      lcd.print("b2p1 ");
      break;
    case 4:
      lcd.print("b2p2 ");
      break;
    case 5:
      lcd.print("b2p3 ");
      break;
    case 6:
      lcd.print("b3p1 ");
      break;
    case 7:
      lcd.print("b3p2 ");
      break;
    case 8:
      lcd.print("b3p3 ");
      break;
  }
}
void loop() {
    int xAngle, yAngle;
    fillAngles(xAngle, yAngle);
    //SendValue('x', xAngle);
    //SendValue('y', yAngle);
    //delay(100);

    if  (xAngle > lastXAngle + eeprom.angleSensitivity || xAngle < lastXAngle - eeprom.angleSensitivity)
    {
      lastXAngle = xAngle;
    }
    displayValue(lastXAngle, 0, 2, 3, 0);
    lcd.write(0xDF); //°
    lcd.print("|  ");

    if  (yAngle > lastYAngle + eeprom.angleSensitivity || yAngle < lastYAngle - eeprom.angleSensitivity)
    {
      lastYAngle = yAngle;
    }
    else
    displayValue(lastYAngle, 1, 2, 3, 0);
    lcd.write(0xDF); //°
    lcd.print("-  ");
    float pressure = getPressure();
    lcd.setCursor(7, 0);
    switch (mayorMode)
    {
      case 0:
        if (pressure > lastPressure + (double)eeprom.pressureSensitivity / 10.0  || pressure < lastPressure - (double)eeprom.pressureSensitivity / 10.0 )
        {
          lastPressure = pressure;  
        }
        lcd.print(" Pressure");
        displayValue(lastPressure, 1, 12, 6, 1);
        lcd.print("MPa");
        break;
      case 1:
        lcd.print("   Zirafa");
        printZirafaMode();
        displayValue(getZirafaPercentage(pressure), 1, 14, 3, 0);
        lcd.print("%");

        break;
    }
    ProcessCommand(); 

    int buttonsValue = analogRead(BUTTONS_PIN);
    if (buttonsValue < 300) //both released
    {
      digitalWrite(LED_PIN, LOW);
      pressed = false;
    }
    
    else 
    {
      if (!pressed)
      {  
        pressed = true;    
        if (buttonsValue < 500) //button1
        {
          if (mayorMode == 1) //zirafaMode
          {
            zirafaMode++;
            if (zirafaMode == 9)
              zirafaMode = 0;
          }
        }
        else //button2
        {
          pressed = true;
          digitalWrite(LED_PIN, HIGH);
          mayorMode++;
          if (mayorMode > MAX_MAYOR_MODE)
            mayorMode = 0;
        }
      }
    }
    
    /*if (mode == 0)
      digitalWrite(11, HIGH);
    else
      digitalWrite(11, LOW);
   */   
}
