#include "Wire.h"
#include "Math.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <EEPROM.h>
#include <LiquidCrystal.h>

#define NUMBER_BUFFER_SIZE 8
#define PRESSURE_OFFSET 99 //teoretically 102
 
MPU6050 accelerometr;
#define X_COMP 2740 //compensation
#define Y_COMP 14900
#define Z_COMP -200

LiquidCrystal lcd(7, 6, 5, 4, 3, 2);
#define ROWS 2
#define COLUMNS 16

struct Eeprom
{
  int16_t xDegreeComp; //x
  int16_t yDegreeComp; //y
  int16_t pressureOffset; //o
  int16_t pressureDivider; //d
} eeprom;

void fillAngles(int &xAngle, int &yAngle)
{
  const int maxCount = 200;
  
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
     
  x = ax_p / maxCount + X_COMP;
  y = ay_p / maxCount + Y_COMP;
  z = az_p / maxCount + Z_COMP;
     
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
    //TODO
    //if (accelerometr.testConnection())
    //  Serial.println("Spojeni OK...");
      
    lcd.begin(COLUMNS, ROWS);

    //ResetEeprom();
    EEPROM.get(0, eeprom);
    Serial.begin(115200);
}

float getPressure()
{
  /* 5V/1024bit = 4.883 mV/bit
   * offset_bit = 0 MPa  ... 0.5V  ... 102 bit  \
   * range_bit =                                 > 820 bit 
   * max_bit =    16MPa ... 4.5V  ... 922 bit  /
   * 
   * p(Mpa) / range_MPa = (analog_bit - offset_bit) / range_bit;
   * p(Mpa) = (analog_bit - offset_bit) * range_MPa / range_bit; 
   */
  return ((float)analogRead(1) - PRESSURE_OFFSET + eeprom.pressureOffset) * 16 / (820 + eeprom.pressureDivider);
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
    char decider = Serial.read();
    bool setValue = (decider != '?');
    switch (command)
    {
      case 'r':
        ResetEeprom();
        Serial.write("EEPROM reseted");
      break;
      case 'x':
        _ProcessCommand(command, setValue, eeprom.xDegreeComp);
      break;
      case 'y':
        _ProcessCommand(command, setValue, eeprom.yDegreeComp);
      break;
      case 'o':
        _ProcessCommand(command, setValue, eeprom.pressureOffset);
      break;
      case 'd':
        _ProcessCommand(command, setValue, eeprom.pressureDivider);
      break;
      default:
        SendValue(command, 0, false);  
    }      
  }
}
void loop() {
    int xAngle, yAngle;
    fillAngles(xAngle, yAngle);
    displayValue(xAngle, 0, 2, 3, 0);
    lcd.print("| ");
    displayValue(yAngle, 1, 2, 3, 0);
    lcd.print("- ");
    
    displayValue(getPressure(), 0, 12, 6, 1);
    lcd.print("MPa");
    
    //lcd.print(getPressure());
    lcd.print("   ");

    ProcessCommand();
}
