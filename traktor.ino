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

int mode = 0;
#define MAX_MODE 4
bool pressed = false;

struct Eeprom
{
  int16_t xDegreeComp; //x
  int16_t yDegreeComp; //y
  int16_t pressureOffset; //o
  int16_t pressureDivider; //d
  int16_t positionCount; //c
  int16_t multiplier0; //1
  int16_t multiplier1; //1
  int16_t multiplier2; //2
  int16_t multiplier3; //3
  int16_t multiplier4; //4
  int16_t multiplier5; //5
  int16_t multiplier6; //6
  int16_t multiplier7; //7
  int16_t multiplier8; //8
  int16_t multiplier9; //9
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
         Serial.write("Commands:\n");
         Serial.write("x - xDegreeComp\n");
         Serial.write("y - yDegreeComp\n");
         Serial.write("o - pressureOffset\n");
         Serial.write("d - pressureDivider\n");
         Serial.write("c - positionCount\n");
         Serial.write("0 - multiplier0\n");
         Serial.write("...\n");
         Serial.write("9 - multiplier9\n");
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
      case 'c': _ProcessCommand(command, setValue, eeprom.positionCount); break;
      case '0': _ProcessCommand(command, setValue, eeprom.multiplier0); break;
      case '1': _ProcessCommand(command, setValue, eeprom.multiplier1); break;
      case '2': _ProcessCommand(command, setValue, eeprom.multiplier2); break;
      case '3': _ProcessCommand(command, setValue, eeprom.multiplier3); break;
      case '4': _ProcessCommand(command, setValue, eeprom.multiplier4); break;
      case '5': _ProcessCommand(command, setValue, eeprom.multiplier5); break;
      case '6': _ProcessCommand(command, setValue, eeprom.multiplier6); break;
      case '7': _ProcessCommand(command, setValue, eeprom.multiplier7); break;
      case '8': _ProcessCommand(command, setValue, eeprom.multiplier8); break;
      case '9': _ProcessCommand(command, setValue, eeprom.multiplier9); break;
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
int getWeight()
{
  switch (mode)
  {
    case 0: return getPressure() * eeprom.multiplier0;
    case 1: return getPressure() * eeprom.multiplier1;
    case 2: return getPressure() * eeprom.multiplier2;
    case 3: return getPressure() * eeprom.multiplier3;
    case 4: return getPressure() * eeprom.multiplier4;
    case 5: return getPressure() * eeprom.multiplier5;
    case 6: return getPressure() * eeprom.multiplier6;
    case 7: return getPressure() * eeprom.multiplier7;
    case 8: return getPressure() * eeprom.multiplier8;
    case 9: return getPressure() * eeprom.multiplier9;
  }
}
void loop() {
    int xAngle, yAngle;
    fillAngles(xAngle, yAngle);
    //SendValue('x', xAngle);
    //SendValue('y', yAngle);
    //delay(100);
    
    displayValue(xAngle, 0, 2, 3, 0);
    lcd.print("| ");
    displayValue(yAngle, 1, 2, 3, 0);
    lcd.print("- ");
    
    displayValue(getPressure(), 0, 12, 6, 1);
    lcd.print("MPa");

    displayValue(getWeight(), 1, 11, 6, 0);
    lcd.print("kg");

    displayValue(mode + 1, 1, 15, 1, 0);
    
    //lcd.print(getPressure());
    lcd.print("   ");

    ProcessCommand(); 
    
    if (300 < analogRead(BUTTONS_PIN)) //second should be > 500
    {
      digitalWrite(LED_PIN, HIGH);
      if (!pressed)
      {
        pressed = true;
        mode++;
        if (mode == eeprom.positionCount)
          mode = 0;
      }
    }
    else
    {
      digitalWrite(LED_PIN, LOW);
      pressed = false;
    }
    
    /*if (mode == 0)
      digitalWrite(11, HIGH);
    else
      digitalWrite(11, LOW);
   */   
}
