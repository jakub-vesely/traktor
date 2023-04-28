#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <EEPROM.h>
#include <LiquidCrystal.h>

#define NUMBER_BUFFER_SIZE 8
#define PRESSURE_OFFSET 99 //teoretically 102
#define BUTTONS_PIN 7 //A7
#define PRESSURE_PIN 1 //A1
#define LED_PIN 6 //D6
#define BUZZER_PIN 16 //changed to A3=D17 //A2 is D16
#define HALL_INPUT_PIN 2 
#define HALL_LED_1_PIN 11
#define HALL_LED_2_PIN 9
#define HALL_LED_3_PIN 8
#define HALL_LED_4_PIN 7

#define ALARM_STEPS 4
#define TYPE_ANGLE_LIMIT 1
#define TYPE_PRESSURE_LIMIT 2


MPU6050 accelerometr;
int MPUOffsets[6] = { -531, 1764, 1354, 54, -26, -26 }; //source: https://forum.arduino.cc/index.php?action=dlattach;topic=446713.0;attach=193816
LiquidCrystal lcd(12, 10, 5, 4, 3, 14);
#define ROWS 2
#define COLUMNS 16

int mayorMode = 2;
#define MAX_MAYOR_MODE 3

int zirafaMode = 0;
bool pressed = false;
bool isAlarm = false;

float lastPressure = 0;
int lastXAngle = 0;
int lastYAngle = 0;
int hall_counter = 0;
unsigned long last_hall_millis = 0;
unsigned long last_valid_hall_millis = 0;
#define HALL_FILTER_MS 30 //30 ms => 2000 ot /min
int hall_delay = -1; 
int 
struct Eeprom
{
  int16_t xDegreeComp; //x
  int16_t yDegreeComp; //y
  int16_t pressureOffset; //o
  int16_t pressureDivider; //d
  int16_t angleSensitivity; //s
  int16_t angleLimit; //l
  int16_t pressureSensitivity; //S  
  int16_t pressureLimit; //L   
  int16_t maxValue0; //1
  int16_t maxValue1; //1
  int16_t maxValue2; //2
  int16_t maxValue3; //3
  int16_t maxValue4; //4
  int16_t maxValue5; //5
  int16_t maxValue6; //6
  int16_t maxValue7; //7
  int16_t maxValue8; //8
  int16_t rpm_led1_limit;
  int16_t rpm_led2_limit;
  int16_t rpm_led3_limit;
  int16_t rpm_led4_limit;
  uint16_t hall_counter_history;
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
    pinMode(BUZZER_PIN, OUTPUT);
    alarm(1, TYPE_ANGLE_LIMIT); // short beep
    alarm(1, TYPE_PRESSURE_LIMIT);

    pinMode(HALL_INPUT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HALL_INPUT_PIN), hall_interrupt, RISING);

    pinMode(HALL_LED_1_PIN, OUTPUT);
    digitalWrite(HALL_LED_1_PIN, LOW);
    pinMode(HALL_LED_2_PIN, OUTPUT);
    digitalWrite(HALL_LED_2_PIN, LOW);

    pinMode(HALL_LED_3_PIN, OUTPUT);
    digitalWrite(HALL_LED_3_PIN, LOW);

    pinMode(HALL_LED_4_PIN, OUTPUT);
    digitalWrite(HALL_LED_4_PIN, LOW);

    last_hall_millis = millis();
}

void hall_interrupt(){
  unsigned long milis = millis();
  if (last_hall_millis + HALL_FILTER_MS < milis){ //hall sensor generates a lot of interrupts on a sensitivity boundary
    hall_counter++;
    
    if (last_valid_hall_millis > 0){
      hall_delay = milis - last_valid_hall_millis;      
    }
    last_valid_hall_millis = milis;
  }
  last_hall_millis = milis;
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
         Serial.write("l (angleLimit) - Mez nálkonu pro signalizaci ve °\n");
         Serial.write("S (pressureSensitivity) - tlakova senzitivita * 10\n");
         Serial.write("L (pressureLimit) - Mez tlaku pro signalizaci v %\n");   
         Serial.write("o (pressureOffset) - ofset tlakoveho senzoru\n");
         Serial.write("d (pressureDivider) - delitel tlakového senzoru\n");
         Serial.write("A (rpm_led1_limit) - limit ot/min pro první zelenou LED\n");
         Serial.write("B (rpm_led2_limit) - limit ot/min pro druhou zelenou LED\n");
         Serial.write("C (rpm_led3_limit) - limit ot/min pro žlutou LED\n");
         Serial.write("D (rpm_led4_limit) - limit ot/min pro červenou LED\n");
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
      case 'l': _ProcessCommand(command, setValue, eeprom.angleLimit); break;
      case 'S': _ProcessCommand(command, setValue, eeprom.pressureSensitivity); break;
      case 'L': _ProcessCommand(command, setValue, eeprom.pressureLimit); break;
      case 'A': _ProcessCommand(command, setValue, eeprom.rpm_led1_limit); break;
      case 'B': _ProcessCommand(command, setValue, eeprom.rpm_led2_limit); break;
      case 'C': _ProcessCommand(command, setValue, eeprom.rpm_led3_limit); break;
      case 'D': _ProcessCommand(command, setValue, eeprom.rpm_led4_limit); break;
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

void alarm(int state, int type)
{
  int frequency = type == TYPE_ANGLE_LIMIT ? 880 : 1760; //A5,A6 
  int on_duration = state * 60;
  int off_duration = (ALARM_STEPS - state) * 60;
  if (on_duration > 0)
  {
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER_PIN, frequency, on_duration + on_duration / 2); //I have to add some duration to be covered time when is processed main program - only this way is possible to have uninterupted beep
    delay(on_duration);
    //
  }
  if (off_duration > 0)
  {
    digitalWrite(LED_PIN, LOW);
    delay(off_duration);
  }
}

bool testAlarm(int abs_value, int limit, int type)
{
  if (abs_value > limit)
  {
    alarm(4, type);
    return true;
  }
  
  if ((double)abs_value > (double)limit * 0.9)
  {
    alarm(3, type);
    return true;
  }
  
  if ((double)abs_value > (double)limit * 0.8)
  {
    alarm(2, type);
    return true;
  }
  
  if ((double)abs_value > (double)limit * 0.7)
  {
    alarm(1, type);
    return true;
  }
  
  return false; 
}

int process_rpm(){
  int rpm = 0;
  if (hall_delay > 0){
    rpm =  (int)(1.0/ (float)(hall_delay) * 60000) ;
    if (last_valid_hall_millis + 2000 < millis()){ //it is expected that speed will be higher than 30 rot/min 
      hall_delay = 0;
    }
  } 
  digitalWrite(HALL_LED_1_PIN, rpm > eeprom.rpm_led1_limit ? HIGH : LOW);
  digitalWrite(HALL_LED_2_PIN, rpm > eeprom.rpm_led2_limit ? HIGH : LOW);
  digitalWrite(HALL_LED_3_PIN, rpm > eeprom.rpm_led3_limit ? HIGH : LOW);
  digitalWrite(HALL_LED_4_PIN, rpm > eeprom.rpm_led4_limit ? HIGH : LOW);
  return rpm;
}

void loop() {
    int xAngle, yAngle;
    fillAngles(xAngle, yAngle);
    //SendValue('x', xAngle);
    //SendValue('y', yAngle);
    //delay(100);
    if (!isAlarm)
    {
      alarm(0, TYPE_PRESSURE_LIMIT); //any type
    }
    ;isAlarm = false;
    
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

    if (testAlarm(abs(lastYAngle), eeprom.angleLimit, TYPE_ANGLE_LIMIT))
    {
      isAlarm = true;
    }
    
    displayValue(lastYAngle, 1, 2, 3, 0);
    lcd.write(0xDF); //°
    lcd.print("-  ");
    float pressure = getPressure();
    lcd.setCursor(7, 0);

    int rpm = process_rpm();
    switch (mayorMode)
    {
      case 0:
        if (pressure > lastPressure + (double)eeprom.pressureSensitivity / 10.0  || pressure < lastPressure - (double)eeprom.pressureSensitivity / 10.0 )
        {
          lastPressure = pressure;  
        }
        lcd.print("     Tlak");
        displayValue(lastPressure, 1, 12, 6, 1);
        lcd.print("MPa");
        break;
      case 1: {
        lcd.print("   Zirafa");
        printZirafaMode();
        int zirafaPercentage = getZirafaPercentage(pressure);
        displayValue(zirafaPercentage, 1, 14, 3, 0);
        lcd.print("%");
        if (testAlarm(zirafaPercentage, eeprom.pressureLimit, TYPE_PRESSURE_LIMIT)){
          isAlarm = true;
        }
        break;
      }
      case 2: {
        lcd.print("   Otacky");
        lcd.setCursor(7, 1);
        displayValue(rpm, 1, 12, 6, 0);
        lcd.print("o/m");
        break;
      }
      case 3: {
        lcd.print("   Provoz");
        lcd.setCursor(7, 1);
        displayValue(rpm, 1, 12, 6, 0);
        lcd.print("o/m");
        break;
      }
    } 
    ProcessCommand(); 

    int buttonsValue = analogRead(BUTTONS_PIN);
    if (buttonsValue < 300) //both released
    {
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
          mayorMode++;
          if (mayorMode > MAX_MAYOR_MODE)
            mayorMode = 0;
        }
      }
    }  
}
