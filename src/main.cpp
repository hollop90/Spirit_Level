//----------------------------------------
// TITLE  Spirit Level Rev 2
// AUTHOR Ugochukwu Uzoukwu
// CREATED  11/02/2021
//----------------------------------------

//##############################
// PREPROCESSOR DIRECTIVES
//##############################
#include <Arduino.h>
#include "NewPing.h"
#include "LiquidCrystal_I2C.h"
#include <MPU6050_light.h>
#include <Button Lib.h>

#define OUTPUT_DEBUG
#define STATUS_PIN 13
#define INTERRUPT_PIN 2
#define TRIGGER_PIN  5  
#define ECHO_PIN     4  
#define MAX_DISTANCE 200

//##############################
// NEW OBJECTS
//##############################
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
MPU6050 mpu(Wire);
Button switchA(6, true);

//##############################
// VARIABLE DECLARATIONS 
//##############################
int x;
int y;
int angleBuff = 0;

bool singlePress = true;
bool switchMenu = false;
int menuState = 0;

bool init0 = false;
bool init1 = false;

// Distance sensor vars
int distance;
int distanceBuff;
unsigned long sonarInterval = 50;
unsigned long lastPingTime;

//##############################
// FUNCITON PROTOTYPES
//##############################
void getDistance();
void menu0();
void menu1();
void rangeError();

//##########
// SETUP
//##########
void setup()
{
  Serial.begin(115200);
  lcd.init();

  lcd.backlight();
  lcd.setCursor(2, 0);
  lcd.print("Spirit Level");
  lcd.setCursor(1, 1);
  lcd.print("& Rangefinder");

  Serial.println(F("Spirit level with distance meeasurment")); 

  pinMode(12, OUTPUT); // Status LEDs
  pinMode(STATUS_PIN, OUTPUT);
  
  Serial.println(F("Setup complete"));

  delay(500);
  lcd.clear();
  mpu.begin();
  mpu.calcOffsets();
}

//##########
// LOOP
//##########
void loop()
{
  mpu.update();
  x = mpu.getAngleX();
  y = mpu.getAngleY();
  
  if(millis() - lastPingTime >= sonarInterval) // Measure distance every 50ms
  {
    distance = sonar.ping_cm();
    lastPingTime = millis();
  }

  // put your main code here, to run repeatedly:
  if (switchA.getButtonState() == LOW && singlePress == true){
    singlePress = false;
    switchMenu = true;
  }else if (switchA.getButtonState() == HIGH){
    singlePress = true;
  }

  switch(menuState)
  {
    case 0:
      menu0();
    break;

    case 1:
      menu1();
    break;

    default:
      rangeError();
    break;
  }
  if (menuState > 1)
  {
    menuState = 0;
  }

  #ifdef OUTPUT_DEBUG
    Serial.print(F("Dist: "));
    Serial.print(distance);
    Serial.print("  ");
    Serial.print(F("Angle: "));
    Serial.println(mpu.getAngleX());
    //delay(100);
  #endif
}

//##############################
// FUNCTION DEFINTIONS
//##############################
void menu0() // Distance and angle screen
{
  if(!init0)
  {
    lcd.clear();
    lcd.print("Dist:000cm");
    lcd.setCursor(0, 1);
    lcd.print("Anlge:000");
    lcd.print((char)0xDF); // Degree symbol
    init0 = true;
  }

  // if button is pressed change init var
  if(switchMenu == true)
  {
    switchMenu = false;
    init0 = false;
    menuState++;
  }

  if (mpu.getAngleX() > -1 && mpu.getAngleX() < 1)
  {
    digitalWrite(STATUS_PIN, HIGH);
  }else{
    digitalWrite(STATUS_PIN, LOW);
  }
  
  distanceBuff = distance * 10;
  lcd.setCursor(5, 0);
  lcd.print(abs(distanceBuff)/1000); // print first digit
  lcd.print(abs(distanceBuff)/100 % 10); // print second digit
  lcd.print(abs(distanceBuff)/10 % 10); // print third digit
  lcd.print("cm  ");

  angleBuff = mpu.getAngleX() * 10;
  lcd.setCursor(6, 1);
  if(mpu.getAngleX() < 0)
  {
    lcd.print("-");
  }else{
    lcd.print("+");
  }
  lcd.print(abs(angleBuff)/1000); // print first digit
  lcd.print(abs(angleBuff)/100 % 10); // print second digit
  lcd.print(abs(angleBuff)/10 % 10); // print third digit
  lcd.print((char)0xDF);
}

void menu1()
{
  if(!init1)
  {
    lcd.clear();
    lcd.print("Boop:123cm");
    lcd.setCursor(0, 1);
    lcd.print("Apple:456");
    lcd.print((char)0xDF); // Degree symbol
    init1 = true;
  }

  if(switchMenu == true)
  {
    switchMenu = false;
    init1 = false;
    menuState++;
  }
}

void rangeError()
{
  lcd.clear();
  lcd.print("Range Error");
}