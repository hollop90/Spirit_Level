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
Button switchA(8, true);
Button switchB(9, true);

//##############################
// VARIABLE DECLARATIONS 
//##############################

// Orientation Vars
int x;
int y;
int angleBuff = 0;

// Button vars
bool lastButtonStateA = HIGH;
bool lastButtonStateB = HIGH;

// Menu system vars
bool switchMenu = false;
bool menuFunction = false;
bool updateScreen = true;
int incArea = 0;
int menuState = 0;
bool init0 = false;
bool init1 = false;

// Distance sensor vars
int distance;
int distanceBuff;
float distanceA;
float distanceB;
float area;
unsigned long sonarInterval = 50;
unsigned long lastPingTime;

unsigned long buzzerInterval = 500;
unsigned long lastBuzzTime;

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
  pinMode(STATUS_PIN, OUTPUT);

  Serial.begin(115200);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(2, 0);
  lcd.print("Spirit Level");
  lcd.setCursor(1, 1);
  lcd.print("& Rangefinder");
  Serial.println(F("Spirit level with distance meeasurment"));  

  delay(500);
  mpu.begin();
  digitalWrite(STATUS_PIN, LOW);
  mpu.calcOffsets();
  Serial.println(F("Setup complete"));
  lcd.clear();
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

  if(millis() - lastBuzzTime >= buzzerInterval) // "Heatbeat" to show that the program is working
  {
    digitalWrite(12, !(digitalRead(12)));
    lastBuzzTime = millis();
  }

// Polling for button A and B
  if (switchA.getButtonState() == LOW && lastButtonStateA == HIGH){
    lastButtonStateA = LOW;
    switchMenu = true;
  }else if (switchA.getButtonState() == HIGH){
    lastButtonStateA = HIGH;
  }

  if (switchB.getButtonState() == LOW && lastButtonStateB == HIGH){
    lastButtonStateB = LOW;
    menuFunction = true;
  }else if (switchB.getButtonState() == HIGH){
    lastButtonStateB = HIGH;
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
    menuFunction = false;
    updateScreen = true;
    init0 = true;
  }

  // if button is pressed change init var
  if(switchMenu == true)
  {
    switchMenu = false;
    init0 = false;
    menuState++;
  }

  if(menuFunction == true && menuState == 0)
  {
    menuFunction = false;
    updateScreen = !updateScreen;
  }

  if (mpu.getAngleX() > -1 && mpu.getAngleX() < 1)
  {
    digitalWrite(STATUS_PIN, HIGH);
  }else{
    digitalWrite(STATUS_PIN, LOW);
  }
  
  // Hold the values on screen
  if (updateScreen == true)
  {
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
}

void menu1()
{
  // Initialise the menu
  if(!init1)
  {
    lcd.clear();
    lcd.print("A:   cm B:   cm");
    lcd.setCursor(0, 1);
    lcd.print("Area: .  ");
    //lcd.print("m2"); // Degree symbol
    init1 = true;
    updateScreen = false;
  }

  if(switchMenu == true)
  {
    switchMenu = false;
    init1 = false;
    menuState++;
  }

  if(menuFunction == true && menuState == 1)
  {
    menuFunction = false;
    incArea++;
  }

  switch (incArea)
  {
  case 0:
    distanceBuff = distance * 10;
    lcd.setCursor(2, 0);
    lcd.print(abs(distanceBuff)/1000); // print first digit
    lcd.print(abs(distanceBuff)/100 % 10); // print second digit
    lcd.print(abs(distanceBuff)/10 % 10); // print third digit
    break;
  case 1:
    distanceA = distance;
    incArea++;
    break;
  case 2:
    distanceBuff = distance * 10;
    lcd.setCursor(10, 0);
    lcd.print(abs(distanceBuff)/1000); // print first digit
    lcd.print(abs(distanceBuff)/100 % 10); // print second digit
    lcd.print(abs(distanceBuff)/10 % 10); // print third digit
    break;
  case 3:
    distanceB = distance;
    incArea++;
    break;
  case 4:
    area = distanceA*distanceB;
    incArea++;
    break;
  case 5:
    lcd.setCursor(6, 1);
    lcd.print(area);
    incArea++;
  case 6:
    delay(500);
    if(switchB.getButtonState() == LOW)
    {
      init1 = false;
      incArea = 0;
    }
  default:
    break;
  }
}

void rangeError()
{
  lcd.clear();
  lcd.print("Range Error");
}