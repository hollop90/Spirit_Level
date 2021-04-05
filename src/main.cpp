//----------------------------------------
// TITLE  Spirit Level Rev 2
// AUTHOR Ugochukwu Uzoukwu
// VERSION  0.1
// CREATED  11/02/2021
//----------------------------------------
#include <Arduino.h>
#include "NewPing.h"
#include "LiquidCrystal_I2C.h"
#include <MPU6050_light.h>
#include <Button Lib.h>

//#define IMU_DEBUG
#define OUTPUT_DEBUG

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define INTERRUPT_PIN 2 // use pin 15 on ESP8266

#define TRIGGER_PIN  5  
#define ECHO_PIN     4  
#define MAX_DISTANCE 200

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
MPU6050 mpu(Wire);
int x;
int y;
int z;

bool singlePress = true;
bool switchMenu = false;
int menuState = 0;

bool init0 = false;
bool init1 = false;

void menu0();
void menu1();
void rangeError();

Button switchA(6, true);

// Distance sensor vars
int distance;
int distanceBuff;
unsigned long sonarInterval = 50;
unsigned long lastPingTime;
void getDistance();

int angleBuff = 0;

void setup()
{
  Serial.begin(115200);
  mpu.begin();

  lcd.init();
  lcd.backlight();
  lcd.setCursor(2, 0);
  lcd.print("Spirit Level");
  lcd.setCursor(1, 1);
  lcd.print("& Rangefinder");

  Serial.println(F("Spirit level with distance meeasurment")); 

  pinMode(12, OUTPUT); // Status LEDs
  pinMode(13, OUTPUT);

  mpu.calcOffsets();
  Serial.println(F("Setup complete"));

  delay(500);
  lcd.clear();
}

void loop()
{
  mpu.update();
  x = mpu.getAngleX();
  y = mpu.getAngleY();
  z = mpu.getAngleZ();
  
  if(millis() - lastPingTime >= sonarInterval) // Measure distance every 50ms
  {
    distance = sonar.ping_cm();
    lastPingTime = millis();
  }

  // put your main code here, to run repeatedly:
  if (switchA.getButtonState() == LOW && singlePress == true){
    singlePress = false;
    switchMenu = true;
    //menuState++;
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
    /*Serial.print(F("Dist: "));
    Serial.print(distance);
    Serial.print("  ");
    Serial.print(F("Yaw, Pitch, Roll: "));
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("  ");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("  ");
    Serial.print(ypr[2] * 180/M_PI);*/

    Serial.print("Menu: ");
    Serial.println(menuState);
    //delay(100);
  #endif
}

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

  distanceBuff = distance * 10;
  lcd.setCursor(5, 0);
  lcd.print(abs(distanceBuff)/1000); // print first digit
  lcd.print(abs(distanceBuff)/100 % 10); // print second digit
  lcd.print(abs(distanceBuff)/10 % 10); // print third digit
  lcd.print("cm  ");

  angleBuff = x * 10;
  lcd.setCursor(6, 1);
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