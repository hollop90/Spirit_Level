//----------------------------------------
// TITLE  Spirit Level Rev 2
// AUTHOR Ugochukwu Uzoukwu
// VERSION  0.1
// CREATED  11/02/2021
//----------------------------------------
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h" // MPU60 DMP Library
#include "NewPing.h"
#include "LiquidCrystal_I2C.h"

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

MPU6050 mpu;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
LiquidCrystal_I2C lcd(0x27, 16, 2);

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

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void mpu_setup();
void updateOrientation();
void getDistance();

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
  Serial.println(F("Setting up IMU..."));
  mpu_setup();

  pinMode(12, OUTPUT); // Status LEDs
  pinMode(13, OUTPUT);

  Serial.println(F("Setup complete"));

  delay(500);
  lcd.clear();
}

void loop()
{
  //updateOrientation(); // Get orientation froM IMU
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

void mpu_setup()
{
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  // initialize device
  #ifdef IMU_DEBUG
    Serial.println(F("Initializing I2C devices..."));
  #endif
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  #ifdef  IMU_DEBUG
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  #endif

  // load and configure the DMP
  #ifdef IMU_DEBUG
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
  #endif

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(0); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    // turn on the DMP, now that it's ready
    #ifdef IMU_DEBUG
      Serial.println(F("Enabling DMP..."));
    #endif
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    #ifdef IMU_DEBUG
      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)... EDIT: Interrupt is not used in this program"));
    #endif
    //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    #ifdef IMU_DEBUG
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
    #endif  
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    #ifdef IMU_DEBUG
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    #endif  
  }
}

void updateOrientation()
{
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
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