
// Servo Connection
// BROWN - gnd
// red - 5v
// yellow - d10 (pwm on Sero 1)
//        - d11 (servo 2)

// MPU Connection
//
// VCC - 3v3 or 5v
// GND - GND
// SCL - A5 (w/ 10k PuR)
// SCA - A4 (w/ 10k PuR)
// INT - D2 (not used)

#include <Servo.h>
#include "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#endif

#define LED_PIN 13
bool blinkState = true;

Servo Servo1;   
Servo Servo2;   

int Servo1Pos = 0;
int Servo2Pos = 0;

float mpuPitch = 0;
float mpuRoll = 0;
float mpuYaw = 0;



MPU6050 mpu;                   


uint8_t mpuIntStatus;  
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;    
uint8_t fifoBuffer[64]; 

// orientation/motion vars
Quaternion q;           //quaternion container
VectorInt16 aa;         //accel sensor measurements
VectorInt16 aaReal;     //gravity free accel sensor measurements
VectorInt16 aaWorld;    //world-frame accel sensor measurements
VectorFloat gravity;    //gravity vector
float ypr[3];           //yaw/pitch/roll container and gravity vector


#define PITCH   1  
#define ROLL  2    
#define YAW   0     

void setup()
{

  Servo1.attach(10);  
  Servo2.attach(11);  
  delay(50);
  Servo1.write(0); 
  Servo2.write(60); 
  delay(500); 
  Servo1.write(180);
  Servo2.write(120);
  delay(500);
  Servo1.write(0);
  Servo2.write(90);
  delay(500);

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; 
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  
  while (!Serial);     

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP"));
  devStatus = mpu.dmpInitialize();


  /*mpu.setXGyroOffset(118);
  mpu.setYGyroOffset(-44);
  mpu.setZGyroOffset(337);
  mpu.setXAccelOffset(-651);
  mpu.setYAccelOffset(670);
  mpu.setZAccelOffset(1895);*/ 
  mpu.setXGyroOffset(25);
  mpu.setYGyroOffset(43);
  mpu.setZGyroOffset(-47);
  mpu.setXAccelOffset(818);
  mpu.setYAccelOffset(982);
  mpu.setZAccelOffset(1130);


  if (devStatus == 0)
  {
   
    Serial.println(F("Enabling DMP"));
    mpu.setDMPEnabled(true);

   
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)"));
    mpuIntStatus = mpu.getIntStatus();

    
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    Serial.print(F("DMP Initialization failed code = "));
    Serial.println(devStatus);
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

} 

void loop(void)
{
  processAccelGyro();
}  

void processAccelGyro()
{
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  // check for overflow 
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    return;
  }

  if (mpuIntStatus & 0x02) 
  {

    if (fifoCount < packetSize)
      return; //  fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // to track FIFO count here in case there is > 1 packet available
    fifoCount -= packetSize;

    // flush buffer to prevent overflow
    mpu.resetFIFO();


    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpuPitch = ypr[PITCH] * 180 / M_PI;
    mpuRoll = ypr[ROLL] * 180 / M_PI;
    mpuYaw  = ypr[YAW] * 180 / M_PI;


    mpu.resetFIFO();

    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    mpu.resetFIFO();

    Servo1.write(-mpuPitch + 90);
    Servo2.write(mpuRoll + 90);


    mpu.resetFIFO();

  } 
} 
