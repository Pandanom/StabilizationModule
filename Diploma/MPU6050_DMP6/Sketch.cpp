/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

#include "PID.h"
#include "libs/I2Cdev.h"
#include "libs/MPU6050_6Axis_MotionApps20.h"
#include "Kalman.h"
 
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "libs/Wire.h"
//Beginning of Auto generated function prototypes by Atmel Studio
void dmpDataReady();
//End of Auto generated function prototypes by Atmel Studio


#endif
 
MPU6050 mpu;
 
Kalman kalmanX;
// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint32_t timer;
// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
 
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}
 
void setup()
{
  
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

 //Serial.println(F("init start"));
mpu.initialize();
//Serial.println(F("init done"));
devStatus = mpu.dmpInitialize();
 //Serial.println(devStatus);
 //Serial.println(mpu.getDLPFMode());
// supply your own gyro offsets here, scaled for min sensitivity
mpu.setXGyroOffset(220);
mpu.setYGyroOffset(76);
mpu.setZGyroOffset(-85);
mpu.setZAccelOffset(1688); // 1688 factory default for my test chip
 
// make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
	mpu.setDLPFMode(0);
	mpu.setDHPFMode(0);
	//mpu.CalibrateGyro(10);
	//mpu.CalibrateAccel(10);
    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  Serial.begin(115200);
  Serial.flush();
   kalmanX.setAngle(0);
}
 
 
void loop()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
 
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize);
 
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
 
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
 
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
 
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
	/*
    Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.println(q.z);
	*/
	
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
	
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print(",");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print(",");
    Serial.println(ypr[2] * 180/M_PI); // needed to regulate on model
	
	/*
	ypr[2] *= 180/M_PI;
	auto noise = ypr[2] + rand() % 100 - 49;
	double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
	timer = micros();
	auto ka = kalmanX.getAngle(noise, (double)mpu.getRotationX() / 131.0, dt);
	Serial.print(noise);
	Serial.print(",");
	Serial.print(ypr[2]);
	Serial.print(",");
	Serial.println(ka);
	*/
  }
}
