/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

#include "PID.h"
#include "libs/I2Cdev.h"
#include "libs/MPU6050_6Axis_MotionApps20.h"
#include "Kalman.h"
#include "GimbalMot.h"
#include "libs/pidautotuner.h"
 
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "libs/Wire.h"
//Beginning of Auto generated function prototypes by Atmel Studio
void dmpDataReady();
//End of Auto generated function prototypes by Atmel Studio


#endif

//uncomment to turn on data filter
//#define TURN_ON_FILTER
 
 
MPU6050 mpu;
#ifdef TURN_ON_FILTER
Kalman kalmanX, kalmanY, kalmanZ;
#endif
GimbalMot gm;
PID pid(-125,125);
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
 
long tst = 0;
 
 
 
 
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

 
void updateData() 
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
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		ypr[0] *= 180/M_PI;
		ypr[1] *= 180/M_PI;
		ypr[2] *= 180/M_PI;
	}
 }
 
 #ifdef TURN_ON_FILTER
 void filterData(){
	 double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
	 timer = micros();
	 ypr[0] = kalmanZ.getAngle(ypr[0], (double)mpu.getRotationZ() / 131.0, dt);
	 ypr[1] = kalmanY.getAngle(ypr[1], (double)mpu.getRotationY() / 131.0, dt);
	 ypr[2] = kalmanX.getAngle(ypr[2], (double)mpu.getRotationX() / 131.0, dt);
 }
 #endif
 
  //tune PID controller
  //outputs in serial kp, ki and kd  
  void tunePID()
  {
	  Serial.println("tune start");
	  PIDAutotuner tuner = PIDAutotuner();
	  tuner.setTargetInputValue(0);
	  tuner.setLoopInterval(50);
	  tuner.setOutputRange(-125, 125);
	  tuner.setZNMode(PIDAutotuner::ZNModeLessOvershoot);
	  tuner.startTuningLoop(micros());
	  long microseconds;
	  while (!tuner.isFinished()) {
		  updateData();
		  // This loop must run at the same speed as the PID control loop being tuned
		  long prevMicroseconds = microseconds;
		  microseconds = micros();
		  double input = ypr[2];
		  double output = tuner.tunePID(input, microseconds);
		  gm.setVelX(output);
		  while (micros() - microseconds < 50) {
			  delayMicroseconds(1);
			  updateData();
			  }
		Serial.print(".");
	  }
	  Serial.println("");
	  gm.setVelX(0);
	   double kp = tuner.getKp();
	   double ki = tuner.getKi();
	   double kd = tuner.getKd();
	    Serial.print("kp = ");
		Serial.println(kp);
		Serial.print("ki = ");
		Serial.println(ki);
		Serial.print("kd = ");
		Serial.println(kd);
  }
  
  
 
 
void setup()
{
    Serial.begin(115200);
    Serial.flush();
	 Serial.println("init start");
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
	//mpu.setDLPFMode(0);
	//mpu.setDHPFMode(0);
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

  #ifdef TURN_ON_FILTER
  kalmanX.setAngle(0);
  kalmanY.setAngle(0);
  kalmanZ.setAngle(0);
  #endif
  pid.setPos(0);
  gm.init(3, 5, 6, 9, 10, 11);
  Serial.println("init end");
	
  //uncomment if tuning needed 
  //tunePID();
  
  tst = micros();
}
 
 
void loop()
{
	
	updateData();
	#ifdef TURN_ON_FILTER
	filterData();
	#endif
	  
	
    Serial.println(ypr[2]); // needed to be regulated on model
	
	if(micros() - tst > 1000)
	{			
		tst = micros();
		auto vel = pid.calcReg(ypr[2]);
		gm.setVelX(vel);
	}
	//set kp, ki and kd from serial
	if (Serial.available() > 0) {
		auto k = Serial.readStringUntil('\n');
		if(k[0] == 'p')
		{
			k.remove(0,1);
			pid.setKp(k.toDouble());
		}
		else if(k[0] == 'i')
		{
			k.remove(0,1);
			pid.setKi(k.toDouble());
		}
		else if(k[0] == 'd')
		{
			k.remove(0,1);
			pid.setKd(k.toDouble());
		}
	}
 
}
