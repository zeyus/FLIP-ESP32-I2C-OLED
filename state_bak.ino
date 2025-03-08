#include <Arduino.h>
#include <FastTrig.h>
#include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050_6Axis_MotionApps612.h"


MPU6050 mpu;

#define DEBUG
#define IMU_ADDRESS 0x68
#define LED 17
// External interrupt source
#define EXT0_PIN 34

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer
/*---Orientation/Motion Variables---*/ 
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gy;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
uint16_t fifoCount;
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = true;     // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}



bool blinkState = true;

// Setup serial communication

void initSerial() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
}

void initI2C() {
  Wire.begin();
}

void initMpu() {
  // set up MPU
  mpu.reset();
  delay(100);
  mpu.resetSensors();
  delay(100);
  mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setSleepEnabled(false);
  mpu.setStandbyXAccelEnabled(false);
  mpu.setStandbyYAccelEnabled(false);
  mpu.setStandbyZAccelEnabled(false);
  mpu.setExternalFrameSync(MPU6050_EXT_SYNC_DISABLED);

  // See https://github.com/ElectronicCats/mpu6050/blob/master/src/MPU6050.cpp
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);
  delay(100);
  mpu.setDHPFMode(MPU6050_DHPF_HOLD);

  mpu.setMotionDetectionThreshold(10);
  mpu.setMotionDetectionDuration(2);
  mpu.setInterruptMode(MPU6050_INTMODE_ACTIVEHIGH);
  mpu.setAccelerometerPowerOnDelay(2); //max
  mpu.setInterruptLatch(MPU6050_INTLATCH_WAITCLEAR);
  mpu.setInterruptLatchClear(MPU6050_INTCLEAR_STATUSREAD);
  mpu.setIntEnabled(1 << MPU6050_INTERRUPT_MOT_BIT);
  mpu.setInterruptDrive(MPU6050_INTDRV_PUSHPULL);
  mpu.setStandbyXAccelEnabled(false);
  mpu.setStandbyYAccelEnabled(false);
  mpu.setStandbyZAccelEnabled(false);
}


void setup() {
  initSerial();
  initI2C();
  initMpu();
  pinMode(LED, OUTPUT);
  pinMode(EXT0_PIN, INPUT);
  digitalWrite(LED, blinkState);
  Serial.println("Initializing MPU...");
 
  /*Enable Arduino interrupt detection*/
  Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
  Serial.println(F(")..."));
  attachInterrupt(digitalPinToInterrupt(EXT0_PIN), DMPDataReady, RISING);
  // attachInterrupt(digitalPinToInterrupt(EXT0_PIN), DMPDataReady, RISING);
  MPUIntStatus = mpu.getIntStatus();
  // mpu.setInterruptLatchClear(true);
  Serial.println(MPUIntStatus);
  // Serial.println(devStatus);
  /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
  Serial.println(F("DMP ready! Waiting for first interrupt..."));
  DMPReady = true;
  // packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  // }
}


void reportIMU() {
  if (!DMPReady) return; // Stop the program if DMP programming fails.
  if (!MPUInterrupt) return;
  blinkState = !blinkState;
  digitalWrite(LED, blinkState);
  MPUIntStatus = mpu.getIntStatus();
  // mpu.setInterruptLatchClear(true);
  // mpu.setInterruptLat
  MPUInterrupt = false;
  return;
  /* Read a packet from FIFO */
  fifoCount = mpu.getFIFOCount();
	// check for overflow (this should never happen unless our code is too inefficient)
	if ((MPUIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.println(F("FIFO overflow!, giro descompensat!!"));
		Serial.print("&");
		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (MPUIntStatus & 0x02) {
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
        mpu.dmpGetQuaternion(&q, FIFOBuffer);
        mpu.dmpGetGravity(&gravity, &q);
    }
  } else if (MPUIntStatus & (1<<MPU6050_INTERRUPT_MOT_BIT)) 
  //mpu.setInterruptLatchClear(true);
  MPUInterrupt = false;
}


void loop() {
  static uint32_t last_frame = 0;
  //lcd_delay(5000);
  //delay(1000);
  if(millis() - last_frame >= 33) {
    reportIMU();
    last_frame = millis();
  }
  // else {
  //  delay(millis() - last_frame);
  //}
}
