#include <Arduino.h>
#include <lcdgfx.h>
#include <FastTrig.h>
#include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050_6Axis_MotionApps612.h"

// comment out to disable debug output / serial / led
#define DEBUG
// LED for debugging interrupts
#define LED 17

// sleep params
#define SLEEP_AFTER_MS 10000

// External interrupt source
#define EXT0_PIN 34

// Simulation constants
#define SIM_WIDTH 128
#define SIM_HEIGHT 64
#define NUM_PARTICLES 250
#define PARTICLE_RADIUS 2
#define GRAVITY 1.0f
#define DAMPING 0.3f
#define PRESSURE_RADIUS 3.5f
#define PRESSURE_FORCE 0.9f
#define MAX_VEL 2.0f
#define GRID_SIZE 16
#define CELL_SIZE (SIM_WIDTH/GRID_SIZE)

DisplaySSD1306_128x64_I2C display(-1); 
MPU6050 mpu;

// I2C device found at address 0x3C  ! // OLED
// I2C device found at address 0x68  ! // IMU

typedef float f32 __attribute__((aligned(4)));
struct Particle {
  f32 x, y;
  f32 vx, vy;
};


inline float fast_sqrt(float x) {
  union { float f; uint32_t i; } u;
  u.f = x;
  u.i = 0x5f375a86 - (u.i >> 1);
  return u.f * (1.5f - 0.5f * x * u.f * u.f);
}

Particle particles[NUM_PARTICLES];
uint8_t canvasData[SIM_WIDTH*(SIM_HEIGHT/8)]; // because of 1bit display, not RGB
NanoCanvas1 canvas(SIM_WIDTH, SIM_HEIGHT, canvasData);

/*---Sleep/wake vars---*/
uint32_t lastZMot = 0;
bool zMotInterrupt = false;

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
VectorFloat gravity;    // [x, y, z]            Gravity vector
uint16_t fifoCount;

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = true;     // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

// Setup serial communication
// only for debugging
void initSerial() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
}

// Setup I2C communication for IMU
// would be nice to remove this library
// requirement and somehow 
// reuse the implementation from the OLED
void initI2C() {
  Wire.begin();
  Wire.setClock(400000);
}


void initMpu() {
  zMotInterrupt = false;
  lastZMot = 0;
  // avoid unnecessary resets by checking
  // a non-default value
  if (mpu.getAccelerometerPowerOnDelay() == 2) {
    return;
  }
  mpu.initialize();
  mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
  mpu.CalibrateGyro(6);
  // time taken for accel to settle after power on
  // i think 4 is the starting and this adds additional wait time
  mpu.setAccelerometerPowerOnDelay(2); //max 3
  mpu.setTempSensorEnabled(false);
}

void mpuSetInterruptMode(){
  // set the interrupt to latch until data is read
  // this is nice so on mpu.getIntStatus() it will clear the interrupt
  mpu.setInterruptLatch(MPU6050_INTLATCH_WAITCLEAR);
  mpu.setInterruptLatchClear(MPU6050_INTCLEAR_STATUSREAD);

  // i honeslly don't know what the other versions of this is
  // it's either push-pull or open-drain
  mpu.setInterruptDrive(MPU6050_INTDRV_PUSHPULL);
}

/** mpu setup for motion detection
 * this will be used to wake the ESP32
 * from deep sleep (so when ESP is sleeping, accel is in low-ish power mode)
 */
void mpuMotionDetectMode() {
  // set up MPU
  // docs recommend waiting for at least 50ms after reset
  mpu.reset();
  delay(100);
  mpu.resetSensors();
  delay(100);
  mpu.setAccelerometerPowerOnDelay(2); //max 3
  mpu.setTempSensorEnabled(false);
  // disable fifo when sleeping
  mpu.setFIFOEnabled(false);
  // low pass filter, 42Hz
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);
  delay(10);
  // high pass filter, from current value (should not be done while in motion)
  mpu.setDHPFMode(MPU6050_DHPF_HOLD);

  // mpu.setIntEnabled(0);
  // set interrupt to be high when motion detected
  mpu.setInterruptMode(MPU6050_INTMODE_ACTIVEHIGH);
  // apparently this is more accurate
  // See https://github.com/ElectronicCats/mpu6050/blob/master/src/MPU6050.cpp
  mpu.setClockSource(MPU6050_CLOCK_INTERNAL);
  // lowest accel sens
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  // lowest gyro sens
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setDHPFMode(MPU6050_DHPF_RESET); // reset high-pass filter in prep for hold mode

  // // adjust values from calib script example, if needed
  // mpu.setXGyroOffset(-69);
  // mpu.setYGyroOffset(-48);
  // mpu.setZGyroOffset(-19);
  // mpu.setXAccelOffset(-5158);
  // mpu.setYAccelOffset(-4576);
  // mpu.setZAccelOffset(7687);
  // level of movement required to trigger interrupt
  // recommended default is 20
  mpu.setMotionDetectionThreshold(20);
  // number of milliseconds that the sensor must be at the threshold
  mpu.setMotionDetectionDuration(2);

  // enable only motion detection interrupt
  mpu.setIntEnabled(1 << MPU6050_INTERRUPT_MOT_BIT);
  mpuSetInterruptMode();
  // ensure the accellerometers are on
  // mpu.setStandbyXAccelEnabled(false);
  // mpu.setStandbyYAccelEnabled(false);
  // mpu.setStandbyZAccelEnabled(false);

  // we can sleep the gyro
  mpu.setStandbyXGyroEnabled(true);
  mpu.setStandbyYGyroEnabled(true);
  mpu.setStandbyZGyroEnabled(true);
  
}

/** mpu setup for active monitoring
 * this will be used to monitor the IMU
 * while the ESP32 is awake to run the sim
 */
void mpuActiveMonitorMode() {
  // ensure the gyro is on
  mpu.setStandbyXGyroEnabled(false);
  mpu.setStandbyYGyroEnabled(false);
  mpu.setStandbyZGyroEnabled(false);
  devStatus = mpu.dmpInitialize();
  // disable motion detection interrupt
  // and enable data ready interrupt
  // mpu.setIntEnabled(1 << MPU6050_INTERRUPT_DMP_INT_BIT);
  // mpuSetInterruptMode();
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize(); // Get expected DMP packet size for later comparison
    mpu.setIntEnabled((1 << MPU6050_INTERRUPT_ZMOT_BIT) | (1 << MPU6050_INTERRUPT_DMP_INT_BIT));
    mpu.setZeroMotionDetectionDuration(2);
    mpu.setZeroMotionDetectionThreshold(20);
    DMPReady = true;
  }
#ifdef DEBUG
  else {
    Serial.print(F("DMP Initialization failed (code ")); //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }
#endif
}



struct GridCell {
  uint8_t particles[10];
  uint8_t count;
};

GridCell grid[GRID_SIZE][GRID_SIZE];

#ifdef DEBUG
bool blinkState = true;
#endif

void buildSpatialGrid() {
  memset(grid, 0, sizeof(grid));

  for(int i=0; i<NUM_PARTICLES; i++) {
    #ifdef __AVR__
      float x = TO_FLOAT(particles[i].x);
      float y = TO_FLOAT(particles[i].y);
    #else
      float x = particles[i].x;
      float y = particles[i].y;
    #endif
    
    int gx = constrain(x / CELL_SIZE, 0, GRID_SIZE-1);
    int gy = constrain(y / CELL_SIZE, 0, GRID_SIZE-1);
    
    if(grid[gx][gy].count < 10) {
      grid[gx][gy].particles[grid[gx][gy].count++] = i;
    }
  }
}


void setup() {
  DMPReady = false;
#ifdef DEBUG
  initSerial();
#endif
  initI2C();
  initMpu();
  mpuActiveMonitorMode();
  pinMode(EXT0_PIN, INPUT);

#ifdef DEBUG
  pinMode(LED, OUTPUT);
  digitalWrite(LED, blinkState);
  Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
  Serial.print(digitalPinToInterrupt(EXT0_PIN));
  Serial.println(F(")..."));
#endif
  attachInterrupt(digitalPinToInterrupt(EXT0_PIN), DMPDataReady, RISING);
  MPUIntStatus = mpu.getIntStatus();
  // }
  // setup display
  display.begin();
  display.clear();
  canvas.setMode(CANVAS_MODE_TRANSPARENT);

  // Initialize particles in a droplet pattern
  float cx = SIM_WIDTH/2;
  float cy = SIM_HEIGHT/4;
  for(int i=0; i<NUM_PARTICLES; i++) {
    float angle = random(360) * PI / 180.0;
    float radius = random(10);
    particles[i].x = cx + icos(angle) * radius;
    particles[i].y = cy + isin(angle) * radius;
    particles[i].vx = random(-50,50)/25.0;  // -2 to +2
    particles[i].vy = random(-25,50)/25.0;   // -1 to +2
  }
}

void applyPhysics() {
  // Build spatial grid
  buildSpatialGrid();

  // Interactions using grid
  const float PRESSURE_RADIUS_SQ = PRESSURE_RADIUS * PRESSURE_RADIUS;
  
  for(int i=0; i<NUM_PARTICLES; i++) {
    const int gx = particles[i].x / CELL_SIZE;
    const int gy = particles[i].y / CELL_SIZE;

    // Check 3x3 grid around particle
    for(int dx=-1; dx<=1; dx++) {
      for(int dy=-1; dy<=1; dy++) {
        if(gx+dx < 0 || gx+dx >= GRID_SIZE) continue;
        if(gy+dy < 0 || gy+dy >= GRID_SIZE) continue;
        
        GridCell &cell = grid[gx+dx][gy+dy];
        for(int c=0; c<cell.count; c++) {
          const int j = cell.particles[c];
          if(j <= i) continue; // Avoid duplicate pairs

          const float dx = particles[j].x - particles[i].x;
          const float dy = particles[j].y - particles[i].y;
          const float dist_sq = dx*dx + dy*dy;

          if(dist_sq < PRESSURE_RADIUS_SQ && dist_sq > 0.01f) {
            const float dist = fast_sqrt(dist_sq) + 0.001f;
            const float force = PRESSURE_FORCE * (1.0f - dist/PRESSURE_RADIUS);
            
            // Only apply horizontal forces to preserve gravity
            particles[i].vx -= force * dx/dist;
            particles[j].vx += force * dx/dist;
            
            // Reduce vertical force impact
            particles[i].vy -= force * dy/dist * 0.3f;
            particles[j].vy += force * dy/dist * 0.3f;
          }
        }
      }
    }
  }

    // Gravity and movement
  for(int i=0; i<NUM_PARTICLES; i++) {
    particles[i].vy += gravity.x * GRAVITY; // x / y flipped
    particles[i].vx += gravity.y * GRAVITY;
    particles[i].x += particles[i].vx;
    particles[i].y += particles[i].vy;
    particles[i].vx = constrain(particles[i].vx, -MAX_VEL, MAX_VEL);
    particles[i].vy = constrain(particles[i].vy, -MAX_VEL, MAX_VEL);
    // X-axis
    if(particles[i].x <= 0 || particles[i].x >= SIM_WIDTH-1) {
      particles[i].vx *= -DAMPING;
      particles[i].x = constrain(particles[i].x, 1, SIM_WIDTH-2);
    }
    
    // Y-axis
    if(particles[i].y <= 0 || particles[i].y >= SIM_HEIGHT-1) {
      particles[i].vy *= -DAMPING;
      particles[i].y = constrain(particles[i].y, 1, SIM_HEIGHT-2);
    }
  }
}

// Direct canvas buffer access (replace drawParticles)
void drawParticles() {
  // Clear canvas by direct memory access
  memset(canvasData, 0, sizeof(canvasData));
  // canvas.clear();
  for(int i=0; i<NUM_PARTICLES; i++) {
      int x = constrain(static_cast<int>(particles[i].x + 0.5f), 0, SIM_WIDTH-1);
      int y = constrain(static_cast<int>(particles[i].y + 0.5f), 0, SIM_HEIGHT-1);

    #if PARTICLE_RADIUS == 1
      canvas.putPixel(x, y);
    #else
      canvas.drawCircle(x,y,PARTICLE_RADIUS-1);
    #endif
  }
  
  display.drawCanvas(0,0,canvas);
}

void reportIMU() {
  if (!DMPReady) return; // Stop the program if DMP programming fails.
  if (!MPUInterrupt) return;
  MPUIntStatus = mpu.getIntStatus();
#ifdef DEBUG
  Serial.println(MPUIntStatus);
#endif
  /* Read a packet from FIFO */
  fifoCount = mpu.getFIFOCount();
	// check for overflow (this should never happen unless our code is too inefficient)
	if ((MPUIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
#ifdef DEBUG
		Serial.println(F("FIFO overflow, reseting FIFO"));
#endif
	} else if (MPUIntStatus & (1 << MPU6050_INTERRUPT_ZMOT_BIT)) {
    // zero motion interrupt
    // this is used if the device is placed somewhere or stops moving 
    // then we initialize the countdown to sleep
    lastZMot = millis();
    zMotInterrupt = true;
  }

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
	else if (MPUIntStatus & 0x02) {
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
        mpu.dmpGetQuaternion(&q, FIFOBuffer);
        mpu.dmpGetGravity(&gravity, &q); // this is all we care about right now
    }
  }
#ifdef DEBUG
// change LED state
  blinkState = !blinkState;
  digitalWrite(LED, blinkState);
#endif
  // reset interrupt flag
  MPUInterrupt = false;
}

void sleepTimer() {
  if (zMotInterrupt && millis() - lastZMot > SLEEP_AFTER_MS) {
#ifdef DEBUG
    Serial.println(F("Going to sleep"));
#endif
    // detach interrupt
    detachInterrupt(digitalPinToInterrupt(EXT0_PIN));
    // sleep
    mpuMotionDetectMode();
    // clear any pending interrupts
    mpu.getIntStatus();
    // enable wakeup from ext0
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 1);
    esp_deep_sleep_start();
  }
}

void loop() {
  static uint32_t last_frame = 0;
  drawParticles();
  reportIMU();
  if(millis() - last_frame >= 33) {
    sleepTimer();
    last_frame = millis();
    applyPhysics();
  } else {
    lcd_delay(millis() - last_frame);
  }
}
