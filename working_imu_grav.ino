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

/**
 * Class implements SSD1306 128x64 lcd display in 1 bit mode over I2C
 * overrides default implementation to expose private m_i2c
 */
 class DisplaySSD1306_128x64_I2Cx: public DisplaySSD1306_128x64<InterfaceSSD1306<PlatformI2c>>
 {
 public:
     /**
      * @brief Inits 128x64 lcd display over i2c (based on SSD1306 controller): 1-bit mode.
      *
      * Inits 128x64 lcd display over i2c (based on SSD1306 controller): 1-bit mode
      * @param rstPin pin controlling LCD reset (-1 if not used)
      * @param config platform i2c configuration. Please refer to SPlatformI2cConfig.
      */
     explicit DisplaySSD1306_128x64_I2Cx(int8_t rstPin, const SPlatformI2cConfig &config = {-1, 0x3C, -1, -1, 0})
         : DisplaySSD1306_128x64(m_i2c, rstPin)
         , m_i2c(*this, -1,
                 SPlatformI2cConfig{config.busId, static_cast<uint8_t>(config.addr ?: 0x3C), config.scl, config.sda,
                                    config.frequency ?: 400000})
     {
     }
 
     /**
      * Initializes SSD1306 lcd in 1-bit mode
      */
     void begin() override;
 
     /**
      * Closes connection to display
      */
     void end() override;
     InterfaceSSD1306<PlatformI2c> m_i2c;
 };
 void DisplaySSD1306_128x64_I2Cx::begin()
 {
     m_i2c.begin();
     DisplaySSD1306_128x64::begin();
 }
 
 void DisplaySSD1306_128x64_I2Cx::end()
 {
     DisplaySSD1306_128x64::end();
     m_i2c.end();
 }
DisplaySSD1306_128x64_I2Cx display(-1); 
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
VectorInt16 rawAccel; // scaled
VectorInt16 lastAccel = {0, 0, 0};
VectorInt16 accelDelta = {0, 0, 0};
float accelMagnitude = 0;
float lastAccelMagnitude = 0;
float maxAccelImpulse = 0;

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
  display.m_i2c.displayOn();
  display.m_i2c.setContrast(0);
  
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
  // mpu.setInterruptDrive(MPU6050_INTDRV_PUSHPULL);
  mpu.setInterruptMode(MPU6050_INTMODE_ACTIVELOW);
}

/** mpu setup for motion detection
 * this will be used to wake the ESP32
 * from deep sleep (so when ESP is sleeping, accel is in low-ish power mode)
 */
void mpuMotionDetectMode() {
  // Clear any existing settings
  mpu.reset();
  delay(100);
  mpu.initialize();
  
  // Configure motion detection
  mpu.setDLPFMode(MPU6050_DLPF_BW_5); // Even stronger filtering
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setMotionDetectionThreshold(5); // Lower threshold (more sensitive)
  mpu.setMotionDetectionDuration(1);  // Longer duration
  
  // Set interrupt to ACTIVE LOW for deep sleep wake
  mpu.setInterruptMode(MPU6050_INTMODE_ACTIVELOW);
  mpu.setInterruptLatch(true); // stay interrupted until read
  mpu.setMotionDetectionCounterDecrement(MPU6050_DETECT_DECREMENT_1);
  // Enable only motion detection interrupt
  mpu.setIntEnabled(1 << MPU6050_INTERRUPT_MOT_BIT);
  
  
  // Enable accelerometers, disable gyros for power saving
  mpu.setStandbyXAccelEnabled(false);
  mpu.setStandbyYAccelEnabled(false);
  mpu.setStandbyZAccelEnabled(false);
  mpu.setStandbyXGyroEnabled(true);
  mpu.setStandbyYGyroEnabled(true);
  mpu.setStandbyZGyroEnabled(true);
  
  // Force a read of the interrupt status to clear it
  mpu.getIntStatus();
}

/** mpu setup for active monitoring
 * this will be used to monitor the IMU
 * while the ESP32 is awake to run the sim
 */
void mpuActiveMonitorMode() {
  // ensure the gyro is on
  mpu.setWakeCycleEnabled(false);
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
    int gx = constrain(particles[i].x / CELL_SIZE, 0, GRID_SIZE-1);
    int gy = constrain(particles[i].y / CELL_SIZE, 0, GRID_SIZE-1);
    
    if(grid[gx][gy].count < 10) {
      grid[gx][gy].particles[grid[gx][gy].count++] = i;
    }
  }
}

void drawHeart() {
  canvas.clear();
  
  // Heart coordinates
  const uint8_t heartX = SIM_WIDTH / 2;
  const uint8_t heartY = SIM_HEIGHT / 2;
  const uint8_t heartSize = 25;
  
  // Draw heart shape
  for(float t = 0; t < 2*PI; t += 0.01) {
    float x = 16 * pow(sin(t), 3);
    float y = 13 * cos(t) - 5 * cos(2*t) - 2 * cos(3*t) - cos(4*t);
    // Scale and position
    x = x * heartSize / 16 + heartX;
    y = -y * heartSize / 16 + heartY;
    canvas.putPixel(x, y);
  }
  
  // Add "Renee" text in the middle
  canvas.setFixedFont(ssd1306xled_font6x8);
  canvas.printFixed(heartX - 15, heartY - 3, "Renee", STYLE_NORMAL);
  
  // Display and pause
  display.drawCanvas(0, 0, canvas);
  // lcd_delay(2000);  // Show for 2 seconds
}

void setup() {
  DMPReady = false;
#ifdef DEBUG
  initSerial();
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Woke up due to external signal using RTC_IO");
  } else {
    Serial.println("Woke up for other reason");
    Serial.println(wakeup_reason);
  }
#endif
  initI2C();
  // setup display
  display.begin();
  display.clear();
  canvas.setMode(CANVAS_MODE_TRANSPARENT);
  drawHeart();
  initMpu();
  // mpuMotionDetectMode();
  // DMPReady = true;
  mpuActiveMonitorMode();
  pinMode(EXT0_PIN, INPUT);

#ifdef DEBUG
  pinMode(LED, OUTPUT);
  digitalWrite(LED, blinkState);
  Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
  Serial.print(digitalPinToInterrupt(EXT0_PIN));
  Serial.println(F(")..."));
#endif
  attachInterrupt(digitalPinToInterrupt(EXT0_PIN), DMPDataReady, FALLING);
  MPUIntStatus = mpu.getIntStatus();
  // }
  

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

  float baseGravity = GRAVITY;
  float accelBoost = constrain(maxAccelImpulse / 5000.0f, 0.0f, 2.0f);

  // Decay the impulse over time
  maxAccelImpulse *= 0.95f;
  // Apply external forces from accelerometer
  float accelForceX = constrain(rawAccel.x / 50.0f, -1.0f, 1.0f);
  float accelForceY = constrain(rawAccel.y / 50.0f, -1.0f, 1.0f);
  float impulseX = constrain(accelDelta.x / 1000.0f, -2.0f, 2.0f);
  float impulseY = constrain(accelDelta.y / 1000.0f, -2.0f, 2.0f);

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
    particles[i].vy += gravity.x * baseGravity; 
    particles[i].vx += gravity.y * baseGravity;

    // Add accelerometer impulse (decays naturally)
    float impulseX = gravity.y * accelBoost * 2.0f;
    float impulseY = gravity.x * accelBoost * 2.0f;

    // particles[i].vx += accelForceX * 0.8f;
    // particles[i].vy += accelForceY * 0.8f;
    particles[i].vx += impulseX;
    particles[i].vy += impulseY;
    if (accelBoost > 0.5f) {
      particles[i].vx += (random(100) - 50) / 500.0f * accelBoost;
      particles[i].vy += (random(100) - 50) / 500.0f * accelBoost;
    }
    particles[i].x += particles[i].vx;
    particles[i].y += particles[i].vy;
    particles[i].vx = constrain(particles[i].vx, -MAX_VEL, MAX_VEL);
    particles[i].vy = constrain(particles[i].vy, -MAX_VEL, MAX_VEL);

    // Apply stronger damping during high movement
    float adaptiveDamping = DAMPING * (1.0f - accelBoost * 0.2f);
    // X-axis
    if(particles[i].x <= 0 || particles[i].x >= SIM_WIDTH-1) {
      particles[i].vx *= -adaptiveDamping;
      particles[i].x = constrain(particles[i].x, 1, SIM_WIDTH-2);
    }
    
    // Y-axis
    if(particles[i].y <= 0 || particles[i].y >= SIM_HEIGHT-1) {
      particles[i].vy *= -adaptiveDamping;
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
        mpu.dmpGetAccel(&aa, FIFOBuffer);
        rawAccel.x = aa.x / 2048; // Scale to reasonable values
        rawAccel.y = aa.y / 2048;
        rawAccel.z = aa.z / 2048;
        accelDelta.x = aa.x - lastAccel.x;
        accelDelta.y = aa.y - lastAccel.y;
        accelDelta.z = aa.z - lastAccel.z;
        lastAccel = aa;
         // Calculate magnitude of acceleration for impulse detection
        lastAccelMagnitude = accelMagnitude;
        accelMagnitude = sqrt(aa.x*aa.x + aa.y*aa.y + aa.z*aa.z);
        
        // Detect sudden changes (ignoring gravity component of ~16384)
        float accelDelta = abs(accelMagnitude - lastAccelMagnitude);
        maxAccelImpulse = max(maxAccelImpulse * 0.9f, accelDelta);
    }
  }
  else {
    //it's another interrupt
#ifdef DEBUG
    // change LED state
    blinkState = !blinkState;
    digitalWrite(LED, blinkState);
#endif
  }
  // reset interrupt flag
  MPUInterrupt = false;
}

void goToSleep() {
  display.m_i2c.displayOff();
  // detach interrupt
  // detachInterrupt(digitalPinToInterrupt(EXT0_PIN));
  // Prepare MPU6050 for motion detection
  mpuMotionDetectMode();
  
  // Force a read to clear any pending interrupts
  mpu.getIntStatus();
  
  // Add a brief delay to ensure MPU settles
  delay(100);
  
  // Configure the wake-up source (active LOW)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 0);
  
  // Debug message
#ifdef DEBUG
  Serial.println("Going to deep sleep now");
  Serial.flush();
#endif
  
  // Enter deep sleep
  esp_deep_sleep_start();
}

void sleepTimer() {
  if (zMotInterrupt && millis() - lastZMot > SLEEP_AFTER_MS) {
    goToSleep();
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
