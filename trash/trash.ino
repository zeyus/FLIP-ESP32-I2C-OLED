#include <Arduino.h>
#include <lcdgfx.h>
#include <FastTrig.h>
#include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050_6Axis_MotionApps612.h"

DisplaySSD1306_128x64_I2C display(-1); 
MPU6050 mpu;

// Simulation constants
#define SIM_WIDTH 128
#define SIM_HEIGHT 64
#define NUM_PARTICLES 1000
#define PARTICLE_RADIUS 2
#define GRAVITY 1.0f
#define DAMPING 0.4f
#define PRESSURE_RADIUS 3.5f
#define PRESSURE_FORCE 0.9f
#define MAX_VEL 1.0f
#define IMU_ADDRESS 0x68
#define OUTPUT_READABLE_YAWPITCHROLL
#define LED 2

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


#define GRID_SIZE 16
#define CELL_SIZE (SIM_WIDTH/GRID_SIZE)

int const INTERRUPT_PIN = 18;
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



struct GridCell {
  uint8_t particles[10];
  uint8_t count;
};

GridCell grid[GRID_SIZE][GRID_SIZE];

bool blinkState;

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
  // String str; // Hhack
  Serial.begin(38400);
  while (!Serial) {
    ;
  }
  Serial.println("Initializing MPU...");
  /*--Start I2C interface--*/
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(); 
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  mpu.initialize();
  
  Serial.println("Testing MPU6050 connection...");
  // if(mpu.testConnection() ==  false){
  //   Serial.println("MPU6050 connection failed");
  //   while(true);
  // }
  // else{
  devStatus = mpu.dmpInitialize();
  Serial.println("MPU6050 connection successful");
  mpu.setAccelerometerPowerOnDelay(3);
  mpu.setSleepEnabled(false);
  mpu.setStandbyXAccelEnabled(false);
  mpu.setStandbyYAccelEnabled(false);
  mpu.setStandbyZAccelEnabled(false);
  mpu.setDHPFMode(0);
  mpu.setInterruptLatch(false);
  mpu.setIntEnabled(true);
  mpu.setInterruptMode(true);
  mpu.setIntMotionEnabled(true);
  
  mpu.setMotionDetectionDuration(1);
  mpu.setMotionDetectionThreshold(20);
  // mpu.setIntDMPEnabled(true);


  mpu.setXGyroOffset(-69);
  mpu.setYGyroOffset(-48);
  mpu.setZGyroOffset(-19);
  mpu.setXAccelOffset(-5158);
  mpu.setYAccelOffset(-4576);
  mpu.setZAccelOffset(7687);

  mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
  mpu.CalibrateGyro(6);
  Serial.println("These are the Active offsets: ");
  mpu.PrintActiveOffsets();
  Serial.println(F("Enabling DMP..."));   //Turning ON DMP
  mpu.setDMPEnabled(true);
  
  // mpu.setWakeCycleEnabled(true);

  delay(5);

  mpu.setDHPFMode(7);
  // mpu.setWakeFrequency(5);
  // mpu.setWakeCycleEnabled(true);
  // mpu.setStandbyXAccelEnabled(true);
  // mpu.setStandbyYAccelEnabled(true);
  // mpu.setStandbyZAccelEnabled(true);
  // mpu.setIntDMPEnabled(true);
  // mpu.setSleepEnabled(true);
 

  /*Enable Arduino interrupt detection*/
  Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
  Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
  Serial.println(F(")..."));
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
  MPUIntStatus = mpu.getIntStatus();
  Serial.println(MPUIntStatus);
  Serial.println(devStatus);
  /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
  Serial.println(F("DMP ready! Waiting for first interrupt..."));
  DMPReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
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
    //Serial.print("\nParticle ");
    //Serial.print(i);
    //Serial.print(" done");
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
    particles[i].vy += gravity.y * GRAVITY;
    particles[i].vx += gravity.x * GRAVITY;
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
  }
  MPUInterrupt = false;
}


void loop() {
  static uint32_t last_frame = 0;
  //lcd_delay(5000);
  drawParticles();
  reportIMU();
  //delay(1000);
  if(millis() - last_frame >= 33) {
    last_frame = millis();
    applyPhysics();
  } else {
    lcd_delay(millis() - last_frame);
  }
}
