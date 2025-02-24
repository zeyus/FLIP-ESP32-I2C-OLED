#include <Arduino.h>
#include <lcdgfx.h>
#include <FastTrig.h>

// this is working on the mega

DisplaySSD1306_128x64_I2C display(-1); 

// Simulation constants
#define SIM_WIDTH 128
#define SIM_HEIGHT 64
#define NUM_PARTICLES 80
#define PARTICLE_RADIUS 2
#define GRAVITY 0.9f
#define DAMPING 0.92f
#define PRESSURE_RADIUS 4.5f
#define PRESSURE_FORCE 0.6f
#define MAX_VEL 10.0f

#ifdef __AVR__
  typedef int16_t fixed_t;
  #define FIX_SHIFT 6
  #define TO_FIXED(x) ((fixed_t)((x) * (1<<FIX_SHIFT)))
  #define TO_FLOAT(x) ((x) / (1<<FIX_SHIFT))
  
  struct Particle {
    fixed_t x, y;
    fixed_t vx, vy;
  };
  
#else
  typedef float f32 __attribute__((aligned(4)));
  struct Particle {
    f32 x, y;
    f32 vx, vy;
  };
#endif

inline float fast_sqrt(float x) {
  union { float f; uint32_t i; } u;
  u.f = x;
  u.i = 0x5f375a86 - (u.i >> 1);
  return u.f * (1.5f - 0.5f * x * u.f * u.f);
}

Particle particles[NUM_PARTICLES];
uint8_t canvasData[SIM_WIDTH*(SIM_HEIGHT/8)];
NanoCanvas1 canvas(SIM_WIDTH, SIM_HEIGHT, canvasData);


#define GRID_SIZE 16
#define CELL_SIZE (SIM_WIDTH/GRID_SIZE)

struct GridCell {
  uint8_t particles[10];
  uint8_t count;
};

GridCell grid[GRID_SIZE][GRID_SIZE];

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
  display.begin();
  display.clear();
  canvas.setMode(CANVAS_MODE_TRANSPARENT);
  
  // Serial.begin(115200);
  // while (!Serial);

  // Initialize particles in a droplet pattern
  float cx = SIM_WIDTH/2;
  float cy = SIM_HEIGHT/4;

  #ifdef __AVR__
    for(int i=0; i<NUM_PARTICLES; i++) {
      float angle = random(360) * PI / 180.0f;
      float radius = random(10);
      
      float x_offset = icos(angle) * radius; 
      float y_offset = isin(angle) * radius;

      // Constrain BEFORE fixed-point conversion
      float xpos = constrain(cx + x_offset, 0.0f, SIM_WIDTH-1.0f);
      float ypos = constrain(cy + y_offset, 0.0f, SIM_HEIGHT-1.0f);
      
      particles[i].x = TO_FIXED(xpos);
      particles[i].y = TO_FIXED(ypos);

      //Serial.print(str+"Particle "+i+": (angle)"+angle+", (radius)"+radius+" (offsets)"+x_offset+","+y_offset+" -> (x,y)"+particles[i].x+","+particles[i].y+"\n");
      
      // Ensure velocities are within range
      particles[i].vx = TO_FIXED(constrain((random(-50,50)/25.0f), -2.0f, 2.0f));
      particles[i].vy = TO_FIXED(constrain((random(-25,50)/25.0f), -1.0f, 2.0f));
    }
  #else
    for(int i=0; i<NUM_PARTICLES; i++) {
      float angle = random(360) * PI / 180.0;
      float radius = random(10);
      particles[i].x = cx + icos(angle) * radius;
      particles[i].y = cy + isin(angle) * radius;
      particles[i].vx = random(-50,50)/25.0;  // -2 to +2
      particles[i].vy = random(-25,50)/25.0;   // -1 to +2
    }
  #endif
}

void applyPhysics() {
#ifdef __AVR__
  // Fixed-point implementation
  const fixed_t gravity = TO_FIXED(GRAVITY);
  const fixed_t damping = TO_FIXED(DAMPING);
  const fixed_t pressure_radius = TO_FIXED(PRESSURE_RADIUS);
  const fixed_t max_vel = TO_FIXED(MAX_VEL);

  // Convert spatial grid to fixed-point
  buildSpatialGrid();
  const int32_t pressure_radius_sq = ((int32_t)pressure_radius * pressure_radius) >> FIX_SHIFT;

  // Particle interactions
  for(int i=0; i<NUM_PARTICLES; i++) {
    const int gx = TO_FLOAT(particles[i].x) / CELL_SIZE;
    const int gy = TO_FLOAT(particles[i].y) / CELL_SIZE;

    for(int dx=-1; dx<=1; dx++) {
      for(int dy=-1; dy<=1; dy++) {
        if(gx+dx < 0 || gx+dx >= GRID_SIZE) continue;
        if(gy+dy < 0 || gy+dy >= GRID_SIZE) continue;
        
        GridCell &cell = grid[gx+dx][gy+dy];
        for(int c=0; c<cell.count; c++) {
          const int j = cell.particles[c];
          if(j <= i) continue;

          const fixed_t dx = particles[j].x - particles[i].x;
          const fixed_t dy = particles[j].y - particles[i].y;
          const fixed_t dist_sq = (dx*dx + dy*dy) >> FIX_SHIFT;

          if(dist_sq < pressure_radius_sq && dist_sq > TO_FIXED(0.01f)) {
            // Convert to float for precise calculation
            float fx = TO_FLOAT(dx);
            float fy = TO_FLOAT(dy);
            float dist = sqrt(fx*fx + fy*fy) + 0.001f;
            
            // Normalize direction vector
            float nx = fx / dist;
            float ny = fy / dist;
            
            // Calculate force magnitude
            float force = PRESSURE_FORCE * (1.0f - dist/TO_FLOAT(pressure_radius));
            
            // Apply force to both particles
            float fx_impulse = force * nx;
            float fy_impulse = force * ny * 0.7f;  // Reduce vertical effect
            
            particles[i].vx -= TO_FIXED(fx_impulse);
            particles[i].vy -= TO_FIXED(fy_impulse);
            particles[j].vx += TO_FIXED(fx_impulse); 
            particles[j].vy += TO_FIXED(fy_impulse);
          }
        }
      }
    }
  }

  // Physics update with constraints
  for(int i=0; i<NUM_PARTICLES; i++) {
    // Apply gravity
    particles[i].vy += gravity;

    // Update position
    particles[i].x += particles[i].vx;
    particles[i].y += particles[i].vy;

    // Velocity constraints
    particles[i].vx = constrain(particles[i].vx, -TO_FIXED(MAX_VEL*2), TO_FIXED(MAX_VEL*2));
    particles[i].vy = constrain(particles[i].vy, -TO_FIXED(MAX_VEL*2), TO_FIXED(MAX_VEL*2));
    // Simple floor collision
    // if(TO_FLOAT(particles[i].y) > SIM_HEIGHT-1) {
    //   particles[i].y = TO_FIXED(SIM_HEIGHT-1);
    //   particles[i].vy = -particles[i].vy * damping;
    // }
    // Boundary checks
    const float x = TO_FLOAT(particles[i].x);
    const float y = TO_FLOAT(particles[i].y);
    
    if(x <= 0 || x >= SIM_WIDTH-1) {
      particles[i].vx = (-damping * particles[i].vx) >> FIX_SHIFT;
      particles[i].x = TO_FIXED(constrain(x, 1, SIM_WIDTH-2));
    }
    
    if(y <= 0 || y >= SIM_HEIGHT-1) {
      particles[i].vy = (-damping * particles[i].vy) >> FIX_SHIFT;
      particles[i].y = TO_FIXED(constrain(y, 1, SIM_HEIGHT-2));
    }
  }
#else
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
    particles[i].vy += GRAVITY;
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
#endif
}

// Direct canvas buffer access (replace drawParticles)
void drawParticles() {
  // Clear canvas by direct memory access
  memset(canvasData, 0, sizeof(canvasData));
  // canvas.clear();
  // bool occupied[SIM_WIDTH][SIM_HEIGHT] = {false};
  for(int i=0; i<NUM_PARTICLES; i++) {
    #ifdef __AVR__
      int x = static_cast<int>(TO_FLOAT(particles[i].x) + 0.5f);
      int y = static_cast<int>(TO_FLOAT(particles[i].y) + 0.5f);
    #else
      int x = constrain(static_cast<int>(particles[i].x + 0.5f), 0, SIM_WIDTH-1);
      int y = constrain(static_cast<int>(particles[i].y + 0.5f), 0, SIM_HEIGHT-1);
    #endif

    #if PARTICLE_RADIUS == 1
      canvas.putPixel(x, y);
    #else
      canvas.drawCircle(x,y,PARTICLE_RADIUS);
    #endif
    // if(!occupied[x][y]) {
    //   canvas.putPixel(x, y);
    //   occupied[x][y] = true;
      
    //   // Add neighbor pixels if using 2x2
    //   if(x < SIM_WIDTH-1 && !occupied[x+1][y]) {
    //     canvas.putPixel(x+1, y);
    //     occupied[x+1][y] = true;
    //   }
    //   if(y < SIM_HEIGHT-1 && !occupied[x][y+1]) {
    //     canvas.putPixel(x, y+1);
    //     occupied[x][y+1] = true;
    //   }
    // }
  }
  
  display.drawCanvas(0,0,canvas);
}


void loop() {
  static uint32_t last_frame = 0;
  //lcd_delay(5000);
  drawParticles();
  //delay(1000);
  if(millis() - last_frame >= 33) {
    last_frame = millis();
    applyPhysics();
  } else {
    lcd_delay(millis() - last_frame);
  }
}
