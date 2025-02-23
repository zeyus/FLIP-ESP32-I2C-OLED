#include <Arduino.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

// Config
#define PANEL_RES_X 64
#define PANEL_RES_Y 32
#define NUM_PANELS 2
#define SIM_WIDTH (PANEL_RES_X * NUM_PANELS)
#define SIM_HEIGHT PANEL_RES_Y
#define NUM_PARTICLES 100
#define FIXED_SHIFT 8

typedef int16_t fixed_t;
#define TO_FIXED(x) ((fixed_t)((x) * (1 << FIXED_SHIFT)))
#define TO_FLOAT(x) ((float)(x) / (1 << FIXED_SHIFT))

struct Particle {
  fixed_t x, y;
  fixed_t vx, vy;
};

Particle particles[NUM_PARTICLES];
MatrixPanel_I2S_DMA *dma_display;

/*
// Wokwi-compatible pin configuration
HUB75_I2S_CFG::i2s_pins _pins = {
  .r1 = 25, .g1 = 26, .b1 = 27, 
  .r2 = 14, .g2 = 12, .b2 = 13, 
  .a = 23, .b = 19, .c = 5, .d = 17, 
  .e = -1, // Required for 64x64 panels
  .lat = 4, .oe = 15, .clk = 16
};
*/

// ESP32-S3-WROOM-1 HUB75 Pin Mapping
HUB75_I2S_CFG::i2s_pins _pins = {
  .r1 = 1,  .g1 = 2,  .b1 = 3,
  .r2 = 4,  .g2 = 5,  .b2 = 6,
  .a = 7,   .b = 8,   .c = 9,
  .d = 10,  .e = -1,  // 'e' only needed for 64x64 panels
  .lat = 11, .oe = 12, .clk = 13
};

// Color palette
const uint16_t COLORS[] = {
  0x001F, 0x03FF, 0x07FF, 0x7FE0, 0x7F80, 0xFFE0, 0xFD20, 0xF800
};
const int NUM_COLORS = sizeof(COLORS)/sizeof(COLORS[0]);

void setup() {

  HUB75_I2S_CFG mxconfig(
    PANEL_RES_X,
    PANEL_RES_Y,
    NUM_PANELS,
    _pins,
    HUB75_I2S_CFG::FM6126A,
    false,
    HUB75_I2S_CFG::HZ_10M,
    true,
    HUB75_I2S_CFG::SHIFTREG,
    false,
    0
  );

  mxconfig.double_buff = true;
  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  dma_display->begin();
  dma_display->setBrightness(255);

  // Initialize particles
  for (int i = 0; i < NUM_PARTICLES; i++) {
    particles[i].x = TO_FIXED(random(SIM_WIDTH));
    particles[i].y = TO_FIXED(random(SIM_HEIGHT/2));
    particles[i].vx = TO_FIXED((random(-100, 100)/100.0) * 0.7);
    particles[i].vy = TO_FIXED((random(-50, 50)/100.0));
  }
}

void updatePhysics() {
  const fixed_t gravity = TO_FIXED(0.15);
  const fixed_t damping = TO_FIXED(0.82);
  const fixed_t border = TO_FIXED(2.0);

  for (int i = 0; i < NUM_PARTICLES; i++) {
    particles[i].vy += gravity;
    particles[i].x += particles[i].vx;
    particles[i].y += particles[i].vy;

    // Boundary collisions
    if (particles[i].x < border || particles[i].x >= TO_FIXED(SIM_WIDTH) - border) {
      particles[i].vx = -particles[i].vx * damping;
      particles[i].x = constrain(particles[i].x, border, TO_FIXED(SIM_WIDTH) - border);
    }
    if (particles[i].y < border || particles[i].y >= TO_FIXED(SIM_HEIGHT) - border) {
      particles[i].vy = -particles[i].vy * damping;
      particles[i].y = constrain(particles[i].y, border, TO_FIXED(SIM_HEIGHT) - border);
    }
  }
}

void drawParticles() {
  dma_display->fillScreen(0);
  
  for (int i = 0; i < NUM_PARTICLES; i++) {
    int x = constrain(TO_FLOAT(particles[i].x), 0, SIM_WIDTH-1);
    int y = constrain(TO_FLOAT(particles[i].y), 0, SIM_HEIGHT-1);
    
    // Simplified color selection
    uint16_t color = COLORS[(abs(particles[i].vx) + abs(particles[i].vy)) % NUM_COLORS];
    
    dma_display->drawPixel(x, y, color);
  }
}

void loop() {
  static uint32_t last_frame = 0;
  const uint32_t frame_time = 33; // ~30 FPS

  if (millis() - last_frame >= frame_time) {
    updatePhysics();
    drawParticles();
    last_frame = millis();
  }
}
