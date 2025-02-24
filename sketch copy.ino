#include <lcdgfx.h>
#include <nano_engine_v2.h>
#include <nano_gfx_types.h>

#define BAUD_RATE   115200
#include <Arduino.h>
#include <lcdgfx.h>
// The parameters are  RST pin, BUS number, CS pin, DC pin, FREQ (0 means default), CLK pin, MOSI pin
//DisplaySSD1325_128x64_SPI display(3,{-1, 4, 5, 0,-1,-1});   // Use this line for Atmega328p (3=RST, 4=CE, 5=D/C)
//DisplaySSD1325_128x64_I2C display(-1);                    // or (-1,{busId, addr, scl, sda, frequency}). This line is suitable for most platforms by default
//DisplaySSD1325_128x64_SPI display(22,{-1, 5, 21, 0,-1,-1}); // Use this line for ESP32 (VSPI)  (gpio22=RST, gpio5=CE for VSPI, gpio21=D/C)
//DisplaySSD1325_128x64_SPI display(4,{-1, -1, 5, 0,-1,-1});  // Use this line for ESP8266 Arduino style rst=4, CS=-1, DC=5
                                                          // And ESP8266 RTOS IDF. GPIO4 is D2, GPIO5 is D1 on NodeMCU boards


DisplaySSD1306_128x64_I2C display(-1);

#define PANEL_RES_X 128
#define PANEL_RES_Y 64
#define SIM_WIDTH (PANEL_RES_X)
#define SIM_HEIGHT PANEL_RES_Y
#define NUM_PARTICLES 255
#define FIXED_SHIFT 8

typedef int16_t fixed_t;
#define TO_FIXED(x) ((fixed_t)((x) * (1 << FIXED_SHIFT)))
#define TO_FLOAT(x) ((float)(x) / (1 << FIXED_SHIFT))

struct Particle {
  fixed_t x, y;
  fixed_t vx, vy;
};
Particle particles[NUM_PARTICLES];

const int canvasWidth = SIM_WIDTH; // Width must be power of 2, i.e. 16, 32, 64, 128...
const int canvasHeight = SIM_HEIGHT; // Height must be divided on 8, i.e. 8, 16, 24, 32...
const fixed_t gravity = TO_FIXED(0.4);
const fixed_t damping = TO_FIXED(0.6);
const fixed_t border = TO_FIXED(0.1);
const uint32_t frame_time = 33; // ~30 FPS

uint8_t canvasData[canvasWidth*(canvasHeight/8)];
NanoCanvas1 canvas(canvasWidth, canvasHeight, canvasData);

void updatePhysics() {
  const fixed_t max_x = TO_FIXED(SIM_WIDTH-1);
  const fixed_t max_y = TO_FIXED(SIM_HEIGHT-1);

  for (int i = 0; i < NUM_PARTICLES; i++) {
    particles[i].vy += gravity;
    particles[i].x += particles[i].vx;
    particles[i].y += particles[i].vy;

    particles[i].x = constrain(particles[i].x, TO_FIXED(0), max_x);
    particles[i].y = constrain(particles[i].y, TO_FIXED(0), max_y);

    // Boundary collisions
    if (TO_FLOAT(particles[i].x) <= TO_FLOAT(border) || TO_FLOAT(particles[i].x) >= (SIM_WIDTH - TO_FLOAT(border))) {
      particles[i].vx = -particles[i].vx * damping;
      particles[i].x = constrain(particles[i].x, border, TO_FIXED(SIM_WIDTH) - border);
    }
    if (TO_FLOAT(particles[i].y) < TO_FLOAT(border) || TO_FLOAT(particles[i].y) >= (SIM_HEIGHT - TO_FLOAT(border))) {
      particles[i].vy = -particles[i].vy * damping;
      particles[i].y = constrain(particles[i].y, border, TO_FIXED(SIM_HEIGHT) - border);
    }

  }
}

void drawParticles() {
  canvas.clear();
  for (int i = 0; i < NUM_PARTICLES; i++) {
    int x = constrain(TO_FLOAT(particles[i].x), 0, SIM_WIDTH-1);
    int y = constrain(TO_FLOAT(particles[i].y), 0, SIM_HEIGHT-1);
    
    canvas.putPixel(x, y);
  }
  display.drawCanvas(0,0,canvas);
}

// Setup, initialize 
void setup() 
{
  //Serial.begin(115200);
  display.begin();
  display.clear();
  canvas.setMode( CANVAS_MODE_TRANSPARENT );
  // display.setColor( 65535 );
  
  // Initialize particles
  for (int i = 0; i < NUM_PARTICLES; i++) {
    particles[i].x = TO_FIXED(random(SIM_WIDTH-1));
    particles[i].y = TO_FIXED(random(SIM_HEIGHT/2));
    particles[i].vx = TO_FIXED((random(-100, 100)/100.0) * 0.7);
    particles[i].vy = TO_FIXED((random(-50, 50)/100.0));
  }
}

// Loop forever
void loop() 
{
  // static uint32_t last_frame = 0;
  
  
  //if (millis() - last_frame >= frame_time) {
    //Serial.print(display.getColor());
    //Serial.print("Hi");  
    updatePhysics();
    drawParticles();
    //last_frame = millis(); 
  //}
  lcd_delay(frame_time);
}
