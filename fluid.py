import time
import random
import sys

# Simulation constants
SIM_WIDTH = 128
SIM_HEIGHT = 64
NUM_PARTICLES = 200
GRAVITY = 0.5
DAMPING = 0.82
BORDER = 2.0
FRAME_DELAY = 0.033  # ~30 FPS

class Particle:
    def __init__(self):
        self.x = random.uniform(BORDER, SIM_WIDTH - BORDER)
        self.y = random.uniform(BORDER, SIM_HEIGHT/2 - BORDER)
        self.vx = random.uniform(-1, 1) * 0.7
        self.vy = random.uniform(-0.5, 0.5)

def initialize_particles(num_particles):
    return [Particle() for _ in range(num_particles)]

def update_physics(particles):
    for p in particles:
        # Apply gravity
        p.vy += GRAVITY
        
        # Update position
        p.x += p.vx
        p.y += p.vy

        # Horizontal boundary collisions
        if p.x < BORDER or p.x >= SIM_WIDTH - BORDER:
            p.vx *= -DAMPING
            p.x = max(BORDER, min(p.x, SIM_WIDTH - BORDER - 0.1))

        # Vertical boundary collisions
        if p.y < BORDER or p.y >= SIM_HEIGHT - BORDER:
            p.vy *= -DAMPING
            p.y = max(BORDER, min(p.y, SIM_HEIGHT - BORDER - 0.1))

def draw_frame(particles):
    # Initialize empty grid
    grid = [[' ' for _ in range(SIM_WIDTH)] for _ in range(SIM_HEIGHT)]
    
    # Plot particles
    for p in particles:
        x = int(p.x)
        y = int(p.y)
        if 0 <= x < SIM_WIDTH and 0 <= y < SIM_HEIGHT:
            grid[y][x] = 'o'
    
    # Build frame buffer
    buffer = []
    for row in grid:
        buffer.append(''.join(row))
    
    # Clear screen and move cursor to top-left
    sys.stdout.write('\033[H\033[J')
    sys.stdout.write('\n'.join(buffer))
    sys.stdout.flush()

def main():
    particles = initialize_particles(NUM_PARTICLES)
    
    # Hide cursor
    sys.stdout.write('\033[?25l')
    sys.stdout.flush()

    try:
        while True:
            start_time = time.monotonic()
            
            update_physics(particles)
            draw_frame(particles)
            
            # Frame rate control
            elapsed = time.monotonic() - start_time
            sleep_time = FRAME_DELAY - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    except KeyboardInterrupt:
        pass
    finally:
        # Show cursor before exiting
        sys.stdout.write('\033[?25h')
        sys.stdout.flush()

if __name__ == "__main__":
    main()
