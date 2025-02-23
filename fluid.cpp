#include <iostream>
#include <vector>
#include <chrono>
#include <cstdlib>
#include <cmath>

using namespace std;

// Simulation constants
const int SIM_WIDTH = 64;
const int SIM_HEIGHT = 32;
const int NUM_PARTICLES = 100;

struct Particle {
    double x, y;
    double vx, vy;
};

vector<Particle> particles(NUM_PARTICLES);

void initializeParticles() {
    for (auto& p : particles) {
        p.x = rand() % SIM_WIDTH;
        p.y = rand() % (SIM_HEIGHT / 2);
        p.vx = (rand() % 200 - 100) / 100.0 * 0.7;
        p.vy = (rand() % 100 - 50) / 100.0;
    }
}

void updatePhysics() {
    const double gravity = 0.15;
    const double damping = 0.82;
    const double border = 2.0;

    for (auto& p : particles) {
        p.vy += gravity;
        p.x += p.vx;
        p.y += p.vy;

        // Horizontal boundary collisions
        if (p.x < border || p.x >= SIM_WIDTH - border) {
            p.vx = -p.vx * damping;
            p.x = max(border, min(p.x, SIM_WIDTH - border - 0.1));
        }

        // Vertical boundary collisions
        if (p.y < border || p.y >= SIM_HEIGHT - border) {
            p.vy = -p.vy * damping;
            p.y = max(border, min(p.y, SIM_HEIGHT - border - 0.1));
        }
    }
}

void drawFrame() {
    vector<vector<char> > grid(SIM_HEIGHT, vector<char>(SIM_WIDTH, ' '));

    // Plot particles
    for (const auto& p : particles) {
        int x = static_cast<int>(p.x);
        int y = static_cast<int>(p.y);
        x = max(0, min(SIM_WIDTH - 1, x));
        y = max(0, min(SIM_HEIGHT - 1, y));
        grid[y][x] = 'o';
    }

    // Clear screen and reset cursor
    cout << "\033[H";

    // Draw grid
    for (const auto& row : grid) {
        for (char c : row) {
            cout << c;
        }
        cout << '\n';
    }
    cout << flush;
}

int main() {
    srand(time(nullptr));
    initializeParticles();
    
    // Hide cursor
    cout << "\033[?25l";

    auto last_frame = chrono::steady_clock::now();
    const chrono::milliseconds frame_delay(33);

    try {
        while (true) {
            auto now = chrono::steady_clock::now();
            if (now - last_frame >= frame_delay) {
                updatePhysics();
                drawFrame();
                last_frame = now;
            }
        }
    } catch (...) {
        // Show cursor before exiting
        cout << "\033[?25h";
    }

    // Restore cursor
    cout << "\033[?25h";
    return 0;
}
