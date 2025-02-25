I wanted to make an ESP32 based 128x64 I2C OLED FLIP simulator,
inspired by:
https://mitxela.com/projects/fluid-pendant


Flip: Fluid-Implicit-Particle

https://www.sciencedirect.com/science/article/pii/0010465588900203


but with way cheaper and easier components.

It seemse like there's not a huge body of work on this, but I did find this project:
https://wokwi.com/projects/420908950128021505

(no user information)

Now there's another resources I found: (this is amazing)
https://unusualinsights.github.io/fluid_tutorial/



anyway...documentation will go on my site: https://zeyus.com/

I'm going to use the following components:

- ESP32 WROOM
- SSD1315 128x64 OLED I2C display
- MPU6050 6-axis accelerometer/gyro
- 500mAh LiPo battery (402035) - hopefully enough, I can always upgrade it, but I want to try and make it as small as possible
- AMS1117 3.3V voltage regulator
- TBD: Design and 3D print a case
