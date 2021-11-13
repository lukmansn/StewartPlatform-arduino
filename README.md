# StewartPlatform-arduino-self_Balancing_Platform
This is a code for controlling stewart platform with Arduino Mega 2560 with Processing UI via serial monitor communication
The electronics component such as:
- Arduino Mega 2560
- MPU6050
- PCA9685
- Servo MG995 + arm servo
- Push Rod
- Ball joints

This code contain some program part including:
1. Main program for calling all the sub program
2. Reading MPU6050 accelerometer and gyroscope sensor + Complementary filter (Sub program)
3. Inverse kinematic code for controlling stewart platform
4. Fuzzy-PID algorithm for Self balancing platform orientation
5. Servo configuration
