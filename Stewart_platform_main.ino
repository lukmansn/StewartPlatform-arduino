#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <math.h>
#include "ServoSP.h"
#include "variableSerial.h"
#include "conversion.h"
#include "Inversekinematik.h"
#include "IMU_MPU6050.h"
#include "fuzzyPIDRoll.h"
#include "fuzzyPIDPitch.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// setup parameter - parameter pid
// ZN- BASED Kp = 2.16, Ki = 23.60, Kd = 0.049;
float kp = 1.6;    
float ki = 0.0133;
float kd = 0.0128;

float setbalanceX=radians(0), setbalanceY=radians(0);
float Rollvalue  = 0
     ,Pitchvalue = 0;

void setup() {
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  Serial.begin(115200);
  Wire.begin();
  setupMPU();
  gyroError();
  times = millis();
}

void loop() {
  // Read IMU MPU6050
  timevariant();
  recordAccelRegisters();
  recordGyroRegisters();
  calcRollPitchYawData();
  
  Rollvalue  = radians(roll_angle);
  Pitchvalue = radians(pitch_angle+1.7);
  
  // Serial PARSING DATA from processing
  if(Serial.available() > 0) {
     if(Serial.find('<')) {
      for(int i=0; i<7; i++) {
      dj[i] = Serial.parseFloat();          
      }
   }
 }
 
 // TERIMA DATA dari Processing IDE
  for(int i=0;i<6;i++) {
    if(i<3) {
     trans_rot[i] = dj[i]*30;                    // (dj[i]/100)/25.4;  || variasi gain = 30 (siginifikan level adjust) , 20 (level adjust kecil), 25 (medium adjust)
     }else {
     trans_rot[i] = dj[i]*radians(18);           // radians(dj[i]/100); *PI/10 || variasi gain = 20 (siginifikan level adjust), 10 (level adjust kecil), 15 (medium adjust)
    }
  }

  state_variabel = dj[6];

 // MODE MOTION
  if(state_variabel < 1.0) { // balancing = 0.00; //# debug
  calcServopos(trans_rot);
  u[3] = 0;
  u[4] = 0;  
  }

 // MODE SELF-BALANCING
  else if(state_variabel != 0.0) { // balancing = 55.6; // debug
    
    calcFuzzyPID_Roll(Rollvalue, setbalanceX, kp, ki, kd, 1, 2);
    calcFuzzyPID_Pitch(Pitchvalue, setbalanceY, kp, ki, kd, 1, 2);
    u[3] = outPID_roll;
    u[4] = outPID_pitch;
    calcServopos(u);
  }

  // OUTPUT SERVO
  for(int i=0;i<6;i++) {
  pwm.writeMicroseconds(servo[i], servo_pos[i]);
  }
  
 // SEND DATA MPU6050 via SERIAL COMM
   Serial.print(degrees(Rollvalue));Serial.print(","); Serial.print(degrees(Pitchvalue));Serial.print(","); Serial.print(yaw_angle);//Serial.print(",");
 //  Serial.print(roll_angle1);Serial.print(",");Serial.print(pitch_angle1);Serial.print(",");Serial.print(yaw_angle1); Serial.print(",");
 //  Serial.print((consKp + Kpf));Serial.print(",");Serial.print((consKi + Kif));Serial.print(",");Serial.print((consKd + Kdf));Serial.print(",");
 //  Serial.print((consKp_l + Kpf_l));Serial.print(",");Serial.print((consKi_l + Kif_l));Serial.print(",");Serial.print((consKd_l + Kdf_l));
   Serial.println();
}
