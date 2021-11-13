/*This IMU used MPU6050 with discrete complementary filter */
float gForceX, gForceY, gForceZ;
float rotX, rotY, rotZ;

float rotXError = 0;
float rotYError = 0;
float rotZError = 0;

float accX, accY;
float rad_to_deg = 180/PI;
float roll_angle = 0;
float pitch_angle = 0;
float yaw_angle = 0;

float gy_roll = 0;
float gy_pitch = 0;
float gy_spd_roll = 0;
float gy_spd_pitch = 0;

float Acc_roll = 0;
float Acc_pitch = 0;
float Acc_spd_roll = 0;
float Acc_spd_pitch = 0;

float prevTime, times, elapsedTime;

void timevariant() {
  times = millis();
  elapsedTime = (times - prevTime) / 1000;
  prevTime = times;
}

void setupMPU(){
  Wire.beginTransmission(0x68);            // mpu6050 i2c address hex:0x68
  Wire.write(0x6B);                        // mpu6050 i2c address dec:107 device reset/sleep/cycle/temp reset register
  Wire.write(0);                           // di set ke null bosq ben tangi MPU6050 e
  Wire.endTransmission(true);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);                       // power management
  Wire.write(0x03);                       // Selection clock 'PLL with Z axis gyroscope reference'
  Wire.endTransmission(true);
    
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);                             // mpu6050 i2c address Gyro config
  Wire.write(0x00);                             // gyro +/- 250 deg/s
  Wire.endTransmission(true);
 
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);                             // mpu6050 i2c register address accelero config
  Wire.write(0x00);                             // AFSS_SEL=0, Full Scale Range = accel +/- 2g
  Wire.endTransmission(true);

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);                             // DLPF register
  Wire.endTransmission(true);
  
}

void recordAccelRegisters() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);                                    // ACC_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true);
  while(Wire.available() < 6);
  gForceX = (Wire.read()<<8|Wire.read()) / 184;     // calculate raw angle (?)  16384.0; 184.0
  gForceY = (Wire.read()<<8|Wire.read()) / 184;
  gForceZ = (Wire.read()<<8|Wire.read()) / 184;
}

void recordGyroRegisters() {
  Wire.beginTransmission(0x68);
  Wire.write(0x43);                                   // GYRO_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true);
  while(Wire.available() < 6);
  rotX = (Wire.read()<<8|Wire.read()) / 131.0;        // calculate the rotation rate
  rotY = (Wire.read()<<8|Wire.read()) / 131.0;
  rotZ = (Wire.read()<<8|Wire.read()) / 131.0;
}

void gyroError() {
  for (int i = 0; i < 200; i++)
  {
    recordGyroRegisters();
    rotXError += rotX;
    rotYError += rotY;
    rotZError += rotZ;
  }
  rotXError /= 200;
  rotYError /= 200;
  rotZError /= 200;
}

void calcRollPitchYawData() {
  // IMU 6050 WITH COMPLEMENTARY FILTER
  rotX -= rotXError;
  rotY -= rotYError;
  rotZ -= rotZError;
  accX = (atan((gForceY)/sqrt(pow((gForceX),2) + pow((gForceZ),2)))*rad_to_deg);      // convert from raw to degrees
  accY = (atan(-1*(gForceX)/sqrt(pow((gForceY),2) + pow((gForceZ),2)))*rad_to_deg);
  roll_angle  = 0.02 * accX + 0.98 * ((rotX * elapsedTime) + roll_angle);             // filter roll angle
  pitch_angle = 0.02 * accY + 0.98 * ((rotY * elapsedTime) + pitch_angle);            // filter pitch angle
  yaw_angle  += rotZ * elapsedTime;

//  gy_roll = (rotX * elapsedTime) + roll_angle;
//  gy_pitch = (rotY * elapsedTime) + pitch_angle;
//  gy_spd_roll = rotX;
//  gy_spd_pitch = rotY;
//
//  Acc_roll = accX;
//  Acc_pitch = accY;
//  Acc_spd_roll = gForceX;
//  Acc_spd_pitch = gForceY;
  
  //Serial.print("roll_angle : ");
  //Serial.print(roll_angle);Serial.print(",");
  //Serial.print("\t\tpitch_angle : ");
  //Serial.print(pitch_angle);Serial.print(",");
  //Serial.print("\t\tyaw_angle : ");
  //Serial.println(yaw_angle);

  return roll_angle, pitch_angle, yaw_angle;
}
