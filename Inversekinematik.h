#include "variableSP.h"

// Hitung sudut alpha servo
void calcAlpha() {
  float Ai[3], Li[3], Li_rect, L, M, N;
  //float alpha[i] = {0};
  
  for(int i=0; i<6; i++){
    //double min=SERVO_MIN;
    //double max=SERVO_MAX;
    float XpminXb = Qi[0][i] - Bi[0][i];  // Li
    float YpminYb = Qi[1][i] - Bi[1][i];  // Li
 
    // perhitungan Ai vektor ujung servo
    Ai[0] = (panjang_arm * cos(alpha[i]) * cos(Beta[i])) + Bi[0][i];
    Ai[1] = (panjang_arm * cos(alpha[i]) * sin(Beta[i])) + Bi[1][i];
    Ai[2] = (panjang_arm * sin(alpha[i])) + Bi[2][i];
    // perhitungan kaki virtual Li
    Li[0] = Qi[0][i] - Bi[0][i];
    Li[1] = Qi[1][i] - Bi[1][i];
    Li[2] = Qi[2][i] - Bi[2][i];
    Li_rect = sqrt(Li[0]*Li[0] + Li[1]*Li[1] + Li[2]*Li[2]);
    L = Li_rect*Li_rect - (panjang_kaki*panjang_kaki) + (panjang_arm*panjang_arm);
    M = 2 * panjang_arm * (Qi[2][i] - Bi[2][i]);
    N = 2 * panjang_arm * (cos(Beta[i]) * (XpminXb) + sin(Beta[i]) * (YpminYb));
    alpha[i] = asin(L / sqrt((M*M) + (N*N))) - atan2(N,M);
  }
   return alpha;
}

void homeposition() {
  float M0, N0, L0, Home;
  for(int i=0;i<6;i++) {
   float XpminXb = Qi[0][i] - Bi[0][i];  // Li
   float YpminYb = Qi[1][i] - Bi[1][i];  // Li
   Home = sqrt((panjang_kaki*panjang_kaki) + (panjang_arm * panjang_arm) - (XpminXb*XpminXb) - (YpminYb*YpminYb) - Qi[2][i]);
  
   L0 = 2 * panjang_arm * panjang_arm;
   M0 = 2 * panjang_arm * (Pi_p[0][i] - Bi[0][i]);
   N0 = 2 * panjang_arm * (Home + Pi_p[2][i]);
   //Serial.print(M0[0]); Serial.print("||"); Serial.print(N0[0]);Serial.println();
   alpha0[i] = asin(L0/sqrt((M0*M0) + (N0*N0))) - atan2(N0,M0);
  }
  return alpha0;
}

void MatrixRot(float rot[]) {
  float yaw   = rot[5]; 
  float pitch = rot[4];
  float roll  = rot[3];
  
  // Matriks rotasional P_Rb
  P_Rb[0][0] = cos(yaw)*cos(pitch);
  P_Rb[0][1] = (-sin(yaw)*cos(roll)) + (cos(yaw)*sin(pitch)*sin(roll));  
  P_Rb[0][2] = (sin(yaw)*sin(roll)) + (cos(yaw)*sin(pitch)*cos(roll));
  
  P_Rb[1][0] = sin(yaw)*cos(pitch);
  P_Rb[1][1] = (cos(yaw)*cos(roll)) + (sin(yaw)*sin(pitch)*sin(roll));
  P_Rb[1][2] = (-cos(yaw)*sin(roll))+ (sin(yaw)*sin(pitch)*cos(roll));
  
  P_Rb[2][0] = -sin(pitch);
  P_Rb[2][1] = cos(pitch)*sin(roll);
  P_Rb[2][2] = cos(pitch)*cos(roll);
}

void calcQi() {
  for(int i=0;i<6;i++) {
    // rumus e Qi = T + P_Rb*Pi_p
    Qi[0][i] = T[0] + (P_Rb[0][0] * Pi_p[0][i]) + (P_Rb[0][1] * Pi_p[1][i]) + (P_Rb[0][2] * Pi_p[2][i]);  // x
    Qi[1][i] = T[1] + (P_Rb[1][0] * Pi_p[0][i]) + (P_Rb[1][1] * Pi_p[1][i]) + (P_Rb[1][2] * Pi_p[2][i]);  // y
    Qi[2][i] = T[2] + (P_Rb[2][0] * Pi_p[0][i]) + (P_Rb[2][1] * Pi_p[1][i]) + (P_Rb[2][2] * Pi_p[2][i]);  // z
  }
  return Qi;
}

// fungsi vektor translasi movement platform + Z0 home pos
void getTranslasi(float rot[]) {
  T[0] = rot[0]+H0[0];  // x
  T[1] = rot[1]+H0[1];  // y
  T[2] = rot[2]+H0[2];  // z
}

void calcServopos(float rot[]){
  //unsigned char limitcount;
  //const unsigned long eventInterval = 500;
  //unsigned long previousTime = 0;
  //unsigned long currentTime = millis();
  //int pulse_width;
  //limitcount=0;
 
  getTranslasi(rot);
  MatrixRot(rot);
  calcQi();
  homeposition();
  calcAlpha();

  for(int i=0; i<6; i++) {
    if(i==inverse_ser_2||i==inverse_ser_4||i==inverse_ser_6){
      servo_pos[i] = constrain(servo_zero_pos[i] - (alpha[i]-alpha0[i]) * SERVO_Mult, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    }
    else{
      servo_pos[i] = constrain(servo_zero_pos[i] + (alpha[i]-alpha0[i]) * SERVO_Mult, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    }
  }

    /*if(currentTime - previousTime >= eventInterval){
      pwm.writeMicroseconds(servo[i],servo_pos[i]);
      previousTime = currentTime;
    }
  }*/
  return servo_pos;
}
