//================== Define PID variables ================//
#define INTEGRAL_l 1
#define NORMAL_l 2

float outPID_pitch = 0;
float sp_l = 0, pv_l = 0, integralE_l = 0, derivativeE_l = 0, consKp_l = 0, consKi_l = 0, consKd_l = 0, dU_l = 0;
int syntaxError_l = 0;
unsigned long previousMillis_l = 0;
float outP_l = 0, outI_l = 0, outD_l = 0;
float maxOut_l = radians(50);
float minOut_l = radians(-50);
long lastTimeTC_l = 0;
float Tc_l = 0;
int mode_l = 0;
int first_l = 1;
float derror_l = 0, error_l = 0, previouserror_l = 0;

// ========== Define fuzzy variables =========== //
float Kpf_l = 0, Kif_l = 0, Kdf_l = 0;                                  // output defuzzifikasi
float m_error_l = 0, m_derror_l = 0;                                  // derajat keanggotaan
float MAXerror_l = radians(20), MINerror_l = radians(-20);
float MAXderror_l = radians(10), MINderror_l = radians(-10);

float Tab_error_l[7] = {};      // 0 = NB_error, 1 = NM_error, 2 = NS_error, 3 = ZO_error, 4 = PS_error, 5 = PM_error, 6 = PB_error;
float Tab_derror_l[7] = {};     // 0 = NB_derror, 1 = NM_derror, 2 = NS_derror, 3 = ZO_derror, 4 = PS_derror, 5 = PM_derror, 6 = PB_derror;
float ruletable_l[49]= {};      // Tabel Rule base

float Emf_l[9]  = {radians(-25),radians(-20), radians(-15), radians(-10), radians(0), radians(10), radians(15), radians(20), radians(25)};
float DEmf_l[9] = {radians(-15),radians(-10), radians(-5), radians(-2), radians(0), radians(2), radians(5), radians(10), radians(15)};

float mf_error_l(float a_l, float b_l, float c_l)                     // mencari derajat keanggotaan membership function
{
  if (error_l>=a_l&&error_l<b_l) {m_error_l= (error_l-a_l)/(b_l-a_l); }     // derajat keanggotaan region ke-1
  if (error_l>=b_l&&error_l<c_l) {m_error_l = (c_l-error_l)/(c_l-b_l); }     // derajat keanggotaan region ke-2
}

float mf_derror_l(float a1_l, float b1_l, float c1_l)
{
  if (derror_l>=a1_l&&derror_l<b1_l) {m_derror_l = (derror_l-a1_l)/(b1_l-a1_l); }     // derajat keanggotaan region ke-1
  if (derror_l>=b1_l&&derror_l<c1_l) {m_derror_l = (c1_l-derror_l)/(c1_l-b1_l); }     // derajat keanggotaan region ke-2
}

// ===================== MEMBER FUNCTION ==================== //
void fuzzifikasi_l() {
  /* MF error fuzzifikasi */
  if(error_l>=Emf_l[0]&&error_l<=Emf_l[2]){
     mf_error_l(Emf_l[0], Emf_l[1], Emf_l[2]); Tab_error_l[0] = m_error_l;     // NB
    } else {Tab_error_l[0] = 0;}
  if(error_l>=Emf_l[1]&&error_l<=Emf_l[3]){
     mf_error_l(Emf_l[1], Emf_l[2], Emf_l[3]); Tab_error_l[1] = m_error_l;     // NM
    } else {Tab_error_l[1] = 0;}
  if(error_l>=Emf_l[2]&&error_l<=Emf_l[4]){
     mf_error_l(Emf_l[2], Emf_l[3], Emf_l[4]); Tab_error_l[2] = m_error_l;     // NS
    } else {Tab_error_l[2] = 0;}
  if(error_l>=Emf_l[3]&&error_l<=Emf_l[5]){
     mf_error_l(Emf_l[3], Emf_l[4], Emf_l[5]); Tab_error_l[3] = m_error_l;     // ZO
    } else {Tab_error_l[3] = 0;}
  if(error_l>=Emf_l[4]&&error_l<=Emf_l[6]){
     mf_error_l(Emf_l[4], Emf_l[5], Emf_l[6]); Tab_error_l[4] = m_error_l;     // PS
    } else {Tab_error_l[4] = 0;}
  if(error_l>=Emf_l[5]&&error_l<=Emf_l[7]){
     mf_error_l(Emf_l[5], Emf_l[6], Emf_l[7]); Tab_error_l[5] = m_error_l;     // PM
    } else {Tab_error_l[5] = 0;}
  if(error_l>=Emf_l[6]&&error_l<=Emf_l[8]) {
     mf_error_l(Emf_l[6], Emf_l[7], Emf_l[8]); Tab_error_l[6] = m_error_l;     // PB
    } else {Tab_error_l[6] = 0;}

 //Serial.print("MF error = "); Serial.print(Tab_error_l[0]);Serial.print("||");Serial.print(Tab_error_l[1]);Serial.print("||");Serial.print(Tab_error_l[2]);Serial.print("||");
 //Serial.print(Tab_error_l[3]);Serial.print("||");Serial.print(Tab_error_l[4]);Serial.print("||");Serial.print(Tab_error_l[5]);Serial.print("||");Serial.print(Tab_error_l[6]);
 //Serial.println();
  
  /* MF delta error fuzzifikasi */
  if(derror_l>=DEmf_l[0]&&derror_l<=DEmf_l[2]){
     mf_derror_l(DEmf_l[0], DEmf_l[1], DEmf_l[2]); Tab_derror_l[0] = m_derror_l;     // NB
     } else{Tab_derror_l[0] = 0;}
  if(derror_l>=DEmf_l[1]&&derror_l<=DEmf_l[3]){
     mf_derror_l(DEmf_l[1], DEmf_l[2], DEmf_l[3]); Tab_derror_l[1] = m_derror_l;     // NM
     } else{Tab_derror_l[1] = 0;}
  if(derror_l>=DEmf_l[2]&&derror_l<=DEmf_l[4]){
     mf_derror_l(DEmf_l[2], DEmf_l[3], DEmf_l[4]); Tab_derror_l[2] = m_derror_l;     // NS
    } else{Tab_derror_l[2] = 0;}
  if(derror_l>=DEmf_l[3]&&derror_l<=DEmf_l[5]){
     mf_derror_l(DEmf_l[3], DEmf_l[4], DEmf_l[5]); Tab_derror_l[3] = m_derror_l;     // ZO
    } else{Tab_derror_l[3] = 0;}
  if(derror_l>=DEmf_l[4]&&derror_l<=DEmf_l[6]){
     mf_derror_l(DEmf_l[4], DEmf_l[5], DEmf_l[6]); Tab_derror_l[4] = m_derror_l;     // PS
    } else{Tab_derror_l[4] = 0;}
  if(derror_l>=DEmf_l[5]&&derror_l<=DEmf_l[7]){
     mf_derror_l(DEmf_l[5], DEmf_l[6], DEmf_l[7]); Tab_derror_l[5] = m_derror_l;     // PM
    } else{Tab_derror_l[5] = 0;}
  if(derror_l>=DEmf_l[6]&&derror_l<=DEmf_l[8]) {
    mf_derror_l(DEmf_l[6], DEmf_l[7], DEmf_l[8]); Tab_derror_l[6] = m_derror_l;      // PB
   } else{Tab_derror_l[6] = 0;}

 //Serial.print("MF Derror = "); Serial.print(Tab_derror_l[0]);Serial.print("||");Serial.print(Tab_derror_l[1]);Serial.print("||");Serial.print(Tab_derror_l[2]);Serial.print("||");
 //Serial.print(Tab_derror_l[3]);Serial.print("||");Serial.print(Tab_derror_l[4]);Serial.print("||");Serial.print(Tab_derror_l[5]);
 //Serial.println();
}

// ============== RULE EVALUATION TABLE ============= //
void Ruleevaluation_l() {
  int kl = 0;
  for(int ix=0; ix<7;ix++) {
    for(int jx=0; jx<7;jx++) {
      ruletable_l[kl] = min(Tab_error_l[ix], Tab_derror_l[jx]);
      kl++;
    }
  }
  // debugging mung seko tabel 0.0 sampai 0.6
/*
  Serial.print("Rule tables = "); Serial.print(ruletable_l[0]);Serial.print("||");Serial.print(ruletable_l[1]);Serial.print("||");Serial.print(ruletable_l[2]);Serial.print("||");
  Serial.print(ruletable_l[3]);Serial.print("||");Serial.print(ruletable_l[4]);Serial.print("||");Serial.print(ruletable_l[5]);Serial.print("||");Serial.print(ruletable_l[6]);
  Serial.println();
  Serial.print("              "); Serial.print(ruletable_l[7]);Serial.print("||");Serial.print(ruletable_l[8]);Serial.print("||");Serial.print(ruletable_l[9]);Serial.print("||");
  Serial.print(ruletable_l[10]);Serial.print("||");Serial.print(ruletable_l[11]);Serial.print("||");Serial.print(ruletable_l[12]);Serial.print("||");Serial.print(ruletable_l[13]);
  Serial.println();
  Serial.print("              "); Serial.print(ruletable_l[14]);Serial.print("||");Serial.print(ruletable_l[15]);Serial.print("||");Serial.print(ruletable_l[16]);Serial.print("||");
  Serial.print(ruletable_l[17]);Serial.print("||");Serial.print(ruletable_l[18]);Serial.print("||");Serial.print(ruletable_l[19]);Serial.print("||");Serial.print(ruletable_l[20]);
  Serial.println();
  Serial.print("              "); Serial.print(ruletable_l[21]);Serial.print("||");Serial.print(ruletable_l[22]);Serial.print("||");Serial.print(ruletable_l[23]);Serial.print("||");
  Serial.print(ruletable_l[24]);Serial.print("||");Serial.print(ruletable_l[25]);Serial.print("||");Serial.print(ruletable_l[26]);Serial.print("||");Serial.print(ruletable_l[27]);
  Serial.println();
  Serial.print("              "); Serial.print(ruletable_l[28]);Serial.print("||");Serial.print(ruletable_l[29]);Serial.print("||");Serial.print(ruletable_l[30]);Serial.print("||");
  Serial.print(ruletable_l[31]);Serial.print("||");Serial.print(ruletable_l[32]);Serial.print("||");Serial.print(ruletable_l[33]);Serial.print("||");Serial.print(ruletable_l[34]);
  Serial.println();
  Serial.print("              "); Serial.print(ruletable_l[35]);Serial.print("||");Serial.print(ruletable_l[36]);Serial.print("||");Serial.print(ruletable_l[37]);Serial.print("||");
  Serial.print(ruletable_l[38]);Serial.print("||");Serial.print(ruletable_l[39]);Serial.print("||");Serial.print(ruletable_l[40]);Serial.print("||");Serial.print(ruletable_l[41]);
  Serial.println();
  Serial.print("              "); Serial.print(ruletable_l[42]);Serial.print("||");Serial.print(ruletable_l[43]);Serial.print("||");Serial.print(ruletable_l[44]);Serial.print("||");
  Serial.print(ruletable_l[45]);Serial.print("||");Serial.print(ruletable_l[46]);Serial.print("||");Serial.print(ruletable_l[47]);Serial.print("||");Serial.print(ruletable_l[49]);
  Serial.println();
*/
}

// =========== DEFUZZIFIKASI ============= //
void defuzzifikasi_l() {
  float totalKpevalxbobot_l = 0, totalKievalxbobot_l = 0, totalKdevalxbobot_l = 0;
  float Kpeval_l = 0, Kieval_l = 0, Kdeval_l = 0;
  
  float Kpevalxbobot1_l, Kpevalxbobot2_l, Kpevalxbobot3_l, Kpevalxbobot4_l, Kpevalxbobot5_l, Kpevalxbobot6_l, Kpevalxbobot7_l;
  float Kievalxbobot1_l, Kievalxbobot2_l, Kievalxbobot3_l, Kievalxbobot4_l, Kievalxbobot5_l, Kievalxbobot6_l, Kievalxbobot7_l;
  float Kdevalxbobot1_l, Kdevalxbobot2_l, Kdevalxbobot3_l, Kdevalxbobot4_l, Kdevalxbobot5_l, Kdevalxbobot6_l, Kdevalxbobot7_l;
  
  float dKp_l[7] = {-0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3}; //{0,0,0,0,0,0,0};//              // variabel output delta Kp
  float dKi_l[7] = {-0.03, -0.02, -0.01, 0, 0.01, 0.02, 0.03}; //{0,0,0,0,0,0,0};//        // variabel output delta Ki
  float dKd_l[7] = {-0.05, -0.02, -0.01, 0, 0.01, 0.02, 0.05}; //{0,0,0,0,0,0,0};//        // variabel output delta Kd

  //Serial.print("memberdKp = ");Serial.print(dKp[0]);Serial.print("||");Serial.print(dKp[1]);Serial.print("||");Serial.print(dKp[2]);Serial.print("||");Serial.print(dKp[3]);
  //Serial.print("||");Serial.print(dKp[4]);Serial.print("||");Serial.print(dKp[5]);Serial.print("||");Serial.print(dKp[6]);
  //Serial.println();
  
  // defuzzifikasi Kp
  Kpevalxbobot1_l = (ruletable_l[0]*dKp_l[6])+(ruletable_l[1]*dKp_l[6])+(ruletable_l[2]*dKp_l[5])+(ruletable_l[3]*dKp_l[5])+(ruletable_l[4]*dKp_l[4])+(ruletable_l[5]*dKp_l[3])+(ruletable_l[6]*dKp_l[3]);
  Kpevalxbobot2_l = (ruletable_l[7]*dKp_l[6])+(ruletable_l[8]*dKp_l[6])+(ruletable_l[9]*dKp_l[5])+(ruletable_l[10]*dKp_l[4])+(ruletable_l[11]*dKp_l[4])+(ruletable_l[12]*dKp_l[3])+(ruletable_l[13]*dKp_l[3]);
  Kpevalxbobot3_l = (ruletable_l[14]*dKp_l[5])+(ruletable_l[15]*dKp_l[5])+(ruletable_l[16]*dKp_l[5])+(ruletable_l[17]*dKp_l[4])+(ruletable_l[18]*dKp_l[3])+(ruletable_l[19]*dKp_l[2])+(ruletable_l[20]*dKp_l[2]); 
  Kpevalxbobot4_l = (ruletable_l[21]*dKp_l[5])+(ruletable_l[22]*dKp_l[5])+(ruletable_l[23]*dKp_l[4])+(ruletable_l[24]*dKp_l[3])+(ruletable_l[25]*dKp_l[2])+(ruletable_l[26]*dKp_l[1])+(ruletable_l[27]*dKp_l[1]);
  Kpevalxbobot5_l = (ruletable_l[28]*dKp_l[4])+(ruletable_l[29]*dKp_l[4])+(ruletable_l[30]*dKp_l[3])+(ruletable_l[31]*dKp_l[2])+(ruletable_l[32]*dKp_l[2])+(ruletable_l[33]*dKp_l[1])+(ruletable_l[34]*dKp_l[1]);
  Kpevalxbobot6_l = (ruletable_l[35]*dKp_l[4])+(ruletable_l[36]*dKp_l[3])+(ruletable_l[37]*dKp_l[2])+(ruletable_l[38]*dKp_l[1])+(ruletable_l[39]*dKp_l[1])+(ruletable_l[40]*dKp_l[1])+(ruletable_l[41]*dKp_l[0]);
  Kpevalxbobot7_l = (ruletable_l[42]*dKp_l[3])+(ruletable[43]*dKp_l[3])+(ruletable_l[44]*dKp_l[1])+(ruletable_l[45]*dKp_l[1])+(ruletable_l[46]*dKp_l[1])+(ruletable_l[47]*dKp_l[0])+(ruletable_l[48]*dKp_l[0]);

  totalKpevalxbobot_l = Kpevalxbobot1_l + Kpevalxbobot2_l + Kpevalxbobot3_l + Kpevalxbobot4_l + Kpevalxbobot5_l + Kpevalxbobot6_l + Kpevalxbobot7_l;
  for(int i=0; i<49;i++){
    if(ruletable_l[i] != 0){
    Kpeval_l += ruletable_l[i];
    }
  }
  Kpf_l = totalKpevalxbobot_l/Kpeval_l;
  //Serial.print("Debug Kpeval = ");Serial.print(Kpeval);
  //Serial.println();
  //Serial.print("Debug Kpf = ");Serial.print(Kpf);
  //Serial.println();

  // defuzzifikasi Ki
  Kievalxbobot1_l = (ruletable_l[0]*dKi_l[0])+(ruletable_l[1]*dKi_l[0])+(ruletable_l[2]*dKi_l[1])+(ruletable_l[3]*dKi_l[1])+(ruletable_l[4]*dKi_l[2])+(ruletable_l[5]*dKi_l[3])+(ruletable_l[6]*dKi_l[3]);
  Kievalxbobot2_l = (ruletable_l[7]*dKi_l[0])+(ruletable_l[8]*dKi_l[0])+(ruletable_l[9]*dKi_l[1])+(ruletable_l[10]*dKi_l[2])+(ruletable_l[11]*dKi_l[2])+(ruletable_l[12]*dKi_l[3])+(ruletable_l[13]*dKi_l[3]);
  Kievalxbobot3_l = (ruletable_l[14]*dKi_l[1])+(ruletable_l[15]*dKi_l[1])+(ruletable_l[16]*dKi_l[2])+(ruletable_l[17]*dKi_l[2])+(ruletable_l[18]*dKi_l[3])+(ruletable_l[19]*dKi_l[4])+(ruletable_l[20]*dKi_l[4]); 
  Kievalxbobot4_l = (ruletable_l[21]*dKi_l[1])+(ruletable_l[22]*dKi_l[1])+(ruletable_l[23]*dKi_l[3])+(ruletable_l[24]*dKi_l[4])+(ruletable_l[25]*dKi_l[4])+(ruletable_l[26]*dKi_l[5])+(ruletable_l[27]*dKi_l[5]);
  Kievalxbobot5_l = (ruletable_l[28]*dKi_l[1])+(ruletable_l[29]*dKi_l[2])+(ruletable_l[30]*dKi_l[3])+(ruletable_l[31]*dKi_l[4])+(ruletable_l[32]*dKi_l[4])+(ruletable_l[33]*dKi_l[5])+(ruletable_l[34]*dKi_l[5]);
  Kievalxbobot6_l = (ruletable_l[35]*dKi_l[3])+(ruletable_l[36]*dKi_l[3])+(ruletable_l[37]*dKi_l[4])+(ruletable_l[38]*dKi_l[4])+(ruletable_l[39]*dKi_l[5])+(ruletable_l[40]*dKi_l[6])+(ruletable_l[41]*dKi_l[6]);
  Kievalxbobot7_l = (ruletable_l[42]*dKi_l[3])+(ruletable_l[43]*dKi_l[3])+(ruletable_l[44]*dKi_l[4])+(ruletable_l[45]*dKi_l[5])+(ruletable_l[46]*dKi_l[5])+(ruletable_l[47]*dKi_l[6])+(ruletable_l[48]*dKi_l[6]);

  totalKievalxbobot_l = Kievalxbobot1_l + Kievalxbobot2_l + Kievalxbobot3_l + Kievalxbobot4_l + Kievalxbobot5_l + Kievalxbobot6_l + Kievalxbobot7_l;
  for(int i=0; i<49;i++){
    if(ruletable_l[i] != 0){
    Kieval_l += ruletable_l[i];
    }
  }
  Kif_l = totalKievalxbobot_l/Kieval_l;
  //Serial.print("Debug Kieval = ");Serial.print(Kieval);
  //Serial.println();
  //Serial.print("Debug Kif = ");Serial.print(Kif);
  //Serial.println();

  // defuzzifikasi Kd
  Kdevalxbobot1_l = (ruletable_l[0]*dKd_l[4])+(ruletable_l[1]*dKd_l[2])+(ruletable_l[2]*dKd_l[0])+(ruletable_l[3]*dKd_l[0])+(ruletable_l[4]*dKd_l[0])+(ruletable_l[5]*dKd_l[1])+(ruletable_l[6]*dKd_l[4]);
  Kdevalxbobot2_l = (ruletable_l[7]*dKd_l[4])+(ruletable_l[8]*dKd_l[2])+(ruletable_l[9]*dKd_l[0])+(ruletable_l[10]*dKd_l[1])+(ruletable_l[11]*dKd_l[1])+(ruletable_l[12]*dKd_l[2])+(ruletable_l[13]*dKd_l[3]);
  Kdevalxbobot3_l = (ruletable_l[14]*dKd_l[3])+(ruletable_l[15]*dKd_l[2])+(ruletable_l[16]*dKd_l[1])+(ruletable_l[17]*dKd_l[1])+(ruletable_l[18]*dKd_l[2])+(ruletable_l[19]*dKd_l[2])+(ruletable_l[20]*dKd_l[3]); 
  Kdevalxbobot4_l = (ruletable_l[21]*dKd_l[3])+(ruletable_l[22]*dKd_l[2])+(ruletable_l[23]*dKd_l[2])+(ruletable_l[24]*dKd_l[3])+(ruletable_l[25]*dKd_l[2])+(ruletable_l[26]*dKd_l[2])+(ruletable_l[27]*dKd_l[3]);
  Kdevalxbobot5_l = (ruletable_l[28]*dKd_l[3])+(ruletable_l[29]*dKd_l[3])+(ruletable_l[30]*dKd_l[3])+(ruletable_l[31]*dKd_l[3])+(ruletable_l[32]*dKd_l[3])+(ruletable_l[33]*dKd_l[3])+(ruletable_l[34]*dKd_l[3]);
  Kdevalxbobot6_l = (ruletable_l[35]*dKd_l[6])+(ruletable_l[36]*dKd_l[2])+(ruletable_l[37]*dKd_l[4])+(ruletable_l[38]*dKd_l[4])+(ruletable_l[39]*dKd_l[4])+(ruletable_l[40]*dKd_l[4])+(ruletable_l[41]*dKd_l[6]);
  Kdevalxbobot7_l = (ruletable_l[42]*dKd_l[6])+(ruletable_l[43]*dKd_l[5])+(ruletable_l[44]*dKd_l[5])+(ruletable_l[45]*dKd_l[5])+(ruletable_l[46]*dKd_l[4])+(ruletable_l[47]*dKd_l[4])+(ruletable_l[48]*dKd_l[4]);

  totalKdevalxbobot_l = Kdevalxbobot1_l + Kdevalxbobot2_l + Kdevalxbobot3_l + Kdevalxbobot4_l + Kdevalxbobot5_l + Kdevalxbobot6_l + Kdevalxbobot7_l;
  for(int i=0; i<49;i++){
    if(ruletable_l[i] != 0){
    Kdeval_l += ruletable_l[i];
    }
  }
  Kdf_l = totalKdevalxbobot_l/Kdeval_l;
  //Serial.print("Debug Kdeval = ");Serial.print(Kdeval);
  //Serial.println();
  //Serial.print("totalKdevalxbobot = ");Serial.print(totalKdevalxbobot);
  //Serial.println();
  //Serial.print("Debug Kdf = ");Serial.print(Kdf);
  //Serial.println();
  //Serial.print("Debugging Kpf, Kif, Kdf = "); Serial.print(Kpf); Serial.print("||");Serial.print(Kif);Serial.print("||");Serial.print(Kdf);
  //Serial.println();

}

void constPID_l(float vKp_l,float vKi_l,float vKd_l,int vmode_l) {
  consKp_l = vKp_l;
  consKi_l = vKi_l;
  consKd_l = vKd_l;
  mode_l = vmode_l;
}

void Sensorvalue_l(float vPv_l) {
  pv_l = vPv_l;
}

void setpoint_l(float vsp_l) {
  sp_l = vsp_l;
}

void timesampling_l(int ts_l) {
  Tc_l = ts_l;
}

void calcFuzzyPID_Pitch(float readsensor_l, float inputvalue_l, float Kp_l, float Ki_l, float Kd_l, float tsampling_l, int modePID_l) {
  
  Sensorvalue_l(readsensor_l);
  setpoint_l(inputvalue_l);
  
  // calc error and delta error fuzzyinput()
  error_l  = sp_l - pv_l;
  if(error_l >= MAXerror_l){
     error_l = MAXerror_l;
  } else if (error_l <= MINerror_l){
    error_l = MINerror_l;
  }
  unsigned long counter_l = micros();
  if(counter_l - previousMillis_l >= 3000000) {
  previouserror_l  = error_l;
  previousMillis_l = counter_l;
  }

  derror_l = error_l - previouserror_l;
  if(derror_l >= MAXderror_l){
    derror_l = MAXderror_l;
  } else if (derror_l <= MINderror_l){
    derror_l = MINderror_l;
  }
  
//  Serial.print("DEbug error! = "); Serial.print(error_l); Serial.print("||"); Serial.print(derror_l);
//  Serial.println();
  
  // fuzzy logic controller
  fuzzifikasi_l();
  Ruleevaluation_l();
  defuzzifikasi_l();
  constPID_l(Kp_l,Ki_l,Kd_l,modePID_l);
  timesampling_l(tsampling_l);

  // PID controller
  if(first_l == 1) {
    first_l = 0;
    derivativeE_l = error_l;
  }

  // time sampling detection
  unsigned long Time_l = micros();
  if(Time_l - lastTimeTC_l >= (Tc_l * 1000)){
    float tc_l = (Time_l - lastTimeTC_l) / 1000000.0;
    lastTimeTC_l = Time_l;

    // Proportional calculation
    outP_l = (consKp_l + Kpf_l) * error_l;

    // Integral calculation
    integralE_l += error_l;
    outI_l = (consKi_l + Kif_l) * integralE_l * tc_l;

    // Derivative calculation
    derivativeE_l = error_l - derivativeE_l;
    outD_l = ((consKd_l + Kdf_l) * derivativeE_l) / tc_l;
    derivativeE_l = error_l;

    // delta U output
    dU_l = outP_l + outI_l + outD_l;
    //Serial.print("debug output P,I,D = "); Serial.print(dU); //Serial.print(outP);Serial.print("||");Serial.print(outI);Serial.print("||");Serial.print(outD);
    //Serial.println();

    // case mod
    if(mode_l==1) {
      outPID_pitch += dU_l;
    } else{ outPID_pitch = dU_l;}

    // limit output
    if(maxOut_l != 0 || minOut_l != 0) {
      if(outPID_pitch >= maxOut_l) {
        outPID_pitch = maxOut_l;
        integralE_l -= error_l;
      } else if (outPID_pitch <= minOut_l) {
        outPID_pitch = minOut_l;
        integralE_l -= error_l;
      }
    }
    //Serial.print("Output PID = "); Serial.print(outPID_pitch);
//    Serial.print("Integral = "); Serial.print(integralE_l);
    //Serial.println();
    return outPID_pitch;

  }
}
