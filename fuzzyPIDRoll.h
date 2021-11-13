//================== Define PID variables ================//
#define INTEGRAL 1
#define NORMAL 2

float outPID_roll = 0;
float sp = 0, pv = 0, integralE = 0, derivativeE = 0, consKp = 0, consKi = 0, consKd = 0, dU = 0;
int syntaxError = 0;
unsigned long previousMillis = 0;
float outP = 0, outI = 0, outD = 0;
float maxOut = radians(50);
float minOut = radians(-50);
long lastTimeTC = 0;
float Tc = 0;
int mode = 0;
int first = 1;
float derror = 0, error = 0, previouserror = 0;

// ========== Define fuzzy variables =========== //
float Kpf = 0, Kif = 0, Kdf = 0;                                  // output defuzzifikasi
float m_error = 0, m_derror = 0;                                  // derajat keanggotaan
float MAXerror = radians(20), MINerror = radians(-20);
float MAXderror = radians(10), MINderror = radians(-10);

float Tab_error[7] = {};      // 0 = NB_error, 1 = NM_error, 2 = NS_error, 3 = ZO_error, 4 = PS_error, 5 = PM_error, 6 = PB_error;
float Tab_derror[7] = {};     // 0 = NB_derror, 1 = NM_derror, 2 = NS_derror, 3 = ZO_derror, 4 = PS_derror, 5 = PM_derror, 6 = PB_derror;
float ruletable[49]= {};      // Tabel Rule base

float Emf[9]  = {radians(-25),radians(-20), radians(-15), radians(-10), radians(0), radians(10), radians(15), radians(20), radians(25)};
float DEmf[9] = {radians(-15),radians(-10), radians(-5), radians(-2), radians(0), radians(2), radians(5), radians(10), radians(15)};

float GainKp = 1, GainKi = 1, GainKd = 1;

float mf_error(float a, float b, float c)                     // mencari derajat keanggotaan membership function
{
  if (error>=a&&error<b) {m_error = (error-a)/(b-a); }     // derajat keanggotaan region ke-1
  if (error>=b&&error<c) {m_error = (c-error)/(c-b); }     // derajat keanggotaan region ke-2
}

float mf_derror(float a1, float b1, float c1)
{
  if (derror>=a1&&derror<b1) {m_derror = (derror-a1)/(b1-a1); }     // derajat keanggotaan region ke-1
  if (derror>=b1&&derror<c1) {m_derror = (c1-derror)/(c1-b1); }     // derajat keanggotaan region ke-2
}

// ===================== MEMBER FUNCTION ==================== //
void fuzzifikasi() {
  /* MF error fuzzifikasi */
  Tab_error[7] = {};
  if(error>=Emf[0]&&error<=Emf[2]){
     mf_error(Emf[0], Emf[1], Emf[2]); Tab_error[0] = m_error;     // NB
    } else {Tab_error[0] = 0;}
  if(error>=Emf[1]&&error<=Emf[3]){
     mf_error(Emf[1], Emf[2], Emf[3]); Tab_error[1] = m_error;     // NM
    } else {Tab_error[1] = 0;}
  if(error>=Emf[2]&&error<=Emf[4]){
     mf_error(Emf[2], Emf[3], Emf[4]); Tab_error[2] = m_error;     // NS
    } else {Tab_error[2] = 0;}
  if(error>=Emf[3]&&error<=Emf[5]){
     mf_error(Emf[3], Emf[4], Emf[5]); Tab_error[3] = m_error;     // ZO
    } else {Tab_error[3] = 0;}
  if(error>=Emf[4]&&error<=Emf[6]){
     mf_error(Emf[4], Emf[5], Emf[6]); Tab_error[4] = m_error;     // PS
    } else {Tab_error[4] = 0;}
  if(error>=Emf[5]&&error<=Emf[7]){
     mf_error(Emf[5], Emf[6], Emf[7]); Tab_error[5] = m_error;     // PM
    } else {Tab_error[5] = 0;}
  if(error>=Emf[6]&&error<=Emf[8]) {
     mf_error(Emf[6], Emf[7], Emf[8]); Tab_error[6] = m_error;     // PB
    } else {Tab_error[6] = 0;}

  //Serial.print("MF error = "); Serial.print(Tab_error[0]);Serial.print("||");Serial.print(Tab_error[1]);Serial.print("||");Serial.print(Tab_error[2]);Serial.print("||");
  //Serial.print(Tab_error[3]);Serial.print("||");Serial.print(Tab_error[4]);Serial.print("||");Serial.print(Tab_error[5]);Serial.print("||");Serial.print(Tab_error[6]);
  // Serial.println();
  
  Tab_derror[7] = {};
  /* MF delta error fuzzifikasi */
  if(derror>=DEmf[0]&&derror<=DEmf[2]){
     mf_derror(DEmf[0], DEmf[1], DEmf[2]); Tab_derror[0] = m_derror;     // NB
     } else{Tab_derror[0] = 0;}
  if(derror>=DEmf[1]&&derror<=DEmf[3]){
     mf_derror(DEmf[1], DEmf[2], DEmf[3]); Tab_derror[1] = m_derror;     // NM
     } else{Tab_derror[1] = 0;}
  if(derror>=DEmf[2]&&derror<=DEmf[4]){
     mf_derror(DEmf[2], DEmf[3], DEmf[4]); Tab_derror[2] = m_derror;     // NS
    } else{Tab_derror[2] = 0;}
  if(derror>=DEmf[3]&&derror<=DEmf[5]){
     mf_derror(DEmf[3], DEmf[4], DEmf[5]); Tab_derror[3] = m_derror;     // ZO
    } else{Tab_derror[3] = 0;}
  if(derror>=DEmf[4]&&derror<=DEmf[6]){
     mf_derror(DEmf[4], DEmf[5], DEmf[6]); Tab_derror[4] = m_derror;     // PS
    } else{Tab_derror[4] = 0;}
  if(derror>=DEmf[5]&&derror<=DEmf[7]){
     mf_derror(DEmf[5], DEmf[6], DEmf[7]); Tab_derror[5] = m_derror;     // PM
    } else{Tab_derror[5] = 0;}
  if(derror>=DEmf[6]&&derror<=DEmf[8]){
    mf_derror(DEmf[6], DEmf[7], DEmf[8]); Tab_derror[6] = m_derror;      // PB
   } else{Tab_derror[6] = 0;}

  //Serial.print("MF Derror = "); Serial.print(Tab_derror[0]);Serial.print("||");Serial.print(Tab_derror[1]);Serial.print("||");Serial.print(Tab_derror[2]);Serial.print("||");
  //Serial.print(Tab_derror[3]);Serial.print("||");Serial.print(Tab_derror[4]);Serial.print("||");Serial.print(Tab_derror[5]);
  //Serial.println();
}

// ============== RULE EVALUATION TABLE ============= //
void Ruleevaluation() {
  int k = 0;
  for(int i=0; i<7;i++) {
    for(int j=0; j<7;j++) {
      ruletable[k] = min(Tab_error[i], Tab_derror[j]);
      k++;
    }
  }
  
/*  
  // debugging mung seko tabel 0.0 sampai 0.6
  Serial.print("Rule tables = "); Serial.print(ruletable[0]);Serial.print("||");Serial.print(ruletable[1]);Serial.print("||");Serial.print(ruletable[2]);Serial.print("||");
  Serial.print(ruletable[3]);Serial.print("||");Serial.print(ruletable[4]);Serial.print("||");Serial.print(ruletable[5]);Serial.print("||");Serial.print(ruletable[6]);
  Serial.println();
  Serial.print("              "); Serial.print(ruletable[7]);Serial.print("||");Serial.print(ruletable[8]);Serial.print("||");Serial.print(ruletable[9]);Serial.print("||");
  Serial.print(ruletable[10]);Serial.print("||");Serial.print(ruletable[11]);Serial.print("||");Serial.print(ruletable[12]);Serial.print("||");Serial.print(ruletable[13]);
  Serial.println();
  Serial.print("              "); Serial.print(ruletable[14]);Serial.print("||");Serial.print(ruletable[15]);Serial.print("||");Serial.print(ruletable[16]);Serial.print("||");
  Serial.print(ruletable[17]);Serial.print("||");Serial.print(ruletable[18]);Serial.print("||");Serial.print(ruletable[19]);Serial.print("||");Serial.print(ruletable[20]);
  Serial.println();
  Serial.print("              "); Serial.print(ruletable[21]);Serial.print("||");Serial.print(ruletable[22]);Serial.print("||");Serial.print(ruletable[23]);Serial.print("||");
  Serial.print(ruletable[24]);Serial.print("||");Serial.print(ruletable[25]);Serial.print("||");Serial.print(ruletable[26]);Serial.print("||");Serial.print(ruletable[27]);
  Serial.println();
  Serial.print("              "); Serial.print(ruletable[28]);Serial.print("||");Serial.print(ruletable[29]);Serial.print("||");Serial.print(ruletable[30]);Serial.print("||");
  Serial.print(ruletable[31]);Serial.print("||");Serial.print(ruletable[32]);Serial.print("||");Serial.print(ruletable[33]);Serial.print("||");Serial.print(ruletable[34]);
  Serial.println();
  Serial.print("              "); Serial.print(ruletable[35]);Serial.print("||");Serial.print(ruletable[36]);Serial.print("||");Serial.print(ruletable[37]);Serial.print("||");
  Serial.print(ruletable[38]);Serial.print("||");Serial.print(ruletable[39]);Serial.print("||");Serial.print(ruletable[40]);Serial.print("||");Serial.print(ruletable[41]);
  Serial.println();
  Serial.print("              "); Serial.print(ruletable[42]);Serial.print("||");Serial.print(ruletable[43]);Serial.print("||");Serial.print(ruletable[44]);Serial.print("||");
  Serial.print(ruletable[45]);Serial.print("||");Serial.print(ruletable[46]);Serial.print("||");Serial.print(ruletable[47]);Serial.print("||");Serial.print(ruletable[49]);
  Serial.println();
*/
}

// =========== DEFUZZIFIKASI ============= //
void defuzzifikasi() {
  float totalKpevalxbobot = 0, totalKievalxbobot = 0, totalKdevalxbobot = 0;
  float Kpeval = 0, Kieval = 0, Kdeval = 0;
  
  float Kpevalxbobot1, Kpevalxbobot2, Kpevalxbobot3, Kpevalxbobot4, Kpevalxbobot5, Kpevalxbobot6, Kpevalxbobot7;
  float Kievalxbobot1, Kievalxbobot2, Kievalxbobot3, Kievalxbobot4, Kievalxbobot5, Kievalxbobot6, Kievalxbobot7;
  float Kdevalxbobot1, Kdevalxbobot2, Kdevalxbobot3, Kdevalxbobot4, Kdevalxbobot5, Kdevalxbobot6, Kdevalxbobot7;
  
  float dKp[7] = {-0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3}; //{0,0,0,0,0,0,0};//              // variabel output delta Kp
  float dKi[7] = {-0.03, -0.02, -0.01, 0, 0.01, 0.02, 0.03}; //{0,0,0,0,0,0,0};//              // variabel output delta Ki
  float dKd[7] = {-0.05, -0.02, -0.01, 0, 0.01, 0.02, 0.05}; //{0,0,0,0,0,0,0};//              // variabel output delta Kd

  //Serial.print("memberdKp = ");Serial.print(dKp[0]);Serial.print("||");Serial.print(dKp[1]);Serial.print("||");Serial.print(dKp[2]);Serial.print("||");Serial.print(dKp[3]);
  //Serial.print("||");Serial.print(dKp[4]);Serial.print("||");Serial.print(dKp[5]);Serial.print("||");Serial.print(dKp[6]);
  //Serial.println();
  
  // defuzzifikasi Kp
  Kpevalxbobot1 = (ruletable[0]*dKp[6])+(ruletable[1]*dKp[6])+(ruletable[2]*dKp[5])+(ruletable[3]*dKp[5])+(ruletable[4]*dKp[4])+(ruletable[5]*dKp[3])+(ruletable[6]*dKp[3]);
  Kpevalxbobot2 = (ruletable[7]*dKp[6])+(ruletable[8]*dKp[6])+(ruletable[9]*dKp[5])+(ruletable[10]*dKp[4])+(ruletable[11]*dKp[4])+(ruletable[12]*dKp[3])+(ruletable[13]*dKp[3]);
  Kpevalxbobot3 = (ruletable[14]*dKp[5])+(ruletable[15]*dKp[5])+(ruletable[16]*dKp[5])+(ruletable[17]*dKp[4])+(ruletable[18]*dKp[3])+(ruletable[19]*dKp[2])+(ruletable[20]*dKp[2]); 
  Kpevalxbobot4 = (ruletable[21]*dKp[5])+(ruletable[22]*dKp[5])+(ruletable[23]*dKp[4])+(ruletable[24]*dKp[3])+(ruletable[25]*dKp[2])+(ruletable[26]*dKp[1])+(ruletable[27]*dKp[1]);
  Kpevalxbobot5 = (ruletable[28]*dKp[4])+(ruletable[29]*dKp[4])+(ruletable[30]*dKp[3])+(ruletable[31]*dKp[2])+(ruletable[32]*dKp[2])+(ruletable[33]*dKp[1])+(ruletable[34]*dKp[1]);
  Kpevalxbobot6 = (ruletable[35]*dKp[4])+(ruletable[36]*dKp[3])+(ruletable[37]*dKp[2])+(ruletable[38]*dKp[1])+(ruletable[39]*dKp[1])+(ruletable[40]*dKp[1])+(ruletable[41]*dKp[0]);
  Kpevalxbobot7 = (ruletable[42]*dKp[3])+(ruletable[43]*dKp[3])+(ruletable[44]*dKp[1])+(ruletable[45]*dKp[1])+(ruletable[46]*dKp[1])+(ruletable[47]*dKp[0])+(ruletable[48]*dKp[0]);

  totalKpevalxbobot = Kpevalxbobot1 + Kpevalxbobot2 + Kpevalxbobot3 + Kpevalxbobot4 + Kpevalxbobot5 + Kpevalxbobot6 + Kpevalxbobot7;
  for(int i=0; i<49;i++){
    if(ruletable[i] != 0){
    Kpeval += ruletable[i];
    }
  }
  Kpf = (totalKpevalxbobot/Kpeval)*GainKp;
  //Serial.print("Debug Kpeval = ");Serial.print(Kpeval);
  //Serial.println();
  //Serial.print("Debug Kpf = ");Serial.print(Kpf);
  //Serial.println();

  // defuzzifikasi Ki
  Kievalxbobot1 = (ruletable[0]*dKi[0])+(ruletable[1]*dKi[0])+(ruletable[2]*dKi[1])+(ruletable[3]*dKi[1])+(ruletable[4]*dKi[2])+(ruletable[5]*dKi[3])+(ruletable[6]*dKi[3]);
  Kievalxbobot2 = (ruletable[7]*dKi[0])+(ruletable[8]*dKi[0])+(ruletable[9]*dKi[1])+(ruletable[10]*dKi[2])+(ruletable[11]*dKi[2])+(ruletable[12]*dKi[3])+(ruletable[13]*dKi[3]);
  Kievalxbobot3 = (ruletable[14]*dKi[1])+(ruletable[15]*dKi[1])+(ruletable[16]*dKi[2])+(ruletable[17]*dKi[2])+(ruletable[18]*dKi[3])+(ruletable[19]*dKi[4])+(ruletable[20]*dKi[4]); 
  Kievalxbobot4 = (ruletable[21]*dKi[1])+(ruletable[22]*dKi[1])+(ruletable[23]*dKi[3])+(ruletable[24]*dKi[4])+(ruletable[25]*dKi[4])+(ruletable[26]*dKi[5])+(ruletable[27]*dKi[5]);
  Kievalxbobot5 = (ruletable[28]*dKi[1])+(ruletable[29]*dKi[2])+(ruletable[30]*dKi[3])+(ruletable[31]*dKi[4])+(ruletable[32]*dKi[4])+(ruletable[33]*dKi[5])+(ruletable[34]*dKi[5]);
  Kievalxbobot6 = (ruletable[35]*dKi[3])+(ruletable[36]*dKi[3])+(ruletable[37]*dKi[4])+(ruletable[38]*dKi[4])+(ruletable[39]*dKi[5])+(ruletable[40]*dKi[6])+(ruletable[41]*dKi[6]);
  Kievalxbobot7 = (ruletable[42]*dKi[3])+(ruletable[43]*dKi[3])+(ruletable[44]*dKi[4])+(ruletable[45]*dKi[5])+(ruletable[46]*dKi[5])+(ruletable[47]*dKi[6])+(ruletable[48]*dKi[6]);

  totalKievalxbobot = Kievalxbobot1 + Kievalxbobot2 + Kievalxbobot3 + Kievalxbobot4 + Kievalxbobot5 + Kievalxbobot6 + Kievalxbobot7;
  for(int i=0; i<49;i++){
    if(ruletable[i] != 0){
    Kieval += ruletable[i];
    }
  }
  Kif = (totalKievalxbobot/Kieval)*GainKi;
  //Serial.print("Debug Kieval = ");Serial.print(Kieval);
  //Serial.println();
  //Serial.print("Debug Kif = ");Serial.print(Kif);
  //Serial.println();

  // defuzzifikasi Kd
  Kdevalxbobot1 = (ruletable[0]*dKd[4])+(ruletable[1]*dKd[2])+(ruletable[2]*dKd[0])+(ruletable[3]*dKd[0])+(ruletable[4]*dKd[0])+(ruletable[5]*dKd[1])+(ruletable[6]*dKd[4]);
  Kdevalxbobot2 = (ruletable[7]*dKd[4])+(ruletable[8]*dKd[2])+(ruletable[9]*dKd[0])+(ruletable[10]*dKd[1])+(ruletable[11]*dKd[1])+(ruletable[12]*dKd[2])+(ruletable[13]*dKd[3]);
  Kdevalxbobot3 = (ruletable[14]*dKd[3])+(ruletable[15]*dKd[2])+(ruletable[16]*dKd[1])+(ruletable[17]*dKd[1])+(ruletable[18]*dKd[2])+(ruletable[19]*dKd[2])+(ruletable[20]*dKd[3]); 
  Kdevalxbobot4 = (ruletable[21]*dKd[3])+(ruletable[22]*dKd[2])+(ruletable[23]*dKd[2])+(ruletable[24]*dKd[3])+(ruletable[25]*dKd[2])+(ruletable[26]*dKd[2])+(ruletable[27]*dKd[3]);
  Kdevalxbobot5 = (ruletable[28]*dKd[3])+(ruletable[29]*dKd[3])+(ruletable[30]*dKd[3])+(ruletable[31]*dKd[3])+(ruletable[32]*dKd[3])+(ruletable[33]*dKd[3])+(ruletable[34]*dKd[3]);
  Kdevalxbobot6 = (ruletable[35]*dKd[6])+(ruletable[36]*dKd[2])+(ruletable[37]*dKd[4])+(ruletable[38]*dKd[4])+(ruletable[39]*dKd[4])+(ruletable[40]*dKd[4])+(ruletable[41]*dKd[6]);
  Kdevalxbobot7 = (ruletable[42]*dKd[6])+(ruletable[43]*dKd[5])+(ruletable[44]*dKd[5])+(ruletable[45]*dKd[5])+(ruletable[46]*dKd[4])+(ruletable[47]*dKd[4])+(ruletable[48]*dKd[4]);

  totalKdevalxbobot = Kdevalxbobot1 + Kdevalxbobot2 + Kdevalxbobot3 + Kdevalxbobot4 + Kdevalxbobot5 + Kdevalxbobot6 + Kdevalxbobot7;
  for(int i=0; i<49;i++){
    if(ruletable[i] != 0){
    Kdeval += ruletable[i];
    }
  }
  Kdf = (totalKdevalxbobot/Kdeval)*GainKd;
  //Serial.print("Debug Kdeval = ");Serial.print(Kdeval);
  //Serial.println();
  //Serial.print("totalKdevalxbobot = ");Serial.print(totalKdevalxbobot);
  //Serial.println();
  //Serial.print("Debug Kdf = ");Serial.print(Kdf);
  //Serial.println();
  //Serial.print("Debugging Kpf, Kif, Kdf = "); Serial.print(Kpf); Serial.print("||");Serial.print(Kif);Serial.print("||");Serial.print(Kdf);
  //Serial.println();

}

void constPID(float vKp,float vKi,float vKd,int vmode) {
  consKp = vKp;
  consKi = vKi;
  consKd = vKd;
  mode = vmode;
}

void Sensorvalue(float vPv) {
  pv = vPv;
}

void setpoint(float vsp) {
  sp = vsp;
}

void timesampling(int ts) {
  Tc = ts;
}

void calcFuzzyPID_Roll(float readsensor, float inputvalue, float Kp, float Ki, float Kd, float tsampling, int modePID) {
  
  Sensorvalue(readsensor);
  setpoint(inputvalue);
  
  // calc error and delta error fuzzyinput()
    error  = sp - pv;
  if(error >= MAXerror){
    error = MAXerror;
  } else if (error <= MINerror){
    error = MINerror;
  }
  unsigned long counter = micros();
  if(counter - previousMillis >= 3000000) {
  previouserror  = error;
  previousMillis = counter;
  }

  derror = error - previouserror;
  if(derror >= MAXderror){
    derror = MAXderror;
  } else if (derror <= MINderror){
    derror = MINderror;
  }

//  Serial.print("DEbug error! = "); Serial.print(error); Serial.print("||"); Serial.print(derror);
//  Serial.println();
  
  // fuzzy logic controller
  fuzzifikasi();
  Ruleevaluation();
  defuzzifikasi();
  constPID(Kp,Ki,Kd,modePID);
  timesampling(tsampling);

  // PID controller
  if(first == 1) {
    first = 0;
    derivativeE = error;
  }

  // time sampling detection
  unsigned long Time = micros();
  if(Time - lastTimeTC >= (Tc * 1000)){
    float tc = (Time - lastTimeTC) / 1000000.0;
    lastTimeTC = Time;

    // Proportional calculation
    outP = (consKp + Kpf) * error;

    // Integral calculation
    integralE = error;
    outI = (consKi + Kif) * integralE * tc;

    // Derivative calculation
    derivativeE = error - derivativeE;
    outD = ((consKd + Kdf) * derivativeE) / tc;
    derivativeE = error;

    // delta U output
    dU = outP + outI + outD;
//    Serial.print("debug output PID = "); 
//    Serial.print(dU); Serial.print(","); 
//    Serial.print("|| Debug [P],[I],[D]: "); 
//    Serial.print(outP);Serial.print(",");Serial.print(",");Serial.print(outD);
//    Serial.print(outI);
//    Serial.println();

    // case mod
    if(mode==1) {
      outPID_roll += dU;
    } else{ outPID_roll = dU;}

    // limit output
    if(maxOut != 0 || minOut != 0) {
      if(outPID_roll >= maxOut) {
        outPID_roll = maxOut;
        integralE -= error;
      } else if (outPID_roll <= minOut) {
        outPID_roll = minOut;
        integralE -= error;
      }
    }
//    Serial.print("Output PID = "); Serial.print(outPID_roll);
//    Serial.print("Integral = "); Serial.print(integralE);
//    Serial.println();
    return outPID_roll;
  }
}
