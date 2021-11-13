/* ======== General setup untuk kinematika stewart platform ======== */
// set servo posisi 90 deg
static int servo_zero_pos[6] = {950, 2210, 850, 2350, 950, 2200};           //{1000, 2100, 900, 2270, 1000, 2150};        // << perlu dicek satu satu lagi

// variable array untuk translasi (x,y,z) dan rotasi (roll, pitch, yaw)
static float trans_rot[6] = {0,0.0,0, 0, 0, 0};                            // << simpan variable masukan
static float alpha[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};                    // << simpan variable sudut servo saat melakukan rotasi
static float alpha0[6];

int servo_pos[6]={0};

// servo min dan max posisi horizontal
const float SERVO_Mult = 400/(PI/4); // = 509.554
                     
// Spesifikasi sudut - sudut
// sudut basis
const float sudutbasis[]    = {240, 300, 360, 60, 120, 180};
const float basisbelok[]    = {3.078, -3.078, 3.078, -3.078, 3.078, -3.078};
// sudut platform
const float sudutplatform[] = {220, 320, 340, 75, 105, 200};
// sudut beta
const float Beta[] = {0, -PI, PI*2/3, -PI/3, PI*4/3, -PI/3*5}; // 0, -180, 120, -60 (300), 240, -300 (60) //0, -PI, PI*2/3, -PI/3, PI*4/3, -PI/3*5               
// Spesifikasi ukuran basis dan platform
const float tinggi_awal      = 155.00         // skala tinggi awal
            ,radius_basis    = 93.115         // jari jari basis round 1 = 84.32 | jari jari basis round 2 = 93.115
            ,radius_platform = 79.253         // jari jari platform
            ,panjang_arm     = 24
            ,panjang_kaki    = 165;
// Penentuan vektor posisi dari joint Basis
const float Bi[3][6]={
  {
     radius_basis * cos(radians(sudutbasis[0])) + radians(basisbelok[0]),
     radius_basis * cos(radians(sudutbasis[1])) + radians(basisbelok[1]),
     radius_basis * cos(radians(sudutbasis[2])) + radians(basisbelok[2]),
     radius_basis * cos(radians(sudutbasis[3])) + radians(basisbelok[3]),
     radius_basis * cos(radians(sudutbasis[4])) + radians(basisbelok[4]),
     radius_basis * cos(radians(sudutbasis[5])) + radians(basisbelok[5]),
  },
  {
     radius_basis * sin(radians(sudutbasis[0])) + radians(basisbelok[0]),
     radius_basis * sin(radians(sudutbasis[1])) + radians(basisbelok[1]),
     radius_basis * sin(radians(sudutbasis[2])) + radians(basisbelok[2]),
     radius_basis * sin(radians(sudutbasis[3])) + radians(basisbelok[3]),
     radius_basis * sin(radians(sudutbasis[4])) + radians(basisbelok[4]),
     radius_basis * sin(radians(sudutbasis[5])) + radians(basisbelok[5]),
  },
  {0,0,0,0,0,0}
 },
 Pi_p[3][6]={
  {
    radius_platform * cos(radians(sudutplatform[0])),
    radius_platform * cos(radians(sudutplatform[1])),
    radius_platform * cos(radians(sudutplatform[2])),
    radius_platform * cos(radians(sudutplatform[3])),
    radius_platform * cos(radians(sudutplatform[4])),
    radius_platform * cos(radians(sudutplatform[5])),
  },
  {
    radius_platform * sin(radians(sudutplatform[0])),
    radius_platform * sin(radians(sudutplatform[1])),
    radius_platform * sin(radians(sudutplatform[2])),
    radius_platform * sin(radians(sudutplatform[3])),
    radius_platform * sin(radians(sudutplatform[4])),
    radius_platform * sin(radians(sudutplatform[5])),
  },
  {0,0,0,0,0,0}
 };

// P_Rb = rotation matrix || Qi = T + P_Rb * Pi_p || T = translasi matriks || Home = vektor posisi awal
float P_Rb[3][3], Qi[3][6], T[3], H0[3] = {0,0,tinggi_awal}, Home;
