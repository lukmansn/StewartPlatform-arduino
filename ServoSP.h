// Servo spesifikasi
#define MIN_PULSE_WIDTH   450
#define MAX_PULSE_WIDTH   2700     // 3000
#define FREQUENCY         50       // 50

// Define I/O pin servo
int static servo[6] = {0,1,2,3,4,5};  // pin servo di PCA9685

// Define servo position number
// servo positive
#define ser_1 0
#define ser_3 2
#define ser_5 4
// servo negative
#define inverse_ser_2 1 
#define inverse_ser_4 3
#define inverse_ser_6 5
