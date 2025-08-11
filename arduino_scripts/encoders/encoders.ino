#include <TimerOne.h>
#include <Encoder.h>
// define motors pins
#define M1R_DIR 5	// RIGHT motor direction | Set pin to LOW to move front ! HIGH to move back
#define M1R_PWM 4	// Set 0 to 255 to set RIGHT motor speed
#define M2L_DIR 6	// LEFT motor direction  | Set pin to LOW to move front ! HIGH to move back
#define M2L_PWM 9	// Set 0 to 255 to set LEFT motor speed

// define range sensors pins (IR sensors)
#define IR_FRONT A2	// IF1
#define IR_RIGHT A3	// IF2
#define IR_LEFT  A4	// IF3

// Encoders Pins
#define ENC_M1R_A 3// RIGHT encoder channel A - C1
#define ENC_M1R_B 2	// RIGHT encoder channel B - C2
#define ENC_M2L_A 18 	// LEFT encoder channel A - C1
#define ENC_M2L_B 19 	// LEFT encoder channel B - C2

// SMART LED Pin
#define SMART_LED_P 10
#define NUMPIXELS   2

// geometrical parameters
float r = 1.6;
float b = 10.6;
int gear_box=298;
float C = 0.00241;


Encoder enc_left(ENC_M2L_A,ENC_M2L_B);
Encoder enc_right(ENC_M1R_A,ENC_M1R_B);



volatile bool flag = false;
long enc_left_count=0;
long enc_right_count=0;
float DL=0;
float DR=0;
void setup() {
  Serial.begin(9600);
  Timer1.initialize(100);         // 10Hz = 100 ms
  Timer1.attachInterrupt(callback);  
  digitalWrite(M1R_DIR ,LOW);
  digitalWrite(M2L_DIR ,LOW);
 digitalWrite(M1R_PWM,200);
  digitalWrite(M2L_PWM,200);
  
}

void loop() {
  if (flag) {
   
    flag = false;
  }
   
  if(DL>=10.0 ){
    digitalWrite(M1R_PWM,0);
  digitalWrite(M2L_PWM,0);
  }
  


}

void callback() {
  long right=enc_right.read();
  long left= enc_left.read();
  long count_left= left-enc_left_count;
  long count_right= right -enc_right_count;
  enc_left_count= left;
  enc_right_count= right;
  DL= DL+C*count_left;
  DR= DR+C*count_right;
  flag = true;  
}

