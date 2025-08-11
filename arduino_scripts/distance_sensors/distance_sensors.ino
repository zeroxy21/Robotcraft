
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
#define ENC_M1R_A 18	// RIGHT encoder channel A - C1
#define ENC_M1R_B 19	// RIGHT encoder channel B - C2
#define ENC_M2L_A 3	// LEFT encoder channel A - C1
#define ENC_M2L_B 2	// LEFT encoder channel B - C2

// SMART LED Pin
#define SMART_LED_P 10
#define NUMPIXELS   2
float sensF(){
    float sum=0.0;
    for(int i=0;i<1000;i++){
      float sens=analogRead(IR_FRONT);
      sum=sum+sens;
    }
    float res=sum/1000.0;
    res=49.7-0.134*res+0.000104*res*res;
    Serial.println(res);
   return res;
}
float sensL(){
   float sum=0.0;
    for(int i=0;i<1000;i++){
      float sens=analogRead(IR_LEFT);
      sum=sum+sens;
    }
    float res=sum/1000.0;
    res=46.9-0.132*res+0.000106*res*res;
    Serial.println(res);
   return res;
}
float sensR(){
  float sum=0.0;
    for(int i=0;i<1000;i++){
      float sens=analogRead(IR_RIGHT);
      sum=sum+sens;
    }
    float res=sum/1000.0;
    res=51.9-0.144*res+0.000114*res*res;
    Serial.println(res);
   return res;
}
void setup() {
  Serial.begin(9600);

}

void loop() {
    sensL();
    delay(1500);
  // put your main code here, to run repeatedly:

}
