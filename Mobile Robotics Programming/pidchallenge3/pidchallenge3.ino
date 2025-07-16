
#include "math.h"
#include "encoder.h"
// define motors pins
#define M1R_DIR 5  // RIGHT motor direction | Set pin to LOW to move front ! HIGH to move back
#define M1R_PWM 4 // Set 0 to 255 to set RIGHT motor speed
#define M2L_DIR 6 // LEFT motor direction  | Set pin to LOW to move front ! HIGH to move back
#define M2L_PWM 9 // Set 0 to 255 to set LEFT motor speed
// Encoders Pins
#define ENC_M1R_A 18  // RIGHT encoder channel A - C1
#define ENC_M1R_B 19  // RIGHT encoder channel B - C2
#define ENC_M2L_A 3 // LEFT encoder channel A - C1
#define ENC_M2L_B 2 // LEFT encoder channel B - C2
volatile long encoderCount = 0;
long lastEncoderCount = 0;
unsigned long lastTime = 0;
float Kp = 1.5;
float Ki = 0.5;
float Kd = 0.1;

float previousError = 0;
float integral = 0;
void setup() {
  // put your setup code here, to run once:
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(3),encFunc,CHANGE);
  lastTime = millis();

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(encoderCount);
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 100) { 

    long pulses = encoderCount - lastEncoderCount;
    float realSpeed = pulses;
    output=pid_controller;
    analogWrite(9, (int)output);
    previousError = error;
    lastEncoderCount = encoderCount;
    lastTime = currentTime;

}
void encFunc(){
  encoderCount++;
}
float pid_controller(float desiredSpeed, float realSpeed) {
  float error = desiredSpeed - realSpeed;

  integral += error;
  float derivative = error - previousError;

  float output = Kp * error + Ki * integral + Kd * derivative;

  previousError = error;

  return constrain(output, 0, 255);
}
