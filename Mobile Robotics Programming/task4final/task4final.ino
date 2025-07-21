#include "math.h"

// Motor pins
#define M1R_DIR 5    // RIGHT motor direction
#define M1R_PWM 4    // RIGHT motor PWM
#define M2L_DIR 6    // LEFT motor direction
#define M2L_PWM 9    // LEFT motor PWM

// Encoder pins
#define ENC_M1R_A 18  // RIGHT encoder channel A
#define ENC_M1R_B 19  // RIGHT encoder channel B
#define ENC_M2L_A 3   // LEFT encoder channel A
#define ENC_M2L_B 2   // LEFT encoder channel B

// Robot parameters
const int r = 50;     // Wheel radius (mm)
const int b = 200;    // Wheelbase (mm)
const int C = 100;    // Encoder resolution (ticks/rev)

// Global variables
volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;
long lastLeftCount = 0;
long lastRightCount = 0;
unsigned long lastTime = 0;

double x = 0, y = 0, teta = 0;

// PID variables
float Kp = 0.5, Ki = 0, Kd = 0;
float previousErrorLeft = 0, previousErrorRight = 0;
float integralLeft = 0, integralRight = 0;

void setup() {
  Serial.begin(115200);
  
  // Motor control setup
  pinMode(M1R_DIR, OUTPUT);
  pinMode(M1R_PWM, OUTPUT);
  pinMode(M2L_DIR, OUTPUT);
  pinMode(M2L_PWM, OUTPUT);
  
  // Encoder setup
  pinMode(ENC_M2L_A, INPUT);
  pinMode(ENC_M2L_B, INPUT);
  pinMode(ENC_M1R_A, INPUT);
  pinMode(ENC_M1R_B, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENC_M2L_A), leftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_M1R_A), rightEncoder, CHANGE);
  
  lastTime = millis();
}

void loop() {
  // Handle velocity commands
  if (Serial.available() > 0) {
    double V, W;
    cmd_vel(&V, &W);
    
    double wl, wr;
    cmd_vel2wheels(V, W, &wl, &wr);
    
    // Convert angular velocities to encoder targets
    float desiredLeftSpeed = (wl * C) / (2 * M_PI);
    float desiredRightSpeed = (wr * C) / (2 * M_PI);
    
    // Store targets for PID control
    static float targetLeft = 0, targetRight = 0;
    targetLeft = desiredLeftSpeed;
    targetRight = desiredRightSpeed;
  }

  // PID control at fixed interval
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 100) {  // 100ms sampling
    // Calculate actual speeds
    float leftSpeed = encoderLeftCount - lastLeftCount;
    float rightSpeed = encoderRightCount - lastRightCount;
    
    // Reset counters
    lastLeftCount = encoderLeftCount;
    lastRightCount = encoderRightCount;
    lastTime = currentTime;
    
    // Update odometry
    int NL = leftSpeed;
    int NR = rightSpeed;
    double x1, y1, teta1, V, W;
    poseUpdate1(NL, NR, r, b, C, &x1, &y1, &teta1, &V, &W);
    
    x += x1;
    y += y1;
    teta += teta1;
    
    // PID control (using targets from cmd_vel)
    float leftOutput = pid_controller(targetLeft, leftSpeed, &integralLeft, &previousErrorLeft);
    float rightOutput = pid_controller(targetRight, rightSpeed, &integralRight, &previousErrorRight);
    
    // Motor control with direction
    digitalWrite(M2L_DIR, (targetLeft < 0) ? HIGH : LOW);
    analogWrite(M2L_PWM, constrain(abs(leftOutput), 0, 255));
    
    digitalWrite(M1R_DIR, (targetRight < 0) ? HIGH : LOW);
    analogWrite(M1R_PWM, constrain(abs(rightOutput), 0, 255));
    
    // Debug output
    Serial.print("Pos:(");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.print(") Theta:");
    Serial.print(teta);
    Serial.print(" L:");
    Serial.print(leftSpeed);
    Serial.print(" R:");
    Serial.println(rightSpeed);
  }
}

// Encoder interrupts
void leftEncoder() {
  if (digitalRead(ENC_M2L_B)) {
    encoderLeftCount--;
  } else {
    encoderLeftCount++;
  }
}

void rightEncoder() {
  if (digitalRead(ENC_M1R_B)) {
    encoderRightCount--;
  } else {
    encoderRightCount++;
  }
}

// PID controller (your original function)
float pid_controller(float desiredSpeed, float realSpeed, float* integral, float* previousError) {
  float error = desiredSpeed - realSpeed;
  
  *integral += error;
  *integral = constrain(*integral, -255/Ki, 255/Ki);  // Anti-windup
  
  float derivative = error - *previousError;
  float output = Kp * error + Ki * (*integral) + Kd * derivative;
  
  *previousError = error;
  
  return output;
}

// Your original functions with minimal changes
void cmd_vel(double* V, double* W) {
  while (Serial.available() < 1) delay(10);
  *V = Serial.parseFloat();
  
  while (Serial.available() < 1) delay(10);
  *W = Serial.parseFloat();
  
  Serial.print("Cmd: V=");
  Serial.print(*V);
  Serial.print(" W=");
  Serial.println(*W);
}

void cmd_vel2wheels(double V, double W, double* wl, double* wr) {
  *wl = (V - 0.5 * b * W) / r;
  *wr = (V + 0.5 * b * W) / r;
}

void poseUpdate1(int NL, int NR, int r, int b, int C, double* x, double* y, double* theta, double* V, double* W) {
  double Dl = 2 * M_PI * r * NL / C;
  double Dr = 2 * M_PI * r * NR / C;
  
  *theta = (Dr - Dl) / b;
  *x = ((Dl + Dr) / 2) * cos(*theta);
  *y = ((Dl + Dr) / 2) * sin(*theta);
  *V = (Dr + Dl) / 2;
  *W = (Dr - Dl) / b;
}
