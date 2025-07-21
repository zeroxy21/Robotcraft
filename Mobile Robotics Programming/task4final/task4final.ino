#include <math.h>

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
const float r = 50.0f;     // Wheel radius (mm)
const float b = 200.0f;    // Wheelbase (mm)
const int C = 100;         // Encoder resolution (ticks/rev)

// Global variables
volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;
long lastLeftCount = 0;
long lastRightCount = 0;
unsigned long lastTime = 0;

float x = 0.0f, y = 0.0f, theta = 0.0f;
float targetLeft = 0.0f, targetRight = 0.0f;  // Moved from loop() to global scope

// PID variables
float Kp = 6.8755f, Ki = 5.5004f, Kd = 1.4921f;
float previousErrorLeft = 0.0f, previousErrorRight = 0.0f;
float integralLeft = 0.0f, integralRight = 0.0f;

void setup() {
  Serial.begin(115200);
  
  // Motor control setup
  pinMode(M1R_DIR, OUTPUT);
  pinMode(M1R_PWM, OUTPUT);
  pinMode(M2L_DIR, OUTPUT);
  pinMode(M2L_PWM, OUTPUT);
  
  // Encoder setup - enable pullup resistors
  pinMode(ENC_M2L_A, INPUT_PULLUP);
  pinMode(ENC_M2L_B, INPUT_PULLUP);
  pinMode(ENC_M1R_A, INPUT_PULLUP);
  pinMode(ENC_M1R_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENC_M2L_A), leftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_M1R_A), rightEncoder, CHANGE);
}

void loop() {
  // Handle velocity commands
  //if (Serial.available() > 0) {
    float V=255, W=20;
    //cmd_vel(&V, &W);
    
    float wl, wr;
    cmd_vel2wheels(V, W, &wl, &wr);
    
    // Convert angular velocities to encoder targets (ticks per 100ms)
    targetLeft = (wl * C) / (2.0f * M_PI)* 10.0f;
    targetRight = (wr * C) / (2.0f * M_PI)* 10.0f;
 // }

  // PID control at fixed interval
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 50) {  // 100ms sampling
    float deltaT = (currentTime - lastTime) / 1000.0f;  // Convert to seconds
    
    // Calculate actual speeds (ticks per sample period)
    float leftSpeed = (encoderLeftCount - lastLeftCount)* 10.0f ;
    float rightSpeed = (encoderRightCount - lastRightCount)* 10.0f ;
    
    // Reset counters
    lastLeftCount = encoderLeftCount;
    lastRightCount = encoderRightCount;
    lastTime = currentTime;
    
    // Update odometry
    int NL = leftSpeed * deltaT;  // Convert back to ticks for this period
    int NR = rightSpeed * deltaT;
    float x1, y1, theta1, V, W;
    poseUpdate1(NL, NR, r, b, C, &x1, &y1, &theta1, &V, &W);
    
    x += x1;
    y += y1;
    theta += theta1;
    
    // PID control (using targets from cmd_vel)
    float leftOutput = pid_controller(targetLeft, leftSpeed, &integralLeft, &previousErrorLeft);
    float rightOutput = pid_controller(targetRight, rightSpeed, &integralRight, &previousErrorRight);
    
    // Motor control with direction
    digitalWrite(M2L_DIR, (leftOutput < 0) ? HIGH : LOW);
    analogWrite(M2L_PWM, constrain(abs(leftOutput), 0, 255));
    
    digitalWrite(M1R_DIR, (rightOutput < 0) ? HIGH : LOW);
    analogWrite(M1R_PWM, constrain(abs(rightOutput), 0, 255));
    
    // Debug output
    // POUR VOIR L4ERREUR persistante 
    Serial.print("ErrL:"); Serial.print(targetLeft - leftSpeed);
    Serial.print(" ErrR:"); Serial.println(targetRight - rightSpeed);
    //Serial.print("Pos:(");
    //Serial.print(x);
   // Serial.print(",");
    //Serial.print(y);
    //Serial.print(") Theta:");
    //Serial.print(theta);
    //Serial.print(" L:");
    //Serial.print(leftSpeed);
   // Serial.print(" R:");
    //Serial.println(rightSpeed);
  }
}

// Encoder interrupts
void leftEncoder() {
  if (digitalRead(ENC_M2L_A) == digitalRead(ENC_M2L_B)) {
    encoderLeftCount--;
  } else {
    encoderLeftCount++;
  }
}

void rightEncoder() {
  if (digitalRead(ENC_M1R_A) == digitalRead(ENC_M1R_B)) {
    encoderRightCount--;
  } else {
    encoderRightCount++;
  }
}

// PID controller
float pid_controller(float desiredSpeed, float realSpeed, float* integral, float* previousError) {
  float error = desiredSpeed - realSpeed;
  
  *integral += error;
  *integral = constrain(*integral, -255.0f/Ki, 255.0f/Ki);  // Anti-windup
  
  float derivative = error - *previousError;
  float output = Kp * error + Ki * (*integral) + Kd * derivative;
  
  *previousError = error;
  
  return constrain(output, -255.0f, 255.0f);
}

void cmd_vel(float* V, float* W) {
  while (Serial.available() < 1) delay(10);
  *V = Serial.parseFloat();
  
  while (Serial.available() < 1) delay(10);
  *W = Serial.parseFloat();
  
  Serial.print("Cmd: V=");
  Serial.print(*V);
  Serial.print(" W=");
  Serial.println(*W);
}

void cmd_vel2wheels(float V, float W, float* wl, float* wr) {
  float base = V / r;
  float delta = (b * W) / (2.0f * r);

  // On ajoute un offset à une des roues, mais sans inverser le sens
  *wl = base - delta;
  *wr = base + delta;

  // On s'assure que les deux sont positives (si V est trop petit, on les limite à min 0)
  *wl = max(*wl, 0.0f);
  *wr = max(*wr, 0.0f);
}

void poseUpdate1(int NL, int NR, float r, float b, int C, float* x, float* y, float* theta, float* V, float* W) {
  float Dl = 2.0f * M_PI * r * NL / C;
  float Dr = 2.0f * M_PI * r * NR / C;
  
  *theta = (Dr - Dl) / b;
  *x = ((Dl + Dr) / 2.0f) * cos(*theta);
  *y = ((Dl + Dr) / 2.0f) * sin(*theta);
  *V = (Dr + Dl) / 2.0f;
  *W = (Dr - Dl) / b;
}
