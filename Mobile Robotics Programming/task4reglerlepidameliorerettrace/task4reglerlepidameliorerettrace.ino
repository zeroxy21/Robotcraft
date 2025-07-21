#include "math.h"

// Define motor pins
#define M1R_DIR 5   // RIGHT motor direction
#define M1R_PWM 4   // RIGHT motor speed
#define M2L_DIR 6   // LEFT motor direction
#define M2L_PWM 9   // LEFT motor speed

// Encoder pins
#define ENC_M1R_A 18  // RIGHT encoder A
#define ENC_M1R_B 19  // RIGHT encoder B
#define ENC_M2L_A 3   // LEFT encoder A
#define ENC_M2L_B 2   // LEFT encoder B

// Encoder variables
volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;
long lastLeftCount = 0;
long lastRightCount = 0;

// PID variables
unsigned long lastTime = 0;
const unsigned long sampleTime = 100;  // 100ms
float Kp = 1.1, Ki = 0.05, Kd = 0.1;
float previousErrorLeft = 0, previousErrorRight = 0;
float integralLeft = 0, integralRight = 0;

// Test variables
float desiredSpeed = 0;
bool testRunning = false;
unsigned long testStartTime = 0;
const float maxTestSpeed = 100.0;

// Reduced data logging
const int maxDataPoints = 100;  // Reduced from 500
float timeData[maxDataPoints];
float leftSpeedData[maxDataPoints];
int dataIndex = 0;

void setup() {
  Serial.begin(115200);
  
  // Motor configuration
  pinMode(M1R_DIR, OUTPUT);
  pinMode(M1R_PWM, OUTPUT);
  pinMode(M2L_DIR, OUTPUT);
  pinMode(M2L_PWM, OUTPUT);
  
  // Encoder configuration
  pinMode(ENC_M2L_A, INPUT);
  pinMode(ENC_M2L_B, INPUT);
  pinMode(ENC_M1R_A, INPUT);
  pinMode(ENC_M1R_B, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENC_M2L_A), leftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_M1R_A), rightEncoder, CHANGE);
  
  Serial.println(F("System ready. Send 's' to start test."));
}

void loop() {
  unsigned long currentTime = millis();
  
  // Start test on serial command
  if (Serial.available() && !testRunning) {
    if (Serial.read() == 's') {
      startTest();
    }
  }
  
  if (testRunning) {
    // Update desired speed (ramp up over 1 second)
    float elapsed = (currentTime - testStartTime) / 1000.0;
    desiredSpeed = min(maxTestSpeed, maxTestSpeed * elapsed);
    
    // End test after 5 seconds
    if (currentTime - testStartTime >= 5000) {
      endTest();
    }
  }
  
  if (currentTime - lastTime >= sampleTime) {
    // Calculate speed
    float leftSpeed = (encoderLeftCount - lastLeftCount) * (1000.0 / sampleTime);
    float rightSpeed = (encoderRightCount - lastRightCount) * (1000.0 / sampleTime);
    
    lastLeftCount = encoderLeftCount;
    lastRightCount = encoderRightCount;
    lastTime = currentTime;
    
    // PID control
    float leftOutput = pid_controller(desiredSpeed, leftSpeed, &integralLeft, &previousErrorLeft);
    float rightOutput = pid_controller(desiredSpeed, rightSpeed, &integralRight, &previousErrorRight);
    
    // Motor control
    digitalWrite(M2L_DIR, LOW);
    analogWrite(M2L_PWM, constrain(abs(leftOutput), 0, 255));
    digitalWrite(M1R_DIR, LOW);
    analogWrite(M1R_PWM, constrain(abs(rightOutput), 0, 255));
    
    // Log only left speed to save memory
    if (testRunning && dataIndex < maxDataPoints) {
      timeData[dataIndex] = (currentTime - testStartTime) / 1000.0;
      leftSpeedData[dataIndex] = leftSpeed;
      dataIndex++;
    }
    
    // Reduced serial output
    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime >= 200) {  // Print every 200ms
      Serial.print(F("T:"));
      Serial.print((currentTime - testStartTime) / 1000.0, 1);
      Serial.print(F(",D:"));
      Serial.print(desiredSpeed, 0);
      Serial.print(F(",L:"));
      Serial.print(leftSpeed, 0);
      Serial.print(F(",R:"));
      Serial.println(rightSpeed, 0);
      lastPrintTime = currentTime;
    }
  }
}

void startTest() {
  testRunning = true;
  testStartTime = millis();
  dataIndex = 0;
  desiredSpeed = 0;
  integralLeft = integralRight = 0;
  previousErrorLeft = previousErrorRight = 0;
  Serial.println(F("Test started..."));
}

void endTest() {
  testRunning = false;
  analogWrite(M2L_PWM, 0);
  analogWrite(M1R_PWM, 0);
  
  // Print collected data
  Serial.println(F("Test data (Time,LeftSpeed):"));
  for (int i = 0; i < dataIndex; i++) {
    Serial.print(timeData[i], 2);
    Serial.print(',');
    Serial.println(leftSpeedData[i], 0);
  }
  Serial.println(F("Test complete. Send 's' to run again."));
}

void leftEncoder() {
  encoderLeftCount += digitalRead(ENC_M2L_B) ? -1 : 1;
}

void rightEncoder() {
  encoderRightCount += digitalRead(ENC_M1R_B) ? -1 : 1;
}

float pid_controller(float desired, float actual, float* integral, float* prevError) {
  float error = desired - actual;
  *integral += error;
  if (Ki != 0) *integral = constrain(*integral, -255/Ki, 255/Ki);
  float output = Kp * error + Ki * (*integral) + Kd * (error - *prevError);
  *prevError = error;
  return output;
}