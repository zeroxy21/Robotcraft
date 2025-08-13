#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <TimerOne.h>
#include <Encoder.h>
#include <math.h>

// --- Pins moteurs ---
#define M1R_DIR 5
#define M1R_PWM 4
#define M2L_DIR 6
#define M2L_PWM 9

// --- Pins capteurs IR ---
#define IR_FRONT A2
#define IR_RIGHT A3
#define IR_LEFT  A4

// --- Pins encodeurs ---
#define ENC_M1R_A 3
#define ENC_M1R_B 2
#define ENC_M2L_A 18
#define ENC_M2L_B 19

// --- Paramètres géométriques ---
float r = 0.016;     // rayon roue en m
float b = 0.106;     // entraxe en m
float C = 0.00241;   // distance parcourue par tick

Encoder enc_left(ENC_M2L_A, ENC_M2L_B);
Encoder enc_right(ENC_M1R_A, ENC_M1R_B);

// --- Variables odométrie ---
volatile bool flag = false;
long enc_left_count = 0;
long enc_right_count = 0;

float x = 0.0, y = 0.0, theta = 0.0;

// --- Variables PID ---
float targetLeft = 0.0, targetRight = 0.0;
float Kp = 6.8755, Ki = 5.5004, Kd = 1.4921;
float integralLeft = 0.0, integralRight = 0.0;
float prevErrorLeft = 0.0, prevErrorRight = 0.0;
long lastPIDTime = 0;
long lastLeftCountPID = 0, lastRightCountPID = 0;

// --- Flag pour vérifier si commande reçue ---
bool cmd_vel_received = false;

// --- ROS ---
ros::NodeHandle nh;

std_msgs::Float32 front_Ir_msg;
std_msgs::Float32 right_Ir_msg;
std_msgs::Float32 left_Ir_msg;
geometry_msgs::Pose2D pose_msg;

ros::Publisher front_Ir("/front_distance", &front_Ir_msg);
ros::Publisher right_Ir("/right_distance", &right_Ir_msg);
ros::Publisher left_Ir("/left_distance", &left_Ir_msg);
ros::Publisher pose_pub("/pose", &pose_msg);

// --- Odométrie ---
void callback_encoders() {
    long right = enc_right.read();
    long left  = enc_left.read();

    long count_left  = left - enc_left_count;
    long count_right = right - enc_right_count;

    enc_left_count  = left;
    enc_right_count = right;

    float dL = C * count_left;
    float dR = C * count_right;

    float dS     = (dR + dL) / 2.0;
    float dTheta = (dR - dL) / b;

    x     += dS * cos(theta + dTheta / 2.0);
    y     += dS * sin(theta + dTheta / 2.0);
    theta += dTheta;

    if (theta > M_PI)  theta -= 2.0 * M_PI;
    if (theta < -M_PI) theta += 2.0 * M_PI;

    pose_msg.x = x;
    pose_msg.y = y;
    pose_msg.theta = theta;

    flag = true;
}

// --- PID ---
float pid_controller(float desired, float real, float *integral, float *prevErr) {
    float error = desired - real;
    *integral += error;
    *integral = constrain(*integral, -255.0/Ki, 255.0/Ki);
    float derivative = error - *prevErr;
    float output = Kp*error + Ki*(*integral) + Kd*derivative;
    *prevErr = error;
    return constrain(output, -255.0, 255.0);
}

// --- Conversion cmd_vel → vitesses roues ---
void cmd_vel2wheels(float V, float W, float* wl, float* wr) {
    float base = V / r;
    float delta = (b * W) / (2.0 * r);
    *wl = base - delta;
    *wr = base + delta;
}

// --- ROS callbacks ---
void callback_cmd_vel(const geometry_msgs::Twist &msg) {
    float wl, wr;
    cmd_vel2wheels(msg.linear.x, msg.angular.z, &wl, &wr);

    targetLeft  = wl * 0.1;
    targetRight = wr * 0.1;

    cmd_vel_received = true;
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &callback_cmd_vel);

void setup() {
    nh.initNode();

    pinMode(M1R_DIR, OUTPUT);
    pinMode(M1R_PWM, OUTPUT);
    pinMode(M2L_DIR, OUTPUT);
    pinMode(M2L_PWM, OUTPUT);

    Timer1.initialize(100000); // 100ms
    Timer1.attachInterrupt(callback_encoders);

    nh.advertise(front_Ir);
    nh.advertise(right_Ir);
    nh.advertise(left_Ir);
    nh.advertise(pose_pub);

    nh.subscribe(cmd_vel_sub);
}

unsigned long last_pub_time = 0;

void loop() {
    unsigned long now = millis();

    // --- PID loop tous les 100 ms ---
    if (now - lastPIDTime >= 100) {
        long leftCount = enc_left.read();
        long rightCount = enc_right.read();

        float leftSpeed = (leftCount - lastLeftCountPID) * 10.0;
        float rightSpeed = (rightCount - lastRightCountPID) * 10.0;

        lastLeftCountPID = leftCount;
        lastRightCountPID = rightCount;
        lastPIDTime = now;

        if(cmd_vel_received) {
            float leftOut = pid_controller(targetLeft, leftSpeed, &integralLeft, &prevErrorLeft);
            float rightOut = pid_controller(targetRight, rightSpeed, &integralRight, &prevErrorRight);

            // --- Inversion sens roue pour avancer correctement ---
            digitalWrite(M2L_DIR, (leftOut < 0) ? LOW : HIGH);
            analogWrite(M2L_PWM, constrain(abs(leftOut), 0, 255));

            digitalWrite(M1R_DIR, (rightOut < 0) ? LOW : HIGH);
            analogWrite(M1R_PWM, constrain(abs(rightOut), 0, 255));
        } else {
            analogWrite(M1R_PWM, 0);
            analogWrite(M2L_PWM, 0);
        }
    }

    // --- Publication ROS tous les 100 ms ---
    if (now - last_pub_time >= 100) {
        last_pub_time = now;
        if (flag) {
            pose_pub.publish(&pose_msg);
            flag = false;
        }
    }

    nh.spinOnce();
}

