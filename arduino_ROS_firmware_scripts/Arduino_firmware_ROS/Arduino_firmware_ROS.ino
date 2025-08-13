#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

#include <TimerOne.h>
#include <Encoder.h>
#include <math.h> // pour M_PI, cos, sin

// === Pins ===
#define IR_FRONT A2
#define IR_RIGHT A3
#define IR_LEFT  A4

#define ENC_M1R_A 3
#define ENC_M1R_B 2
#define ENC_M2L_A 18
#define ENC_M2L_B 19

// === Paramètres géométriques ===
float r = 1.6;       // rayon roue en cm
float b = 10.6;      // entraxe en cm
float C = 0.00241;   // conversion ticks -> cm

Encoder enc_left(ENC_M2L_A, ENC_M2L_B);
Encoder enc_right(ENC_M1R_A, ENC_M1R_B);

// === Variables odométrie ===
volatile bool publish_flag = false;
long enc_left_count = 0;
long enc_right_count = 0;
float x = 0.0, y = 0.0, theta = 0.0;

// === ROS ===
ros::NodeHandle nh;

std_msgs::Float32 front_Ir_msg;
std_msgs::Float32 right_Ir_msg;
std_msgs::Float32 left_Ir_msg;
geometry_msgs::Pose2D pose_msg;

ros::Publisher front_Ir("/front_distance", &front_Ir_msg);
ros::Publisher right_Ir("/right_distance", &right_Ir_msg);
ros::Publisher left_Ir("/left_distance", &left_Ir_msg);
ros::Publisher pose_pub("/pose", &pose_msg);

// === Fonctions capteurs IR ===
float sensF() {
    float sum = 0.0;
    for (int i = 0; i < 100; i++) sum += analogRead(IR_FRONT);
    float res = sum / 100.0;
    return (49.7 - 0.134 * res + 0.000104 * res * res) / 100.0;
}
float sensL() {
    float sum = 0.0;
    for (int i = 0; i < 100; i++) sum += analogRead(IR_LEFT);
    float res = sum / 100.0;
    return (46.9 - 0.132 * res + 0.000106 * res * res) / 100.0;
}
float sensR() {
    float sum = 0.0;
    for (int i = 0; i < 100; i++) sum += analogRead(IR_RIGHT);
    float res = sum / 100.0;
    return (51.9 - 0.144 * res + 0.000114 * res * res) / 100.0;
}

// === Timer ISR : juste mettre un flag ===
void timer_isr() {
    publish_flag = true;
}

// === Mise à jour odométrie ===
void update_odometry() {
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
}

void setup() {
    nh.initNode();

    nh.advertise(front_Ir);
    nh.advertise(right_Ir);
    nh.advertise(left_Ir);
    nh.advertise(pose_pub);

    Timer1.initialize(100000); // 100 ms = 10 Hz
    Timer1.attachInterrupt(timer_isr);
}

void loop() {
    if (publish_flag) {
        publish_flag = false;

        // 1. Mettre à jour odométrie
        update_odometry();

        // 2. Lire capteurs IR
        front_Ir_msg.data = sensF();
        right_Ir_msg.data = sensR();
        left_Ir_msg.data  = sensL();

        // 3. Publier ROS
        front_Ir.publish(&front_Ir_msg);
        right_Ir.publish(&right_Ir_msg);
        left_Ir.publish(&left_Ir_msg);
        pose_pub.publish(&pose_msg);
    }

    nh.spinOnce();
}

