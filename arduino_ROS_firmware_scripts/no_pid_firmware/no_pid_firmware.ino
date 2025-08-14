#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

#include <TimerOne.h>
#include <Encoder.h>
#include <math.h> // pour M_PI, cos, sin

// === Pins moteurs ===
#define M1R_DIR 5
#define M1R_PWM 4
#define M2L_DIR 6
#define M2L_PWM 9

// === Pins capteurs IR ===
#define IR_FRONT A2
#define IR_RIGHT A3
#define IR_LEFT  A4

// === Pins encodeurs ===
#define ENC_M1R_A 3
#define ENC_M1R_B 2
#define ENC_M2L_A 18
#define ENC_M2L_B 19

// === Paramètres géométriques en mètre ===
float r = 0.016;      // rayon roue en m (1.6 cm)
float b = 0.106;      // entraxe en m (10.6 cm)
int gear_box = 298;
float C = 0.0000241;  // conversion impulsions → m (0.00241 cm → 0.0000241 m)

Encoder enc_left(ENC_M2L_A, ENC_M2L_B);
Encoder enc_right(ENC_M1R_A, ENC_M1R_B);

// === Variables globales odométrie ===
volatile bool flag = false;
long enc_left_count = 0;
long enc_right_count = 0;

float x = 0.0;
float y = 0.0;
float theta = 0.0; // en radians

// === Fonctions capteurs IR (renvoie distance en m) ===
float sensF() {
    float sum = 0.0;
    for (int i = 0; i < 1000; i++) sum += analogRead(IR_FRONT);
    float res = sum / 1000.0;
    return (49.7 - 0.134 * res + 0.000104 * res * res) / 100.0; // m
}

float sensL() {
    float sum = 0.0;
    for (int i = 0; i < 1000; i++) sum += analogRead(IR_LEFT);
    float res = sum / 1000.0;
    return (46.9 - 0.132 * res + 0.000106 * res * res) / 100.0; // m
}

float sensR() {
    float sum = 0.0;
    for (int i = 0; i < 1000; i++) sum += analogRead(IR_RIGHT);
    float res = sum / 1000.0;
    return (51.9 - 0.144 * res + 0.000114 * res * res) / 100.0; // m
}

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

// === Odométrie via encodeurs ===
void callback_encoders() {
    long right = enc_right.read();
    long left  = enc_left.read();

    long count_left  = left - enc_left_count;
    long count_right = right - enc_right_count;

    enc_left_count  = left;
    enc_right_count = right;

    float dL = C * count_left;  // m
    float dR = C * count_right; // m

    float dS     = (dR + dL) / 2.0;
    float dTheta = (dR - dL) / b; // rad

    x     += dS * cos(theta + dTheta / 2.0);
    y     += dS * sin(theta + dTheta / 2.0);
    theta += dTheta;

    // Normalisation de theta
    if (theta > M_PI)  theta -= 2.0 * M_PI;
    if (theta < -M_PI) theta += 2.0 * M_PI;

    // Remplir le message ROS
    pose_msg.x = x;
    pose_msg.y = y;
    pose_msg.theta = theta;

    flag = true;
}

// === Pilotage moteurs ===
void setMotor(int pwm_pin, int dir_pin, float speed) {
    // speed entre -1.0 et 1.0
    if (speed > 1.0) speed = 1.0;
    if (speed < -1.0) speed = -1.0;

    if (speed >= 0) {
        digitalWrite(dir_pin, LOW);
        analogWrite(pwm_pin, (int)(speed * 255.0));
    } else {
        digitalWrite(dir_pin, HIGH);
        analogWrite(pwm_pin, (int)(-speed * 255.0));
    }
}

// === Callbacks ROS ===
void callback_cmd_vel(const geometry_msgs::Twist &msg) {
    float v = msg.linear.x;   // m/s
    float w = msg.angular.z;  // rad/s

    // Conversion vitesse linéaire et angulaire en vitesse roue
    float vL = v - (b/2.0) * w; // m/s
    float vR = v + (b/2.0) * w; // m/s

    // Normalisation en PWM (vitesse max 0.2 m/s)
    float vmax = 0.20; // m/s
    float left_pwm  = vL / vmax;
    float right_pwm = vR / vmax;

    setMotor(M2L_PWM, M2L_DIR, left_pwm);
    setMotor(M1R_PWM, M1R_DIR, right_pwm);
}

void callback_rgb_leds(const std_msgs::UInt8MultiArray &msg) {
    // msg.data[0..2] -> R,G,B
}

void callback_set_pose(const std_msgs::UInt8MultiArray &msg) {
    if(msg.data_length >= 3) {
        x = msg.data[0] / 100.0; // cm -> m
        y = msg.data[1] / 100.0; // cm -> m
        theta = msg.data[2] * M_PI / 180.0; // deg -> rad
        enc_left_count = enc_left.read();
        enc_right_count = enc_right.read();
    }
}

// === Subscribers ===
ros::Subscriber<std_msgs::UInt8MultiArray> rgb_leds_sub("/rgb_leds", &callback_rgb_leds);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &callback_cmd_vel);
ros::Subscriber<std_msgs::UInt8MultiArray> set_pose_sub("/set_pose", &callback_set_pose);

void setup() {
    nh.initNode();

    // Timer pour encoder callback (10 Hz = 100000 µs)
    Timer1.initialize(100000);
    Timer1.attachInterrupt(callback_encoders);

    // Publis
    nh.advertise(front_Ir);
    nh.advertise(right_Ir);
    nh.advertise(left_Ir);
    nh.advertise(pose_pub);

    // Subs
    nh.subscribe(rgb_leds_sub);
    nh.subscribe(cmd_vel_sub);
    nh.subscribe(set_pose_sub);

    // Pins moteurs en sortie
    pinMode(M1R_DIR, OUTPUT);
    pinMode(M1R_PWM, OUTPUT);
    pinMode(M2L_DIR, OUTPUT);
    pinMode(M2L_PWM, OUTPUT);
}

unsigned long last_pub_time = 0;  // Pour cadencer les publications à 10 Hz

void loop() {
    unsigned long now = millis();
    if (now - last_pub_time >= 100) {  // 100 ms = 10 Hz
        last_pub_time = now;

        // Lecture IR (version rapide)
        front_Ir_msg.data = sensF();
        right_Ir_msg.data = sensR();
        left_Ir_msg.data  = sensL();

        front_Ir.publish(&front_Ir_msg);
        right_Ir.publish(&right_Ir_msg);
        left_Ir.publish(&left_Ir_msg);

        // Publier la pose seulement si mise à jour
        if (flag) {
            pose_pub.publish(&pose_msg);
            flag = false;
        }
    }

    nh.spinOnce();
}

