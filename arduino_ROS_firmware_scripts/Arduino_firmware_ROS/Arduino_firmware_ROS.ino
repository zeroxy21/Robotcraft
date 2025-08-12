#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>


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
//IR fucntions start
float sensF(){
    float sum=0.0;
    for(int i=0;i<1000;i++){
      float sens=analogRead(IR_FRONT);
      sum=sum+sens;
    }
    float res=sum/1000.0;
    res=49.7-0.134*res+0.000104*res*res;
    
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
    
   return res;
}

// IR functions end

ros::NodeHandle nh;

// === Messages séparés pour chaque publisher ===
std_msgs::Float32 front_Ir_msg;
std_msgs::Float32 right_Ir_msg;
std_msgs::Float32 left_Ir_msg;
geometry_msgs::Pose2D pose_msg;

// === Publishers ===
ros::Publisher front_Ir("/front_distance", &front_Ir_msg);
ros::Publisher right_Ir("/right_distance", &right_Ir_msg);
ros::Publisher left_Ir("/left_distance", &left_Ir_msg);
ros::Publisher pose_pub("/pose", &pose_msg);

// === Callbacks ===

void callback_encoders() {
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

void callback_rgb_leds(const std_msgs::UInt8MultiArray &msg) {
  // TODO : gérer les LED RGB
}

void callback_cmd_vel(const geometry_msgs::Twist &msg) {
  // TODO : gérer la commande de vitesse
}

void callback_set_pose(const std_msgs::UInt8MultiArray &msg) {
  // TODO : définir la position
}

// === Subscribers ===
ros::Subscriber<std_msgs::UInt8MultiArray> rgb_leds_sub("/rgb_leds", &callback_rgb_leds);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &callback_cmd_vel);
ros::Subscriber<std_msgs::UInt8MultiArray> set_pose_sub("/set_pose", &callback_set_pose);

void setup() {
  nh.initNode();
 // Timer1.initialize(100);         // 10Hz = 100 ms
  //Timer1.attachInterrupt(callback_encoders);  
  
  
  // Publis
  nh.advertise(front_Ir);
  nh.advertise(right_Ir);
  nh.advertise(left_Ir);
  nh.advertise(pose_pub);

  // Subs
  nh.subscribe(rgb_leds_sub);
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(set_pose_sub);
}

void loop() {
  front_Ir_msg.data=sensF();
  right_Ir_msg.data=sensR();
  left_Ir_msg.data=sensL();
 // === Publier ===
  front_Ir.publish(&front_Ir_msg);
  right_Ir.publish(&right_Ir_msg);
  left_Ir.publish(&left_Ir_msg);
  pose_pub.publish(&pose_msg);
  nh.spinOnce();
}

