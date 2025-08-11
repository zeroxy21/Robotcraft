#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

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
  // === Simuler des valeurs capteurs ===
  front_Ir_msg.data = 1.0;  // distance avant
  right_Ir_msg.data = 2.0;  // distance droite
  left_Ir_msg.data  = 3.0;  // distance gauche

  // === Simuler la pose ===
  pose_msg.x = 1.23;     // position X en mètres
  pose_msg.y = 4.56;     // position Y en mètres
  pose_msg.theta = 0.78; // orientation en radians

  // === Publier ===
  front_Ir.publish(&front_Ir_msg);
  right_Ir.publish(&right_Ir_msg);
  left_Ir.publish(&left_Ir_msg);
  pose_pub.publish(&pose_msg);

  nh.spinOnce();
  delay(100); // éviter la surcharge de la liaison série
}

