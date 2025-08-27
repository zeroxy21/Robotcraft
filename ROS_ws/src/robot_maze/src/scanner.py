#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist, Pose2D, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
import tf2_ros
from tf.transformations import quaternion_from_euler
import os
import subprocess
import time

seuil = 0.15
LOST = 0
TURN_LEFT = 1
WALL1 = 2
WALL2 = 3
STATE = LOST

FOV = 0.034906585
MIN_RANGE = 0.1
MAX_RANGE = 0.8
RADIATION_TYPE = Range.INFRARED

save_delay = 600.0  # 10 minutes
start_time = time.time()
map_saved = False

def pose_callback(msg):
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"

    odom_msg.pose.pose.position.x = msg.x
    odom_msg.pose.pose.position.y = msg.y
    odom_msg.pose.pose.position.z = 0.0

    q = quaternion_from_euler(0, 0, msg.theta)
    odom_msg.pose.pose.orientation.x = q[0]
    odom_msg.pose.pose.orientation.y = q[1]
    odom_msg.pose.pose.orientation.z = q[2]
    odom_msg.pose.pose.orientation.w = q[3]

    odom_pub.publish(odom_msg)

    t = TransformStamped()
    t.header.stamp = odom_msg.header.stamp
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    tf_broadcaster.sendTransform(t)

def make_range_msg(value, frame_id):
    msg = Range()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.radiation_type = RADIATION_TYPE
    msg.field_of_view = FOV
    msg.min_range = MIN_RANGE
    msg.max_range = MAX_RANGE
    msg.range = value
    return msg

class Basic_solver:
    def __init__(self):
        rospy.init_node("scanner_node")
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.front_dist = float('inf')
        self.left_dist = float('inf')
        self.right_dist = float('inf')
        rospy.Subscriber("/front_distance", Float32, self.front_cb)
        rospy.Subscriber("/right_distance", Float32, self.right_cb)
        rospy.Subscriber("/left_distance", Float32, self.left_cb)

    def left_cb(self, msg):
        self.left_dist = msg.data
        ir_left_pub.publish(make_range_msg(self.left_dist, "left_ir"))

    def front_cb(self, msg):
        self.front_dist = msg.data
        ir_front_pub.publish(make_range_msg(self.front_dist, "front_ir"))

    def right_cb(self, msg):
        self.right_dist = msg.data
        ir_right_pub.publish(make_range_msg(self.right_dist, "right_ir"))

    def status(self):
        global STATE
        if STATE == LOST:
            self.lost()
        elif STATE == TURN_LEFT:
            self.TL()
        elif STATE == WALL1:
            self.w1()
        elif STATE == WALL2:
            self.w2()

    def lost(self):
        global STATE
        msg = Twist()
        if self.front_dist > seuil:
            msg.linear.x = 0.4
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)
        else:
            STATE = TURN_LEFT

    def TL(self):
        global STATE
        msg = Twist()
        if self.front_dist < seuil:
            msg.linear.x = 0.0
            msg.angular.z = 0.5
            self.cmd_vel_pub.publish(msg)
        else:
            STATE = WALL1

    def w1(self):
        global STATE
        msg = Twist()
        if self.right_dist > seuil:
            msg.linear.x = 0.1
            msg.angular.z = -0.3
        else:
            STATE = WALL2
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def w2(self):
        global STATE
        msg = Twist()
        if self.right_dist < seuil:
            msg.linear.x = 0.0
            msg.angular.z = 0.5
            self.cmd_vel_pub.publish(msg)
        else:
            if self.front_dist > seuil:
                STATE = WALL1
            elif self.left_dist < seuil:
                STATE = TURN_LEFT

if __name__ == "__main__":
    solver = Basic_solver()
    rate = rospy.Rate(20)
    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    ir_front_pub = rospy.Publisher("/ir_front_sensor", Range, queue_size=10)
    ir_right_pub = rospy.Publisher("/ir_right_sensor", Range, queue_size=10)
    ir_left_pub = rospy.Publisher("/ir_left_sensor", Range, queue_size=10)
    rospy.Subscriber("/pose", Pose2D, pose_callback)

    while not rospy.is_shutdown():
        solver.status()
        rate.sleep()

        # --- Sauvegarde automatique après 10 minutes ---
        if not map_saved and (time.time() - start_time) > save_delay:
            map_saved = True
            print("10 minutes écoulées, sauvegarde de la map...")
            if not os.path.exists('maps'):
                os.makedirs('maps')
            subprocess.call([
                'rosrun', 'map_server', 'map_saver',
                '-f', 'maps/robot_maze_map'
            ])
            print("Map sauvegardée dans maps/robot_maze_map.{pgm,yaml}")
