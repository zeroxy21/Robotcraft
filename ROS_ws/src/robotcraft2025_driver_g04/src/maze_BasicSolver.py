#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D, TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
import tf2_ros
from tf.transformations import quaternion_from_euler

# === Constants for IR range messages ===
FOV = 0.034906585  # 2 degrees in radians
MIN_RANGE = 0.1
MAX_RANGE = 0.8
RADIATION_TYPE = Range.INFRARED  # = 1
seuil = 0.012

# --- Variables globales pour distances ---
front_dist = 0.0
right_dist = 0.0
left_dist = 0.0

def pose_callback(msg):
    # Create and publish Odometry
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"

    odom_msg.pose.pose.position.x = msg.x
    odom_msg.pose.pose.position.y = msg.y
    odom_msg.pose.pose.position.z = 0.0

    # Convert theta (yaw) to quaternion
    q = quaternion_from_euler(0, 0, msg.theta)
    odom_msg.pose.pose.orientation.x = q[0]
    odom_msg.pose.pose.orientation.y = q[1]
    odom_msg.pose.pose.orientation.z = q[2]
    odom_msg.pose.pose.orientation.w = q[3]

    odom_pub.publish(odom_msg)

    # Broadcast TF
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

# --- Callbacks IR avec global ---
def front_cb(msg):
    global front_dist
    front_dist = msg.data
    ir_front_pub.publish(make_range_msg(msg.data, "front_ir"))

def right_cb(msg):
    global right_dist
    right_dist = msg.data
    ir_right_pub.publish(make_range_msg(msg.data, "right_ir"))

def left_cb(msg):
    global left_dist
    left_dist = msg.data
    ir_left_pub.publish(make_range_msg(msg.data, "left_ir"))

# --- Fonction de contrôle simple ---
def run(front_dist, right_dist, left_dist):
    msg = Twist()
    if front_dist < seuil:
        msg.linear.x = 0.0
        msg.angular.z = -0.3
    elif right_dist < seuil:
        msg.linear.x = 0.0
        msg.angular.z = -0.3
    elif left_dist < seuil:
        msg.linear.x = 0.0
        msg.angular.z = 0.3
    else:
        msg.linear.x = 0.5
        msg.angular.z = 0.0
    cmd_vel_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("Basic_solver_node")

    # Publishers et TF
    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    ir_front_pub = rospy.Publisher("/ir_front_sensor", Range, queue_size=10)
    ir_right_pub = rospy.Publisher("/ir_right_sensor", Range, queue_size=10)
    ir_left_pub = rospy.Publisher("/ir_left_sensor", Range, queue_size=10)
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    # Subscribers
    rospy.Subscriber("/pose", Pose2D, pose_callback)
    rospy.Subscriber("/front_distance", Float32, front_cb)
    rospy.Subscriber("/right_distance", Float32, right_cb)
    rospy.Subscriber("/left_distance", Float32, left_cb)

    rospy.loginfo("Basic_solver_node started, listening to /pose and distance topics...")

    rate = rospy.Rate(20)  # 20 Hz pour cmd_vel

    while not rospy.is_shutdown():
        run(front_dist, right_dist, left_dist)
        rate.sleep()
