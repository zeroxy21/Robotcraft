#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros

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
    import math
    from tf.transformations import quaternion_from_euler
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

if __name__ == "__main__":
    rospy.init_node("pose_to_odom")

    # Publisher for Odometry
    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)

    # TF broadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Subscriber to /pose
    rospy.Subscriber("/pose", Pose2D, pose_callback)

    rospy.loginfo("pose_to_odom node started, listening to /pose...")
    rospy.spin()
