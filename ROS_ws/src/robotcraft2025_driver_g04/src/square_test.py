#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
import tf.transformations as tft

class SquareControllerP:
    def __init__(self):
        rospy.init_node('square_controller_proportional')
        self.side_length = 1.0
        self.linear_speed = 0.3
        self.safety_margin = 0.1
        self.emergency_stop = False
        self.current_pose = None

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/ir_front', Range, self.ir_cb)
        rospy.sleep(1.0)

        # Coefficient proportionnel pour la correction angulaire
        self.kp_angle = 2.0

    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose

    def ir_cb(self, msg):
        if msg.range < self.safety_margin:
            self.emergency_stop = True
            rospy.logerr(f"OBSTACLE DÉTECTÉ! Distance: {msg.range:.2f} m")

    def move_straight_with_correction(self, distance):
        if self.current_pose is None:
            return

        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        _, _, start_yaw = tft.euler_from_quaternion([
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ])

        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.emergency_stop:
            dx = self.current_pose.position.x - start_x
            dy = self.current_pose.position.y - start_y
            traveled = math.hypot(dx, dy)

            _, _, yaw = tft.euler_from_quaternion([
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            ])
            error_yaw = math.atan2(math.sin(start_yaw - yaw), math.cos(start_yaw - yaw))

            cmd = Twist()
            cmd.linear.x = self.linear_speed
            cmd.angular.z = self.kp_angle * error_yaw
            self.cmd_pub.publish(cmd)

            if traveled >= distance:
                break
            rate.sleep()

        self.cmd_pub.publish(Twist())

    def rotate(self, angle):
        if self.current_pose is None:
            return

        _, _, start_yaw = tft.euler_from_quaternion([
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ])
        target_yaw = start_yaw + angle

        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.emergency_stop:
            _, _, yaw = tft.euler_from_quaternion([
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            ])
            error = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))

            cmd = Twist()
            cmd.angular.z = self.kp_angle * error
            self.cmd_pub.publish(cmd)

            if abs(error) < 0.01:
                break
            rate.sleep()

        self.cmd_pub.publish(Twist())

    def run(self):
        rospy.loginfo("Démarrage boucle carrée avec correcteur proportionnel...")
        while not rospy.is_shutdown() and not self.emergency_stop:
            for i in range(4):
                self.move_straight_with_correction(self.side_length)
                self.rotate(math.pi/2)
        self.cmd_pub.publish(Twist())

if __name__ == '__main__':
    try:
        controller = SquareControllerP()
        controller.run()
    except rospy.ROSInterruptException:
        pass
