#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import tf2_ros
import tf.transformations as tft

class SquareControllerP:
    def __init__(self):
        rospy.init_node("square_controller_proportional")

        # === Paramètres ===
        self.side_length = rospy.get_param("~side_length", 1.0)
        self.linear_speed = rospy.get_param("~linear_speed", 0.1)   # lent
        self.kp_angle = rospy.get_param("~kp_angle", 0.8)           # virages doux
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 0.3)
        self.angle_tolerance = rospy.get_param("~angle_tolerance", 0.01)
        self.safety_margin = rospy.get_param("~safety_margin", 0.1)

        # TF frames
        self.world_frame = rospy.get_param("~world_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        self.emergency_stop = False

        # === Pub/Sub ===
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        rospy.Subscriber("/ir_front_sensor", Range, self.ir_cb)
        rospy.Subscriber("/ir_left_sensor", Range, self.ir_cb)
        rospy.Subscriber("/ir_right_sensor", Range, self.ir_cb)

        # === TF2 Listener ===
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    # ---------- Sécurité ----------
    def ir_cb(self, msg):
        if msg.range < self.safety_margin:
            self.emergency_stop = True
            rospy.logerr(f"🚨 OBSTACLE détecté par {msg.header.frame_id} à {msg.range:.2f} m ! Arrêt d'urgence.")
            self.cmd_pub.publish(Twist())  # stop immédiat

    # ---------- Outils ----------
    @staticmethod
    def saturate(value, limit):
        return max(-limit, min(limit, value))

    def get_pose(self):
        """Retourne (x, y, yaw) depuis TF (odom -> base_link)."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.5)
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
            return x, y, yaw
        except:
            return None

    # ---------- Contrôleurs ----------
    def move_straight_with_correction(self, distance):
        pose = self.get_pose()
        if pose is None:
            rospy.logwarn("Pas de TF disponible pour avancer")
            return
        start_x, start_y, start_yaw = pose

        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.emergency_stop:
            pose = self.get_pose()
            if pose is None:
                continue
            x, y, yaw = pose

            dx = x - start_x
            dy = y - start_y
            traveled = math.hypot(dx, dy)

            error_yaw = math.atan2(math.sin(start_yaw - yaw), math.cos(start_yaw - yaw))

            cmd = Twist()
            cmd.linear.x = self.linear_speed
            cmd.angular.z = self.saturate(self.kp_angle * error_yaw, self.max_angular_speed)
            self.cmd_pub.publish(cmd)

            if traveled >= distance:
                break
            rate.sleep()

        self.cmd_pub.publish(Twist())

    def rotate(self, angle):
        pose = self.get_pose()
        if pose is None:
            rospy.logwarn("Pas de TF disponible pour rotation")
            return
        _, _, start_yaw = pose
        target_yaw = start_yaw + angle

        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.emergency_stop:
            pose = self.get_pose()
            if pose is None:
                continue
            _, _, yaw = pose

            error = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))

            cmd = Twist()
            cmd.angular.z = self.saturate(self.kp_angle * error, self.max_angular_speed)
            self.cmd_pub.publish(cmd)

            if abs(error) < self.angle_tolerance:
                break
            rate.sleep()

        self.cmd_pub.publish(Twist())

    # ---------- Cycle principal ----------
    def run(self):
        rospy.loginfo("Démarrage : un seul carré avec vitesses réduites...")
        for i in range(4):
            if self.emergency_stop:
                rospy.logwarn("Arrêt du carré à cause d'un obstacle 🚧")
                break
            self.move_straight_with_correction(self.side_length)
            self.rotate(math.pi / 2)

        self.cmd_pub.publish(Twist())
        rospy.loginfo("Carré terminé ✅")

if __name__ == "__main__":
    try:
        controller = SquareControllerP()
        controller.run()
    except rospy.ROSInterruptException:
        pass
