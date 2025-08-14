#!/usr/bin/env python3
import rospy
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
import tf.transformations as tft

class SquareController:
    def __init__(self):
        rospy.init_node('square_controller_v4')
        
        # Configuration améliorée
        self.side_length = 1.0  # m
        self.linear_speed = 0.5    # m/s
        self.angular_speed = 1.8  # rad/s (~45°/s)
        self.safety_margin = 0.1   # m
        self.timeout = 5.0         # seconds
        
        # Système redondant
        self.odom_ready = False
        self.last_cmd_time = time.time()
        self.emergency_stop = False
        
        # Contrôle temporel
        self.move_time = self.side_length / self.linear_speed
        self.turn_time = (math.pi/2) / self.angular_speed
        
        # Publishers/Subscribers robustes
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=10)
        rospy.Subscriber('/ir_front', Range, self.ir_cb, queue_size=5)
        
        # Timer de sécurité
        self.safety_timer = rospy.Timer(rospy.Duration(0.1), self.check_safety)
        
    def odom_cb(self, msg):
        """Callback d'odométrie avec vérification de validité"""
        try:
            self.current_pose = msg.pose.pose
            self.odom_ready = True
        except:
            rospy.logwarn("Problème avec les données d'odométrie!")
    
    def ir_cb(self, msg):
        """Callback de capteur avec arrêt d'urgence"""
        if msg.range < self.safety_margin:
            self.emergency_stop = True
            rospy.logerr(f"OBSTACLE DÉTECTÉ! Distance: {msg.range:.2f}m")
    
    def check_safety(self, event):
        """Vérifications redondantes"""
        # Timeout global
        if time.time() - self.last_cmd_time > self.timeout:
            self.emergency_stop = True
            rospy.logerr("Timeout de sécurité déclenché!")
        
        # Vérification de l'odométrie
        if not hasattr(self, 'current_pose'):
            rospy.logwarn_throttle(5, "En attente de la première odométrie...")
            return
    
    def execute_move(self, duration, linear, angular):
        """Commande de mouvement avec temporisation"""
        start_time = time.time()
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        
        while (time.time() - start_time) < duration and not rospy.is_shutdown():
            if self.emergency_stop:
                self.cmd_pub.publish(Twist())
                return False
            
            self.cmd_pub.publish(twist)
            self.last_cmd_time = time.time()
            rospy.sleep(0.05)
            
        return True
    
    def run_square(self):
        """Séquence principale robuste"""
        rospy.loginfo("Démarrage de la séquence carrée")
        
        for i in range(4):
            if rospy.is_shutdown() or self.emergency_stop:
                break
                
            # Phase de déplacement
            rospy.loginfo(f"Début côté {i+1}/4")
            if not self.execute_move(self.move_time, self.linear_speed, 0):
                break
                
            # Phase de rotation
            rospy.loginfo(f"Début rotation {i+1}/4")
            if not self.execute_move(self.turn_time, 0, self.angular_speed):
                break
                
        self.cmd_pub.publish(Twist())
        rospy.loginfo("Séquence terminée")

if __name__ == '__main__':
    try:
        controller = SquareController()
        rospy.sleep(1)  # Attente initiale
        controller.run_square()
    except rospy.ROSInterruptException:
        pass