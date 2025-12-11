import gest_recog_camera
import rclpy
from rclpy.node import Node
from threading import Thread, Event
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped, Point
from std_msgs.msg import Header
from time import sleep
import math


class gestures(Node):
   def __init__(self):
       super().__init__('do_gesture')
       self.velpub = self.create_publisher(Twist, 'cmd_vel_mod', 10)
       self.velread = self.create_subscription(Twist, 'cmd_vel', self.gestured, 10)
       self.prev_gesture = "None"


   def gestured(self, msg):
       g = gest_recog_camera.latest_gesture


       if g == "Gesture: Thumbs Up" and self.prev_gesture != g:
           mod = Twist()
           mod.linear.x = msg.linear.x + 0.5
           if mod.linear.x > 0.3: # max neato speed
               mod.linear.x = 0.3
           self.velpub.publish(mod)
           self.prev_gesture = g


       if g == "Gesture: Thumbs Down" and self.prev_gesture!= g:
           mod = Twist()
           mod.linear.x = msg.linear.x - 0.5
           if mod.linear.x < 0: # min neato speed
               mod.linear.x = 0.0
           self.velpub.publish(mod)
           self.prev_gesture = g
      
       if g == "Gesture: Victory" and self.prev_gesture != g:
           mod = Twist()
           for i in list(range(1,5)):
               ang_vel = 0.3
               mod.angular.z = ang_vel
               self.velpub.publish(mod)
               sleep((50*(math.pi/180))/ang_vel)
               mod.angular.z = -ang_vel
               self.velpub.publish(mod)
           self.prev_gesture = g


def main(args=None):
   rclpy.init(args=args)
   node = gestures()
   rclpy.spin(node)
   rclpy.shutdown()