import Gesture_neato.Gesture_neato.gest_recog_camera as gest_recog_camera
import rclpy
from rclpy.node import Node
from threading import Thread, Event
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped, Point
from std_msgs.msg import Header
from time import sleep
import math
from take_picture import save_fist
from geometry_msgs.msg import Twist

class gestures(Node):
    def __init__(self):
       super().__init__('gesture_fsm')
       self.gespub = self.create_publisher(str, 'gesture', 10)
       self.gessub = self.create_subscription(str, 'gesture', self.handle_gesture, 10)
       self.velpub = self.create_publisher(Twist, 'cmd_vel_mod', 10)
       self.velread = self.create_subscription(Twist, 'cmd_vel', self.handle_gesture, 10)


    def handle_gesture(self, msg):
        if msg == "Thumbs Up":
            self.speed_up(msg=self.velread)
        if msg == "Thumbs Down":
            self.slow_down(msg=self.velread)
        if msg == "Victory":
            self.wiggle(msg=self.velread)


    
    def speed_up(self, msg):
        mod = Twist()
        mod.linear.x = msg.linear.x + 0.5
        if mod.linear.x > 0.3: # max neato speed
            mod.linear.x = 0.3
            self.velpub.publish(mod)

    def slow_down(self, msg):
           mod = Twist()
           mod.linear.x = msg.linear.x - 0.5
           if mod.linear.x < 0: # min neato speed
               mod.linear.x = 0.0
           self.velpub.publish(mod)
           
    def wiggle(self, msg):
        mod = Twist()
        for i in list(range(1,5)):
            ang_vel = 0.3
            mod.angular.z = ang_vel
            self.velpub.publish(mod)
            sleep((50*(math.pi/180))/ang_vel)
            mod.angular.z = -ang_vel
            self.velpub.publish(mod)

def main():
    node.handle_gesture()


if __name__ == '__main__':
    main()
