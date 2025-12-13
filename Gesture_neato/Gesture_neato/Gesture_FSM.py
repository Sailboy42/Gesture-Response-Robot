import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from time import sleep
import math


class Gestures(Node):
    def __init__(self):
        super().__init__('gesture_fsm')

        # Publishers
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel_mod', 10)

        # Subscribers
        self.gesture_sub = self.create_subscription(
            String,
            'gesture',
            self.gesture_cb,
            10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_cb,
            10
        )

        # Store latest velocity
        self.current_vel = Twist()

        self.get_logger().info("Gesture FSM node started")

    # ---------------- Callbacks ---------------- #

    def cmd_vel_cb(self, msg: Twist):
        """Store the most recent velocity command"""
        self.current_vel = msg

    def gesture_cb(self, msg: String):
        """React to gestures"""
        gesture = msg.data

        if gesture == "Thumbs Up":
            self.speed_up()
        elif gesture == "Thumbs Down":
            self.slow_down()
        elif gesture == "Victory":
            self.wiggle()

    # ---------------- Actions ---------------- #

    def speed_up(self):
        mod = Twist()
        mod.linear.x = min(self.current_vel.linear.x + 0.05, 0.3)
        self.vel_pub.publish(mod)

    def slow_down(self):
        mod = Twist()
        mod.linear.x = max(self.current_vel.linear.x - 0.05, 0.0)
        self.vel_pub.publish(mod)

    def wiggle(self):
        mod = Twist()
        ang_vel = 0.3
        duration = (50 * math.pi / 180) / ang_vel  # 50 degrees

        for _ in range(4):
            mod.angular.z = ang_vel
            self.vel_pub.publish(mod)
            sleep(duration)

            mod.angular.z = -ang_vel
            self.vel_pub.publish(mod)
            sleep(duration)

        # stop rotation
        mod.angular.z = 0.0
        self.vel_pub.publish(mod)


# ---------------- Main ---------------- #

def main():
    rclpy.init()
    node = Gestures()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





























































# import Gesture_neato.Gesture_neato.gest_recog_camera as gest_recog_camera
# import rclpy
# from rclpy.node import Node
# from threading import Thread, Event
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist, PointStamped, Point
# from std_msgs.msg import Header
# from time import sleep
# import math
# from take_picture import save_fist
# from geometry_msgs.msg import Twist

# class gestures(Node):
#     def __init__(self):
#        super().__init__('gesture_fsm')
#        self.gespub = self.create_publisher(str, 'gesture', 10)
#        self.gessub = self.create_subscription(str, 'gesture', self.handle_gesture, 10)
#        self.velpub = self.create_publisher(Twist, 'cmd_vel_mod', 10)
#        self.velread = self.create_subscription(Twist, 'cmd_vel', self.handle_gesture, 10)


#     def handle_gesture(self, msg):
#         if msg == "Thumbs Up":
#             self.speed_up(msg=self.velread)
#         if msg == "Thumbs Down":
#             self.slow_down(msg=self.velread)
#         if msg == "Victory":
#             self.wiggle(msg=self.velread)


    
#     def speed_up(self, msg):
#         mod = Twist()
#         mod.linear.x = msg.linear.x + 0.5
#         if mod.linear.x > 0.3: # max neato speed
#             mod.linear.x = 0.3
#             self.velpub.publish(mod)

#     def slow_down(self, msg):
#            mod = Twist()
#            mod.linear.x = msg.linear.x - 0.5
#            if mod.linear.x < 0: # min neato speed
#                mod.linear.x = 0.0
#            self.velpub.publish(mod)
           
#     def wiggle(self, msg):
#         mod = Twist()
#         for i in list(range(1,5)):
#             ang_vel = 0.3
#             mod.angular.z = ang_vel
#             self.velpub.publish(mod)
#             sleep((50*(math.pi/180))/ang_vel)
#             mod.angular.z = -ang_vel
#             self.velpub.publish(mod)

# def main():
#    gestures.handle_gesture(self)

# if __name__ == '__main__':
#     main()
