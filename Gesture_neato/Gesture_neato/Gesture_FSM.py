#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PointStamped
from time import sleep, time
import math
import os
import cv2 as cv


class Gestures(Node):
    def __init__(self):
        super().__init__("gesture_fsm")

        # Publisher: command the base
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.last_saved_ms = 0

        # Subscriber: recognized gesture string
        self.gesture_sub = self.create_subscription(
            String, "gesture", self.gesture_cb, 10
        )

        # Person tracker subscription
        self.person_tracker_sub = self.create_subscription(
            PointStamped, "/human_tracker/position", self.person_tracker_cb, 10
        )

        # Persistent command we keep streaming
        self.cmd = Twist()

        # Neato / diff-drive drivers usually need a steady cmd_vel stream
        self.timer = self.create_timer(0.05, self._tick)  # 20 Hz

        self.wiggle_active = False
        self.wiggle_step = 0
        self.wiggle_deadline_ns = 0
        self.saved_linear = 0.0

        self.ang_vel = 0.3
        self.duration_s = (50 * math.pi / 180.0) / self.ang_vel  # time for 50 degrees

        # Person tracking state
        self.person_visible = False
        self.person_angle = 0.0
        self.last_tracker_update = 0.0
        self.following_mode = False
        self.tracker_timeout = 0.5  # seconds

        #  self.get_logger().info("Gesture FSM node started (streaming cmd_vel @ 20 Hz)")

    def _publish_cmd(self):
        """Continuously publish the latest desired command."""
        self.vel_pub.publish(self.cmd)

    def gesture_cb(self, msg: String):
        """React to gestures."""
        raw = msg.data
        g = raw.strip().lower().replace("_", " ")

        self.get_logger().info(f"gesture raw={raw!r} normalized={g!r}")

        if g in ("like", "like"):
            # if g in ("thumbs up", "thumb up"):
            self.get_logger().info("speeding up")
            self.speed_up()
        elif g in ("dislike", "dislike"):
            # elif g in ("thumbs down", "thumb down"):
            self.get_logger().info("slowing down")
            self.slow_down()
        elif g in ("rock", "middle_finger"):
            # elif g in ("victory", "peace", "v sign"):
            self.get_logger().info("wiggling")
            self.start_wiggle()
        elif g in ("iloveyou"):
            # Toggle person following mode
            self.following_mode = not self.following_mode
            if self.following_mode:
                self.get_logger().info("Person following mode: ON")
            else:
                self.get_logger().info("Person following mode: OFF")
                # Stop robot when turning off following
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
        else:
            self.get_logger().warn(f"unrecognized gesture: {raw!r}")

    def person_tracker_cb(self, msg: PointStamped):
        """Callback for person tracker position updates"""
        self.person_visible = True
        self.person_angle = msg.point.z  # Angle in radians
        self.last_tracker_update = time.time()

    def _tick(self):
        """Runs at 20 Hz: update wiggle + publish."""
        if self.wiggle_active:
            self._advance_wiggle()
        elif self.following_mode:
            self._update_person_follow()

        self.vel_pub.publish(self.cmd)

    # ---------------- Actions ---------------- #

    def speed_up(self):
        # increase forward speed, cap at 0.3 m/s
        self.cmd.linear.x = min(self.cmd.linear.x + 0.1, 0.3)
        # optional: stop turning when changing speed
        self.cmd.angular.z = 0.0

    def slow_down(self):
        # decrease forward speed, floor at 0
        self.cmd.linear.x = max(self.cmd.linear.x - 0.005, 0.0)
        self.cmd.angular.z = 0.0

    def start_wiggle(self):
        if self.wiggle_active:
            return  # ignore retriggers while wiggling

        self.get_logger().info("Starting wiggle")
        self.wiggle_active = True
        self.wiggle_step = 0
        self.saved_linear = self.cmd.linear.x

        # start immediately
        self.wiggle_deadline_ns = self.get_clock().now().nanoseconds

    def _advance_wiggle(self):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns < self.wiggle_deadline_ns:
            return

        # Each step lasts duration_s. Pattern: +, -, +, -, +, -, +, -, stop
        if self.wiggle_step < 8:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = (
                self.ang_vel if (self.wiggle_step % 2 == 0) else -self.ang_vel
            )
            self.wiggle_step += 1
            self.wiggle_deadline_ns = now_ns + int(self.duration_s * 1e9)
        else:
            # done
            self.cmd.angular.z = 0.0
            self.cmd.linear.x = self.saved_linear
            self.wiggle_active = False
            self.get_logger().info("Wiggle complete")

    def _update_person_follow(self):
        """Update person following behavior (called from _tick)"""
        # Check if tracker data is stale
        current_time = time.time()
        if current_time - self.last_tracker_update > self.tracker_timeout:
            self.person_visible = False

        if self.person_visible:
            # Person is detected - follow them
            max_angular_speed = 0.5  # rad/s
            max_linear_speed = 0.2  # m/s

            # Proportional control for angular velocity
            angular_kp = 1.0
            angular_vel = angular_kp * self.person_angle
            angular_vel = max(-max_angular_speed, min(max_angular_speed, angular_vel))

            # Move forward if person is roughly centered (angle < ~11 degrees)
            if abs(self.person_angle) < 0.2:
                self.cmd.linear.x = max_linear_speed
            else:
                # Turn first, then move forward at reduced speed
                self.cmd.linear.x = max_linear_speed * 0.5

            self.cmd.angular.z = angular_vel
        else:
            # Person lost - stop and search (slow rotation)
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.3  # Slow rotation to search for person


def main():
    rclpy.init()
    node = Gestures()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
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
