#!/usr/bin/env python3
"""
Human Follower Node for Neato Robot
This node subscribes to human tracking data and controls the robot to follow a person.
Integrates with the human tracker to provide smooth following behavior.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
import math


class HumanFollowerNode(Node):
    """
    ROS2 Node that controls a Neato robot to follow a detected human.
    Subscribes to human tracker position data and publishes velocity commands.
    """

    def __init__(self):
        super().__init__("human_follower_node")

        # Control parameters
        self.max_linear_speed = 0.3  # m/s
        self.max_angular_speed = 0.5  # rad/s
        self.target_distance = 0.8  # meters (desired distance from person)
        self.linear_kp = 0.5  # Proportional gain for linear velocity
        self.angular_kp = 1.0  # Proportional gain for angular velocity
        self.min_distance = 0.5  # meters (stop if closer than this)
        self.max_distance = 2.0  # meters (max distance to follow)

        # Tracking state
        self.person_detected = False
        self.person_angle = 0.0
        self.person_distance = 0.0  # Will be estimated or from lidar

        # Publishers and subscribers
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.tracker_sub = self.create_subscription(
            PointStamped, "/human_tracker/position", self.tracker_callback, 10
        )

        # Control timer (runs at 10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Human follower node started")
        self.get_logger().info("Subscribing to /human_tracker/position")
        self.get_logger().info("Publishing to cmd_vel")

    def tracker_callback(self, msg):
        """
        Callback for human tracker position updates.

        Args:
            msg: PointStamped message with:
                - point.x: X offset in pixels (positive = right)
                - point.y: Y offset in pixels (positive = down)
                - point.z: Angle in radians (positive = right)
        """
        self.person_detected = True
        self.person_angle = msg.point.z  # Angle in radians

        # Estimate distance based on Y offset (closer person = larger in frame)
        # This is a simple heuristic - you may want to use lidar data instead
        y_offset = abs(msg.point.y)
        # Simple mapping: larger Y offset (person lower in frame) = closer
        # This is a rough estimate and should be calibrated
        estimated_distance = max(0.5, 2.0 - (y_offset / 100.0))
        self.person_distance = estimated_distance

        self.get_logger().debug(
            f"Person detected: angle={math.degrees(self.person_angle):.1f}°, "
            f"estimated_distance={self.person_distance:.2f}m"
        )

    def control_loop(self):
        """
        Main control loop that calculates and publishes velocity commands.
        """
        msg = Twist()

        if not self.person_detected:
            # No person detected - stop
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.vel_pub.publish(msg)
            return

        # Reset detection flag (will be set again if tracker publishes)
        # This allows timeout if person is lost
        self.person_detected = False

        # Calculate linear velocity based on distance
        distance_error = self.person_distance - self.target_distance

        # Only move forward if person is within reasonable range
        if self.min_distance < self.person_distance < self.max_distance:
            # Proportional control for linear velocity
            linear_vel = self.linear_kp * distance_error
            # Clamp to max speed
            linear_vel = max(
                -self.max_linear_speed * 0.5, min(self.max_linear_speed, linear_vel)
            )

            # Don't move backward if too close (just stop)
            if self.person_distance < self.target_distance:
                linear_vel = 0.0
        else:
            linear_vel = 0.0

        # Calculate angular velocity to turn towards person
        # person_angle is positive when person is to the right
        angular_vel = self.angular_kp * self.person_angle
        # Clamp to max angular speed
        angular_vel = max(
            -self.max_angular_speed, min(self.max_angular_speed, angular_vel)
        )

        # If person is very close, prioritize turning over moving forward
        if self.person_distance < self.min_distance:
            linear_vel = 0.0

        msg.linear.x = linear_vel
        msg.angular.z = angular_vel

        self.vel_pub.publish(msg)

        self.get_logger().debug(
            f"Control: linear={linear_vel:.2f} m/s, " f"angular={angular_vel:.2f} rad/s"
        )


def main(args=None):
    rclpy.init(args=args)
    node = HumanFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before shutting down
        stop_msg = Twist()
        node.vel_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
