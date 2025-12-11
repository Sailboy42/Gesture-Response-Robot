#!/usr/bin/env python3
"""
Human Tracking using MediaPipe Pose
This script tracks a human figure in real-time using MediaPipe Pose detection.
Can be used standalone with a camera or as a ROS2 node for robot control.
"""

import cv2
import numpy as np
import mediapipe as mp
import math

# Optional ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, PointStamped
    from cv_bridge import CvBridge, CvBridgeError

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class HumanTracker:
    def __init__(
        self,
        use_ros2=False,
        camera_topic="/camera/image_raw",
        publish_topic="/human_tracker/position",
    ):
        """
        Initialize the human tracker.

        Args:
            use_ros2: If True, run as ROS2 node. If False, use direct camera.
            camera_topic: ROS2 topic to subscribe to for camera images.
            publish_topic: ROS2 topic to publish human position data.
        """
        self.use_ros2 = use_ros2 and ROS2_AVAILABLE
        self.cap = None
        self.bridge = None

        # Initialize MediaPipe Pose
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
            model_complexity=1,  # 0, 1, or 2 (higher = more accurate but slower)
        )
        self.mp_drawing = mp.solutions.drawing_utils

        # Tracking state
        self.person_center = None  # (x, y) in image coordinates
        self.person_visible = False
        self.image_center = None  # (x, y) center of the image
        self.offset_from_center = None  # (x_offset, y_offset)
        self.person_angle = None  # Angle from center in radians

        # ROS2 setup
        if self.use_ros2:
            self.node = None
            self.camera_topic = camera_topic
            self.publish_topic = publish_topic
            self.position_publisher = None
            self.image_subscription = None

    def detect_human(self, frame):
        """
        Detect human figure in the frame using MediaPipe Pose.

        Args:
            frame: Input BGR frame

        Returns:
            tuple: (frame_with_annotations, person_center, visible)
                - frame_with_annotations: Frame with pose landmarks drawn
                - person_center: (x, y) center of detected person, or None
                - visible: Boolean indicating if person was detected
        """
        # Convert BGR to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process the frame
        results = self.pose.process(rgb_frame)

        # Get image dimensions
        h, w = frame.shape[:2]
        self.image_center = (w // 2, h // 2)

        person_center = None
        visible = False

        # Draw pose landmarks on the frame
        annotated_frame = frame.copy()

        if results.pose_landmarks:
            visible = True

            # Draw pose landmarks
            self.mp_drawing.draw_landmarks(
                annotated_frame,
                results.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS,
                self.mp_drawing.DrawingSpec(
                    color=(0, 255, 0), thickness=2, circle_radius=2
                ),
                self.mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2),
            )

            # Calculate person center using key body points
            # Use shoulders and hips for a more stable center point
            landmarks = results.pose_landmarks.landmark

            # Get key points (if visible)
            left_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
            right_shoulder = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
            left_hip = landmarks[self.mp_pose.PoseLandmark.LEFT_HIP]
            right_hip = landmarks[self.mp_pose.PoseLandmark.RIGHT_HIP]

            # Calculate center from visible points
            visible_points = []
            if left_shoulder.visibility > 0.5:
                visible_points.append((left_shoulder.x * w, left_shoulder.y * h))
            if right_shoulder.visibility > 0.5:
                visible_points.append((right_shoulder.x * w, right_shoulder.y * h))
            if left_hip.visibility > 0.5:
                visible_points.append((left_hip.x * w, left_hip.y * h))
            if right_hip.visibility > 0.5:
                visible_points.append((right_hip.x * w, right_hip.y * h))

            if visible_points:
                # Calculate average center
                person_center = (
                    int(np.mean([p[0] for p in visible_points])),
                    int(np.mean([p[1] for p in visible_points])),
                )

                # Draw center point
                cv2.circle(annotated_frame, person_center, 10, (255, 0, 0), -1)

                # Draw line from image center to person center
                cv2.line(
                    annotated_frame, self.image_center, person_center, (255, 255, 0), 2
                )

        self.person_center = person_center
        self.person_visible = visible

        # Calculate offset from center
        if person_center:
            offset_x = person_center[0] - self.image_center[0]
            offset_y = person_center[1] - self.image_center[1]
            self.offset_from_center = (offset_x, offset_y)

            # Calculate angle from center (for robot turning)
            # Normalize to [-1, 1] range where -1 is left edge, 1 is right edge
            normalized_offset = offset_x / (w / 2)
            # Convert to approximate angle in radians (field of view ~60 degrees)
            self.person_angle = normalized_offset * math.radians(30)
        else:
            self.offset_from_center = None
            self.person_angle = None

        return annotated_frame, person_center, visible

    def get_tracking_info(self):
        """
        Get current tracking information for robot control.

        Returns:
            dict: Tracking information with keys:
                - visible: bool, whether person is detected
                - center: tuple (x, y) or None
                - offset: tuple (x_offset, y_offset) or None
                - angle: float, angle in radians from center or None
        """
        return {
            "visible": self.person_visible,
            "center": self.person_center,
            "offset": self.offset_from_center,
            "angle": self.person_angle,
        }

    def track_human_standalone(self):
        """
        Main method to track human in real-time video stream (standalone mode).
        """
        # Initialize video capture
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            print("Error: Could not open camera")
            return

        print("Human Tracking Started (Standalone Mode)")
        print("Press 'q' to quit")

        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Error: Could not read frame")
                break

            # Flip frame horizontally for mirror effect
            frame = cv2.flip(frame, 1)

            # Detect human
            annotated_frame, person_center, visible = self.detect_human(frame)

            # Add text overlay with tracking info
            info_text = f"Person: {'Detected' if visible else 'Not Detected'}"
            cv2.putText(
                annotated_frame,
                info_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
            )

            if visible and person_center:
                offset_text = f"Offset: ({self.offset_from_center[0]:.0f}, {self.offset_from_center[1]:.0f})"
                angle_text = f"Angle: {math.degrees(self.person_angle):.1f}°"
                cv2.putText(
                    annotated_frame,
                    offset_text,
                    (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 0),
                    2,
                )
                cv2.putText(
                    annotated_frame,
                    angle_text,
                    (10, 110),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 0),
                    2,
                )

            # Show the frame
            cv2.imshow("Human Tracking", annotated_frame)

            # Break loop on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        # Cleanup
        self.cap.release()
        cv2.destroyAllWindows()
        self.pose.close()
        print("Human Tracking Stopped")

    def image_callback(self, msg):
        """
        ROS2 callback for processing camera images.

        Args:
            msg: sensor_msgs.msg.Image message
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.node.get_logger().error(f"CvBridge error: {e}")
            return

        # Detect human
        annotated_frame, person_center, visible = self.detect_human(frame)

        # Publish position if person is detected
        if visible and person_center and self.position_publisher:
            point_msg = PointStamped()
            point_msg.header.stamp = self.node.get_clock().now().to_msg()
            point_msg.header.frame_id = "camera_frame"
            point_msg.point.x = float(self.offset_from_center[0])  # X offset in pixels
            point_msg.point.y = float(self.offset_from_center[1])  # Y offset in pixels
            point_msg.point.z = (
                float(self.person_angle) if self.person_angle else 0.0
            )  # Angle in radians
            self.position_publisher.publish(point_msg)

        # Log tracking status
        if visible:
            self.node.get_logger().debug(
                f"Person detected at center: {person_center}, "
                f"angle: {math.degrees(self.person_angle):.1f}°"
            )

        # Show the frame (optional, for debugging)
        cv2.imshow("Human Tracking", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.node.get_logger().info("q pressed, shutting down...")
            rclpy.shutdown()

    def track_human_ros2(self):
        """
        Main method to track human as a ROS2 node.
        """
        if not ROS2_AVAILABLE:
            print("Error: ROS2 not available. Use standalone mode instead.")
            return

        rclpy.init()

        # Create ROS2 node
        self.node = Node("human_tracker_node")
        self.bridge = CvBridge()

        # Create publisher for human position
        self.position_publisher = self.node.create_publisher(
            PointStamped, self.publish_topic, 10
        )

        # Subscribe to camera topic
        self.image_subscription = self.node.create_subscription(
            Image, self.camera_topic, self.image_callback, 10
        )

        self.node.get_logger().info(
            f"Human tracker node started. Listening on {self.camera_topic}, "
            f"publishing to {self.publish_topic}"
        )

        # Create OpenCV window
        cv2.namedWindow("Human Tracking", cv2.WINDOW_NORMAL)

        try:
            rclpy.spin(self.node)
        finally:
            # Cleanup
            self.node.destroy_node()
            self.pose.close()
            cv2.destroyAllWindows()
            rclpy.shutdown()
            print("Human Tracking Stopped")


class HumanTrackerNode(Node):
    """
    ROS2 Node wrapper for HumanTracker.
    This allows the tracker to be used as a proper ROS2 node.
    """

    def __init__(
        self, camera_topic="/camera/image_raw", publish_topic="/human_tracker/position"
    ):
        super().__init__("human_tracker_node")
        self.tracker = HumanTracker(use_ros2=False)  # Don't create nested node
        self.bridge = CvBridge()
        self.camera_topic = camera_topic
        self.publish_topic = publish_topic

        # Create publisher for human position
        self.position_publisher = self.create_publisher(
            PointStamped, self.publish_topic, 10
        )

        # Subscribe to camera topic
        self.image_subscription = self.create_subscription(
            Image, self.camera_topic, self.image_callback, 10
        )

        self.get_logger().info(
            f"Human tracker node started. Listening on {self.camera_topic}, "
            f"publishing to {self.publish_topic}"
        )

        # Create OpenCV window
        cv2.namedWindow("Human Tracking", cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        """ROS2 callback for processing camera images."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        # Detect human
        annotated_frame, person_center, visible = self.tracker.detect_human(frame)

        # Publish position if person is detected
        if visible and person_center:
            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = "camera_frame"
            point_msg.point.x = float(
                self.tracker.offset_from_center[0]
            )  # X offset in pixels
            point_msg.point.y = float(
                self.tracker.offset_from_center[1]
            )  # Y offset in pixels
            point_msg.point.z = (
                float(self.tracker.person_angle) if self.tracker.person_angle else 0.0
            )  # Angle in radians
            self.position_publisher.publish(point_msg)

        # Log tracking status periodically
        if visible:
            self.get_logger().debug(
                f"Person detected at center: {person_center}, "
                f"angle: {math.degrees(self.tracker.person_angle):.1f}°"
            )

        # Show the frame (optional, for debugging)
        cv2.imshow("Human Tracking", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("q pressed, shutting down...")
            rclpy.shutdown()


def main():
    """Main function to run human tracking."""
    import sys

    # Check if ROS2 mode is requested
    use_ros2 = "--ros2" in sys.argv or "--ros" in sys.argv

    tracker = HumanTracker(use_ros2=use_ros2)

    if use_ros2 and ROS2_AVAILABLE:
        # Use the node wrapper for cleaner ROS2 integration
        rclpy.init()
        node = HumanTrackerNode()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            node.tracker.pose.close()
            cv2.destroyAllWindows()
            rclpy.shutdown()
    else:
        # Standalone mode
        tracker.track_human_standalone()


if __name__ == "__main__":
    main()
