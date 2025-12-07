#!/usr/bin/env python3

import time
import cv2 as cv
import numpy as np
import mediapipe as mp
from mediapipe.tasks.python.core.base_options import BaseOptions

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# -------------- MediaPipe setup --------------

# Callback function to print result of gesture recognition
def print_result(result):
    global latest_gesture

    if result.gestures:
        category = result.gestures[0][0]
        name = category.category_name
        latest_gesture = f"Gesture: {name}"
        print(latest_gesture)
    else:
        latest_gesture = "None"

model_path = '/home/tabby305/Downloads/gesture_recognizer.task'

GestureRecognizer = mp.tasks.vision.GestureRecognizer
GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
GestureRecognizerResult = mp.tasks.vision.GestureRecognizerResult
VisionRunningMode = mp.tasks.vision.RunningMode

options = GestureRecognizerOptions(
    base_options=BaseOptions(model_asset_path=model_path),
    running_mode=VisionRunningMode.LIVE_STREAM,
    result_callback=print_result
)

# Create recognizer once, reuse in callback
recognizer = GestureRecognizer.create_from_options(options)

bridge = CvBridge()
start_time = time.time()


# -------------- ROS2 node --------------

class GestureRecognizerNode(Node):
    def __init__(self):
        super().__init__('gesture_recognizer_node')

        # Subscribe to your camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.first_frame = False
        self.get_logger().info('Gesture recognizer node started. Listening on /camera/image/raw')

    def image_callback(self, msg):
        global start_time

        try:
            frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        if not self.first_frame:
            self.get_logger().info("Received first frame from camera topic")
            self.first_frame = True

        # Convert frame to RGB for MediaPipe
        rgb_frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

        frame_timestamp_ms = int((time.time() - start_time) * 1000)

        # Run recognizer asynchronously
        recognizer.recognize_async(mp_image, frame_timestamp_ms)

        # Show the frame 
        cv.imshow('frame', frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("q pressed, shutting down...")
            rclpy.shutdown()


def main():
    rclpy.init()

    # Create OpenCV window 
    cv.namedWindow('frame', cv.WINDOW_NORMAL)

    node = GestureRecognizerNode()

    try:
        rclpy.spin(node)
    finally:
        # Clean up
        node.destroy_node()
        recognizer.close()
        cv.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
