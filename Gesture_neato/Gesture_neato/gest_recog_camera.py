#!/usr/bin/env python3
import time
import cv2 as cv
import numpy as np
import mediapipe as mp
from mediapipe.tasks.python.core.base_options import BaseOptions
from Gesture_neato.take_picture import save_fist

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
#from take_picture import save_fist

#from take_picture import save_fist

# -------------- MediaPipe setup --------------

current_gesture = None  


model_path = '../../gesture_custom/exported_model/gesture_recognizer.task'
#model_path = '/home/tabby305/Downloads/gesture_recognizer.task'
#model_path = '/home/bhargavi/Downloads/gesture_recognizer.task'
# GestureRecognizer = mp.tasks.vision.GestureRecognizer
# GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
# GestureRecognizerResult = mp.tasks.vision.GestureRecognizerResult
# VisionRunningMode = mp.tasks.vision.RunningMode

# options = GestureRecognizerOptions(
#     base_options=BaseOptions(model_asset_path=model_path),
#     running_mode=VisionRunningMode.LIVE_STREAM,
#     result_callback= self.print_result

# # Create recognizer once, reuse in callback
# recognizer = GestureRecognizer.create_from_options(options)

# bridge = CvBridge()
# start_time = time.time()


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
        # Create publisher for topic called gesture
        self.gespub = self.create_publisher(String, 'gesture', 10)



        self.first_frame = False
        self.get_logger().info('Gesture recognizer node started. Listening on /camera/image/raw')

        self.bridge = CvBridge()
        self.start_time = time.time()
        
        GestureRecognizer = mp.tasks.vision.GestureRecognizer
        GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
        GestureRecognizerResult = mp.tasks.vision.GestureRecognizerResult
        VisionRunningMode = mp.tasks.vision.RunningMode
        options = GestureRecognizerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=VisionRunningMode.LIVE_STREAM,
            result_callback= self.print_result
        )

        self.recognizer = GestureRecognizer.create_from_options(options)
        self.first_frame = False
        self.get_logger().info('Gesture recognizer node started. Listening on /camera/image_raw')

    def print_result(self, result, output_image, timestamp_ms):
        if result.gestures:
            category = result.gestures[0][0]
            name = category.category_name
        else:
            name = ""
        print(name)

        if name == "Closed_Fist":
            save_fist(output_image=output_image, timestamp_ms=timestamp_ms)

        # publish a real string
        self.gespub.publish(String(data=name))

        # optional: debug
        self.get_logger().info(f"Gesture: {name}")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        if not self.first_frame:
            self.get_logger().info("Received first frame from camera topic")
            self.first_frame = True

        

        # Convert frame to RGB for MediaPipe
        rgb_frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

        frame_timestamp_ms = int((time.time() - self.start_time) * 1000)

        # Run recognizer async
        self.recognizer.recognize_async(mp_image, frame_timestamp_ms)

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
        node.recognizer.close()
        cv.destroyAllWindows()
        rclpy.shutdown()

    return current_gesture 


if __name__ == '__main__':
    print(main())  




