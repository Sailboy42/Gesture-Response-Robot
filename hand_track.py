#!/usr/bin/env python3
"""
Hand Tracking using OpenCV
This script tracks a hand in real-time using OpenCV's computer vision techniques.
"""

import cv2
import numpy as np


class HandTracker:
    def __init__(self):
        """Initialize the hand tracker."""
        self.cap = None

    def detect_skin(self, frame):
        """
        Detect skin color in the frame using HSV color space.
        Returns a binary mask of detected skin regions.
        """
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define range for skin color in HSV
        # These values work well for various skin tones
        lower_skin = np.array([0, 20, 70], dtype=np.uint8)
        upper_skin = np.array([20, 255, 255], dtype=np.uint8)

        # Create mask for skin color
        mask = cv2.inRange(hsv, lower_skin, upper_skin)

        # Apply morphological operations to remove noise
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        return mask

    def find_hand_contour(self, mask):
        """
        Find the largest contour which is likely the hand.
        Returns the contour.
        """
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        # Find the largest contour (likely the hand)
        largest_contour = max(contours, key=cv2.contourArea)

        # Filter out small contours (noise)
        if cv2.contourArea(largest_contour) < 5000:
            return None

        return largest_contour

    def track_hand(self):
        """
        Main method to track hand in real-time video stream.
        """
        # Initialize video capture
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            print("Error: Could not open camera")
            return

        print("Hand Tracking Started")
        print("Press 'q' to quit")

        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Error: Could not read frame")
                break

            # Flip frame horizontally for mirror effect
            frame = cv2.flip(frame, 1)

            # Detect skin
            skin_mask = self.detect_skin(frame)

            # Find hand contour
            contour = self.find_hand_contour(skin_mask)

            # Draw hand contour if found
            if contour is not None:
                cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)

            # Show the frame
            cv2.imshow("Hand Tracking", frame)

            # Show the mask (optional, for debugging)
            cv2.imshow("Skin Mask", skin_mask)

            # Break loop on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        # Cleanup
        self.cap.release()
        cv2.destroyAllWindows()
        print("Hand Tracking Stopped")


def main():
    """Main function to run hand tracking."""
    tracker = HandTracker()
    tracker.track_hand()


if __name__ == "__main__":
    main()
