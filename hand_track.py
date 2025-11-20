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
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(detectShadows=True)

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
        Returns the contour and its convex hull.
        """
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None, None

        # Find the largest contour (likely the hand)
        largest_contour = max(contours, key=cv2.contourArea)

        # Filter out small contours (noise)
        if cv2.contourArea(largest_contour) < 5000:
            return None, None

        # Find convex hull
        hull = cv2.convexHull(largest_contour)

        return largest_contour, hull

    def find_hand_center(self, contour):
        """
        Calculate the center point of the hand contour.
        Returns (cx, cy) coordinates.
        """
        if contour is None:
            return None

        # Calculate moments
        M = cv2.moments(contour)

        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return (cx, cy)

        return None

    def find_fingertips(self, contour, hull):
        """
        Detect fingertips using convexity defects.
        Returns list of fingertip coordinates.
        """
        if contour is None or hull is None:
            return []

        # Find convexity defects
        try:
            defects = cv2.convexityDefects(
                contour, cv2.convexHull(contour, returnPoints=False)
            )
        except (cv2.error, ValueError):
            return []

        if defects is None:
            return []

        fingertips = []

        for i in range(defects.shape[0]):
            s, e, f, d = defects[i, 0]
            start = tuple(contour[s][0])
            end = tuple(contour[e][0])
            far = tuple(contour[f][0])

            # Calculate the angle between vectors
            a = np.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
            b = np.sqrt((far[0] - start[0]) ** 2 + (far[1] - start[1]) ** 2)
            c = np.sqrt((end[0] - far[0]) ** 2 + (end[1] - far[1]) ** 2)

            angle = np.arccos((b**2 + c**2 - a**2) / (2 * b * c))

            # Filter defects that are likely fingertips
            if angle <= np.pi / 2 and d > 10000:
                fingertips.append(end)

        return fingertips

    def draw_hand_tracking(self, frame, contour, hull, center, fingertips):
        """
        Draw hand tracking visualization on the frame.
        """
        if contour is not None:
            # Draw contour
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)

        if hull is not None:
            # Draw convex hull
            cv2.drawContours(frame, [hull], -1, (255, 0, 0), 2)

        if center is not None:
            # Draw center point
            cv2.circle(frame, center, 10, (0, 0, 255), -1)
            cv2.putText(
                frame,
                "Center",
                (center[0] + 15, center[1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                2,
            )

        # Draw fingertips
        for i, fingertip in enumerate(fingertips):
            cv2.circle(frame, fingertip, 8, (255, 255, 0), -1)
            cv2.putText(
                frame,
                f"F{i+1}",
                (fingertip[0] + 10, fingertip[1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 0),
                2,
            )

        return frame

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

            # Find hand contour and convex hull
            contour, hull = self.find_hand_contour(skin_mask)

            # Find hand center
            center = self.find_hand_center(contour)

            # Find fingertips
            fingertips = self.find_fingertips(contour, hull)

            # Draw tracking visualization
            frame = self.draw_hand_tracking(frame, contour, hull, center, fingertips)

            # Display hand count
            finger_count = len(fingertips)
            cv2.putText(
                frame,
                f"Fingers: {finger_count}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2,
            )

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
