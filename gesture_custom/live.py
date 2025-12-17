"""
Live webcam gesture recognition using a MediaPipe Gesture Recognizer model.
Overlays the top recognized gesture and its confidence on the webcam feed.
Requires a trained gesture recognizer model exported as a .task bundle.
Make sure to run `train_gesture_recognizer.py` first to create the model."""
import time
import cv2
import mediapipe as mp


def main():
    """
    Main function to run live gesture recognition from webcam.
    Uses MediaPipe GestureRecognizer in LIVE_STREAM mode.
    Overlays the top recognized gesture and its confidence on the webcam feed.
    Press 'q' to quit the application.
    """
    model_path = "./exported_model/gesture_recognizer.task"

    BaseOptions = mp.tasks.BaseOptions
    GestureRecognizer = mp.tasks.vision.GestureRecognizer
    GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
    VisionRunningMode = mp.tasks.vision.RunningMode

    latest_result = {"text": ""}

    def on_result(result, output_image, timestamp_ms: int):
        """
        Callback function to handle gesture recognition results.
         Args:
             result: GestureRecognizerResult from MediaPipe.
             output_image: The image with any overlays (not used here).
             timestamp_ms: Timestamp of the frame in milliseconds.
         """
        # result.gestures is a list per hand; each is a list of Category
        if result.gestures and len(
                result.gestures) > 0 and len(
                result.gestures[0]) > 0:
            top = result.gestures[0][0]
            latest_result["text"] = f"{top.category_name} ({top.score:.2f})"
        else:
            latest_result["text"] = ""

    options = GestureRecognizerOptions(
        base_options=BaseOptions(model_asset_path=model_path),
        running_mode=VisionRunningMode.LIVE_STREAM,
        result_callback=on_result,
        # Tunables:
        # num_hands=1,
        # min_hand_detection_confidence=0.5,
        # min_hand_presence_confidence=0.5,
        # min_tracking_confidence=0.5,
    )

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Could not open webcam.")

    with GestureRecognizer.create_from_options(options) as recognizer:
        while True:
            ok, frame_bgr = cap.read()
            if not ok:
                break

            # Convert to RGB for MediaPipe Image
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(
                image_format=mp.ImageFormat.SRGB,
                data=frame_rgb)

            timestamp_ms = int(time.time() * 1000)
            recognizer.recognize_async(mp_image, timestamp_ms)

            # Overlay latest result (async callback updates it)
            text = latest_result["text"]
            if text:
                cv2.putText(frame_bgr, text, (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

            cv2.imshow("Gesture Recognizer (Live)", frame_bgr)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
