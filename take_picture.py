# take_picture.py

import os
import cv2 as cv

last_saved_ms = 0  # 


def save_fist(
    output_image,
    timestamp_ms,
    cooldown_ms=1000,
    save_subdir="Downloads",
):
    """
    Save a frame to ~/Downloads when the given result contains a closed fist.

    Parameters
    ----------
    result : GestureRecognizerResult
        MediaPipe gesture result.
    output_image : mp.Image
        MediaPipe image corresponding to this result (RGB).
    timestamp_ms : int
        Timestamp for uniqueness / wait time
    gesture_name : str
        Name of the gesture to trigger on (default: 'Closed_Fist').
    min_score : float
        Minimum confidence to accept the gesture.
    cooldown_ms : int
        Minimum time between saves.
    save_subdir : str
        Directory under the home folder to save the image.
    """
    global last_saved_ms

    #if not result.gestures:
    #    return

    #category = result.gestures[0][0]
    #name = category.category_name
    #score = category.score

    p#rint(f"Gesture: {name} ({score:.2f})")

    ## Only act if it's the gesture we care about
    #if name != gesture_name or score < min_score:
    #    return

    # Enforce cooldown
    if timestamp_ms - last_saved_ms <= cooldown_ms:
        return

    last_saved_ms = timestamp_ms

    # Convert MediaPipe Image (RGB) -> NumPy -> BGR for OpenCV
    rgb_frame = output_image.numpy_view()
    bgr_frame = cv.cvtColor(rgb_frame, cv.COLOR_RGB2BGR)

    # Build save path: ~/Downloads/closed_fist_<timestamp>.png
    home_dir = os.path.expanduser("~")
    save_dir = os.path.join(home_dir, save_subdir)
    os.makedirs(save_dir, exist_ok=True)

    filename = f"closed_fist_{timestamp_ms}.png"
    filepath = os.path.join(save_dir, filename)

    success = cv.imwrite(filepath, bgr_frame)
    if success:
        print(f"[INFO] Saved closed fist frame to: {filepath}")
    else:
        print(f"[WARN] Failed to save image to: {filepath}")


save_fist()