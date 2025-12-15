# take_picture.py
import os
import cv2 as cv

last_saved_ms = 0

def save_fist(output_image, timestamp_ms, cooldown_ms=1000):
    global last_saved_ms
    print("fist is saving")

    # Enforce cooldown
   # if timestamp_ms - last_saved_ms <= cooldown_ms:
     #   return
    #last_saved_ms = timestamp_ms

    # Convert MediaPipe Image (RGB) -> NumPy -> BGR for OpenCV
    rgb_frame = output_image.numpy_view()
    bgr_frame = cv.cvtColor(rgb_frame, cv.COLOR_RGB2BGR)

    # Save to ~/Downloads/<save_subdir>/
    home_dir = "/home/tabby305"
    save_dir = os.path.join(home_dir, "Downloads")
    
    #os.makedirs(save_dir, exist_ok=True)

    filename = f"closed_fist_{timestamp_ms}.png"
    filepath = os.path.join(save_dir, filename)

    success = cv.imwrite(filepath, bgr_frame)
    if success:
        print(f"[INFO] Saved closed fist frame to: {filepath}")
    else:
        print(f"[WARN] Failed to save image to: {filepath}")
