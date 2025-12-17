import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.tasks.python.core.base_options import BaseOptions

import time
import numpy as np
import cv2 as cv

# Callback function to print result of gesture recognition on Webcam

def print_result(result, output_image, timestamp_ms):
    if result.gestures:
        category = result.gestures[0][0]
        name = category.category_name
        print(f"Gesture: {name}")
    
    # Here we can put if statements to run neato commands based on what the gesture is recognized to be
    # We'll need to write it based on when the gesture changes not every time it is detected because
    # it will keep writing "Thumbs Up" until the gesture changes
   
        

# This is where the MediaPipe model lives on my laptop, download the task and change per person
model_path = '../../gesture_custom/exported_model/gesture_recognizer.task'


# Initializing the model and starting the standard options for live streaming video
base_options = BaseOptions(model_asset_path=model_path)

GestureRecognizer = mp.tasks.vision.GestureRecognizer
GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
GestureRecognizerResult = mp.tasks.vision.GestureRecognizerResult
VisionRunningMode = mp.tasks.vision.RunningMode

options = GestureRecognizerOptions(
    base_options=BaseOptions(model_asset_path= model_path),
    running_mode=VisionRunningMode.LIVE_STREAM,
    result_callback=print_result)



# Start Live Camera Feed to Use as Input
cap = cv.VideoCapture(0)
# Record Start Time
start_time = time.time()

if not cap.isOpened():
    print("Cannot open camera")
    exit()
# initializing an instance of the gesture recognizer as recognizer    
with GestureRecognizer.create_from_options(options) as recognizer:
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        rgb_frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB) # need to convert frame to image form that the mediapipe model takes
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
        frame_timestamp_ms = int((time.time() - start_time) * 1000) # time per frame, needed so that model can run without interuptions

        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        
        # Runs the recognizer asynchronously to detect gesture without interupting the live video
        recognizer.recognize_async(mp_image, frame_timestamp_ms)  
        # Display the resulting frame
        cv.imshow('frame',frame)
        if cv.waitKey(1) == ord('q'):
            break

# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
