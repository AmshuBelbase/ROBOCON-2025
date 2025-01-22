import cv2
import ultralytics

model = ultralytics.YOLO('best1.pt')

def segment(frame):
    results = model.predict(source=frame,conf=0.25,save=False)
    return results

