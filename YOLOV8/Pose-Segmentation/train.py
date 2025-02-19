if __name__ == '__main__':
    from ultralytics import YOLO
    import cv2

    model = YOLO('yolo11m-pose.pt')
    model.train(data='dataset.yaml',epochs=100,imgsz=640,device='0',batch=4)