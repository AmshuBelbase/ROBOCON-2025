if __name__ == '__main__':
    from ultralytics import YOLO
    import cv2

    model = YOLO('best.pt')
    model.train(data='dataset.yaml',epochs=80,imgsz=640,device='0',batch=4)