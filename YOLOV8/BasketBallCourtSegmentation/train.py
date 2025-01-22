if __name__ == '__main__':
    from ultralytics import YOLO
    import torch

    model = YOLO('yolov8n.pt')

    model.train(data="myconfig.yaml",imgsz=640,batch=10,epochs=100,device=0)

    metrics = model.val()
    print(metrics)
