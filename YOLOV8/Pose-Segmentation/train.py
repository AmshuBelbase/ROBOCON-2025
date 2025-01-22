if __name__ == '__main__':
    from ultralytics import YOLO
    import cv2

    model=YOLO('best.pt')
    # model.train(data='dataset.yaml',epochs=100,imgsz=640,device='0')
    cap = cv2.VideoCapture('video4.mp4')
    while True:
        succ,frame = cap.read()
        if not succ:
            print("Shits up")
            break
        results=model.predict(source=frame)
        cv2.imshow('frame',results[0].plot())
        if cv2.waitKey(1)==ord('q'):
            break
cap.release()
cv2.destroyAllWindows()

    