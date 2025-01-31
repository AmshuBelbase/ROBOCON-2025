if __name__ == '__main__':
    from ultralytics import YOLO
    import cv2
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotfile import showplot

    def quadarea(point):
        points=np.array(point,np.float64)
        x1, y1 = points[0]
        x2, y2 = points[1]
        x3, y3 = points[2]
        x4, y4 = points[3]
        return abs(x1*y2 + x2*y3 + x3*y4 + x4*y1 - (y1*x2 + y2*x3 + y3*x4 + y4*x1)) / 2

    model=YOLO('bestidk.pt') #Load The YOLO Model
    img = cv2.imread('frame_0163.jpg') #Read the frame
    cap = cv2.VideoCapture('video4.mp4')
    while True: 
        succ,frame = cap.read()
        if not succ:
            break
        results = model.predict(frame) #Run Inference on the frame
        
        if results[0].boxes:
            image_height = img.shape[0] #Extract the image shapes
            image_width = img.shape[1] #Extract the image shapes
            keypoints = results[0].keypoints #Extract the tensor information
            
            x = keypoints.cpu().numpy()  #Convert the tensor information to nummpy array
            coords = np.vstack(x.xy) #Get the x-y coordinates of the keypoints
            
            box_coords = np.array(np.vstack(results[0].boxes.xywh.cpu().numpy()),dtype=np.uint32) #Get the bounding boxes coordinates 
            bbox_tlx , bbox_tly , bbox_width , bbox_height = box_coords[0,:] #Extract that in variables
            print(box_coords) #Print the box coords data
            
            
            print(coords) #print the Keypoint coordinates
            
            area = quadarea(coords)
            print(area)

            length_difference = coords[1][1]-coords[2][1] - (coords[0][1]-coords[3][1])
            print(length_difference)
            imag = cv2.putText(results[0].plot() , f"Area is: {area}" , (bbox_tlx-100,bbox_tly+5) , cv2.FONT_HERSHEY_SIMPLEX , 1 , color=(255,0,0) , thickness=2)
            image = cv2.putText(imag , f"Difference: {length_difference}" , (bbox_tlx-100,bbox_tly-15) , cv2.FONT_HERSHEY_SIMPLEX , 1 , color=(255,0,0) , thickness=2)
            cv2.imshow("Window" , image)
        else:
            cv2.imshow("Window",frame)
        # showplot(image_height,image_width,coords) #Display the plot
        if cv2.waitKey(1)==ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows




















#     model.train(data='dataset.yaml',epochs=100,imgsz=640,device='0')
#     cap = cv2.VideoCapture('video4.mp4')
#     while True:
#         succ,frame = cap.read()
#         if not succ:
#             print("Shits up")
#             break
#         results=model.predict(source=frame)
#         cv2.imshow('frame',results[0].plot())
#         if cv2.waitKey(1)==ord('q'):
#             break
# cap.release()
# cv2.destroyAllWindows()