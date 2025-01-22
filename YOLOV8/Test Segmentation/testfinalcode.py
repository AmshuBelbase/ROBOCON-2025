if __name__ == '__main__':
    # Import all the libraries
    import cv2
    import numpy as np
    from ultralytics import YOLO
    import matplotlib.pyplot as plt 
    from segmenting import segment
    from masked_frame import masking
    from matplotfile import matplot , showplot
    from coords_extraction import bestcor
    import os

    cap = cv2.VideoCapture('video2.mp4') #Capture the video frame
    count=0
    os.chdir(r'C:\ROBOCON\YOLOV8\Test Segmentation\writefiles')
    while True:
        succ,frame = cap.read() #Capture a frame from the video

        if not succ:
            print("couldnt Read file")
            break

        # cv2.imshow("Original Video Frame" , frame) #Display the captured from as the original video

        image_height=frame.shape[0]
        image_width=frame.shape[1]

        pred_result = segment(frame) #Get the segmented result object
        seg_frame = pred_result[0].plot() #Get the annotated/segmented frame
        cv2.imshow("Segmented Frame",seg_frame) #Display the segmented frame

        mframe,mask_object = masking(pred_result) #Get the masked frame and the masked object
        masked_frame=cv2.resize(mframe,(image_width,image_height))
        cv2.imshow("Masked Frame",masked_frame) #Display the masked frame

        # m_frame = (masked_frame * 255).astype(np.uint8)
        # cv2.imwrite(f"image{count}.jpg",m_frame)
        # count=count+1
        coords = matplot(mask_object,image_height,image_width) #Display the plotted boundary
        print(coords)

        # outercor,area = bestcor(coords) #Get the best outer coords
        # print(outercor,area) #print the outer coords
        # showplot(image_height,image_width,np.array(outercor))#Visualize the outer coords


        key = cv2.waitKey(0)
        if key == ord('q'):
            continue
        elif key==ord('e'):
            break

    cap.release()
    cv2.destroyAllWindows()


# ([596, 390], [714, 266], [714, 202], [688, 44])