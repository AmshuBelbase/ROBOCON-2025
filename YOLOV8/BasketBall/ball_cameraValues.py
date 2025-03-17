if __name__ == '__main__':
    from ultralytics import YOLO
    import cv2
    import matplotlib.pyplot as plt
    import numpy as np
    import threading
    import time

    model = YOLO('best1.pt')

    cap = cv2.VideoCapture(2)

    x = np.array([0])
    y = np.array([0])
    fig , ax = plt.subplots()

    ax.set_ylim(0,720)
    ax.set_xlim(0,1280)

    
    sc = plt.scatter(x,y)
    plt.ion()

    drawing = False  
    start_point = (0, 0)
    end_point = (0, 0)
    
    line1x = np.array([0,0])
    line1y = np.array([0,0])
    line1, = ax.plot(line1x,line1y,'ro-')

    line2x = np.array([0,0])
    line2y = np.array([0,0])
    line2, = ax.plot(line2x , line2y , 'ro-')


    def draw_line(event, x, y, flags, param):
        global drawing, start_point, end_point , line1x 

        if event == cv2.EVENT_LBUTTONDOWN:  
            drawing = True
            start_point = (x, y)
            line2x[0] = x
            line2y[0] = y

        elif event == cv2.EVENT_MOUSEMOVE and drawing:  
            end_point = (x, y)
            line2x[1] = x
            line2y[1] = y

        elif event == cv2.EVENT_LBUTTONUP:  
            drawing = False
            end_point = (x, y)
            line2x[1] = x
            line2y[1] = y
        elif event == cv2.EVENT_MBUTTONDOWN:
            line1x[0] , line1x[1] = x , x
            


    cv2.namedWindow("frame")
    cv2.setMouseCallback("frame", draw_line)


    distancePerPixels = 0.1329 #cms/pixel

    plt.show()

    start_time = time.perf_counter()

    while cap.isOpened():
        succ , frame = cap.read()
        if not succ:
            print("Failed")
            break
        frame_width = frame.shape[1]
        frame_height = frame.shape[0]
        results = model.predict(source=frame , conf = 0.65 , save = False)
        resultFrame = results[0].plot()

        print(f"Frame width: {frame_width} , Frame height: {frame_height}")

        

        for c in results[0].boxes:
            x_center, y_center, width, height = c.xywh[0].cpu().numpy()
            heightBall = 0
            centerHeight = frame_height - y_center
            cv2.putText(resultFrame , f"Centre(NR): {int(centerHeight)} , Height : {int(heightBall)}" , (int(x_center - width/2), int(y_center - height/2) - 20),cv2.FONT_HERSHEY_SIMPLEX , 0.7 , color=(0,255,0) , thickness=2)
            cv2.putText(resultFrame , f"Centre(R): {int(line2y[0]-y_center)} , Height : {int(heightBall)}" , (int(x_center - width/2), int(y_center - height/2) - 40),cv2.FONT_HERSHEY_SIMPLEX , 0.7 , color=(0,0,255) , thickness=2)
            
            if x_center > line1x[0] and centerHeight > frame_height - line2y[0]:
                x = np.append(x,x_center)
                y = np.append(y,centerHeight)
                print(f"Non reference distance: {distancePerPixels*centerHeight}")
                print(f"reference distance: {distancePerPixels*(line2y[0]-y_center)}")
                sc.set_offsets(np.c_[x, y])
                plt.draw()
                plt.gcf().canvas.flush_events()
        
        if line1x[0]!=0:
            line1y[0] , line1y[1] = (0 , frame_width)
            line1.set_xdata(line1x)
            line1.set_ydata(line1y)
            cv2.line(resultFrame , (line1x[0],line1y[0]) , (line1x[1] , line1y[1]) , (255,0,0) , 2)
            plt.draw()
            plt.pause(0.001)
            plt.gcf().canvas.flush_events()
            
        if start_point != end_point:
            cv2.line(resultFrame, start_point, end_point, (0, 255, 0), 2)  
            angle  = np.arctan2(end_point[1]-start_point[1],end_point[0]-start_point[0])
            cv2.putText(resultFrame , f"Angle: {angle}" ,(start_point[0]+10 , start_point[1]-10) ,cv2.FONT_HERSHEY_SIMPLEX , 0.7 , color=(255,0,0) , thickness=2)

            line2.set_xdata(line2x)
            line2.set_ydata(frame_height-line2y)
            plt.draw()
            plt.pause(0.001)
            plt.gcf().canvas.flush_events()

        cv2.imshow("frame" , resultFrame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('e'):
            x , y  = np.array([0]) , np.array([0])
            sc.set_offsets(np.c_[x,y])
            plt.draw()
            plt.gcf().canvas.flush_events()

    max_pixel = y.max()
    max_height = distancePerPixels*max_pixel
    print(f"Max pixels: {max_pixel} , Max height: {max_height}")
    
    plt.ioff()
    cap.release()
    cv2.destroyAllWindows