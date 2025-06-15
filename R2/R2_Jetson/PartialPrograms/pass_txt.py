import cv2
import numpy as np
import socket  
import time
import pygame
from multiprocessing import Queue , Process , Manager
import pyzed.sl as sl 
from ultralytics import YOLO
from typing import List
import struct
import select
from pupil_apriltags import Detector


#Define packing format (4 integers , 6 short integers - 4 bytes , 2 bytes)
pack_format = '<6i8h'

#Globals to hold motor speeds
bldc_rpm = 0
mapped_pwm = 0
pass_bldc=0
mapped_pwm_april=0
#Global joystick Object
joystick = None


#Process for Camera and YOLO Detection
def zed2_process(shared):

    #Class for Normal Webcam
    class Normal_WebCam:
        def __init__(self):
            print("Initializing Camera...")
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                print("Cannot open camera") 
            else:
                print("Initialized Camera")

        def get_feed(self):
            ret, frame = self.cap.read()
            if not ret:
                print("Can't receive frame. Exiting ...") 
                return None
            else:
                #aman added  it to reverse the frame
                # frame = cv2.flip(frame, -1) 
                return frame
            
    #Class For ZED2 Feed
    class ZED2_feed:
        def __init__(self): 
            print("Initializing Camera...")
            self.zed = sl.Camera()
            self.input_type = sl.InputType() 

            # Create a InitParameters object and set configuration parameters
            init_params = sl.InitParameters() 
            init_params.camera_resolution = sl.RESOLUTION.HD1080 # Use HD720 opr HD1200 video mode, depending on camera type.
            init_params.camera_fps = 30  # Set fps at 30 
            # Set configuration parameters 
            init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE # Use ULTRA depth mode
            init_params.coordinate_units = sl.UNIT.MILLIMETER # Use millimeter units (for depth measurements)
            init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
            init_params.depth_maximum_distance = 10000
            init_params.depth_minimum_distance = 1000


            self.runtime_params = sl.RuntimeParameters()
            status = self.zed.open(init_params)

            if status != sl.ERROR_CODE.SUCCESS:
                print(repr(status))
                exit()

            self.image_left_tmp = sl.Mat()
            self.depth_map = sl.Mat()

            print("Initialized Camera")
            
        def get_feed(self):      
            err = self.zed.grab(self.runtime_params) 
            if err == sl.ERROR_CODE.SUCCESS: # Check that a new image is successfully acquire
                self.zed.retrieve_image(self.image_left_tmp, sl.VIEW.LEFT)
                self.zed.retrieve_measure(self.depth_map, sl.MEASURE.DEPTH) # Retrieve depth
                image_net = self.image_left_tmp.get_data()
                frame = cv2.cvtColor(image_net, cv2.COLOR_RGBA2RGB)
                return frame
            else:
                print("Error during capture : ", err) 
                return None
            
    # Constants for hoop distance
    CONFIDENCE_THRESHOLD = 0.25
    FRAME_SKIP = 5
    BASKET_WIDTH_CM = 48.5
    CONST = 487.0 #Calibrated constant

    #constants for passing (apriltag)
    calib_data = np.load(r'/home/robocon/Desktop/Dominic/MultiMatrixLenevo.npz')  # Replace with your filename
    camera_matrix = calib_data['camMatrix']
    dist_coeffs = calib_data['distCoef']
    # === Extract intrinsics for AprilTag detector ===
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    camera_params = [fx, fy, cx, cy]
    # === Tag parameters ===
    tag_size = 30.5  # Real-world tag size in centimeters (e.g., 15 cm)
    # === Initialize AprilTag detector ===
    detector = Detector(families='tag36h11',
                        nthreads=1,
                        quad_decimate=1.0,
                        quad_sigma=0.0,
                        refine_edges=1,
                        decode_sharpening=0.25,
                        debug=0)

    # Load YOLO model
    model = YOLO(r'/home/robocon/Desktop/Dominic/n_hoop.pt')

    # Initialize camera
    z = Normal_WebCam()
    # z = ZED2_feed()

    #Try to access the feed continously if not available
    while True:
        try:
            #Current frame count since last active time of Camera
            frame_count = 0

            #Mapping function for offset to pwm    
            def map_range(value, in_min, in_max, out_min, out_max):
                return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

            #Get frames continously
            while True:
                
                frame = z.get_feed()  
                frame_height, frame_width = frame.shape[:2]                 
                
                #Skip frames
                frame_count += 1
                if frame_count % FRAME_SKIP != 0:
                    print("Skip: ", frame_count)
                    continue
                
                frame_undistorted = frame#cv2.undistort(frame, camera_matrix, dist_coeffs)
                gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)

                distance_pass=0
                # === Detect AprilTags ===
                tags = detector.detect(gray, estimate_tag_pose=True,
                                    camera_params=camera_params, tag_size=tag_size)
                #Mid band threshold values FOR APRIL TAG
                threshold_april = 0.01
                right_threshold_april = int(frame_width / 2.0 + frame_width * threshold_april)
                left_threshold_april = int(frame_width / 2.0 - frame_width * threshold_april) 
                for tag in tags:
                    tag_id = tag.tag_id
                    
                    pose_t = tag.pose_t  # [x, y, z] translation in meters
                    pose_r = tag.pose_R  # 3x3 rotation matrix

                    distance_pass = np.linalg.norm(pose_t)

                    # Draw tag center and info
                    center = tuple(np.int32(tag.center))
                    cv2.circle(frame, center, 6, (0, 255, 0), -1)
                    cv2.putText(frame, f"ID: {tag_id}  Dist: {distance_pass:.2f}m",
                                (center[0] + 10, center[1]), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (255, 0, 0), 2)
                    error_offset_april = 0
                    #Final pwm for the turntable
                    mapped_pwm_april = 0


                    # detected and with the offset
                    if right_threshold_april < center[0] or center[0] < left_threshold_april:
                        #Calculate the pwm
                        error_offset_april = center[0] - (frame_width // 2) 
                        mapped_pwm_april = map_range(abs(error_offset_april), 0, (frame_width//2),10, 128) #####CHECK KAR YE JOYSTICK MAE KAISE MAP KARNA HAI
                        mapped_pwm_april = -mapped_pwm_april if error_offset_april<0 else mapped_pwm_april
                    
                        #Update values in shared dictionary
                        shared['pass_turn'] = mapped_pwm_april
                    else:
                        #Send default values when basket inside the boundaries
                        error_offset_april =0                             
                        mapped_pwm_april = 1
                        shared['pass_turn'] = mapped_pwm

                    # Draw tag border
                    for i in range(4):
                        pt1 = tuple(np.int32(tag.corners[i]))
                        pt2 = tuple(np.int32(tag.corners[(i + 1) % 4]))
                        cv2.line(frame, pt1, pt2, (0, 255, 255), 2)
                shared['dist_pass']=distance_pass
                #Get frame height and width
                
                
                #Mid band threshold values FOR HOOP
                threshold = 0.01
                right_threshold = int(frame_width / 2.0 + frame_width * threshold)
                left_threshold = int(frame_width / 2.0 - frame_width * threshold) 

                #run yolo inference
                results = model.predict(source=frame, conf=CONFIDENCE_THRESHOLD, save=False, verbose=False)
                

                #Flag to check if Basket detected or not
                det = False
                for res in results:
                    for box in res.boxes:

                        #Coordinates and data of the detected objects
                        x_center, y_center, width, height = box.xywh[0].cpu().numpy()
                        x_center, y_center, width, height = float(x_center), float(y_center), float(width), float(height)
                        class_id = int(box.cls[0].cpu().numpy())
                        class_name = results[0].names.get(class_id, "Unknown") 

                        #Check if object is basket otherwise check other object
                        if class_name != "basket": 
                            continue

                        det = True
                        
                        #Calculate the distance to basket
                        unit_length = BASKET_WIDTH_CM / width
                        distance = unit_length * CONST
                        print(f"Distance to basket: {distance:.2f} cm")

                        #Coordinates to draw the bounding boxes
                        xmin = x_center - (width / 2)
                        ymin = y_center - (height / 2)
                        xmax = x_center + (width / 2)
                        ymax = y_center + (height / 2)

                        confidence = round(box.conf[0].item(), 3)
                        frame = cv2.rectangle(frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (255, 0, 0), 2)

                        #Error offset from tehe middle of the screen
                        error_offset = 0

                        #Final pwm for the turntable
                        mapped_pwm = 0


                        # detected and with the offset
                        if right_threshold < x_center or x_center < left_threshold:
                            #Calculate the pwm
                            error_offset = x_center - (frame_width // 2) 
                            mapped_pwm = map_range(abs(error_offset), 0, (frame_width//2),10, 200)
                            mapped_pwm = -mapped_pwm if error_offset<0 else mapped_pwm
                        
                            #Update values in shared dictionary
                            shared['pwm'] = mapped_pwm
                            shared['distance'] =distance
                        else:
                            #Send default values when basket inside the boundaries
                            error_offset =0                             
                            mapped_pwm = -0
                            shared['pwm'] = mapped_pwm
                            shared['distance'] = distance
                                
                        #Display details - frame width , distance , error offset
                        cv2.putText(frame,str(frame_width), (320, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
                        cv2.putText(frame, f"Dist: {distance:.2f}", (500, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
                        cv2.putText(frame, f"Offset: {round(error_offset,2)}", (1000, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
                if not det:
                    #If basket not detected send default values
                    mapped_pwm = 0
                    shared['pwm'] = mapped_pwm
                    shared['distance'] = 0
                
                #Draw the boundary lines
                frame= cv2.line(frame, (frame_width // 2,0), (frame_width // 2,frame_height), (255,255,255), 1)
                frame= cv2.line(frame, (right_threshold,0), (right_threshold,frame_height), (255,0,255), 1)
                frame= cv2.line(frame, (left_threshold,0), (left_threshold,frame_height), (255,255,0), 1)
                cv2.imshow("distance", frame)
                # cv2.imshow("AprilTag Detection", frame_undistorted)

                if cv2.waitKey(1) == ord('q'):
                    break

            cv2.destroyAllWindows()
        except Exception as e: 
            #If camera feed is not captured
            #Send error message and try again
            shared['pwm']=0
            # z=ZED2_feed()
            shared['distance']=0
            print("Can't receive frame. Trying again ...") 
            print(f"Ye error hua camera ke saath: {e}")
            continue

def ps4_process(shared):
    #Global rpm value for bldc
    global bldc_rpm , joystick, pass_bldc

    #Function to get rpm from distance
    def get_rpm_hoop(distance):
        distances = np.array([390,317,285,245,220,415,460])  # centimeters
        rpm = np.array([3535,3265,3050,2830,2755,3750,4025])
        coefficients = np.polyfit(distances,rpm ,2)
        return coefficients[0] * distance**2 + coefficients[1] * distance + coefficients[2]
    def get_rpm_pass(distance):
        distances = np.array([390,317,285,245,220,415,460])  # centimeters
        rpm = np.array([3535,3265,3050,2830,2755,3750,4025]) #currently rndom array, need to find proper values later
        coefficients = np.polyfit(distances,rpm ,2)
        return coefficients[0] * distance**2 + coefficients[1] * distance + coefficients[2]
    

    #Initialize pygame and joystick module
    pygame.init()
    pygame.joystick.init()


    #Function to initialize a joystick
    def init_joystick():
        print("Initializing JoyStick")
        global joystick
        if pygame.joystick.get_count() > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            print(f"Joystick connected: {joystick.get_name()}")
        else:
            joystick = None
            print("No Controller")

    #Map stick values
    def map_stick(val):
        return round((val + 1) * 127.5)  # -1 to 1 to 0 to 255
    
    def map_to_255(value: float) -> int:
        value = max(0.0, min(1.0, value))
        return int(round(value * 255))



    #Initialize joystick
    init_joystick()  
        
    while True:

        #Call pygame event handler
        pygame.event.pump()

        #Check event queue for device addition and removal
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEADDED:
                print("Control Andar")
                init_joystick()
            elif event.type == pygame.JOYDEVICEREMOVED:
                print("Control Bahar")
                joystick = None
        
        #Check if joystick initialized
        if joystick:
            
            try:
                #Access joystick values
                lx = round(joystick.get_axis(0), 2)
                ly = -round(joystick.get_axis(1), 2)
                rx = round(joystick.get_axis(2), 2)
                ry = -round(joystick.get_axis(5), 2)

                lx_mapped = map_stick(lx)
                ly_mapped = map_stick(ly)
                rx_mapped = map_stick(rx)
                ry_mapped = map_stick(ry)

                l2 = map_to_255(joystick.get_axis(3))
                r2 = map_to_255(joystick.get_axis(4))

                cross = joystick.get_button(1)
                circle = joystick.get_button(2)
                square = joystick.get_button(0)
                triangle = joystick.get_button(3)

                l1 = joystick.get_button(4)
                r1 = joystick.get_button(5)

                # dx , dy = joystick.get_hat(0)
                
                # distance to bldc rpm
                bldc_rpm = get_rpm_hoop(shared['distance'])
                pass_bldc=get_rpm_pass(shared['dist_pass'])
                if shared['distance']==0 or bldc_rpm>100000:
                    bldc_rpm = 0
                if shared['dist_pass']==0 or pass_bldc>100000:
                    pass_bldc = 0
                number=0
                with open('rpm.txt', 'r') as file:
                    number = int(file.readline().strip())

                

                #Assign all values to the shared dictionary
                shared["axis"] = [lx_mapped, ly_mapped, rx_mapped, ry_mapped]
                shared["l1"] = l1
                shared["r1"] = r1
                shared["l2"] = l2
                shared["r2"] = r2
                shared["cross"] = cross
                shared["square"] = square
                shared["circle"] = circle
                shared["triangle"] = triangle
                shared["bldc_rpm"] = bldc_rpm
                shared["pass_bldc"]=number

                #Print the values
                print(int(shared['axis'][0]), int(shared['axis'][1]), int(shared['axis'][2]), int(shared['axis'][3]), int(shared['l2']),int(shared['r2']),
                        int(shared['r1']),int(shared['l1']), int(shared['cross']), int(shared['square']), int(shared['circle']),
                            int(shared['triangle']), int(shared['pwm']), int(shared['bldc_rpm']),int(shared['pass_bldc']),int(shared['dist_pass']),int(shared['pass_turn']))
                

            except pygame.error:
                print("Error in accessing values")
                joystick = None
        #If joystick not found try again        
        else:
            time.sleep(0.1)


#Process to 
def send_data(shared):

    #Set up the server socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.bind(('0.0.0.0', 45000)) #10069
    sock.listen()
    sock.setblocking(False)

    #List of socket objects for incoming client connections
    inputs = [sock]
    clients = {}

    #Keep monitoring client connections to server
    while True:
        #Check which sockets are ready to read
        readable, _, _ = select.select(inputs, [], [], 0.001)
        #Go through all sockets with data
        for s in readable:
            #If socket is server sock
            if s is sock:
                #Add the server-client socket to the socket list
                conn, addr = s.accept()
                conn.setblocking(False)
                inputs.append(conn)
                clients[conn] = {'buffer': b''}
            #If socket is not server socket
            else:
                try:
                    #Check if data recieved is '0x01'
                    data = s.recv(1)
                    if data == b'\x01':
                        #Send a structured response and print it for debugging
                        if(int(shared['l1'])==0):
                            response = struct.pack(
                                pack_format,
                                int(shared['axis'][0]), int(shared['axis'][1]), int(shared['axis'][2]), int(shared['axis'][3]),int(shared['l2']),int(shared['r2']),
                                int(shared['r1']),int(shared['l1']), int(shared['cross']),int(shared['square']) ,int(shared['circle']),
                                int(shared['triangle']), int(shared['pass_turn']), int(shared['pass_bldc'])
                            )
                            print("--- Sent: ", int(shared['axis'][0]), int(shared['axis'][1]), int(shared['axis'][2]), int(shared['axis'][3]),int(shared['l2']),int(shared['r2']),
                                int(shared['r1']),int(shared['l1']), int(shared['cross']),int(shared['square']) ,int(shared['circle']),
                                int(shared['triangle']), int(shared['pass_turn']), int(shared['pass_bldc']))
                        else:
                            response = struct.pack(
                                pack_format,
                                int(shared['axis'][0]), int(shared['axis'][1]), int(shared['axis'][2]), int(shared['axis'][3]),int(shared['l2']),int(shared['r2']),
                                int(shared['r1']),int(shared['l1']), int(shared['cross']),int(shared['square']) ,int(shared['circle']),
                                int(shared['triangle']), int(shared['pwm']), int(shared['pass_bldc'])
                            )
                            print("Sent: ", int(shared['axis'][0]), int(shared['axis'][1]), int(shared['axis'][2]), int(shared['axis'][3]),int(shared['l2']),int(shared['r2']),
                                int(shared['r1']),int(shared['l1']), int(shared['cross']),int(shared['square']) ,int(shared['circle']),
                                int(shared['triangle']), int(shared['pwm']), int(shared['pass_bldc']))
                            # print("pass_dist",shared['dist_pass'])
                        s.sendall(response)
                except:
                    #Handle disconnected or broken clients and remove socket from the list and close the socket
                    inputs.remove(s)
                    s.close()

if __name__ == "__main__":

    #Create a shared dictionary
    manager = Manager()
    shared = manager.dict({
        "axis": [128, 128, 128, 128],
        "l1":0,
        "r1": 0,
        "l2":0,
        "r2": 0,
        "cross": 0,
        "circle": 0,
        "triangle": 0,
        "square": 0,
        "bldc_rpm": 0,
        'pwm' : 0,
        'distance' : 0,
        'dist_pass': 0,
        'pass_bldc':0,
        'pass_turn':0,
        'txt_rpm':0
    })
    
    #Define the processes
    p1 = Process(target = zed2_process , args=(shared,))
    p2 = Process(target = ps4_process, args=(shared,))
    p3 = Process(target = send_data, args=(shared,))
    
    #Start the processes
    p1.start()
    p2.start()
    p3.start()

    p1.join()
    p2.join()
    p3.join()