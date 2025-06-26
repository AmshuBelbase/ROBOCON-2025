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
import math

cam_index = 0

#Define packing format (6 integers , 9 short integers - 4 bytes , 2 bytes)
pack_format = '<6i9h'

#Globals to hold motor speeds
bldc_rpm = 0
mapped_pwm = 0 
mapped_pwm_april=0
#Global joystick Object
joystick = None
 

#Process for Camera and YOLO Detection
def zed2_process(shared):

    #Class for Normal Webcam
    class Normal_WebCam:
        def __init__(self):
            print("Initializing Camera...")
            self.cap = cv2.VideoCapture(cam_index)
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
            init_params.camera_fps = 15  # Set fps at 30 
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
             
    
    class Passing:
        def __init__(self):
            # === Tag parameters ===
            self.tag_size = 0.305  # Real-world tag size in centimeters (e.g., 15 cm)  
            #constants for passing (apriltag)
            self.calib_data = np.load(r'/home/robocon/Desktop/Dominic/MultiMatrixLenevo.npz')  # Replace with your filename
            self.camera_matrix = self.calib_data['camMatrix']
            self.dist_coeffs = self.calib_data['distCoef']
            # === Extract intrinsics for AprilTag detector ===
            self.fx = self.camera_matrix[0, 0]
            self.fy = self.camera_matrix[1, 1]
            self.cx = self.camera_matrix[0, 2]
            self.cy = self.camera_matrix[1, 2]
            self.camera_params = [self.fx, self.fy, self.cx, self.cy]
            # === Initialize AprilTag detector ===
            self.detector = Detector(families='tag36h11',
                                nthreads=1,
                                quad_decimate=1.0,
                                quad_sigma=0.0,
                                refine_edges=1,
                                decode_sharpening=0.25,
                                debug=0)


        def get_tags(self, frame):
            self.frame = frame
            self.frame_undistorted = self.frame #cv2.undistort(frame, camera_matrix, dist_coeffs)
            self.gray = cv2.cvtColor(self.frame_undistorted, cv2.COLOR_BGR2GRAY)

            # === Detect AprilTags ===
            self.tags = self.detector.detect(self.gray, estimate_tag_pose=True,
                                camera_params=self.camera_params, tag_size=self.tag_size) 

        def draw_text_with_background(self, text, org, font, font_scale, text_color, thickness, bg_color, f = None):
            # Get the text size
            (w, h), baseline = cv2.getTextSize(text, font, font_scale, thickness)

            # Calculate the rectangle coordinates
            x, y = org
            rect_start = (x, y - h - baseline)
            rect_end = (x + w, y + baseline)

            if f is None:
                f = self.frame

            # Draw rectangle (background)
            cv2.rectangle(f, rect_start, rect_end, bg_color, cv2.FILLED)

            # Draw the text
            cv2.putText(f, text, org, font, font_scale, text_color, thickness)

        def process_tags(self, err_off):
            self.offset = 0
            self.pose_t = None
            for tag in self.tags:
                tag_id = tag.tag_id
                
                self.pose_t = tag.pose_t  # [x, y, z] translation in meters
                self.pose_r = tag.pose_R  # 3x3 rotation matrix
                
                # Draw tag center and info
                center = tuple(np.int32(tag.center))

                cv2.circle(self.frame, center, 6, (0, 255, 0), -1)

                frame_height, frame_width = self.frame.shape[:2]

                cv2.circle(self.frame, (frame_width//2, frame_height), 6, (255, 255, 0), -1)
                cv2.line(self.frame, (frame_width//2, frame_height), center, (0, 255, 255), 2)


                               
                # Size of the AprilTag (front face of cube) in meters
                self.tag_size = 0.305  # Adjust according to actual tag size
                cube_depth = -0.5  # Depth of the cube in meters

                # Define 8 corners of the cube in tag's local coordinate system
                cube_points_3d = np.array([
                    [0, 0, 0],
                    [self.tag_size, 0, 0],
                    [self.tag_size, self.tag_size, 0],
                    [0, self.tag_size, 0],
                    [0, 0, -cube_depth],
                    [self.tag_size, 0, -cube_depth],
                    [self.tag_size, self.tag_size, -cube_depth],
                    [0, self.tag_size, -cube_depth]
                ], dtype=np.float32)

                # Offset cube so it's centered on the tag
                cube_points_3d -= np.array([self.tag_size/2, self.tag_size/2, 0])

                # Convert rotation matrix to rotation vector
                rvec, _ = cv2.Rodrigues(self.pose_r)
                tvec = self.pose_t.reshape(-1, 1)

                # Project the 3D points to 2D image plane
                cube_points_2d, _ = cv2.projectPoints(cube_points_3d, rvec, tvec, self.camera_matrix, self.dist_coeffs)
                cube_points_2d = cube_points_2d.reshape(-1, 2).astype(int)

                # Draw cube edges
                # Base square (tag face)
                for i in range(4):
                    pt1 = tuple(cube_points_2d[i])
                    pt2 = tuple(cube_points_2d[(i + 1) % 4])
                    cv2.line(self.frame, pt1, pt2, (0, 255, 255), 2)

                # Top square (opposite face)
                for i in range(4, 8):
                    pt1 = tuple(cube_points_2d[i])
                    pt2 = tuple(cube_points_2d[4 + (i + 1) % 4])
                    cv2.line(self.frame, pt1, pt2, (0, 255, 0), 2)

                # Vertical edges
                for i in range(4):
                    pt1 = tuple(cube_points_2d[i])
                    pt2 = tuple(cube_points_2d[i + 4])
                    cv2.line(self.frame, pt1, pt2, (255, 0, 0), 2)

                back_face_center_3d = np.mean(cube_points_3d[4:8], axis=0)
                back_face_center_2d, _ = cv2.projectPoints(
                    np.array([back_face_center_3d]), rvec, tvec, self.camera_matrix, self.dist_coeffs
                )
                back_face_center_2d = tuple(back_face_center_2d[0][0].astype(int)) 
                cv2.line(frame, (frame_width//2, frame_height), back_face_center_2d, (0, 0, 255), 2)
                
                # Draw tag border
                for i in range(4):
                    pt1 = tuple(np.int32(tag.corners[i]))
                    pt2 = tuple(np.int32(tag.corners[(i + 1) % 4])) 
                    cv2.line(self.frame, pt1, pt2, (0, 255, 255), 2)

                delta_y = frame_height - center[1]
                delta_x = center[0] - frame_width//2
                angle_radians = math.atan2(delta_y, delta_x)
                angle_degrees_y = math.degrees(angle_radians) 

                delta_y = frame_height - back_face_center_2d[1]
                delta_x = back_face_center_2d[0] - frame_width//2
                angle_radians = math.atan2(delta_y, delta_x)
                angle_degrees_r = math.degrees(angle_radians)

                self.offset = delta_x
 
                text = f"Angle Y: {90 - angle_degrees_y:.2f} Angle R: {90 - angle_degrees_r:.2f}"
                position = (center[0] - 30, center[1])
                self.draw_text_with_background(
                    text=text, org=position,
                    font=cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale=0.5,
                    text_color=(255, 255, 255),   # White text
                    thickness=2,
                    bg_color=(0, 0, 0)            # Black background
                )
                    
        def get_distance_then_rpm(self):
            if self.pose_t is None:
                return 0
            distance_pass = np.linalg.norm(self.pose_t) * 100 
            distances = np.array([155,187,214,237,270,300])  # centimeters
            rpm = np.array([1960,2150,2300,2450,2600,2750]) #currently rndom array, need to find proper values later
            coefficients = np.polyfit(distances,rpm ,2)
            pass_rpm = (coefficients[0] * distance_pass**2 + coefficients[1] * distance_pass + coefficients[2])*0.90

            text = f"Distance {distance_pass} RPM {pass_rpm}"
            position = (0 + 40, 0 + 20)
            self.draw_text_with_background(
                text=text, org=position,
                font=cv2.FONT_HERSHEY_SIMPLEX,
                font_scale=0.5,
                text_color=(255, 255, 255),   # White text
                thickness=2,
                bg_color=(0, 0, 0)            # Black background
            )  

            return pass_rpm

        def get_alignment_pwm(self):
            text = f"OFFSET {self.offset}"
            position = (0 + 40, 0 + 40)
            self.draw_text_with_background(
                text=text, org=position,
                font=cv2.FONT_HERSHEY_SIMPLEX,
                font_scale=0.5,
                text_color=(255, 255, 255),   # White text
                thickness=2,
                bg_color=(0, 0, 0)            # Black background
            )  
            return self.offset


    
    # Initialize camera

    z = Normal_WebCam()
    # z = ZED2_feed()  

    FRAME_SKIP = 5

    pass_obj = Passing()
    count_missed = 0

    def map_range(value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    #Try to access the feed continously if not available
    while True:
        try:
            # Current frame count since last active time of Camera
            frame_count = 0

            # Get frames continously
            while True:
                frame = z.get_feed()                  
                frame_height, frame_width = frame.shape[:2] 
                # Skip frames
                frame_count += 1
                if frame_count % FRAME_SKIP != 0:
                    # print("Skip: ", frame_count)
                    continue

                offset = 0
                err_off = 0
                center = frame_width // 2
                cv2.rectangle(frame, (center - err_off,0), (center + err_off,frame_height), color=(255, 0, 0), thickness=-1)


                # get offset of target and required bldc rpm based on Target MODE

                if shared["turn_mode"] == "Passing":
                    FRAME_SKIP = 1
                    err_off = 30
                    pass_obj.get_tags(frame = frame)
                    pass_obj.process_tags(err_off = err_off)
                    offset = pass_obj.get_alignment_pwm()
                    shared["bldc_rpm"] = pass_obj.get_distance_then_rpm()

                elif shared["turn_mode"] == "Shooting":
                    FRAME_SKIP = 5
                    offset = 0
                    shared["bldc_rpm"]
                else:
                    offset = 0
                    shared["bldc_rpm"]
                
                pwm_min , pwm_max = 0, 0

                # Get Rotational RPM based on the offset and required Mapping
                if abs(offset) < err_off:
                    shared["pwm"] = 0
                    # count_missed = 0
                # elif offset == 0:
                #     count_missed += 1
                #     if count_missed >= 1:
                #         count_missed = 0
                #         shared["pwm"] = 0
                elif abs(offset) > err_off:
                    count_missed = 0
                    def offset_to_rpm(offset, rpm_min = 0, rpm_max = 20, offset_min = 0, offset_max = 300):
                        ease = (offset - offset_min) / (offset_max - offset_min)
                        x = math.acos(1 - 2 * ease) / math.pi
                        rpm = rpm_min + x * (rpm_max - rpm_min)
                        return rpm
                    0
                    thresholds = [
                        (int(center*0.8), 100),
                        (int(center*0.6), 80),
                        (int(center*0.4), 60),
                        (int(center*0.2), 45)
                    ]
                    thres_mins = {100: 0, 80: 0, 60: 0, 45: 0}

                    # Loop to find the appropriate pwm_max
                    for threshold, value in thresholds:
                        if abs(offset) > threshold:
                            pwm_max = value
                            pwm_min = thres_mins[value]
                            break
                        else:
                            pwm_max = thresholds[-1][1]  # Default to the smallest value
                            pwm_min = thres_mins[pwm_max]



                    shared["pwm"] = offset_to_rpm(abs(offset), pwm_min, pwm_max, 0, frame_width//2)

                    # shared["pwm"] = map_range(abs(offset), 0, frame_width//2, 0, 15) 
                    shared["pwm"] = -shared["pwm"] if offset < 0 else shared["pwm"]


                
                text = f"{shared['turn_mode']} PWM {shared['pwm']} {pwm_min} {pwm_max}"
                position = (0 + 40, 0 + 60)
                pass_obj.draw_text_with_background(
                    text=text, org=position,
                    font=cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale=0.5,
                    text_color=(255, 255, 255),   # White text
                    thickness=2,
                    bg_color=(0, 0, 0),            # Black background
                    f = frame
                )
                text = f"{center - err_off} {offset} {center + err_off}"
                position = (0 + 40, 0 + 80)
                pass_obj.draw_text_with_background(
                    text=text, org=position,
                    font=cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale=0.5,
                    text_color=(255, 255, 255),   # White text
                    thickness=2,
                    bg_color=(0, 0, 0),            # Black background
                    f = frame
                )  
                
                for i in range(2,9,2):
                    frame= cv2.line(frame, (int(center - (center*(i/10))), 0), (int(center - (center*(i/10))),frame_height), (255,255,255), 1)
                    frame= cv2.line(frame, (int(center + (center*(i/10))), 0), (int(center + (center*(i/10))),frame_height), (255,255,255), 1)
                

                frame= cv2.line(frame, (center, 0), (center,frame_height), (255,255,255), 1)
                frame= cv2.line(frame, (center - err_off,0), (center - err_off,frame_height), (255,0,255), 1)
                frame= cv2.line(frame, (center + err_off,0), (center + err_off,frame_height), (255,255,0), 1)
                

                cv2.imshow("distance", frame)
                
                if cv2.waitKey(1) == ord('q'):
                    break 
 
            cv2.destroyAllWindows()
 
        except Exception as e: 
            #If camera feed is not captured
            #Send error message and try again
            shared['pwm'] = 0
            shared['bldc_rpm'] = 0 
            print("Can't receive frame. Trying again ...") 
            print(f"Ye error hua camera ke saath: {e}")
            continue

def ps4_process(shared):
    #Global rpm value for bldc
    global joystick
 

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

    last_touch = shared['touch_button']
        
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

                touch_button = joystick.get_button(13)

                dpad_tup = joystick.get_hat(0)
                dpad_up = 1 if dpad_tup[1]>0 else 0
                dpad_down = 1 if dpad_tup[1]<0 else 0
                dpad_right = 1 if dpad_tup[0]>0 else 0
                dpad_left = 1 if dpad_tup[0]<0 else 0
 
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

                if last_touch == 0 and touch_button == 1:
                    shared["touch_button"] = not shared["touch_button"]

                last_touch = touch_button

                if dpad_up == 1:
                    turn_mode = "Shooting"
                elif dpad_left == 1:
                    turn_mode = "Passing"
                elif dpad_down == 1:
                    turn_mode = "None"
                else:
                    turn_mode = shared["turn_mode"]

                shared["turn_mode"] = turn_mode 
 
                print(int(shared['axis'][0]), int(shared['axis'][1]), int(shared['axis'][2]), int(shared['axis'][3]), int(shared['l2']),int(shared['r2']),
                        int(shared['r1']),int(shared['l1']), int(shared['cross']), int(shared['square']), int(shared['circle']),
                            int(shared['triangle']), dpad_up, dpad_left, dpad_down, shared["touch_button"], shared["turn_mode"])

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
        # print("0")
        readable, _, _ = select.select(inputs, [], [], 0.001) 
        #Go through all sockets with data
        for s in readable: 
            print("1")
            #If socket is server sock
            if s is sock:
                print("2")
                #Add the server-client socket to the socket list
                conn, addr = s.accept()
                conn.setblocking(False)
                inputs.append(conn)
                clients[conn] = {'buffer': b''}
            #If socket is not server socket
            else:
                try:
                    print("3") 

                    #Check if data recieved is '0x01'
                    data = s.recv(1)
                    if data == b'\x01':
                        #Send a structured response and print it for debugging
                        response = struct.pack(
                            pack_format,
                            int(shared['axis'][0]), int(shared['axis'][1]), int(shared['axis'][2]), int(shared['axis'][3]),int(shared['l2']),int(shared['r2']),
                            int(shared['r1']), int(shared['l1']), int(shared['cross']),int(shared['square']) ,int(shared['circle']),
                            int(shared['triangle']), int(shared['touch_button']), int(shared['pwm']), int(shared['bldc_rpm'])
                        )
                        print("Sent: ", int(shared['axis'][0]), int(shared['axis'][1]), int(shared['axis'][2]), int(shared['axis'][3]),int(shared['l2']),int(shared['r2']),
                            int(shared['r1']),int(shared['l1']), int(shared['cross']),int(shared['square']) ,int(shared['circle']),
                            int(shared['triangle']), int(shared['touch_button']), int(shared['pwm']), int(shared['bldc_rpm']))
                   
                        s.sendall(response)
                except Exception as e:
                    #Handle disconnected or broken clients and remove socket from the list and close the socket
                    inputs.remove(s)
                    print("mkc: ", e)
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
        "touch_button": 0,
        "bldc_rpm": 0,
        "pwm" : 0, 
        "turn_mode":"None",
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