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

pack_format = '<4i6h'
bldc_rpm = 0
mapped_pwm = 0
joystick = None

def zed2_process(shared):
    class Normal_WebCam:
        def __init__(self):
            print("Initializing Camera...")
            self.cap = cv2.VideoCapture(1)
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
                return frame

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
            
    # Constants
    CONFIDENCE_THRESHOLD = 0.25
    FRAME_SKIP = 2
    BASKET_WIDTH_CM = 47.2
    CONST = 1100 #alibrated constant

    # Load YOLO model
    model = YOLO(r'best.pt')

    # Initialize camera
    # z = Normal_WebCam()
    z = ZED2_feed()
    while True:
        try:

            frame_count = 0

            def map_range(value, in_min, in_max, out_min, out_max):
                return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

            while True:
                # global mapped_pwm
                frame = z.get_feed()
                if frame is None or not frame.any():
                    z = ZED2_feed()
                    shared['pwm']=0
                    shared['distance']=0
                    
                    print("Can't receive frame. Trying again ...")
                    break

                # print(type(frame))
                
                frame_count += 1
                if frame_count % FRAME_SKIP != 0:
                    continue

                # temp = frame[0:int(frame.shape[0]), 0:int(frame.shape[1] / 2)]
                # frame = temp

                frame_height, frame_width = frame.shape[:2]
                # print(f"{frame_height} , {frame_width}")

                threshold = 0.01#percent
                right_threshold = int(frame_width / 2.0 + frame_width * threshold)
                left_threshold = int(frame_width / 2.0 - frame_width * threshold) 

                results = model.predict(source=frame, conf=CONFIDENCE_THRESHOLD, save=False, verbose=False)
                det = False
                for res in results:
                    for box in res.boxes:
                        # global mapped_pwm
                        x_center, y_center, width, height = box.xywh[0].cpu().numpy()
                        x_center, y_center, width, height = float(x_center), float(y_center), float(width), float(height)
                        class_id = int(box.cls[0].cpu().numpy())
                        class_name = results[0].names.get(class_id, "Unknown") 
                        if class_name != "basket": 
                            continue
                        det = True
                        unit_length = BASKET_WIDTH_CM / width
                        distance = unit_length * CONST
                        print(f"Distance to basket: {distance:.2f} cm")

                        xmin = x_center - (width / 2)
                        ymin = y_center - (height / 2)
                        xmax = x_center + (width / 2)
                        ymax = y_center + (height / 2)

                        confidence = round(box.conf[0].item(), 3)
                        frame = cv2.rectangle(frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (255, 0, 0), 2)

                        # 1 means detected and in center
                        error_offset = 0

                        mapped_pwm = 0
                        # detected and with the offset
                        if right_threshold < x_center or x_center < left_threshold:
                            error_offset = x_center - (frame_width // 2) 
                            
                            mapped_pwm = map_range(abs(error_offset), 0, (frame_width//2),10, 200)
                            mapped_pwm = -mapped_pwm if error_offset<0 else mapped_pwm
                            # q.put(mapped_pwm)
                            # q2.put(distance)
                            shared['pwm'] = mapped_pwm
                            shared['distance'] =distance
                        else:

                            error_offset =0 
                            
                            mapped_pwm = -0
                            shared['pwm'] = mapped_pwm
                            shared['distance'] = distance
                                

                        cv2.putText(frame,str(frame_width), (320, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
                        cv2.putText(frame, f"Dist: {distance:.2f}", (500, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
                        cv2.putText(frame, f"Offset: {round(error_offset,2)}", (1000, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
                if not det:
                    
                    mapped_pwm = 0
                    shared['pwm'] = mapped_pwm
                    shared['distance'] = 0
                    
                frame= cv2.line(frame, (frame_width // 2,0), (frame_width // 2,frame_height), (255,255,255), 1)
                frame= cv2.line(frame, (right_threshold,0), (right_threshold,frame_height), (255,0,255), 1)
                frame= cv2.line(frame, (left_threshold,0), (left_threshold,frame_height), (255,255,0), 1)
                cv2.imshow("distance", frame)

                if cv2.waitKey(1) == ord('q'):
                    break

            cv2.destroyAllWindows()
        except Exception as e:
            print(f"Ye error hua camera ke saath: {e}")
            continue

def ps4_process(shared):
    global bldc_rpm
    def get_rpm(distance):
        distances = np.array([390,317,285,245,220,415,460])  # meters
        rpm = np.array([3535,3265,3050,2830,2755,3750,4025])
        coefficients = np.polyfit(distances,rpm ,2)
        return coefficients[0] * distance**2 + coefficients[1] * distance + coefficients[2]
    
        # time.sleep(1)

    global joystick
    

    sock = socket.socket(socket.AF_INET , socket.SOCK_STREAM)

    prev_lx = 128
    prev_ly = 128
    prev_rx = 128
    prev_ry = 128
    prev_r1 = 0

    pygame.init()
    pygame.joystick.init()

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

    def map_stick(val):
        return round((val + 1) * 127.5)  # -1 to 1 to 0 to 255

    init_joystick()  
        
    while True:
        pygame.event.pump()

        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEADDED:
                print("Control Andar")
                init_joystick()
            elif event.type == pygame.JOYDEVICEREMOVED:
                print("Control Bahar")
                joystick = None
        print(joystick)
        if joystick:
            print("Check")
            try:
                
                lx = round(joystick.get_axis(0), 2)
                ly = -round(joystick.get_axis(1), 2)
                rx = round(joystick.get_axis(2), 2)
                ry = -round(joystick.get_axis(5), 2)

                lx_mapped = map_stick(lx)
                ly_mapped = map_stick(ly)
                rx_mapped = map_stick(rx)
                ry_mapped = map_stick(ry)

                # l2 = int(joystick.get_axis(3) + 1)
                # r2 = int(joystick.get_axis(4) + 1)

                cross = joystick.get_button(1)
                circle = joystick.get_button(2)
                # square = joystick.get_button(0)
                triangle = joystick.get_button(3)

                l1 = joystick.get_button(4)
                r1 = joystick.get_button(5)

                # dx , dy = joystick.get_hat(0)
                # count = count + 1
                
                # distance to bldc rpm
                bldc_rpm = get_rpm(shared['distance'])
                if shared['distance']==0 or bldc_rpm>4500:
                    bldc_rpm = 0
                

                shared["axis"] = [lx_mapped, ly_mapped, rx_mapped, ry_mapped]
                shared["r1"] = r1
                shared["cross"] = cross
                shared["circle"] = circle
                shared["triangle"] = triangle
                shared["bldc_rpm"] = bldc_rpm
                
            except pygame.error:
                print("Gadbad Kuch toh")
                joystick = None
        else:
            time.sleep(0.1)

def send_data(shared):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.bind(('0.0.0.0', 10069))
    sock.listen()
    sock.setblocking(False)

    inputs = [sock]
    clients = {}

    while True:
        readable, _, _ = select.select(inputs, [], [], 0.001)
        for s in readable:
            if s is sock:
                conn, addr = s.accept()
                conn.setblocking(False)
                inputs.append(conn)
                clients[conn] = {'buffer': b''}
            else:
                try:
                    data = s.recv(1)
                    if data == b'\x01':
                        
                        response = struct.pack(
                            pack_format,
                            int(shared['axis'][0]), int(shared['axis'][1]), int(shared['axis'][2]), int(shared['axis'][3]),
                            int(shared['r1']), int(shared['cross']), int(shared['circle']),
                            int(shared['triangle']), int(shared['pwm']), int(shared['bldc_rpm'])
                        )
                        s.sendall(response)
                except:
                    inputs.remove(s)
                    s.close()

if __name__ == "__main__":
    manager = Manager()
    shared = manager.dict({
        "axis": [128, 128, 128, 128],
        "r1": 0,
        "cross": 0,
        "circle": 0,
        "triangle": 0,
        "bldc_rpm": 0,
        'pwm' : 0,
        'distance' : 0,
    })
    
    
    p1 = Process(target = zed2_process , args=(shared,))
    p2 = Process(target = ps4_process, args=(shared,))
    p3 = Process(target = send_data, args=(shared,))
    

    p1.start()
    p2.start()
    p3.start()

    p1.join()
    p2.join()
    p3.join()