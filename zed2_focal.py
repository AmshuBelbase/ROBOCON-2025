import cv2
import pyzed.sl as sl 
from ultralytics import YOLO

model = YOLO(r'C:\Users\Happy Home-\OneDrive\Desktop\ROBOCON\best.pt') 

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
            return frame

class ZED2_feed:
    def __init__(self): 
        print("Initializing Camera...")
        self.zed = sl.Camera()
        self.input_type = sl.InputType() 

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters() 
        init_params.camera_resolution = sl.RESOLUTION.HD2K # Use HD720 opr HD1200 video mode, depending on camera type.
        init_params.camera_fps = 15  # Set fps at 30 
        # Set configuration parameters 
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL # Use ULTRA depth mode
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


############### USAGE ############## 

z = ZED2_feed()
# z = Normal_WebCam()

while cv2.waitKey(5) != 113:
    frame = z.get_feed()
    results = model.predict(source=frame, conf=0.25, save=True)
    
    for res in results:
        for box in res.boxes:
            # Extract coordinates and other values
            x_center, y_center, width, height = box.xywh[0].cpu().numpy()
            x_center, y_center, width, height = float(x_center), float(y_center), float(width), float(height)
            
            class_id = int(box.cls[0].cpu().numpy())
            class_name = results[0].names.get(class_id, "Unknown")
            
            if class_name == "basket": 
                distance=350 #known distance
                # Calculate distance using a constant diameter (47.2 cm) of the hoop
                unit_length = 50 / width  # cm per pixel (approx.)
                const = distance/unit_length  # your calibrated constant
                print(f"Distance to basket: {const:.2f} centimeters")
                
                # Calculate bounding box coordinates
                xmin = x_center - (width / 2)
                ymin = y_center - (height / 2)
                xmax = x_center + (width / 2)
                ymax = y_center + (height / 2)
                
                confidence = round(box.conf[0].item(), 3)
                frame = cv2.rectangle(frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (255, 0, 0), 2)
                
                # Determine safety based on horizontal offset
                xoffset = x_center - 320  # assumes a 640-pixel width
                # s = "safe" if -3 < xoffset < 3 else "unsafe"
                # cv2.putText(frame, s, (320, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
                cv2.putText(frame, str(const), (500, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
                # cv2.putText(frame, str(confidence), (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
                cv2.putText(frame, str(xoffset), (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)

    
    cv2.imshow("distance", frame)
    if cv2.waitKey(1) == ord('q'):
        break