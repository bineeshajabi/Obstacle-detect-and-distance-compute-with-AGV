from matplotlib import image
from matplotlib.pyplot import hsv
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def empty(a):
    pass

class Hsv_filter_node(Node):
    def __init__(self):
        #Used to initialise parameters
        super().__init__('hsv_filter_node')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', 10)
        self.back_sub = cv2.createBackgroundSubtractorMOG2(history=700, varThreshold=25, detectShadows=True)
    
        
        #Red
        #Range 1
        self.red1_ll = np.array([0,70,50])
        self.red1_hl = np.array([10,255,255])
        #Range 2
        self.red2_ll = np.array([170,70,50])
        self.red2_hl = np.array([180,255,255])

        #Blue
        self.blue_ll = np.array([92,57,50])
        self.blue_ul = np.array([142,153,178])

        #Green
        self.green_ll = np.array([35,50,50])
        self.green_ul = np.array([85,255,255])

        #White
        self.white_ll = np.array([0,0,180])
        self.white_ul = np.array([179,30,255])

        self.color_data = [
        {"name": "Blue", "lower": self.blue_ll, "upper": self.blue_ul, "color_bgr": (255, 0, 0)},
        {"name": "Green", "lower": self.green_ll, "upper": self.green_ul, "color_bgr": (0, 255, 0)},
        # Red is handled separately due to dual ranges
        {"name": "White", "lower": self.white_ll, "upper": self.white_ul, "color_bgr": (255, 255, 255)},
        ]

        self.get_logger().info("Color Detection Node Started")
        
        # Timer for continuous processing
        self.timer = self.create_timer(0.1, self.process_frame)  

        self.bridge = CvBridge()

    def process_frame(self):

        ret,frame=self.cap.read()

        if ret==True:           
            self.current_frame = self.bridge.imgmsg_to_cv2(frame, desired_encoding='bgr8')   

            hsv = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2HSV)
            combined_mask = np.zeros_like(hsv[:, :, 0])

            for color in self.color_data:
                # Generate mask using cv2.inRange and numpy arrays
                mask = cv2.inRange(hsv, color["lower"], color["upper"])
                combined_mask = cv2.bitwise_or(combined_mask, mask)
                # Contour Detection and Tracing for each color
                cnts, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                for c in cnts:
                    area=cv2.contourArea(c)
                    if area > 2000: # Threshold for area size
                        # Draw contour in the color's BGR
                        cv2.drawContours(frame, [c], -1, color["color_bgr"], 3)
                        M=cv2.moments(c)
                        if M["m00"] != 0:
                            cx=int(M["m10"]/M["m00"])
                            cy=int(M["m01"]/M["m00"])
                            cv2.circle(frame, (cx, cy), 7, (0, 0, 0), -1) # Center circle (black)
                            # Text label in the color's BGR
                            cv2.putText(frame, color["name"], (cx-20, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 1, color["color_bgr"], 2)
            # Process Red (dual range)
            mask_red1 = cv2.inRange(hsv, self.red1_ll, self.red1_hl)
            mask_red2 = cv2.inRange(hsv, self.red2_ll, self.red2_hl)
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)
            combined_mask = cv2.bitwise_or(combined_mask, mask_red)
            # Contour Detection and Tracing for Red
            cnts_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            red_color_bgr = (0, 0, 255) # Red in BGR
            for c in cnts_red:
                area = cv2.contourArea(c)
                if area > 2000:
                    cv2.drawContours(frame, [c], -1, red_color_bgr, 3)
                    M=cv2.moments(c)
                    if M["m00"] != 0:
                        cx=int(M["m10"]/M["m00"])
                        cy=int(M["m01"]/M["m00"])
                        cv2.circle(frame, (cx, cy), 7, (0, 0, 0), -1)
                        cv2.putText(frame, "Red", (cx-20, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 1, red_color_bgr, 2)
            # Display Results using cv2
            res = cv2.bitwise_and(frame, frame, mask=combined_mask)     

            cv2.imshow("live transmission", frame)
            cv2.imshow("combined mask", combined_mask)
            cv2.imshow("res (color isolation)", res)


def main(args=None):
    rclpy.init()
    hsv_filter_node = Hsv_filter_node()  
    try:
        rclpy.spin(hsv_filter_node)
    except KeyboardInterrupt:
        pass       
    hsv_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
