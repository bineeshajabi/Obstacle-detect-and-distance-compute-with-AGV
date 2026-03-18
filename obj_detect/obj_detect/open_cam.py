import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class Cam_node(Node):
    def __init__(self):
        #Used to initialise parameters
        super().__init__('camera_node')
        self.pub=self.create_publisher(Image,'cam_fd',10)

        #Used to create OpenCV video capture object and set the camera device number
        self.camDevicenum=0
        self.cap = cv2.VideoCapture(self.camDevicenum)

        #Adding timer
        self.period_communication=0.1
        self.timer=self.create_timer(self.period_communication,self.timer_callback)

        #Used to create CvBridge which helps convert ROS images to OpenCV images
        self.bridge=CvBridge()

#Used to define timer callback function
    def timer_callback(self):   

        ret,frame=self.cap.read()
        if ret==True:
            #Using CvBridge to convert OpenCV image to ROS image and publish and so 'cv2_to_imgmsg' 
            self.pub.publish(self.bridge.cv2_to_imgmsg(frame,'bgr8'))
            self.get_logger().info('Publishing video frame')
        else:
            self.get_logger().info('No frame captured from camera')  

def main(args=None):
    rclpy.init(args=args)
    obj_subscriber=Cam_node()
    rclpy.spin(obj_subscriber)
    obj_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

