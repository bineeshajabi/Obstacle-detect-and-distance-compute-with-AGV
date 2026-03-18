import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDistanceNode(Node):
    def __init__(self):
        super().__init__('color_distance_node')
        
        self.bridge = CvBridge()
        
        # camera info values
        # filled when /camera_info received
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        # depth image stored here
        # used inside image callback
        self.depth_image = None
        
        # HSV range — change these
        # for your colored object
        self.lower = np.array([35, 50, 50])   # green
        self.upper = np.array([85, 255, 255])  # green

        # subscribers
        self.image_sub = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            10)

        self.depth_sub = self.create_subscription(
            Image,
            'camera/depth_image',
            self.depth_callback,
            10)

        self.info_sub = self.create_subscription(
            CameraInfo,
            'camera/camera_info',
            self.info_callback,
            10)

    # ─────────────────────────────
    # CAMERA INFO CALLBACK
    # get focal length + optical center
    # only need once so destroy after
    # ─────────────────────────────
    def info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.get_logger().info(
            f'Camera info received: '
            f'fx={self.fx:.1f} fy={self.fy:.1f}')
        # destroy after first message
        # dont need it again
        self.destroy_subscription(self.info_sub)

    # ─────────────────────────────
    # DEPTH CALLBACK
    # store latest depth frame
    # ─────────────────────────────
    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='32FC1')

    # ─────────────────────────────
    # IMAGE CALLBACK
    # main processing loop
    # ─────────────────────────────
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8')

        # ── STEP 1: HSV masking ──
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)

        # ── STEP 2: find contours ──
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            cv2.imshow('frame', frame)
            cv2.waitKey(1)
            return

        # ── STEP 3: largest contour ──
        largest = max(contours, key=cv2.contourArea)
        
        # ignore small detections
        # avoids noise
        if cv2.contourArea(largest) < 500:
            cv2.imshow('frame', frame)
            cv2.waitKey(1)
            return

        # ── STEP 4: centroid ──
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return
            
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        # ── STEP 5: depth at centroid ──
        distance = None
        if self.depth_image is not None:
            # get depth value at centroid pixel
            depth_value = float(
                self.depth_image[cy, cx])
            
            # check valid reading
            # NaN and 0 mean no reading
            if not np.isnan(depth_value) and depth_value > 0:
                distance = depth_value

        # ── STEP 6: 3D position ──
        # only if camera info received
        if distance and self.fx:
            X = (cx - self.cx) * distance / self.fx
            Y = (cy - self.cy) * distance / self.fy
            Z = distance

        # ── STEP 7: draw on frame ──
        # draw contour
        cv2.drawContours(
            frame, [largest], -1, (0, 255, 0), 2)
        
        # draw centroid
        cv2.circle(
            frame, (cx, cy), 8, (0, 0, 255), -1)
        
        # draw distance text
        if distance:
            cv2.putText(
                frame,
                f'Distance: {distance:.2f}m',
                (cx - 50, cy - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (255, 255, 255), 2)
            
            if self.fx:
                cv2.putText(
                    frame,
                    f'X:{X:.2f} Y:{Y:.2f} Z:{Z:.2f}',
                    (cx - 50, cy - 45),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 0), 2)
            
            self.get_logger().info(
                f'Centroid: ({cx},{cy}) '
                f'Distance: {distance:.2f}m')

        cv2.imshow('frame', frame)
        cv2.imshow('mask', mask)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()