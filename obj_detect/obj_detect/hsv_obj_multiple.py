import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class MultiColorDetector(Node):
    def __init__(self):
        super().__init__('multi_color_detector')
        
        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None
        self.depth_image = None

        # ── define all colors to detect ──
        # add or remove colors here
        # each color = one obstacle type
        self.color_ranges = {
            'red1': {
                'lower': np.array([0, 70, 50]),
                'upper': np.array([10, 255, 255]),
                'bgr':   (0, 0, 255)
            },
            'red2': {
                'lower': np.array([170,70,50]),
                'upper': np.array([180,255,255]),
                'bgr':   (0, 0, 255)
            },
            'blue': {
                'lower': np.array([100, 150, 50]),
                'upper': np.array([140, 255, 255]),
                'bgr':   (255, 0, 0)
            },
            'green': {
                'lower': np.array([35, 50, 50]),
                'upper': np.array([85, 255, 255]),
                'bgr':   (0, 255, 0)
            },
            'yellow': {
                'lower': np.array([25, 50, 50]),
                'upper': np.array([35, 255, 255]),
                'bgr':   (0, 255, 255)
            }
        }

        self.image_sub = self.create_subscription(
            Image, 'camera/image',
            self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, 'camera/depth_image',
            self.depth_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, 'camera/camera_info',
            self.info_callback, 10)

    def info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.destroy_subscription(self.info_sub)

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='32FC1')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # store all detections
        detections = []

        # ── loop through each color ──
        for color_name, params in self.color_ranges.items():

            mask = cv2.inRange(
                hsv,
                params['lower'],
                params['upper'])

            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)

            if not contours:
                continue

            # ── get ALL contours not just largest ──
            for contour in contours:

                # ignore small noise
                if cv2.contourArea(contour) < 500:
                    continue

                M = cv2.moments(contour)
                if M['m00'] == 0:
                    continue

                # centroid
                px = int(M['m10'] / M['m00'])
                py = int(M['m01'] / M['m00'])

                # depth at centroid
                distance = None
                if self.depth_image is not None:
                    d = float(self.depth_image[py, px])
                    if not np.isnan(d) and d > 0:
                        distance = d

                # 3D position
                X = Y = Z = None
                if distance and self.fx:
                    X = (px - self.cx) * distance / self.fx
                    Y = (py - self.cy) * distance / self.fy
                    Z = distance

                # store detection
                detections.append({
                    'color':    color_name,
                    'pixel':    (px, py),
                    'distance': distance,
                    'X': X, 'Y': Y, 'Z': Z,
                    'contour':  contour,
                    'bgr':      params['bgr']
                })

                # ── draw on frame ──
                cv2.drawContours(
                    frame, [contour], -1,
                    params['bgr'], 2)

                cv2.circle(
                    frame, (px, py),
                    8, params['bgr'], -1)

                if distance:
                    label = (f"{color_name} "
                             f"{distance:.2f}m")
                    cv2.putText(
                        frame, label,
                        (px - 40, py - 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, params['bgr'], 2)

                    if X is not None:
                        xyz = f"X:{X:.2f} Y:{Y:.2f}"
                        cv2.putText(
                            frame, xyz,
                            (px - 40, py - 35),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.4, (255, 255, 255), 1)

        # ── log all detections ──
        for d in detections:
            self.get_logger().info(
                f"{d['color']} at "
                f"pixel{d['pixel']} "
                f"distance: {d['distance']:.2f}m"
                if d['distance']
                else f"{d['color']} detected "
                     f"no depth")

        cv2.imshow('Multi Color Detection', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = MultiColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
