# hsv_obstacle_detection

Detects colored obstacles in Gazebo using HSV filtering and estimates their distance using a depth camera. Tested on ROS2 Jazzy + Gazebo Harmonic with a TurtleBot3 Waffle.

---

## What it does

Reads the RGB and depth camera topics, runs HSV color segmentation per frame, finds contours, and uses the depth value at the centroid pixel to get real-world distance. Multiple colors are detected in a single pass — each one labeled separately on the output window.

```
/rgbd_camera/image + /rgbd_camera/depth_image
            ↓
    HSV mask per color
            ↓
    contour → centroid pixel
            ↓
    depth[cy, cx] → distance (m)
            ↓
    X Y Z in camera frame
```

---

## Setup

```bash
# dependencies
sudo apt install ros-jazzy-ros-gz-bridge \
                 ros-jazzy-ros-gz-sim

pip install opencv-python numpy

# build
cd ~/your_ws
colcon build --packages-select hsv_obstacle_detection
source install/setup.bash
```

---

## Running

```bash
# terminal 1 — simulation
ros2 launch your_bringup gazebo.launch.py

# terminal 2 — gz bridge
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:=path/to/gz_bridge.yaml

# terminal 3 — detection node
ros2 run hsv_obstacle_detection multi_color_detector
```

---

## Topics

| Direction | Topic | Type |
|-----------|-------|------|
| sub | `/rgbd_camera/image` | `sensor_msgs/Image` |
| sub | `/rgbd_camera/depth_image` | `sensor_msgs/Image` |
| sub | `/rgbd_camera/camera_info` | `sensor_msgs/CameraInfo` |

---

## Adding colors

Edit the `color_ranges` dict in `multi_color_detector.py`:

```python
self.color_ranges = {
    'red': {
        'lower': np.array([0, 120, 70]),
        'upper': np.array([10, 255, 255]),
        'bgr':   (0, 0, 255)
    },
    'blue': {
        'lower': np.array([100, 150, 50]),
        'upper': np.array([140, 255, 255]),
        'bgr':   (255, 0, 0)
    },
}
```

To find HSV values for a new color run the trackbar script:

```bash
python3 hsv_trackbar_calibration.py
```

Adjust sliders until the mask shows only the target object, note the values.

---

## Distance computation

Depth image encoding is 32FC1 — each pixel is already a float in meters. Camera intrinsics from `/rgbd_camera/camera_info` are used for 3D position:

```
X = (cx - optical_cx) * depth / fx
Y = (cy - optical_cy) * depth / fy
Z = depth
```

Intrinsics are read once on startup and the subscription is destroyed after the first message.

---

## Notes

- Contours below 500px area are ignored as noise
- NaN depth values at object edges are filtered out
- Tested at 640x480 — higher resolutions slow things down
- The detector slot (HSV) can be swapped for YOLO or ArUco — rest of the pipeline stays the same

---

## Platform

- Ubuntu 24.04
- ROS2 Jazzy
- Gazebo Harmonic
- TurtleBot3 Waffle
