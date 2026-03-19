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
cd ~/obj_hsv_ws
colcon build 
source install/setup.bash
```

---

## Running

```bash
# terminal 1 — simulation
ros2 launch obj_detect agv_world.launch.py

# terminal 2 — single colour detection
ros2 run obj_detect dist_hsv

# terminal 3 — multiple colour detection node
ros2 run obj_detect dist_multiple_hsv
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

Edit the `color_ranges` dict in `hsv_obj_multiple.py`:

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
