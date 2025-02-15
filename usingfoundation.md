# Need to make Changes

installing python packages and packages :

```bash
sudo apt update
sudo apt install python3-pip python3-venv
pip install torch torchvision opencv-python numpy pyrealsense2 rclpy sensor_msgs std_msgs cv_bridge
```

hoping you have run all the necessery code in setupandrun in realsense of my repository

Step 1 : Colne and Set up Foundation Stereo 

```bash
cd ~/ros2_ws/src
git clone https://github.com/NVlabs/FoundationStereo.git
cd FoundationStereo
```

Step 5 : Install Model Requirements 

```bash
pip install -r requirements.txt
```

Step 6: Download Pre-trained model weights 
Download FoundationStereo’s pre-trained weights:
```bash
wget https://nvlabs.github.io/FoundationStereo/foundationstereo_checkpoint.pth
```

## Set Up ROS2 Package for Hole Detection

Step 7 : Create a New ROS2 Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python detect_holes_ros2
cd detect_holes_ros2/detect_holes_ros2
touch detect_holes.py
chmod +x detect_holes.py
```

4. Write the ROS2 Node for Hole Detection

Edit detect_holes.py and add the following code:

```python
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
import torch
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge
from model import FoundationStereo  # Import FoundationStereo model

class HoleDetectorNode(Node):
    def __init__(self):
        super().__init__("hole_detector")
        self.bridge = CvBridge()

        # ROS2 Publishers
        self.hole_detected_pub = self.create_publisher(Bool, "hole_detected", 10)
        self.depth_pub = self.create_publisher(Float32, "hole_depth", 10)
        self.image_pub = self.create_publisher(Image, "processed_image", 10)

        # Initialize RealSense camera
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)

        # Load FoundationStereo model
        self.model = FoundationStereo()
        self.model.load_state_dict(torch.load("foundationstereo_checkpoint.pth"))
        self.model.eval()

        # Camera intrinsic parameters
        self.focal_length = 615.0  # Adjust based on your RealSense camera
        self.baseline = 0.05  # Distance between stereo cameras in meters

        self.timer = self.create_timer(0.1, self.process_frame)  # Run at 10Hz

    def process_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        # Convert RealSense frames to OpenCV format
        left_image = np.asanyarray(color_frame.get_data())
        right_image = np.asanyarray(depth_frame.get_data())

        # Preprocess images for FoundationStereo
        left_tensor = self.preprocess_image(left_image)
        right_tensor = self.preprocess_image(right_image)

        # Run FoundationStereo
        with torch.no_grad():
            disparity_map = self.model(left_tensor, right_tensor)

        disparity_map = disparity_map.squeeze(0).cpu().numpy()

        # Convert disparity to depth
        disparity_map[disparity_map == 0] = 0.01  # Avoid division by zero
        depth_map = (self.focal_length * self.baseline) / disparity_map
        depth_map = (depth_map / np.max(depth_map) * 255).astype(np.uint8)

        # Detect holes
        hole_detected, depth, processed_image = self.detect_holes(depth_map, left_image)

        # Publish results
        self.hole_detected_pub.publish(Bool(data=hole_detected))
        if hole_detected:
            self.depth_pub.publish(Float32(data=depth))
            self.get_logger().info(f"Hole Detected - Depth: {depth:.2f}m")
        
        # Publish processed image
        processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding="bgr8")
        self.image_pub.publish(processed_msg)

    def preprocess_image(self, img):
        img = cv2.resize(img, (640, 480))  # Resize to match input size
        img = img.astype(np.float32) / 255.0  # Normalize pixel values
        img = np.transpose(img, (2, 0, 1))  # Convert HWC → CHW format
        return torch.tensor(img).unsqueeze(0)  # Add batch dimension

    def detect_holes(self, depth_map, frame):
        gray = cv2.cvtColor(depth_map, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        hole_detected = False
        hole_depth = 0.0

        for contour in contours:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)

            if radius > 5:  # Ignore small false positives
                hole_detected = True
                depth_value = depth_map[center[1], center[0]] / 255.0 * self.baseline * self.focal_length
                hole_depth = depth_value

                # Draw the detected hole
                cv2.circle(frame, center, radius, (0, 255, 0), 2)
                cv2.circle(frame, center, 3, (0, 0, 255), -1)
                cv2.putText(frame, f"Depth: {depth_value:.2f}m", (center[0]+10, center[1]-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        return hole_detected, hole_depth, frame

def main(args=None):
    rclpy.init(args=args)
    node = HoleDetectorNode()
    rclpy.spin(node)
    node.pipeline.stop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

 5. Build & Run the ROS2 Node
 6. 
Step 8: Add Executable to setup.py

```bash
entry_points={
    'console_scripts': [
        'hole_detector = detect_holes_ros2.detect_holes:main',
    ],
},
```

 Step 9: Build & Source ROS2 Package

```bash
cd ~/ros2_ws
colcon build --packages-select detect_holes_ros2
source install/setup.bash
```

Step 10: Run the Node

```bash
ros2 run detect_holes_ros2 hole_detector
```
 
