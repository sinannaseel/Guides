# ROS2 Humble Hawksbill Depth Camera Integration on Ubuntu 22.04 (VMWare Workstation)

This guide explains how to set up and use an Intel RealSense depth camera with ROS2 Humble Hawksbill on Ubuntu 22.04 in a VMWare Workstation environment.

## Step 0: Pre-requisites

### ROS2 Installation
Follow the [official instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) to install the ROS2 Humble Hawksbill distribution.

### Important Terminal Command
Run this command every time you open a new terminal to source ROS2 setup:
```bash
source /opt/ros/humble/setup.bash
```
---

## Step 1: Install Intel RealSense SDK 2.0

```bash
sudo apt install ros-humble-librealsense2*
```
---
## Step 2: Install ROS2 RealSense Package

```bash
sudo apt install ros-humble-realsense2-*
```
---
## Step 3: Run RealSense Camera Node and Visualize Data
### Step 3.1: Enable Device Sharing in VMWare

Before starting the virtual machine, configure the USB device sharing settings. Ensure the RealSense camera can connect to the VM.
### Step 3.2: Connect the RealSense Camera

Plug the Depth camera into your laptop. If the VM is already running, disconnect and reconnect the device to pass the connection through to the VM.
### Step 3.3: Launch RealSense Node and Publish PointCloud Data

```bash
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x15 pointcloud.enable:=true
```
### Step 3.4: Visualize in RViz

Launch RViz:
```bash
rviz2 rviz
```
Change the Fixed Frame from map to camera_link. Add a PointCloud2 display in RViz and update the topic.
### Step 3.5: Check Available Topics

Use this command to list all available topics:
```bash
ros2 topic list
```
## Step 4: Creating a ROS2 Package and Extracting Data
### Step 4.1: Create Directory and Package
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python depth_stream_pkg
```

```plain text
src/
└── depth_stream_pkg/
    ├── depth_stream_pkg/
    │   ├── __init__.py
    ├── package.xml
    ├── resource/
    │   └── depth_stream_pkg
    └── setup.py

```
### Step 4.2: Write the Depth Stream Subscriber
#### Step 4.2.1: Add Dependencies

Edit package.xml to include these dependencies:

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>cv_bridge</exec_depend>
```
#### Step 4.2.2: Install cv_bridge

```bash
sudo apt install ros-humble-cv-bridge
```

#### Step 4.2.3: Write the Subscriber Node

Create a file for the depth stream subscriber:
```bash
touch ~/ros2_ws/src/depth_stream_pkg/depth_stream_pkg/depth_subscriber.py
chmod +x ~/ros2_ws/src/depth_stream_pkg/depth_stream_pkg/depth_subscriber.py

```

Edit depth_subscriber.py with the following code:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge

class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',  # Replace with your depth topic
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info("Depth Subscriber Node has been started.")

    def listener_callback(self, msg):
        try:
            # Convert the depth image to a NumPy array
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            depth_array = np.array(depth_image, dtype=np.float32)

            # Get the depth value at the center of the image
            height, width = depth_array.shape
            center_depth = depth_array[height // 2, width // 2]
            self.get_logger().info(f"Depth at center: {center_depth:.2f} mm")
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly.")
    except Exception as e:
        node.get_logger().error(f"Exception in node: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Step 4.2.4: Update setup.py

Edit the setup.py file to configure the package:
```python
from setuptools import setup

package_name = 'depth_stream_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A package to subscribe to depth stream and process it',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_subscriber = depth_stream_pkg.depth_subscriber:main',
        ],
    },
)
```
---
## Step 5: Build and Run the Package
### Step 5.1: Build the Package
```bash
cd ~/ros2_ws
colcon build
```

Source the workspace:

```
source install/setup.bash
```

### Step 5.2: Run the Depth Subscriber
```bash
ros2 run depth_stream_pkg depth_subscriber
```

### Step 5.3: Verify the ROS Topic (Optional)

Check the available topics:
```bash
ros2 topic list
```

Verify the topic type:
```bash
ros2 topic info /camera/camera/depth/image_rect_raw
```

You should see:
```bash
Type: sensor_msgs/Image
```
Check if data is being published:
```bash
ros2 topic echo /camera/camera/depth/image_rect_raw
```

---
## Step 6: Viewing the Video Stream of the Camera
### Step 6.1: Create an RGB Data Stream Script

Create a file for the RGB stream:
```bash
touch ~/ros2_ws/src/depth_stream_pkg/depth_stream_pkg/check_rgb.py
chmod +x ~/ros2_ws/src/depth_stream_pkg/depth_stream_pkg/check_rgb.py
```

Edit check_rgb.py with the following code:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RGBChecker(Node):
    def __init__(self):
        super().__init__('rgb_checker')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            cv2.imshow("RGB Image", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error displaying RGB image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = RGBChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 6.2: Run the RGB Stream Script
```bash
ros2 run depth_stream_pkg check_rgb
```

If OpenCV requires a BGR stream, modify the script:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class BGRImageProcessor(Node):
    def __init__(self):
        super().__init__('bgr_image_processor')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # Subscribe to the RGB topic
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            # Convert the image from ROS to OpenCV (in RGB format by default)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")

            # Convert RGB to BGR for OpenCV compatibility
            bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

            # Display the BGR image
            cv2.imshow("BGR Image", bgr_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = BGRImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Don’t forget to update the setup.py, then rebuild and source the package:
```bash
colcon build
source install/setup.bash
```
---

## Step 7: Testing and Troubleshooting

### Step 7.1: Verify Camera Connection
Make sure the RealSense camera is connected properly and visible in your ROS2 environment. If there are issues with the connection or the topic, check the following:

1. **USB Device Pass-through**: Ensure that the camera is properly passed through to the VM. If necessary, reconnect it after starting the VM.
2. **Check Camera Topics**:
   ```bash
   ros2 topic list
```

Ensure that /camera/camera/depth/image_rect_raw and /camera/camera/color/image_raw (or similar topics) are listed.

3. **Topic Information** : Verify the message type for each topic to ensure correct data is being published:
```bash
ros2 topic info /camera/camera/depth/image_rect_raw
ros2 topic info /camera/camera/color/image_raw
```

4. **Data Flow Check** : Use the ros2 topic echo command to confirm that the topics are streaming data:
```bash
ros2 topic echo /camera/camera/depth/image_rect_raw
ros2 topic echo /camera/camera/color/image_raw
```

### Step 7.2: Debugging Errors in the Subscriber Node

If you encounter errors in the depth_subscriber.py node:

1. **Check ROS2 Logs** : Logs can provide more details on the issue. Use the following command to check logs:

```bash
ros2 log
```
Check Dependencies: Ensure all required dependencies are installed:

```bash
sudo apt install ros-humble-sensor-msgs ros-humble-cv-bridge
```

Verify Topic Name: Double-check the topic name in the subscriber node. You can use ros2 topic list to confirm the correct topic.

---

## Step 8: Clean-up and Conclusion

After you have successfully set up the depth camera and your ROS2 package, you can start integrating additional functionalities, such as object detection, path planning, or SLAM. Here are some possible next steps:

    Depth Image Processing: You can extend the depth image processing to extract features, perform segmentation, or compute 3D point clouds.
    Real-time Visualization: Consider integrating the real-time depth image and point cloud visualizations into your robotics project.
    Custom ROS2 Nodes: Create additional ROS2 nodes to process and react to the data from the depth camera in a robotic application.

You now have a working setup for utilizing the Intel RealSense depth camera in your ROS2 Humble Hawksbill environment on Ubuntu 22.04.

---

## Troubleshooting Tips

### USB Device Passthrough Issues
- **Problem**: The RealSense camera does not appear in the VM or cannot be accessed.
- **Solution**: Ensure the VM is set to share the USB device (RealSense) with the guest OS. Reconnect the camera after starting the VM if needed.
  - In VMware, go to the VM settings > USB Controller > Ensure that the “Automatically connect new USB devices” option is enabled.
  - After connecting the camera, check if it appears by running:
    ```bash
    lsusb
    ```
    You should see an Intel RealSense device listed.

### Depth Image Not Showing or Corrupted
- **Problem**: The depth image does not display correctly in `rviz2` or is corrupted.
- **Solution**:
  - Check if the topic name is correct by running:
    ```bash
    ros2 topic list
    ```
  - Verify if data is being streamed by running:
    ```bash
    ros2 topic echo /camera/camera/depth/image_rect_raw
    ```
  - If you see raw data, ensure the right topic is selected in `rviz2` and that the Fixed Frame is set to `camera_link`.

### RealSense Camera Lag or Low Frame Rate
- **Problem**: The camera feed is slow or has noticeable lag.
- **Solution**: This can be caused by system resource constraints. Try lowering the resolution and frame rate for better performance. Modify the launch command for lower settings:
    ```bash
    ros2 launch realsense2_camera rs_launch.py depth_module.profile:=320x240x15 pointcloud.enable:=true
    ```
    You can also adjust the VM settings to allocate more CPU cores and RAM.

### No Data in RViz
- **Problem**: RViz shows no data after launching.
- **Solution**:
  - Ensure that `rviz2` is set to the correct topic and frame (`camera_link`).
  - Double-check if the depth data is published correctly using:
    ```bash
    ros2 topic echo /camera/camera/depth/image_rect_raw
    ```
  - Make sure the depth camera node is running properly and without errors.

---

## Step 9: Possible Enhancements

1. **3D Point Cloud Processing**:
   The point cloud data from the depth stream can be processed for object detection, obstacle avoidance, or 3D mapping. Explore integrating packages like `pcl_ros` or using OpenCV for advanced depth image processing.

2. **Integrating with Robot Hardware**:
   Once your depth camera is fully integrated with ROS2, you can use it with robot platforms like TurtleBot, custom robots, or drones to enable features like autonomous navigation and SLAM.

3. **Using Depth for Augmented Reality (AR)**:
   The depth camera data can be used to build AR applications, such as creating depth maps or enabling object interaction within a 3D environment.

---

## Conclusion

Congratulations on setting up and running the Intel RealSense camera with ROS2 Humble Hawksbill on Ubuntu 22.04 in your VMWare workstation! This guide walked you through the steps of installing necessary packages, configuring the camera, and writing a subscriber node for depth data. You should now be able to explore more advanced applications involving depth sensing, real-time visualization, and robot control.

---

## Feedback and Support

If you run into any issues or have feedback, feel free to leave a comment or seek help from the ROS2 or Intel RealSense communities:

- [ROS2 Community](https://discourse.ros.org/)
- [Intel RealSense Community](https://community.intelrealsense.com/)

---

## Additional Resources

Here are some helpful links to dive deeper into ROS2 and Intel RealSense integration:

1. **[ROS2 Humble Documentation](https://docs.ros.org/en/humble/)**  
   The official documentation for ROS2 Humble Hawksbill. This will guide you through various features, best practices, and tools within ROS2, including message passing, nodes, topics, services, and much more.

2. **[Intel RealSense SDK Documentation](https://dev.intelrealsense.com/docs)**  
   The official Intel RealSense SDK documentation, which provides an overview of the SDK, installation instructions, and API details for working with RealSense depth cameras. This is essential for understanding how to use the camera's advanced features, such as depth sensing, motion tracking, and 3D mapping.

3. **[ROS2 RealSense Package Documentation](https://github.com/IntelRealSense/ros2_intel_realsense)**  
   GitHub repository for the ROS2 RealSense package, which integrates Intel RealSense cameras with ROS2. This package provides nodes for camera control, point cloud generation, and image stream management. You’ll find installation instructions, usage examples, and source code here.

4. **[Intel RealSense Forums](https://community.intelrealsense.com/)**  
   The official Intel RealSense community forums where you can ask questions, find troubleshooting tips, and engage with other RealSense users and developers.

5. **[ROS Answers](https://answers.ros.org/)**  
   A Q&A platform for ROS users to help with troubleshooting, coding questions, and learning best practices in robotics development using ROS.

6. **[PCL (Point Cloud Library)](https://pointclouds.org/)**  
   The official Point Cloud Library (PCL) website. PCL is widely used for processing 3D point cloud data, and is often used in conjunction with RealSense cameras for 3D mapping, object recognition, and more.

---

These resources should help you further develop your understanding of both the ROS2 framework and RealSense SDK, enabling you to create more sophisticated robotic applications.


