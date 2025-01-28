I'm using ROS2 humble hawksbill distro, ubuntu 22.04 in VMWare workstation in a windows 10 OS. Laptop : Legion Pro 7i.
Mac or ubuntu based on arm 64 architect wasnt so helpful and also had some problems with the latest version of ubuntu and distro

### Step 0 : Download the pre-requisites 

ROS2 Distro : [Humble hawksbill]!{https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html}
Always remember to run this when you open a terminal 
'''
source /opt/ros/humble/setup.bash
'''bash
you can kinda automate this but that is for later or HW

### Step 1 : Install latest intel RealSense SDK 2.0

'''
sudo apt install ros-humble-librealsense2*
'''bash

### Step 2: Install debian pacakage from ROS Servers

'''
sudo apt install ros-humble-realsense2-*
'''bash

### Step 3: Run Realsese camera node and visualize data

Step 3.1 : Make sure the VM has enabled the sharing of the device 
This should be done before starting the VM, there is an option of what to do with the removable devices

Step 3.2 : Connect 
Connect the Depth camera to the laptop, you need to remove and connect again when the VM is running else it will not let the connection pass through

Step 3.3 : Visualize 

Launch Realsense node and publish pointcloud data:

'''
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x15 pointcloud.enable:=true
'''bash

launch rviz to visualize

'''
rviz2 rviz
'''bash

In Rviz, change \bold{Fixed Frame} from map to camera link and add PointCloud2 to display

Go to topic in pointcloud2 in the added list and change the topic list

if you dont know what is the topic name you can chake using 

'''
ros2 topic list
'''bash

## Step 4 : Creating Ros2 package and Extracting data

### Step 4.1 : make directory and create a package

'''
mkdir ros2_ws/src
ros2 pkg create --build-type ament_python depth_stream_pkg
'''bash

The folder hierarchy should look like this : 

'''
src/
└── depth_stream_pkg/
    ├── depth_stream_pkg/
    │   ├── __init__.py
    ├── package.xml
    ├── resource/
    │   └── depth_stream_pkg
    └── setup.py
'''schematics

### Step 4.2 : Write the Depth Stream subscriber 

Step 4.2.1 : Add dependencies 

Edit the package.xml file to include the dependencies for the sensor_msgs and cv_bridge packages:

'''
<exec_depend>rclpy</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>cv_bridge</exec_depend>
'''xml

### 4.2.2 Install cv_bridge

You’ll need to install cv_bridge for Python:

'''
sudo apt install ros-humble-cv-bridge  # Replace "humble" with your ROS 2 distro
'''bash

### 4.2.3 Write the Subscriber Node

Create a new file for the node:

'''
touch ~/ros2_ws/src/depth_stream_pkg/depth_stream_pkg/depth_subscriber.py
chmod +x ~/ros2_ws/src/depth_stream_pkg/depth_stream_pkg/depth_subscriber.py
'''bash

Now, edit depth_subscriber.py with the following code:

'''
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

'''python

### 4.2.4 Update the setup.py

Open the setup.py file and configure it to install the Python script:

'''
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

'''python

## Step 5: Build and Run the Package
5.1 Build the Package
Go back to your workspace root and build the package:
'''
cd ~/ros2_ws
colcon build
'''bash

Source the workspace after building:

'''
source install/setup.bash
'''bash

5.2 Run the Subscriber

Run the depth subscriber node:

'''
ros2 run depth_stream_pkg depth_subscriber
'''bash

You should start seeing the depth values (e.g., at the center of the depth image) printed to the terminal. if you dont 

5.3 Check the ros topic is correct 

'''
ros2 topic list
'''bash

Also Verify the Topic Type: Run the following command to confirm that /camera/camera/depth/image_rect_raw publishes a sensor_msgs/Image message:

'''
ros2 topic info /camera/camera/depth/image_rect_raw
'''bash

you should see : 

'''
Type: sensor_msgs/Image
'''bash

Also Check if the Depth Data is Being Published: Use the following command to check if data is flowing through the topic:

'''
ros2 topic echo /camera/camera/depth/image_rect_raw
'''bash

## Step 6 seeing the video stream of the Camera

check for RGB data stream : 

create a python file check_rgb.py

'''
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

'''python

Run the script:

'''
ros2 run depth_stream_pkg check_rgb
'''bash

but if you using OpenCV you might need to configure it to BGR Stream

'''
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

'''python

dont forget to add them in the setup, then build and source too.
