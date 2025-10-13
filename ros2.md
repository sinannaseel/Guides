colcon build 
source install/setup.bash

cd ~/ros2_tutorial_workspace/src
ros2 pkg create the_simplest_python_package \
--build-type ament_python

cd ~/ros2_tutorial_workspace/src
ros2 pkg create python_package_with_a_node \
--build-type ament_python \
--node-name sample_python_node

specific packages:

colcon build --packages-select <name-of-pkg>

