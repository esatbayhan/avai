# Simple Camera Package for ROS2 Foxy

## Build Instruction
```bash
rosdep install -i --from-path src --rosdistro foxy -y
colcon build
```

## Execution Instruction
```bash
. install/setup.bash # inside workspace
ros2 run campkg camera_node # on TurtleBot3
ros2 run campkg image_processing_node # on TurtleBot3
ros2 run campkg image_display_node # on remote computer
```

## Usage Image Display Node
Press S to save manually an image

## Architecture
![Architecture rosgraph](/docs/rosgraph.png)