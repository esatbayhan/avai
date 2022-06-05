# Simple Camera Package for ROS2 Foxy

## Build Instruction
```bash
rosdep install -i --from-path src --rosdistro foxy -y
colcon build
```

## Execution Instruction
```bash
. install/setup.bash # inside workspace
ros2 run campkg camera # on TurtleBot3
ros2 run campkg processing # on TurtleBot3
ros2 run campkg display # on remote computer
```
