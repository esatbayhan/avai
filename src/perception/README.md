# Perception

## Installation
```bash
cw
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlin-install
```

## Usage
```bash
ros2 run perception detector # without display
ros2 launch perception detector_display_launch.py # with display
```
