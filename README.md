# Autonomous Vehicles and Artificial Intelligence

## Packaged
- campkg
- perception


# Get started with the simulation:
- make sure you clone the entire repo in the following file structure: HOME/turtlebot3_ws/src
- download zip: https://drive.google.com/file/d/1Uj5RQQPWN1KfQzIC3PBE3yP42D1mGnsx/view?usp=sharing
- inside "HOME/turtlebot3_ws/src/avai/src" unpack the zip (containing other repos + custom simulation files)
- delete all files in HOME/turtlebot3_ws/ other than the "src" folder
- run the following commands in HOME/turtlebot3_ws/

colcon build
export TURTLEBOT3_MODEL=waffle_pi
. install/setup.bash
ros2 launch turtlebot3_gazebo track_1.launch.py
ros2 launch perception detector_display_launch.py
ros2 run turtlebot3_teleop teleop_keyboard

if everything works you can do the same as in this video: https://drive.google.com/file/d/1YtI3s_NveSBI50pj2B6MthdQEDRCGTfG/view?usp=sharing
