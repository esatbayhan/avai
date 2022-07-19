# Autonomous Vehicles and Artificial Intelligence

## Start Working
- Follow the emanual of TurtleBot3 to install all tools (use foxy)
  - [emanual.robotis.com](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)
- Clone the project with submodules as colcon_ws in your home folder
  - `git clone --recurse-submodules -j8 https://github.com/esatbayhan/avai ~/colcon_ws`
- Install all dependencies required by packages
  - `rosdep install --from-paths src --ignore-src -r -y`

## Stack
TurtleBot3
1. Start Camera `ros2 run campkg camera`
2. Start Camera Processing `ros2 run campkg processing`
3. Start Heartbeat (Emergency Stop) `ros2 run controls heartbeat`

Computer

4. Start Cone Detection `ros2 run perception detector`
5. Start Controller (Autonomous Driving) `ros2 launch controls turtlebot3_burger.launch.py`

## Simulation
1. Start Simulation `ros2 launch turtlebot3_gazebo track_1.launch.py`
2. Start Simulated Camera `ros2 run campkg simcam`

## Demonstration
### Simulation with Detection, Sensor Filtering, Waypoint calculation and autonomous driving
https://user-images.githubusercontent.com/69524126/178964943-cebe4831-9a8c-4f18-9498-0b6279cd4a6b.mp4

### Live Demo
https://user-images.githubusercontent.com/69524126/179793282-fffd812d-bc40-44b8-b9b7-4b5e079fb6a9.mp4

