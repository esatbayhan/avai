# Autonomous Vehicles and Artificial Intelligence

## Start Working
- Follow the emanual of TurtleBot3 to install all tools (use foxy)
  - [emanual.robotis.com](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)
- Clone the project with submodules as colcon_ws in your home folder
  - `git clone --recurse-submodules -j8 https://github.com/esatbayhan/avai ~/colcon_ws`
- Install all dependencies required by packages
  - `rosdep install --from-paths src --ignore-src -r -y`

## Stack
### Currently
TurtleBot3
1. Start Camera `ros2 run campkg camera`
2. Start Camera Processing `ros2 run campkg processing`

Computer

3. Start Cone Detection `ros2 run perception detector`
4. Start LiDAR Filter `ros2 run lidar_filter filter`

### Future
Computer

5. Calculate and publish Waypoints based on LaserScan published by lidar_filter.filter
6. Subscribe Waypoints and move TurtleBot3 to specified Waypoint

## Simulation
1. Start Simulation `ros2 launch turtlebot3_gazebo track_1.launch.py`
2. Start Simulated Camera `ros2 run campkg simcam`

## Demonstration
### Simulation with Detection & LiDAR Filtering
https://user-images.githubusercontent.com/69524126/178010672-4d66b507-c91b-4a6a-b23f-e7b26b3d5333.mp4

