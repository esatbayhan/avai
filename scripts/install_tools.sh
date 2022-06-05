echo "[Install additional tools]"
source ~/.bashrc
sudo apt-get install -yq graphicsmagick-libmagick-dev-compat
sudo apt-get install -yq \
    ros-foxy-gazebo-* \
    ros-foxy-cartographer \
    ros-foxy-cartographer-ros \
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup \
    ros-foxy-dynamixel-sdk \
    ros-foxy-turtlebot3-msgs \
    ros-foxy-turtlebot3 \
    python3-rosdep2