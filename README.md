# Switched-Turn-Trajectory-Source-Localization
This is Software-In-The-Loop implemenation of Switched Turn/Trajectory Source localization Algorithms.

 ## Installation
 1. Install ros-noetic-desktop-full, follow the instructions in https://wiki.ros.org/noetic/Installation/Ubuntu
 2. Download and setup Px4 Firmware, https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html#ros-gazebo-classic
 3. After setting up Px4 Autopilot, run the following in Px4-Autopilot directory
    ```
    cd ~/Px4-Autopilot
    make px4_sitl gazebo-classic
    ```
    Add the following the to .bashrc
    ```
    source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
    ```
 4. Install MAVROS to communicate with Px4 through ROS, https://docs.px4.io/main/en/ros/mavros_installation.html
 5. Create ros workspace, install dependent packages and clone the src_loc package
    ```
    sudo apt-get install python3-catkin-tools
    sudo apt-get install ros-noetic-grid-map
    mkdir -p catkin_ws/src
    cd catkin_ws/src
    git clone ---
    cd ..
    catkin build
    ```

  

