# CSLAM STORAGE

This is a ROS2 package for the [Swarm-Slam project.](https://github.com/MISTLab/Swarm-SLAM) This package runs a node that store the necessary map data (Point cloud keyframes and pose graph). The goal of doing this is make possible to use the map data from explorations in other applications such as ML training.

## Table of Contents

TODO

- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Installation and building
Make sure that you followed the installation guide in [Swarm-Slam.](https://github.com/MISTLab/Swarm-SLAM) 

```
    # Make sure that you are in <your_swarm_slam_workspace>/src/ 
    git clone https://github.com/roman2veces/cslam_storage.git
    cd ..
    conda activate <your_conda_environment>
    colcon build
```

## How to test this project
You will need at least 2 terminals:

First terminal (simulating the base station)
```
    # Make sure that you built the workspace and you are in <your_swarm_slam_workspace> 
    conda activate <your_conda_environment>
    source /opt/ros/foxy/setup.bash
    source install/setup.bash
    ros2 launch cslam_visualization visualization_lidar.launch.py
```


Second terminal (simulating a robot using a dataset, you can launch as many process like this as robots)
```
    # Make sure that you built the workspace and you are in <your_swarm_slam_workspace> 
    conda activate <your_conda_environment>
    source /opt/ros/foxy/setup.bash
    source install/setup.bash
    <!-- You can also use kitti_lidar_01.launch.py, or create your own launch file inspired from this one -->
    ros2 launch cslam_experiments kitti_lidar_00.launch.py launch_delay_s:=0
```

