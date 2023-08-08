# CSLAM STORAGE

This is a ROS2 package for the [Swarm-Slam project.](https://github.com/MISTLab/Swarm-SLAM) This package runs a node that store the necessary map data (Point cloud keyframes and pose graph). The goal of doing this is make possible to use the map data from explorations in other applications such as ML training.

## Table of Contents

TODO

- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Installation and building
```
    # Make sure that you are in <your_swarm_slam_workspace>/src/ 
    git clone https://github.com/roman2veces/cslam_storage.git
    cd ..
    colcon build
```