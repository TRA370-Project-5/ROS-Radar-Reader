# Installation

Clone the repository to the ROS workspace

## Libraries

https://github.com/stelzo/ros_pointcloud2

https://github.com/sequenceplanner/r2r

bincode

serde

serde-big-array

Add using  

    cargo add <name>

Build as usual with colcon and then source install folder in ros workspace

# Interface
Right now it publishes Pointcloud2 topics on /pointcloud

# Run

Before first launch every new session source the installation folder in the ros workspace, if there is no install folder then you can source the native ros installation folder
    source /opt/$ROS_DISTRO/install/setup.bash
or
    source /ros_ws/install/setup.bash
    
Launch using

    ros2 run radar_reader radar_reader 
