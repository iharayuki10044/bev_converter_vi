# bev_converter

## Enviornment
- Ubuntu 16.04 or 18.04
- ROS Kinetic or Melodic

## Requirement
- PCL 1.8
-  amslabtech/dynamic_cloud_detector ... https://github.com/amslabtech/dynamic_cloud_detector.git

## Install and Build

```
cd catkin_workspace/src
git clone https://github.com/YoshitakaNagai/bev_converter.git
cd ..
catkin_make
```

## Nodes
### bev_converter
#### Published topics
- /bev/grid (nav_msgs/OccupancyGrid)

#### Subscribed topics
- /cloud/dynamic (sensor_msgs/PointCloud2)
- /odom (nav_msgs/Odometry)

## How to Use
```
roslaunch bev_converter bev_flow_estimator.launch
```
