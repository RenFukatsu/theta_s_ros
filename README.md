# theta_s_ros

## Environment
- Ubuntu 18.04 or 20.04
- ROS Melodic or Noetic
- OpenCV 3 or 4

## Install and Build

```
cd catkin_ws/src
git clone --recursive https://github.com/RenFukatsu/theta_s_ros.git
cd ..
catkin build
```

## Nodes
### theta_s_camera
[libuvc_camera camera_node](http://wiki.ros.org/libuvc_camera)
### converter
Converting from fisheye image to equirectanguler image
#### Subscribed topics
- /camera/image_raw (sensor_msgs/Image)
#### Published topics
- /equirectanguler/image_raw (sensor_msgs/Image)

## How to Use
Theta S in live mode and then
```
roslaunch theta_s_ros theta_s_ros.launch
```

