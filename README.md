# experimental_robotics_course

## Description

This repository contains the code for the experimental robotics course.
![Setup](https://github.com/yongming-qin/experimental_robotics_course/media/setup.jpg)

Hardware:

* The robot uses a differential drive mechanism.
* A realsense D435i 3D camera.
* A joystick is used for teleoperation and stopping the autonomous movements when needed.
* An arduino board is used to subscribe command velocity.

Software:

* realsense camera driver
* rtabmap
* simple_drive
* etc.

Code: 

* `src` for ROS
* `arduino_cmd_vel` for Arduino

Videos:

* `navigation_phone_1_ppt.mp4, navigation_phone_2_ppt.mp4` are recorded using a phone.
* `navigation_screen.mp4` is the screen recording.

## Running
`git clone` the repository. For final project, the folders of `realsense-ros, rtabmap_ros, turtlebot_files, zed-ros-wrapper` can be deleted if rtabmap and realsense driver have been installed, 

`catkin_make` the `src` code in a workspace. Install the needed packages.

Upload `arduino_cmd_vel` to the Arduino board.





```
roslaunch simple_drive course_drive.launch
roslaunch thk_nav course_slam_realsense.launch
roslaunch thk_nav course_navigation.launch
```

The joystick controls the robot via `teleop/cmd_vel`. The `move_base` node of navigation stack controls the robot via `move_base/cmd_vel`. The two are subscribed by `simple_drive/cmd_vel_mux` that determines which one is sent to Arduino controller through `cmd_vel`.


## Support for ouster 3D lidar
Please install rtabmap from source so that lidars can be supported. https://github.com/introlab/rtabmap_ros
* Install `libpointmatcher, libnano` via complilation.
* Install `rtabmap` (1. not anaconda python 2. swtich to melodic-devel branch ).  Download the `rtabmap_ros` (swtich to melodic-devel branch).