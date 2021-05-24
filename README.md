# experimental_robotics_course

## Description

This repository contains the code for the experimental robotics course.  
<img src="media/setup.jpg" width="300">

Hardware:

* The robot uses a differential drive mechanism.
* A realsense D435i 3D camera.
* [optional] A ouster 3D lidar.
* A joystick is used for teleoperation and stopping the autonomous movements when needed.
* An arduino board is used to subscribe command velocity.

Software:

* realsense camera driver
* rtabmap
* simple_drive
* etc.

Code:  
For realsense D435i, please use the branch of `realsense`.  
For ouster, please use the branch of `ouster`.  

* `src` for ROS
* `arduino_cmd_vel` for Arduino

Videos:

* `navigation_phone_1_ppt.mp4, navigation_phone_2_ppt.mp4` are recorded using a phone.
* `navigation_screen.mp4` is the screen recording.

## Running
`git clone` the repository.

`catkin_make` the `src` code in a workspace. Install the needed packages.
This command helps to install the dependency but the installation is not complete.
`rosdep install --from-paths src --ignore-src -r -y`

Upload `arduino_cmd_vel` to the Arduino board. Change the PWM pins to match the robot Aruduino.



```
roslaunch simple_drive course_drive.launch
roslaunch thk_nav course_slam_realsense.launch
roslaunch thk_nav course_navigation_realsense.launch
```

The joystick controls the robot via `teleop/cmd_vel`. The `move_base` node of navigation stack controls the robot via `move_base/cmd_vel`. The two are subscribed by `simple_drive/cmd_vel_mux` that determines which one is sent to Arduino controller through `cmd_vel`.


## Support for ouster 3D lidar
Please install rtabmap from source so that lidars can be supported. https://github.com/introlab/rtabmap_ros
* Install `libpointmatcher, libnano` via complilation.
* Install `rtabmap` (1. not anaconda python 2. swtich to melodic-devel branch ).  Download the `rtabmap_ros` (swtich to melodic-devel branch).
