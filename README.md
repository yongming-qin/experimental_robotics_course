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
`git clone` the repository. Rename `experimental_robotics_course` to `exp_ws`. Or you can just use the ROS packages in src folder.

`catkin build` the `src` code in a workspace. Install the needed packages.
This command helps to install the dependency but the installation is not complete.
`rosdep install --from-paths src --ignore-src -r -y`

Upload `arduino_cmd_vel` to the Arduino board. Change the PWM pins to match the robot Aruduino.


For realsense:
```
roslaunch simple_drive course_drive.launch
roslaunch thk_nav course_slam_realsense.launch
roslaunch thk_nav course_navigation_realsense.launch
```

For ouster:
```
roslaunch simple_drive course_drive.launch
roslaunch thk_nav course_slam_ouster.launch
roslaunch thk_nav course_navigation_ouster.launch
```

The joystick controls the robot via `teleop/cmd_vel`. The `move_base` node of navigation stack controls the robot via `move_base/cmd_vel`. The two are subscribed by `simple_drive/cmd_vel_mux` that determines which one is sent to Arduino controller through `cmd_vel`.

