# arti_hardware  [Transcend Robotics](http://transcendrobotics.com/)
The Hardware ROS package for ARTI3 robot
## How to install
```
sudo apt-get install ros-$ROS_DISTRO-serial
cd "your_workspace"/src
git clone https://github.com/transcendrobotics/arti_hardware.git
cd ..
catkin_make
```
## How to use it
Once you have the robot, and installed ROS and this package, you can simply do
```
roslaunch arti_hardware arit_base.launch
```
## How to control the robot
Simply just publish veloctiy command on "/cmd_vel" you should be able to control the robot, or you can publish a left and right voltage value to control it.
For example:
### Use keyboard
```
sudo apt-get install ros-$ROS_DISTRO-teleop-twist-keyboard
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
See the instruction to control the robot.
### Use joystick
```
sudo apt-get install ros-$ROS_DISTRO-teleop-twist-keyboard
roslaunch teleop_twist_joy teleop_twist_joy.launch
```
## Notes
1. The code was capable with odometry from encoder but we don't want to use encoder anymore because track robots slip a lot.
2. The hardware doesn't have velocity control, the command sends to the robot is open loop, works as voltage command.

## Parameters
 Parameter                    |           Description                                       |              Value          
------------------------------|-------------------------------------------------------------|-------------------------    
port                          | The port address of the hardware                            | string               
baud_rate                     | Hardware Baudrate, should not be changed                    | double
body_width                    | The width of the body (m)                                   | double
serial_time_out               | Serial Communication Timeout (ms)                           | double
cmd_time_out                  | Command Timeout (ms)                                        | double
control_rate                  | The rate of sending command to hardware (Hz)                | double
odom_rate                     | The rate of publishing odom  (Hz)                           | double
odom_window                   | The window size to smooth odometry                          | int
wheel_multiplier              | The multiplier for wheel diameter, used in odoemtry         | double
flip_lr                       | Whether to flip the direction of left and right wheel       | bool        
publish_tf                    | Whether to publishing TF                                    | bool        
odom_bias                     | the bias of odometry reading on left wheel                  | bool        
temp_cutoff_value             | When the robot's temperature reaches this value, it will stop to protect the battery(&deg; C) | double
## Aurthor: [Di Zeng](https://www.linkedin.com/in/dizeng)
