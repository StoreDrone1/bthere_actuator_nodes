# bThere actuator nodes

This repository is intended to be a collection of open-source ROS nodes to easy interface between software and mechanical actuators such as motors. It currently only has two nodes, the raspberry pi motor controller and the
maestro servol controller.

These nodes are developed for use with the bThere console, but are not dependent on it.

# The nodes

## Pi motor controller

This node uses raspberry pi GPIO to control a tank- or mecanum-style robot chassis. It listens to a twist topic (default `/cmd_vel/teleop` but configurable) and outputs pwm to configured pins. Currently this configuration is done in the script, but could be replaced with a system to read from a config file. See the internal docstrings and comments for configuration instructions.

Note: This node does not incorporate any kind of deadman switch or smoothing. Optimally it should be used with these things to avoid damaging the robot, though they are not strictly necessary.

### to use

```bash
$ catkin_make
$ source devel/setup.bash
$ roslaunch pi_motor_controller pi_motor_controller.launch
```

### parameters

- `topic_name`: Defaults to `/cmd_vel/teleop`, set this to the topic you would like the node to listen to.


## Maestro servo controller

This node uses a Pololu maestro servo controller to control a pair of servos, typically for panning and tilting
a camera. It listens to a twist topic (default /camera_servo1/teleop) and outputs pwm to the servos via command
packets to the maestro controller. The maestro controller is expected to be connected to the robot via serial
cable.

Configuration of the serial device is in linux_config.cfg. There are other configuration parameters in the scripts.

Note that an out-of-box maestro servo controller must first be configured to use usb dual port mode and to have
per-channel settings for servo extents, acceleration, etc. See the documenation, config util, and sample
config files in the util directory.

### to use

```bash
$ catkin_make
$ source devel/setup.bash
$ roslaunch maestro_servo_controller maestro_servo_controller.launch
```

### parameters

- `topic_name`: Defaults to `/camera_servo1/teleop`, set this to the topic you would like the node to listen to.