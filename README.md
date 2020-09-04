# bThere actuator nodes

This repository is intended to be a collection of open-source ROS nodes to easy interface between software and mechanical actuators such as motors. It currently only has one node, the raspberry pi motor controller.

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
