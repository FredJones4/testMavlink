# Motivation
This code was written to test the capabilities of MAVSDK-Python when used with PX4 v 1.14.

# Result
#### High Data Quality, Poor Data speed
The data gathering, while it includes the airspeed that is not currently available in PX4 ROS 2 topics, runs so slowly that it is not worth it.

**TODO: insert table here with empirical results from data gathering.**

#### No support for Actuator/Motor Control

My discussion with developers on the issue: https://discuss.px4.io/t/which-servo-to-which-set-actuator-value/39972/7 

#### Easy to understand (compared to ROS 2)

# Main files

```
1_and_3_dean_new_connect_receive_data.py
1-1_dean_alpha_beta.py
2_1_test_both_control
```

# How to use
1. Set up PX4 Gazebo (and QGroundControl, if desired for debugging)
2. Run the python control file of choice.