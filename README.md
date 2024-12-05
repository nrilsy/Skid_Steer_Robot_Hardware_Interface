# Skid_Steer_Robot_Hardware_Interface
This repo is created to implement a hardware interface for a DC Motor (which is driven by L298N Motor Driver) for real world mobile robot project. Aim is to create a bunch of packages that will be deployed on Raspberry Pi 3b to provide a direct control mechanism to the robot. 
- Since the motors have no encoders, the state interfaces are provided by subscribing odometry topic. From odometry topic, the linear_velocity_x and angular_velocity_z values are obtained and necessary velocities of the wheels are calculated by:
`velocity_states_.at(i) =  linear_velocity_x_ +- angular_velocity_z_ * wheel_separation_ / 2.0` equation. + or - sign depends on the wheels location (left or right).
- There is only one velocity command in this project. pigpiod_if2 library is used to control the motor driver(L298N) which is directly connected to the Raspberry Pi 3b pins.


This package is still in progress and do not provide any control on motors for now. 
