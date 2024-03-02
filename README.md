# Sensorized Hand

## About 
This repository contains the control code for a custom, fully-actuated sensorized hand. It is designed to be implemented on a Raspberry Pi with ROS 2, which requires Ubuntu 22.04. 

![Overview](resources/hand.jpg "The custom fully actuated hand.")

## Installation
Create a ROS 2 workspace. Within the source directory clone this repo:
```console
git clone --recurse-submodules git@github.com:OSUrobotics/sensorized_hand.git
```
Install the required dependencies:
```console
python3 -m pip install dynamixel-control
```
Build the workspace by running colcon build from the /src folder:
```console
colcon build --symlink-install
. install/setup.bash
```
Don't forget to add sourcing both the workspace and ROS install to the .bashrc!

## Usage
To launch everything, run:
```console
ros2 launch fingers all_launch.py
```
To launch only the IMUs, TOFs, or motors:
```console
ros2 launch imu bringup_imus_launch.py 
```
```console
ros2 launch tof tof_bringup_launch.py 
```
```console
ros2 run fingers motor_control.py 
```

## Other notes
You may need to downgrade setup tools:
```console
python3 -m pip install setuptools==58.2.0
```
Install readerwriterlock:
```console
python3 -m pip install readerwriterlock
```
To see devices on an I2C bus (number is the bus):
```console
i2cdetect -y 1
```
To see Raspi temp or if it is throttled:
```console
vcgencmd measure_temp
vcgencmd get_throttled
```