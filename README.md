# nxp_precision_9dof
ROS node to publish ADAFRUIT PRECISION NXP 9-DOF BREAKOUT BOARD imu values using Python 2.7

The [NXP 9-DOF Adafruit board](https://learn.adafruit.com/nxp-precision-9dof-breakout/overview) contains a gyroscope ship (FXAS21002C) and an accelerometer/magnetometer ship (FXOS8700).

Adafruit provide nice Python 3.x libraries for the board. [See here for information](https://learn.adafruit.com/nxp-precision-9dof-breakout/python-circuitpython).
If you're fine to use Python 3.x to drive your board just go ahead with those official libraries. If you need for some reason to use Python 2.7 for your project, this repository might be of some use.

This repo is a set of script to publish NXP 9-DOF data as a ROS node (ROS kinetic and melodic work with Python 2.7).
Note: the scripts scripts/Adafruit_FXOS8700.py and scripts/Adafruit_FXAS21002C.py can be use without any problems outside ROS nodes.

# Install

## Install Adafruit GPIO library

Follow the instruction on the [Adafruit GPIO repository](https://github.com/adafruit/Adafruit_Python_GPIO).

## Clone the repository

## Run without ROS

```
cd this_repo/scripts
python imu_pipe_source.py
```

Make sure to set the correct SCL and SDA pin in the main function of imu_pipe_source.py

## Run as ROS node

```
cd catkin_ws/src
git clone this_repo
catkin_make
source devel/setup.bash
rosrun this_repo imu.py
```

By default the data are sent to the '/imu0'
Make sure to set the correct SCL and SDA pin in the devices construcotrs of imu.py

