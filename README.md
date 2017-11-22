# mmuav_gazebo
This repository contains code for simulating various configurations of UAVs. The model used within this package is 3DR Arducopter equipped with apparatus for generating shift in vehicle's CoG. 

Package ```mmuav_arducopter_bridge``` is used for controlling the UAV in real world through files developed in simulation.

## Table Of Contents

- [Installation](#Installation)
- [Basic usage](#BasicUsage)
  * [Running the simulation](#Running)
    * [Mmuav](#Mmuav)
    * [VpcMmc UAV](#MMCUAV)

## <a name="Installation"></a> Installation
Before you start some packages are required. You can install them with:

```sudo apt-get install ros-kinetic-hector-gazebo-plugins ros-kinetic-effort-controllers```

Rotor dynamics are simulated with ```rotors_simulator``` which can be found on [rotors simulator github page](https://github.com/ethz-asl/rotors_simulator). Instructions for workspace creation are supplied within this package.

Finally, to install simply clone this repository and build it:
```
git clone https://github.com/larics/mmuav_gazebo.git
catkin build
```

## <a name="BasicUsage"></a> Basic Usage
This section describes how to run the simulation and lists topis of interest.

### <a name="Running"></a> Running the simulation
There are several different types of simulation that depend on control algorithm in use.

#### <a name="Mmuav"></a> Mmuav
To run simulation with UAV equipped with dual arm manipulator and control based on shift in center of gravity launch:

```roslaunch mmuav_gazebo mmuav_attitude_height.launch``` 

After running the command above, the UAV will hover at 1m height. You can control the UAV through several topics:

**Subscriptions:**

- ``` /mmuav/euler_ref``` -> Desired euler angle reference here to move the UAV.
- ``` /mmuav/vel_ref``` -> Desired height of the UAV.

**Published topics:**

- ``` /mmuav/imu``` -> Imu data from the UAV
- ``` /mmuav/odometry ``` -> Provides position, orientation, linear velocities and angular velocities of the UAV
- ``` /mmuav/pid_*``` -> All topics starting with ```pid_``` are publishing controller status
- ``` /mmuav/camera1/image_raw ``` -> Raw image from camera attached below the UAV used for visualizing dual arm manipulator motion

#### <a name="VPCMMCUAV"></a> VpcMmc UAV

To run simulation for UAV with moving mass controller launch:

```roslaunch mmuav_gazebo vpc_mmcuav_attitude_height.launch ```

Alternatively, one can run simulation with a **rope** attached to the UAV:

```roslaunch mmuav_gazebo vpc_mmcuav_attitude_height.launch model_type:=mmcuav_rope```

After running the command above, the UAV will hover at 1m height. You can control the UAV through several topics:

**Subscriptions:**

- ``` /mmuav/euler_ref``` -> Desired euler angle reference here to move the UAV.
- ``` /mmuav/vel_ref``` -> Desired height of the UAV.

**Published topics:**

- ``` /mmuav/imu``` -> Imu data from the UAV
- ``` /mmuav/odometry ``` -> Provides position, orientation, linear velocities and angular velocities of the UAV
- ``` /mmuav/pid_*``` -> All topics starting with ```pid_``` are publishing controller status