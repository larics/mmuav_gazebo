# mmuav_gazebo
This repository contains code for simulating various configurations of UAVs. The model used within this package is 3DR Arducopter equipped with apparatus for generating shift in vehicle's CoG. 

If you use this repository within your research publication, please cite:
```
@Article{Arbanas2018,
author="Arbanas, Barbara
and Ivanovic, Antun
and Car, Marko
and Orsag, Matko
and Petrovic, Tamara
and Bogdan, Stjepan",
title="Decentralized planning and control for UAV--UGV cooperative teams",
journal="Autonomous Robots",
year="2018",
month="Feb",
day="15",
abstract="In this paper we study a symbiotic aerial vehicle-ground vehicle robotic team where unmanned aerial vehicles (UAVs) are used for aerial manipulation tasks, while unmanned ground vehicles (UGVs) aid and assist them. UGV can provide a UAV with a safe landing area and transport it across large distances, while UAV can provide an additional degree of freedom for the UGV, enabling it to negotiate obstacles. We propose an overall system control framework that includes high-accuracy motion planning for each individual robot and ad-hoc decentralized mission planning for complex missions. Experimental results obtained in a mockup arena for parcel transportation scenario show that the system is able to plan and execute missions in various environments and that the obtained plans result in lower energy consumption.",
issn="1573-7527",
doi="10.1007/s10514-018-9712-y",
url="https://doi.org/10.1007/s10514-018-9712-y"
}
```

Package ```mmuav_arducopter_bridge``` is used for controlling the UAV in real world through files developed in simulation.

## Table Of Contents

- [Installation](#Installation)
- [Basic usage](#BasicUsage)
  * [Running the simulation](#Running)
    * [Mmuav](#Mmuav)
    * [VpcMmc UAV](#VPCMMCUAV)

## <a name="Installation"></a> Installation
Detailed installation instructions can be found in [InstallationInstructions.md](https://github.com/larics/mmuav_gazebo/blob/master/InstallationInstructions.md)

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

To run simulation for UAV with moving mass controller launch and position controller(this simulation supports trajectory following):

```roslaunch mmuav_gazebo vpc_mmcuav_attitude_position.launch ```

Alternatively, one can run simulation with a **rope** attached to the UAV:

```roslaunch mmuav_gazebo vpc_mmcuav_attitude_height.launch model_type:=mmcuav_rope```

After running the command above, the UAV will hover at 1m height. You can control the UAV through several topics:

**Subscriptions:**

- ``` /vpc_mmcuav/euler_ref``` -> Desired euler angle reference here to move the UAV.
- ``` /vpc_mmcuav/vel_ref``` -> Desired height of the UAV.
- ``` /vpc_mmcuav/pose_ref``` -> Position and orientation reference
- ``` /vpc_mmcuav/multi_dof_trajectory``` -> Send trajectory to this topic

**Published topics:**

- ``` /vpc_mmcuav/imu``` -> Imu data from the UAV
- ``` /vpc_mmcuav/odometry ``` -> Provides position, orientation, linear velocities and angular velocities of the UAV
- ``` /vpc_mmcuav/pid_*``` -> All topics starting with ```pid_``` are publishing controller status

# TODO: 
- [ ] Fix README.md
- [ ] Remove redundant files regarding control 
- [ ] Create another robot arm
