# Installation Instructions

**Important Notice**

## Configure workspace

This instructions consider you have [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) installed. Following dependencies have to be installed before configuring the workspace:

```sudo apt-get install python-wstool python-catkin-tools libssh2-1-dev unzip libyaml-cpp0.5v5 libblas-dev liblapack-dev```

Next, initialize workspace using catkin tools:

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/melodic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
cd ~/catkin_ws
catkin build
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Install required simulation packages

If you don't have git you can install it with:
```sudo apt-get install git```

Next, clone and checkout following packages in `src` folder:

```
cd ~/catkin_ws/src
git clone https://github.com/larics/rotors_simulator
cd rotors_simulator
git checkout larics_melodic_master
cd ~/catkin_ws/src
git clone https://github.com/larics/mav_comm
cd mav_comm
git checkout larics_master
```

Before you build, install following dependencies:

```
sudo apt-get install libopencv-dev
sudo apt-get install ros-melodic-joy ros-melodic-octomap-ros ros-melodic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-melodic-control-toolbox ros-melodic-mavros ros-melodic-effort-controllers ros-melodic-position-controllers ros-melodic-robot-controllers ros-melodic-joint-state-controller ros-melodic-controller-manager ros-melodic-gazebo-ros-control ros-melodic-hector-gazebo-plugins ros-melodic-xacro ros-melodic-robot-state-publisher ros-melodic-octomap-ros ros-melodic-dynamic-edt-3d
catkin build
```

Finally, clone mmuav_gazebo repository:

```
cd ~/catkin_ws/src
git clone https://github.com/larics/mmuav_gazebo.git
catkin build
```
