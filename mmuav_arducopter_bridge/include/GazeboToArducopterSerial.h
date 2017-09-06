/******************************************************************************
File name: GazeboToArducopterSerial.h
Description: ROS serial interface for arducopter stepper board
Author: Antun Ivanovic
******************************************************************************/
#include "ros/ros.h"

#include <iostream>
#include <vector>
#include <math.h>
#include <ros/package.h>
#include <string>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

#include <std_msgs/Float64MultiArray.h>
#include <mmuav_arducopter_bridge/StepperParametersConfig.h>
#include <dynamic_reconfigure/server.h>

using namespace std;

class GazeboToArducopterSerial
{
public:
    GazeboToArducopterSerial();
    ~GazeboToArducopterSerial();
    void run();

private:
    // Open and configure serial port.
    struct termios tty;
    struct termios tty_old;
    int USB;
    int SetSerialAttributes(string port, int baudrate);
    int baudrate; string port;
    int SerialWrite(int m[4], unsigned char terminator);
    int SerialRead();

    // ROS-related
    // Node handles
    ros::NodeHandle nhParams, nhTopics;
    ros::Subscriber all_mass_sub;
    void allMassCallback(const std_msgs::Float64MultiArray &msg);
    void reconfigureCallback(mmuav_arducopter_bridge::StepperParametersConfig &config, uint32_t level);

};