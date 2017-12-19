#ifndef UAV_FORCE_CONTROL_H
#define UAV_FORCE_CONTROL_H

#include "ros/ros.h"
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/WrenchStamped.h>
#include <rosgraph_msgs/Clock.h>

class ForceControl{
	private:
		void force_measurement_cb(const geometry_msgs::WrenchStamped &msg);
		void clock_cb(const rosgraph_msgs::Clock &msg);
		float getFilteredForceZ(void);
		bool check_impact(void);

		volatile bool start_flag_;
		float force_x_meas[10], force_z_meas[10], force_y_meas[10];
		int rate_, moving_average_sample_number_;

		rosgraph_msgs::Clock clock_;

		ros::NodeHandle n_;

		ros::Subscriber force_ros_sub_, clock_ros_sub_;

		ros::Publisher force_filtered_pub_, d_force_filtered_pub_;

	public:
		ForceControl(int rate);
		void run();
};

#endif