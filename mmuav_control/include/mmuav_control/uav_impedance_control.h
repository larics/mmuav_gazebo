#ifndef UAV_FORCE_CONTROL_H
#define UAV_FORCE_CONTROL_H

#include "ros/ros.h"
#include <mmuav_control/Tf2.h>
#include <mmuav_control/diff2.h>
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/WrenchStamped.h>
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>
#include <mmuav_control/mraic.h>
#include <mmuav_msgs/MRAIController.h>
#include <mmuav_control/median_filter.h>

#define MAX_MOVING_AVARAGE_SAMPLES_NUM	100


class ImpedanceControl{
	private:
		void force_measurement_cb(const geometry_msgs::WrenchStamped &msg);
		void clock_cb(const rosgraph_msgs::Clock &msg);
		void pose_ref_cb(const geometry_msgs::PoseStamped &msg);
		void force_torque_cb(const geometry_msgs::WrenchStamped &msg);
		void initializeImpedanceFilterTransferFunction(void);
		float getFilteredForceZ(void);
		float getFilteredForceX(void);
		float getFilteredForceY(void);
		float getFilteredTorqueX(void);
		float getFilteredTorqueY(void);
		float getFilteredTorqueZ(void);
		bool check_collision(void);
		bool check_impact(void);
		void publishMRAICstatus(void);
		float* impedanceFilter(float *e, float *Xr);
		float* modelReferenceAdaptiveImpedanceControl(float dt, float *e, float *g0);
		void quaternion2euler(float *quaternion, float *euler);
		void initializeMRACControl(void);

		volatile bool start_flag_, force_sensor_calibration_flag_;
		bool impact_flag_, collision_, ref_flag_;
		float force_x_meas_[MAX_MOVING_AVARAGE_SAMPLES_NUM];
		float force_z_meas_[MAX_MOVING_AVARAGE_SAMPLES_NUM];
		float force_y_meas_[MAX_MOVING_AVARAGE_SAMPLES_NUM];
		float torque_x_meas_[MAX_MOVING_AVARAGE_SAMPLES_NUM];
		float torque_y_meas_[MAX_MOVING_AVARAGE_SAMPLES_NUM];
		float torque_z_meas_[MAX_MOVING_AVARAGE_SAMPLES_NUM];
		float M_[6], B_[6], K_[6], omega_[6], zeta_[6], kd0_[6], sigma3_[6];
		float em0_[6], dem0_[6], wp_[6], wd_[6], fe_[6], kp0_[6], sigma2_[6];
		float a1_[6], b1_[6], c1_[6], a2_[6], b2_[6], c2_[6], sigma1_[6];
		float force_z_offset_, force_y_offset_, force_x_offset_, mrac_time_;
		float torque_y_offset_, torque_x_offset_, torque_z_offset_;
		int rate_, moving_average_sample_number_, targetImpedanceType_;

		median_filter force_z_median;

		rosgraph_msgs::Clock clock_;
		geometry_msgs::PoseStamped pose_ref_;
		geometry_msgs::WrenchStamped force_torque_ref_;
		std_msgs::Float64 yaw_ref_;

		ros::NodeHandle n_;

		ros::Subscriber force_ros_sub_, clock_ros_sub_;
		ros::Subscriber force_torque_ref_ros_sub_, pose_ref_ros_sub_;

		ros::Publisher force_filtered_pub_, position_commanded_pub_, yaw_commanded_pub_;
		ros::Publisher mraic_status_pub_;

		Tf2 Ge_[6], Gxr_[6];
		mraic mraic_[6];

	public:
		ImpedanceControl(int rate, int moving_average_sample_number);
		void setImpedanceFilterMass(float *mass);
		void setImpedanceFilterDamping(float *damping);
		void setImpedanceFilterStiffness(float *stiffness);
		void setTargetImpedanceType(int type);
		void LoadImpedanceControlParameters(std::string file);
		void run();
};

#endif