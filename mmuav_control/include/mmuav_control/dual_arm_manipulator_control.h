#include <mmuav_control/DualArmManipulatorInverseKinematics.h>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include <cmath>

class DualArmManipulatorControl
{
	public:
		DualArmManipulatorControl();
		void start();
		void set_rate(int rate);
		void LoadParameters(std::string file);

	private:
		void left_mapinulator_position_cb_ros(const geometry_msgs::PoseStamped &msg);
		void right_mapinulator_position_cb_ros(const geometry_msgs::PoseStamped &msg);
		void quaternion2euler(float *quaternion, float *euler);
		void getRotationTranslationMatrix(Eigen::Matrix4d &rotationTranslationMatrix, 
			float *orientationEuler, float *position);
		void getAnglesFromRotationTranslationMatrix(Eigen::Matrix4d &rotationTranslationMatrix,
			float *angles);
		void euler2quaternion(float *euler, float *quaternion);
		int joint_criterion_function(float *q1_in, float *q2_in, float *q3_in, float q1_old, float q2_old, float q3_old, float *q_out);
		void joint1_left_controller_state_cb_ros(const control_msgs::JointControllerState &msg);
		void joint2_left_controller_state_cb_ros(const control_msgs::JointControllerState &msg);
		void joint3_left_controller_state_cb_ros(const control_msgs::JointControllerState &msg);
		void joint1_right_controller_state_cb_ros(const control_msgs::JointControllerState &msg);
		void joint2_right_controller_state_cb_ros(const control_msgs::JointControllerState &msg);
		void joint3_right_controller_state_cb_ros(const control_msgs::JointControllerState &msg);

		DualArmManipulatorInverseKinematics manipulator_inverse;
		DualArmManipulatorDirectKinematics manipulator_direct;

		Eigen::Matrix4d T01_, T10_, Torigin_right_, Torigin_left_;
		Eigen::Matrix4d Torigin_right_inv_, Torigin_left_inv_;

		int rate_;
		float left_q1_meas_, left_q2_meas_, left_q3_meas_;
		float right_q1_meas_, right_q2_meas_, right_q3_meas_;

		ros::Subscriber left_manipulator_position_sub_ros_, right_manipulator_position_sub_ros_;
		ros::Subscriber joint1_right_state_sub_ros_, joint2_right_state_sub_ros_, joint3_right_state_sub_ros_;
		ros::Subscriber joint1_left_state_sub_ros_, joint2_left_state_sub_ros_, joint3_left_state_sub_ros_;
		ros::Publisher joint1_right_pub_ros_, joint2_right_pub_ros_, joint3_right_pub_ros_;
		ros::Publisher joint1_left_pub_ros_, joint2_left_pub_ros_, joint3_left_pub_ros_;
		ros::Publisher left_manipulator_position_pub_ros_, right_manipulator_position_pub_ros_;

		ros::NodeHandle n_;

};