#include <mmuav_control/dual_arm_manipulator_control.h>


DualArmManipulatorControl::DualArmManipulatorControl()
{
	T01_ << -1,  0,  0,  0,
		    0, -1,  0,  0,
		    0,  0,  1,  0,
		    0,  0,  0,  1;

	T10_ = T01_.inverse();

	rate_ = 100;

	left_q1_meas_ = 0;
	left_q2_meas_ = 0;
	left_q3_meas_ = 0;

	right_q1_meas_ = 0;
	right_q2_meas_ = 0;
	right_q3_meas_ = 0;

	left_manipulator_position_sub_ros_ = n_.subscribe("left_manipulator/set_point", 1, &DualArmManipulatorControl::left_mapinulator_position_cb_ros, this);
	right_manipulator_position_sub_ros_ = n_.subscribe("right_manipulator/set_point", 1, &DualArmManipulatorControl::right_mapinulator_position_cb_ros, this);
	
	joint1_left_state_sub_ros_ = n_.subscribe("joint1_left_position_controller/state", 1, &DualArmManipulatorControl::joint1_left_controller_state_cb_ros, this);
	joint2_left_state_sub_ros_ = n_.subscribe("joint2_left_position_controller/state", 1, &DualArmManipulatorControl::joint2_left_controller_state_cb_ros, this);
	joint3_left_state_sub_ros_ = n_.subscribe("joint3_left_position_controller/state", 1, &DualArmManipulatorControl::joint3_left_controller_state_cb_ros, this);
	joint1_right_state_sub_ros_ = n_.subscribe("joint1_right_position_controller/state", 1, &DualArmManipulatorControl::joint1_right_controller_state_cb_ros, this);
	joint2_right_state_sub_ros_ = n_.subscribe("joint2_right_position_controller/state", 1, &DualArmManipulatorControl::joint2_right_controller_state_cb_ros, this);
	joint3_right_state_sub_ros_ = n_.subscribe("joint3_right_position_controller/state", 1, &DualArmManipulatorControl::joint3_right_controller_state_cb_ros, this);

	mmuav_position_sub_ros_ = n_.subscribe("odometry", 1, &DualArmManipulatorControl::mmuav_position_cb_ros, this);

	joint1_right_pub_ros_ = n_.advertise<std_msgs::Float64>("joint1_right_position_controller/command", 1);
	joint2_right_pub_ros_ = n_.advertise<std_msgs::Float64>("joint2_right_position_controller/command", 1);
	joint3_right_pub_ros_ = n_.advertise<std_msgs::Float64>("joint3_right_position_controller/command", 1);

	joint1_left_pub_ros_ = n_.advertise<std_msgs::Float64>("joint1_left_position_controller/command", 1);
	joint2_left_pub_ros_ = n_.advertise<std_msgs::Float64>("joint2_left_position_controller/command", 1);
	joint3_left_pub_ros_ = n_.advertise<std_msgs::Float64>("joint3_left_position_controller/command", 1);

	left_manipulator_position_pub_ros_ = n_.advertise<geometry_msgs::PoseStamped>("left_manipulator/position", 1);
	right_manipulator_position_pub_ros_ = n_.advertise<geometry_msgs::PoseStamped>("right_manipulator/position", 1);

	uav_position_commanded_pub_ = n_.advertise<geometry_msgs::Vector3Stamped>("position_control/position_ref", 1);

	nesto = n_.advertise<geometry_msgs::Vector3Stamped>("nesto", 1);

	Tworld_uav_origin_ << 1, 0, 0, 0,
			  			  0, 1, 0, 0, 
			  			  0, 0, 1, 0,
			  			  0, 0, 0, 1;

	Tuav_origin_world_ << 1, 0, 0, 0,
			  			  0, 1, 0, 0, 
			  			  0, 0, 1, 0,
			  			  0, 0, 0, 1;

	Tworld_left_end_effector_ref_ << 1, 0, 0, 0,
			  						 0, 1, 0, 0, 
			  						 0, 0, 1, 0,
			  						 0, 0, 0, 1;

	Tworld_right_end_effector_ref_ << 1, 0, 0, 0,
			  						  0, 1, 0, 0, 
			  						  0, 0, 1, 0,
			  						  0, 0, 0, 1;

}

void DualArmManipulatorControl::LoadParameters(std::string file)
{
	YAML::Node config = YAML::LoadFile(file);
	std::vector<double> theta, a, left_arm_origin, right_arm_origin;
	DualArmManipulatorDirectKinematics::DH_Parameters_TypeDef dhParams;

	theta = config["theta"].as<std::vector<double> >();
	a = config["a"].as<std::vector<double> >();
	left_arm_origin = config["origin"]["left_arm"].as<std::vector<double> >();
	right_arm_origin = config["origin"]["right_arm"].as<std::vector<double> >();

	Tuav_origin_left0_ <<  cos(left_arm_origin[5])*cos(left_arm_origin[4]),  cos(left_arm_origin[5])*sin(left_arm_origin[4])*sin(left_arm_origin[3])-sin(left_arm_origin[5])*cos(left_arm_origin[3]),  cos(left_arm_origin[5])*sin(left_arm_origin[4])*cos(left_arm_origin[3])+sin(left_arm_origin[5])*sin(left_arm_origin[3]), left_arm_origin[0],
					 sin(left_arm_origin[5])*cos(left_arm_origin[4]),  sin(left_arm_origin[5])*sin(left_arm_origin[4])*sin(left_arm_origin[3])+cos(left_arm_origin[5])*cos(left_arm_origin[3]),  sin(left_arm_origin[5])*sin(left_arm_origin[4])*cos(left_arm_origin[3])-cos(left_arm_origin[5])*sin(left_arm_origin[3]), left_arm_origin[1],
					-sin(left_arm_origin[4]),                          cos(left_arm_origin[4])*sin(left_arm_origin[3]),                                      									 cos(left_arm_origin[4])*cos(left_arm_origin[3]),                   													  left_arm_origin[2],
					 0,                                                0,                                                																	     0,                           																							  1;
	Tuav_origin_right0_ <<  cos(right_arm_origin[5])*cos(right_arm_origin[4]),  cos(right_arm_origin[5])*sin(right_arm_origin[4])*sin(right_arm_origin[3])-sin(right_arm_origin[5])*cos(right_arm_origin[3]),  cos(right_arm_origin[5])*sin(right_arm_origin[4])*cos(right_arm_origin[3])+sin(right_arm_origin[5])*sin(right_arm_origin[3]), right_arm_origin[0],
					  sin(right_arm_origin[5])*cos(right_arm_origin[4]),  sin(right_arm_origin[5])*sin(right_arm_origin[4])*sin(right_arm_origin[3])+cos(right_arm_origin[5])*cos(right_arm_origin[3]),  sin(right_arm_origin[5])*sin(right_arm_origin[4])*cos(right_arm_origin[3])-cos(right_arm_origin[5])*sin(right_arm_origin[3]), right_arm_origin[1],
					 -sin(right_arm_origin[4]),                           cos(right_arm_origin[4])*sin(right_arm_origin[3]),                                      									     cos(right_arm_origin[4])*cos(right_arm_origin[3]),                   													       right_arm_origin[2],
					  0,                                                  0,                                                																	         0,                           																							       1;
	
	for (int i=0; i<3; i++)
	{
		dhParams.theta[i] = theta[i];
		dhParams.alpha[i] = 0;
		dhParams.d[i] = 0;
		dhParams.a[i] = a[i];
	}
	
	Tuav_origin_right0_inv_ = Tuav_origin_right0_.inverse();
	Tuav_origin_left0_inv_ = Tuav_origin_left0_.inverse();

	manipulator_direct.LoadParameters(file);
	manipulator_inverse.setDHparams(dhParams);

}

void DualArmManipulatorControl::initialiceUavPositionFilter(void)
{
	float samplingTime;
    int i;

    samplingTime = (float)1.0/(float)rate_;

    for (i = 0; i < 2; i++)
	{
	    Guav_e_[i].reset();
	    Guav_e_[i].setNumerator(1.0, 0.0, 0.0);
	    Guav_e_[i].setDenominator(0, 0, 0);
	    Guav_e_[i].c2d(samplingTime, "zoh");
	       
	    Guav_xr_[i].reset();
	    Guav_xr_[i].setNumerator(0, 0.0, 0.0);
	    Guav_xr_[i].setDenominator(0, 0, 0);
	    Guav_xr_[i].c2d(samplingTime, "zoh");
	}
}

float* DualArmManipulatorControl::uavPositionFilter(float *xr, float *e)
{
	float *Xc = (float * )malloc(sizeof(float)*2);
    int i;

    for (i = 0; i < 2; i++)
    {
		Xc[i] = Guav_e_[i].getDiscreteOutput(e[i]) + Guav_xr_[i].getDiscreteOutput(xr[i]);
	}

	return Xc;
}

void DualArmManipulatorControl::start()
{
	ros::Rate loop_rate(rate_);

	Eigen::Matrix4d T13_left_dk, T13_right_dk, T13_left_ref, T13_right_ref;
	Eigen::Matrix4d Tworld_end_effector_left_dk, Tworld_end_effector_right_dk;

	geometry_msgs::PoseStamped manipulator_pose;
	geometry_msgs::Vector3Stamped uav_commanded_position_msg;
	geometry_msgs::Vector3Stamped nesto_msg;
	std_msgs::Float64 joint_setpoint;

	float orientationEuler_left[3], orientationEuler_right[3];
	float orientationQuaternion_left[4], orientationQuaternion_right[4];
	float q1_left[2], q2_left[2], q3_left[2], q1_right[2], q2_right[2], q3_right[2];
	float Q_left[3], Q_right[3], *x_uav;
	bool left_manipulator_solution = false, right_manipulator_solution = true;

	while (ros::ok())
	{
		ros::spinOnce();

		//direct kinematics
		T13_left_dk = manipulator_direct.dk_calculate(left_q1_meas_,left_q2_meas_,left_q3_meas_);
		T13_right_dk = manipulator_direct.dk_calculate(right_q1_meas_,right_q2_meas_,right_q3_meas_);

		Tworld_end_effector_left_dk = Tworld_uav_origin_*Tuav_origin_left0_*T01_*T13_left_dk;
		Tworld_end_effector_right_dk = Tworld_uav_origin_*Tuav_origin_right0_*T01_*T13_left_dk;

		getAnglesFromRotationTranslationMatrix(Tworld_end_effector_left_dk, orientationEuler_left);
		euler2quaternion(orientationEuler_left, orientationQuaternion_left);
		getAnglesFromRotationTranslationMatrix(Tworld_end_effector_right_dk, orientationEuler_right);

		manipulator_pose.header.stamp = ros::Time::now();
		manipulator_pose.header.frame_id = "left_manipulator";
		manipulator_pose.pose.position.x = Tworld_end_effector_left_dk(0,3);
		manipulator_pose.pose.position.y = Tworld_end_effector_left_dk(1,3);
		manipulator_pose.pose.position.z = Tworld_end_effector_left_dk(2,3);
		manipulator_pose.pose.orientation.x = orientationQuaternion_left[1];
		manipulator_pose.pose.orientation.y = orientationQuaternion_left[2];
		manipulator_pose.pose.orientation.z = orientationQuaternion_left[3];
		manipulator_pose.pose.orientation.w = orientationQuaternion_left[0];
		left_manipulator_position_pub_ros_.publish(manipulator_pose);

		manipulator_pose.header.frame_id = "right_manipulator";
		manipulator_pose.pose.position.x = Tworld_end_effector_right_dk(0,3);
		manipulator_pose.pose.position.y = Tworld_end_effector_right_dk(1,3);
		manipulator_pose.pose.position.z = Tworld_end_effector_right_dk(2,3);
		manipulator_pose.pose.orientation.x = orientationQuaternion_right[1];
		manipulator_pose.pose.orientation.y = orientationQuaternion_right[2];
		manipulator_pose.pose.orientation.z = orientationQuaternion_right[3];
		manipulator_pose.pose.orientation.w = orientationQuaternion_right[0];
		right_manipulator_position_pub_ros_.publish(manipulator_pose);

		//inverse kinematics
		T13_left_ref = T10_ * Tuav_origin_left0_inv_ * Tuav_origin_world_ * Tworld_left_end_effector_ref_;
		T13_right_ref = T10_ * Tuav_origin_right0_inv_ * Tuav_origin_world_ * Tworld_right_end_effector_ref_;

		getAnglesFromRotationTranslationMatrix(T13_left_ref, orientationEuler_left);
		getAnglesFromRotationTranslationMatrix(T13_right_ref, orientationEuler_right);

		manipulator_inverse.ik_calculate(T13_left_ref(0,3), T13_left_ref(1,3), orientationEuler_left[2], q1_left, q2_left, q3_left);
		manipulator_inverse.ik_calculate(T13_right_ref(0,3), T13_right_ref(1,3), orientationEuler_right[2], q1_right, q2_right, q3_right);

		if (joint_criterion_function(q1_left, q2_left, q3_left, left_q1_meas_, left_q2_meas_, left_q3_meas_, Q_left)) left_manipulator_solution = true;
		else left_manipulator_solution = false;

		//if (joint_criterion_function(q1_right, q2_right, q3_right, right_q1_meas_, right_q2_meas_, right_q3_meas_, Q_right)) right_manipulator_solution = true;
		//else right_manipulator_solution = false;

		if (right_manipulator_solution && left_manipulator_solution)
		{
			joint_setpoint.data = Q_left[0];
			joint1_left_pub_ros_.publish(joint_setpoint);

			joint_setpoint.data = Q_left[1];
			joint2_left_pub_ros_.publish(joint_setpoint);

			joint_setpoint.data = Q_left[2];
			joint3_left_pub_ros_.publish(joint_setpoint);

			/*joint_setpoint.data = Q_right[0];
			joint1_right_pub_ros_.publish(joint_setpoint);

			joint_setpoint.data = Q_right[1];
			joint2_right_pub_ros_.publish(joint_setpoint);

			joint_setpoint.data = Q_right[2];
			joint3_right_pub_ros_.publish(joint_setpoint);*/
		}
		else
		{
			uav_commanded_position_msg.vector.x = Tworld_left_end_effector_ref_(0,3);
			uav_commanded_position_msg.vector.y = Tworld_left_end_effector_ref_(1,3);
		}

		//x_uav = uavPositionFilter();


		uav_commanded_position_msg.header.stamp = ros::Time::now();
        uav_commanded_position_msg.vector.z = Tworld_left_end_effector_ref_(2,3);
        uav_position_commanded_pub_.publish(uav_commanded_position_msg);

        nesto_msg.header.stamp = ros::Time::now();
        nesto_msg.vector.x = Tworld_left_end_effector_ref_(0,3);
        nesto.publish(nesto_msg);

        free(x_uav);

		loop_rate.sleep();
	}
}

void DualArmManipulatorControl::left_mapinulator_position_cb_ros(const geometry_msgs::PoseStamped &msg)
{
	float orientationEuler[3], position[3];
	float q[4];

	q[0] = msg.pose.orientation.w;
	q[1] = msg.pose.orientation.x;
	q[2] = msg.pose.orientation.y;
	q[3] = msg.pose.orientation.z;

	position[0] = msg.pose.position.x;
	position[1] = msg.pose.position.y;
	position[2] = msg.pose.position.z;

	quaternion2euler(q, orientationEuler);

	getRotationTranslationMatrix(Tworld_left_end_effector_ref_, orientationEuler, position);
}

void DualArmManipulatorControl::right_mapinulator_position_cb_ros(const geometry_msgs::PoseStamped &msg)
{
	std_msgs::Float64 joint_setpoint;
	float orientationEuler[3], position[3];
	float q[4], q1[2], q2[2], q3[2];
	float Q[3], yaw;
	Eigen::Matrix4d Tuav, T03;

	q[0] = msg.pose.orientation.w;
	q[1] = msg.pose.orientation.x;
	q[2] = msg.pose.orientation.y;
	q[3] = msg.pose.orientation.z;

	position[0] = msg.pose.position.x;
	position[1] = msg.pose.position.y;
	position[2] = msg.pose.position.z;

	quaternion2euler(q, orientationEuler);

	getRotationTranslationMatrix(Tworld_right_end_effector_ref_, orientationEuler, position);

	/*quaternion2euler(q, orientationEuler);
	yaw = orientationEuler[2];
	
	getRotationTranslationMatrix(Tuav, orientationEuler, position);

	T03 = T10_*Tuav_origin_right0_inv_*Tuav;

	getAnglesFromRotationTranslationMatrix(T03, orientationEuler);

	manipulator_inverse.ik_calculate(T03(0,3), T03(1,3), orientationEuler[2], q1, q2, q3);

	if (joint_criterion_function(q1, q2, q3, right_q1_meas_, right_q2_meas_, right_q3_meas_, Q))
	{
		joint_setpoint.data = Q[0];
		joint1_right_pub_ros_.publish(joint_setpoint);

		joint_setpoint.data = Q[1];
		joint2_right_pub_ros_.publish(joint_setpoint);

		joint_setpoint.data = Q[2];
		joint3_right_pub_ros_.publish(joint_setpoint);
	}
	else
	{
		ROS_WARN("RIGHT_MANIPULATOR: There is no solution for point x: %.2f, y: %.2f, yaw: %.2f", position[0], position[1], yaw);
	}*/

}

void DualArmManipulatorControl::mmuav_position_cb_ros(const nav_msgs::Odometry &msg)
{
	ros::Time t;
	ros::Duration dt;
	float position[3], q[4], orientationEuler[3];

	position[0] = msg.pose.pose.position.x;
	position[1] = msg.pose.pose.position.y;
	position[2] = msg.pose.pose.position.z;

	q[0] = msg.pose.pose.orientation.w;
    q[1] = msg.pose.pose.orientation.x;
    q[2] = msg.pose.pose.orientation.y;
    q[3] = msg.pose.pose.orientation.z;

    quaternion2euler(q, orientationEuler);

	getRotationTranslationMatrix(Tworld_uav_origin_, orientationEuler, position);

	Tuav_origin_world_ = Tworld_uav_origin_.inverse();
}

int DualArmManipulatorControl::joint_criterion_function(float *q1_in, float *q2_in, float *q3_in, float q1_old, float q2_old, float q3_old, float *q_out)
{
	float distance[2], temp;
	int temp2;

	distance[0] = 0;
	distance[1] = 0;

	if (std::isnan(q1_in[0]) || std::isnan(q2_in[0]) || std::isnan(q3_in[0])) return 0;
	if (std::isnan(q1_in[1]) || std::isnan(q2_in[1]) || std::isnan(q3_in[1])) return 0;



	for (int i=0; i<2; i++)
	{
		temp = fabs(atan2(sin(q1_in[i] - q1_old), cos(q1_in[i] - q1_old)));
		distance[i] += temp;

		temp = fabs(atan2(sin(q2_in[i] - q2_old), cos(q2_in[i] - q2_old)));
		distance[i] += temp;

		temp = fabs(atan2(sin(q3_in[i] - q3_old), cos(q3_in[i] - q3_old)));
		distance[i] += temp;
	}

	if (distance[0] > distance[1])
	{
		q_out[0] = q1_in[1];
		q_out[1] = q2_in[1];
		q_out[2] = q3_in[1];
	}
	else
	{
		q_out[0] = q1_in[0];
		q_out[1] = q2_in[0];
		q_out[2] = q3_in[0];
	}

	return 1;
}

void DualArmManipulatorControl::joint1_left_controller_state_cb_ros(const mmuav_msgs::PIDController &msg)
{
	left_q1_meas_ = msg.meas;
}

void DualArmManipulatorControl::joint2_left_controller_state_cb_ros(const mmuav_msgs::PIDController &msg)
{
	left_q2_meas_ = msg.meas;
}	

void DualArmManipulatorControl::joint3_left_controller_state_cb_ros(const mmuav_msgs::PIDController &msg)
{
	left_q3_meas_ = msg.meas;
}

void DualArmManipulatorControl::joint1_right_controller_state_cb_ros(const mmuav_msgs::PIDController &msg)
{
	right_q1_meas_ = msg.meas;
}

void DualArmManipulatorControl::joint2_right_controller_state_cb_ros(const mmuav_msgs::PIDController &msg)
{
	right_q2_meas_ = msg.meas;
}

void DualArmManipulatorControl::joint3_right_controller_state_cb_ros(const mmuav_msgs::PIDController &msg)
{
	right_q3_meas_ = msg.meas;
}	

void DualArmManipulatorControl::quaternion2euler(float *quaternion, float *euler)
{
  euler[0] = atan2(2 * (quaternion[0] * quaternion[1] + 
    quaternion[2] * quaternion[3]), 1 - 2 * (quaternion[1] * quaternion[1]
    + quaternion[2] * quaternion[2]));

  euler[1] = asin(2 * (quaternion[0] * quaternion[2] -
    quaternion[3] * quaternion[1]));

  euler[2] = atan2(2 * (quaternion[0]*quaternion[3] +
    quaternion[1]*quaternion[2]), 1 - 2 * (quaternion[2]*quaternion[2] +
    quaternion[3] * quaternion[3]));
}

void DualArmManipulatorControl::euler2quaternion(float *euler, float *quaternion)
{
	float cy = cos(euler[2] * 0.5);
	float sy = sin(euler[2] * 0.5);
	float cr = cos(euler[0] * 0.5);
	float sr = sin(euler[0] * 0.5);
	float cp = cos(euler[1] * 0.5);
	float sp = sin(euler[1] * 0.5);

	quaternion[0] = cy * cr * cp + sy * sr * sp; //w
	quaternion[1] = cy * sr * cp - sy * cr * sp; //x
	quaternion[2] = cy * cr * sp + sy * sr * cp; //y
	quaternion[3] = sy * cr * cp - cy * sr * sp; //z
}

void DualArmManipulatorControl::getRotationTranslationMatrix(Eigen::Matrix4d &rotationTranslationMatrix,
  float *orientationEuler, float *position)
{
  float r11, r12, r13, t1, r21, r22, r23, t2;
  float r31, r32, r33, t3;

  float x, y, z;

  x = orientationEuler[0];
  y = orientationEuler[1];
  z = orientationEuler[2];

  r11 = cos(y)*cos(z);

  r12 = cos(z)*sin(x)*sin(y) - cos(x)*sin(z);

  r13 = sin(x)*sin(z) + cos(x)*cos(z)*sin(y);

  r21 = cos(y)*sin(z);

  r22 = cos(x)*cos(z) + sin(x)*sin(y)*sin(z);

  r23 = cos(x)*sin(y)*sin(z) - cos(z)*sin(x);

  r31 = -sin(y);

  r32 = cos(y)*sin(x);

  r33 = cos(x)*cos(y);

  t1 = position[0];
  t2 = position[1];
  t3 = position[2];


  rotationTranslationMatrix << 
    r11, r12, r13, t1,
    r21, r22, r23, t2,
    r31, r32, r33, t3,
    0,   0,   0,   1;
}

void DualArmManipulatorControl::getAnglesFromRotationTranslationMatrix(Eigen::Matrix4d &rotationTranslationMatrix,
 float *angles)
{
  double r11, r21, r31, r32, r33;
  double roll, pitch, yaw;

  r11 = rotationTranslationMatrix(0,0);
  r21 = rotationTranslationMatrix(1,0);
  r31 = rotationTranslationMatrix(2,0);
  r32 = rotationTranslationMatrix(2,1);
  r33 = rotationTranslationMatrix(2,2);

  roll = atan2(r32, r33);
  pitch = atan2(-r31, sqrt(r32*r32 + r33*r33));
  yaw = atan2(r21, r11);

  angles[0] = roll;
  angles[1] = pitch;
  angles[2] = yaw;
}

void DualArmManipulatorControl::set_rate(int rate)
{
	rate_ = rate;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "inverse_kinematics_node");
	ros::NodeHandle private_node_handle_("~");
	ros::NodeHandle n;
	int rate;

	std::string path = ros::package::getPath("mmuav_control");
	std::string dh_parameters_file;

	DualArmManipulatorControl dam_control;

	private_node_handle_.param("dh_parameters_file", dh_parameters_file, std::string("/config/dual_arm_manipulator_dh_parameters.yaml"));
	private_node_handle_.param("rate", rate, int(50));

	dam_control.LoadParameters(path+dh_parameters_file);
	dam_control.set_rate(rate);
	dam_control.start();

	return 0;
}