#include <mmuav_control/uav_impedance_control.h>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"
#include <mmuav_control/Tf1.h>

ImpedanceControl::ImpedanceControl(int rate, int moving_average_sample_number)
{
    int i;

    for (i = 0; i < MAX_MOVING_AVARAGE_SAMPLES_NUM; i++)
    {
        force_z_meas_[i] = 0;
        force_x_meas_[i] = 0;
        force_y_meas_[i] = 0;
        torque_x_meas_[i] = 0;
        torque_y_meas_[i] = 0;
        torque_z_meas_[i] = 0;
    }

    targetImpedanceType_ = 1;

    for (i = 0; i < 6; i++)
    {   
        M_[i] = 0;
        B_[i] = 0;
        K_[i] = 0;
        em0_[i] = 0;
        dem0_[i] = 0;
        wp_[i] = 0;
        wd_[i] = 0;
        fe_[i] = 0;
        a1_[i] = 0;
        b1_[i] = 0;
        c1_[i] = 0;
        a2_[i] = 0;
        b2_[i] = 0;
        c2_[i] = 0;
        sigma1_[i] = 0;
        sigma2_[i] = 0;
        sigma3_[i] = 0;
        kp0_[i] = 0;
        kd0_[i] = 0;
        omega_[i] = 0;
        zeta_[i] = 0;
    }

	start_flag_ = false;
    force_sensor_calibration_flag_ = false;
	moving_average_sample_number_ = moving_average_sample_number;
	rate_ = rate;

	force_ros_sub_ = n_.subscribe("/force_sensor/ft_sensor", 1, &ImpedanceControl::force_measurement_cb, this);
    pose_ref_ros_sub_ = n_.subscribe("impedance_control/pose_ref", 1, &ImpedanceControl::pose_ref_cb, this);
    force_torque_ref_ros_sub_ = n_.subscribe("impedance_control/force_torque_ref", 1, &ImpedanceControl::force_torque_cb, this);
	clock_ros_sub_ = n_.subscribe("/clock", 1, &ImpedanceControl::clock_cb, this);

	force_filtered_pub_ = n_.advertise<geometry_msgs::WrenchStamped>("/force_sensor/filtered_ft_sensor", 1);
    position_commanded_pub_ = n_.advertise<geometry_msgs::PoseStamped>("dual_arm_manipulator/set_point", 1);
    yaw_commanded_pub_ = n_.advertise<std_msgs::Float64>("position_control/yaw_ref", 1);

    force_z_offset_ = 0.0;
    force_x_offset_ = 0.0;
    force_y_offset_ = 0.0;
    torque_x_offset_ = 0.0;
    torque_y_offset_ = 0.0;
    torque_z_offset_ = 0.0;

    impact_flag_ = false;
    collision_ = false;

}

void ImpedanceControl::setImpedanceFilterMass(float *mass)
{
    for (int i = 0; i < 6; i++) M_[i] = mass[i];
}

void ImpedanceControl::setImpedanceFilterDamping(float *damping)
{
    for (int i = 0; i < 6; i++) B_[i] = damping[i];
}

void ImpedanceControl::setImpedanceFilterStiffness(float *stiffness)
{
    for (int i = 0; i < 6; i++) K_[i] = stiffness[i];
}

void ImpedanceControl::initializeMRACControl(void)
{   
    float samplingTime, a[2], b[2], c[2], sigma[3];
    int i;

    samplingTime = (float)moving_average_sample_number_/(float)rate_;

    for (i = 0; i < 6; i++)
    {   
        a[0] = a1_[i];
        a[1] = a2_[i];
        b[0] = b1_[i];
        b[1] = b2_[i];
        c[0] = c1_[i];
        c[1] = c2_[i];
        sigma[0] = sigma1_[i];
        sigma[1] = sigma2_[i];
        sigma[2] = sigma3_[i];

        mraic_[i].initializeReferenceModel(zeta_[i], omega_[i]);
        mraic_[i].initializeAdaptationLaws(a, b, c, sigma, samplingTime);
        mraic_[i].setWeightingFactors(wp_[i], wd_[i]);
    }
}

void ImpedanceControl::initializeImpedanceFilterTransferFunction(void)
{
    float samplingTime;
    int i;

    samplingTime = (float)moving_average_sample_number_/(float)rate_;

    if (targetImpedanceType_ == 1)
    {
        for (i = 0; i < 6; i++)
        {
            Ge_[i].reset();
            Ge_[i].setNumerator(1.0, 0.0, 0.0);
            Ge_[i].setDenominator(K_[i], B_[i], M_[i]);
            Ge_[i].c2d(samplingTime, "zoh");
        
            Gxr_[i].reset();
            Gxr_[i].setNumerator(K_[i], 0.0, 0.0);
            Gxr_[i].setDenominator(K_[i], B_[i], M_[i]);
            Gxr_[i].c2d(samplingTime, "zoh");
        }
    }
    else if (targetImpedanceType_ == 2)
    {
        for (i = 0; i < 6; i++)
        {
            Ge_[i].reset();
            Ge_[i].setNumerator(1.0, 0.0, 0.0);
            Ge_[i].setDenominator(K_[i], B_[i], M_[i]);
            Ge_[i].c2d(samplingTime, "zoh");

            Gxr_[i].reset();
            Gxr_[i].setNumerator(K_[i], B_[i], 0.0);
            Gxr_[i].setDenominator(K_[i], B_[i], M_[i]);
            Gxr_[i].c2d(samplingTime, "zoh");
        }

    }
    else if (targetImpedanceType_ == 3)
    {
        for (i = 0; i < 6; i++)
        {
            Ge_[i].reset();
            Ge_[i].setNumerator(1.0, 0.0, 0.0);
            Ge_[i].setDenominator(K_[i], B_[i], M_[i]);
            Ge_[i].c2d(samplingTime, "zoh");

            Gxr_[i].reset();
            Gxr_[i].setNumerator(K_[i], B_[i], M_[i]);
            Gxr_[i].setDenominator(K_[i], B_[i], M_[i]);
            Gxr_[i].c2d(samplingTime, "zoh");
        }
    }
}

float* ImpedanceControl::impedanceFilter(float *e, float *Xr)
{
    float *Xc = (float * )malloc(sizeof(float)*6);
    int i;

    for (i = 0; i < 6; i++)
    {
        Xc[i] = Ge_[i].getDiscreteOutput(e[i]) + Gxr_[i].getDiscreteOutput(Xr[i]);
    }

    return Xc;
}

float* ImpedanceControl::modelReferenceAdaptiveImpedanceControl(float dt, float *e, float *g0)
{
    float *Xr = (float * )malloc(sizeof(float)*6);
    bool impact = false;
    int i;

    impact = check_impact();

    for (i = 0; i < 6; i++)
    {
        mraic_[i].setAdaptiveParameterInitialValues(g0[i], kp0_[i], kd0_[i]);
        mraic_[i].setImpact(impact);
        Xr[i] = mraic_[i].compute(dt, e[i]);
    }

    return Xr;
}

void ImpedanceControl::setTargetImpedanceType(int type)
{
    if (type < 1 || type > 3) targetImpedanceType_ = 1;
    else targetImpedanceType_ = type;
}

void ImpedanceControl::force_measurement_cb(const geometry_msgs::WrenchStamped &msg)
{
	if (!start_flag_) start_flag_ = true;

	for (int i=0; i<(moving_average_sample_number_-1); i++)
	{
        force_z_meas_[i] = force_z_meas_[i+1];
        force_x_meas_[i] = force_x_meas_[i+1];
        force_y_meas_[i] = force_y_meas_[i+1];
        torque_x_meas_[i] = torque_x_meas_[i+1];
        torque_y_meas_[i] = torque_y_meas_[i+1];
        torque_z_meas_[i] = torque_z_meas_[i+1];
    }

	force_z_meas_[moving_average_sample_number_-1] = msg.wrench.force.z;
    force_x_meas_[moving_average_sample_number_-1] = msg.wrench.force.x;
    force_y_meas_[moving_average_sample_number_-1] = msg.wrench.force.y;
    torque_x_meas_[moving_average_sample_number_-1] = msg.wrench.torque.x;
    torque_y_meas_[moving_average_sample_number_-1] = msg.wrench.torque.y;
    torque_z_meas_[moving_average_sample_number_-1] = msg.wrench.torque.z;
}

void ImpedanceControl::pose_ref_cb(const geometry_msgs::PoseStamped &msg)
{
    float q[4], euler[3];

    q[0] = msg.pose.orientation.w;
    q[1] = msg.pose.orientation.x;
    q[2] = msg.pose.orientation.y;
    q[3] = msg.pose.orientation.z;

    quaternion2euler(q, euler);

    yaw_ref_.data = euler[2];

    pose_ref_ = msg;
}

void ImpedanceControl::force_torque_cb(const geometry_msgs::WrenchStamped &msg)
{
    force_torque_ref_ = msg;
}

bool ImpedanceControl::check_collision(void)
{
    bool collision;

    if (getFilteredForceZ() > 0.05) collision = true;
    else collision = false;

    return collision;
}

bool ImpedanceControl::check_impact(void)
{
    if (!collision_ && check_collision()) impact_flag_ = true;
    else impact_flag_ = false;

    collision_ = check_collision();

	return impact_flag_;
}

void ImpedanceControl::clock_cb(const rosgraph_msgs::Clock &msg)
{
    clock_ = msg;
}

float ImpedanceControl::getFilteredForceX(void)
{
    float sum = 0;
    float average;

    for (int i = 0; i<moving_average_sample_number_; i++)
        sum += force_x_meas_[i];

    average = sum/moving_average_sample_number_ - force_x_offset_;

    return average;
}

float ImpedanceControl::getFilteredForceY(void)
{
    float sum = 0;
    float average;

    for (int i = 0; i<moving_average_sample_number_; i++)
        sum += force_y_meas_[i];

    average = sum/moving_average_sample_number_ - force_y_offset_;

    return average;
}

float ImpedanceControl::getFilteredForceZ(void)
{
	float sum = 0;
	float average;

	for (int i = 0; i<moving_average_sample_number_; i++)
		sum += force_z_meas_[i];

	average = sum/moving_average_sample_number_ - force_z_offset_;

	return average;
}

float ImpedanceControl::getFilteredTorqueX(void)
{
    float sum = 0;
    float average;

    for (int i = 0; i<moving_average_sample_number_; i++)
        sum += torque_x_meas_[i];

    average = sum/moving_average_sample_number_ - torque_x_offset_;

    return average;
}

float ImpedanceControl::getFilteredTorqueY(void)
{
    float sum = 0;
    float average;

    for (int i = 0; i<moving_average_sample_number_; i++)
        sum += torque_y_meas_[i];

    average = sum/moving_average_sample_number_ - torque_y_offset_;

    return average;
}

float ImpedanceControl::getFilteredTorqueZ(void)
{
    float sum = 0;
    float average;

    for (int i = 0; i<moving_average_sample_number_; i++)
        sum += torque_z_meas_[i];

    average = sum/moving_average_sample_number_ - torque_z_offset_;

    return average;
}

void ImpedanceControl::run()
{
	float dt = 0;
    float *xc, *xr;
    float vector_pose_ref[6] = {0};

	rosgraph_msgs::Clock clock_old;
	geometry_msgs::WrenchStamped filtered_ft_sensor_msg;
    geometry_msgs::PoseStamped commanded_position_msg;
    std_msgs::Float64 commanded_yaw_msg;

	int counter = 1;
    int calibration_counter = 0;
    int force_sensor_init_counter = 0;
    float force_z_sum = 0;
    float force_x_sum = 0;
    float force_y_sum = 0;
    float torque_x_sum = 0;
    float torque_y_sum = 0;
    float torque_z_sum = 0;

    ros::Rate loop_rate(rate_);

    while (ros::Time::now().toSec() == 0 && ros::ok())
    {
        ROS_INFO("Waiting for clock server to start");
    }

    ROS_INFO("Received first clock message");

    while (!start_flag_ && ros::ok())
    {
        ros::spinOnce();

        ROS_INFO("Waiting for the first measurement.");
        ros::Duration(0.5).sleep();
    }

    ROS_INFO("Calibrating force sensor.");

    while (!force_sensor_calibration_flag_ && ros::ok())
    {
        ros::spinOnce();

        if ((counter % moving_average_sample_number_) == 0)
        {
            force_sensor_init_counter++;

            if (force_sensor_init_counter > 100)
            {
                force_z_sum += getFilteredForceZ();
                force_x_sum += getFilteredForceX();
                force_y_sum += getFilteredForceY();
                torque_x_sum += getFilteredTorqueX();
                torque_y_sum += getFilteredTorqueY();
                torque_z_sum += getFilteredTorqueZ();

                calibration_counter++;
            }
        }

        if (calibration_counter >= 500)
        {
            force_z_offset_ = force_z_sum / calibration_counter;
            force_x_offset_ = force_x_sum / calibration_counter;
            force_y_offset_ = force_y_sum / calibration_counter;
            torque_x_offset_ = torque_x_sum / calibration_counter;
            torque_y_offset_ = torque_y_sum / calibration_counter;
            torque_z_offset_ = torque_z_sum / calibration_counter;
            force_sensor_calibration_flag_ = true;
        }

        counter++;

        loop_rate.sleep();
    }

    counter = 1;

    ROS_INFO("Starting impedance control.");

    clock_old = clock_;

    initializeImpedanceFilterTransferFunction();
    initializeMRACControl();

    while (ros::ok())
    {
        ros::spinOnce();

        if ((counter % moving_average_sample_number_) == 0)
        {
            dt = (clock_.clock.toSec() - clock_old.clock.toSec());
            clock_old = clock_;

        	if (dt > 0.0)
			{
                fe_[2] = -(force_torque_ref_.wrench.force.z - getFilteredForceZ()); //ide -e iz razloga jer je force senzor rotiran s obzirom na koordinatni letjlice
                fe_[3] = 0;//-(force_torque_ref_.wrench.torque.y - getFilteredTorqueY());
                fe_[4] = 0;//-(force_torque_ref_.wrench.torque.x - getFilteredTorqueX());
                fe_[5] = 0;//-(force_torque_ref_.wrench.torque.z - getFilteredTorqueZ());

                vector_pose_ref[0] = pose_ref_.pose.position.x;
                vector_pose_ref[1] = pose_ref_.pose.position.y;
                vector_pose_ref[2] = pose_ref_.pose.position.z;
                vector_pose_ref[3] = pose_ref_.pose.position.x;
                vector_pose_ref[4] = pose_ref_.pose.position.y;
                vector_pose_ref[5] = yaw_ref_.data;

                xr = modelReferenceAdaptiveImpedanceControl(dt, fe_, vector_pose_ref);

                xc = impedanceFilter(fe_, xr);

                commanded_position_msg.header.stamp = ros::Time::now();
                commanded_position_msg.pose.position.x = xc[3];
                commanded_position_msg.pose.position.y = xc[4];
                commanded_position_msg.pose.position.z = xc[2];
                commanded_position_msg.pose.orientation.x = 0;
                commanded_position_msg.pose.orientation.y = 0;
                commanded_position_msg.pose.orientation.z = 0.70711;
                commanded_position_msg.pose.orientation.w = 0.70711;
                position_commanded_pub_.publish(commanded_position_msg);

                commanded_yaw_msg.data = xc[5];
                yaw_commanded_pub_.publish(commanded_yaw_msg);

                filtered_ft_sensor_msg.header.stamp = ros::Time::now();
				filtered_ft_sensor_msg.wrench.force.z = getFilteredForceZ();
                filtered_ft_sensor_msg.wrench.force.y = getFilteredForceY();
                filtered_ft_sensor_msg.wrench.force.x = getFilteredForceX();
                filtered_ft_sensor_msg.wrench.torque.x = getFilteredTorqueX();
                filtered_ft_sensor_msg.wrench.torque.y = getFilteredTorqueY();
                filtered_ft_sensor_msg.wrench.torque.z = getFilteredTorqueZ();
				force_filtered_pub_.publish(filtered_ft_sensor_msg);

                free(xc);
                free(xr);
			}

        	counter = 0;
        }

        counter++;

        loop_rate.sleep();
    } 

}

void ImpedanceControl::quaternion2euler(float *quaternion, float *euler)
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

void ImpedanceControl::LoadImpedanceControlParameters(std::string file)
{
    YAML::Node config = YAML::LoadFile(file);
    std::vector<double> B, K, M, omega, zeta, wp, wd, a1, a2;
    std::vector<double> b1, b2, c1, c2, kp0, kd0, sigma1, sigma2, sigma3;

    M = config["IMPEDANCE_FILTER"]["M"].as<std::vector<double> >();
    B = config["IMPEDANCE_FILTER"]["B"].as<std::vector<double> >();
    K = config["IMPEDANCE_FILTER"]["K"].as<std::vector<double> >();

    omega = config["MRAC"]["REFERENCE_MODEL"]["OMEGA"].as<std::vector<double> >();
    zeta = config["MRAC"]["REFERENCE_MODEL"]["ZETA"].as<std::vector<double> >();
    wp = config["MRAC"]["WEIGHTING_FACTORS"]["Wp"].as<std::vector<double> >();
    wd = config["MRAC"]["WEIGHTING_FACTORS"]["Wd"].as<std::vector<double> >();
    a1 = config["MRAC"]["INTEGRAL_ADAPTATION_GAINS"]["a1"].as<std::vector<double> >();
    b1 = config["MRAC"]["INTEGRAL_ADAPTATION_GAINS"]["b1"].as<std::vector<double> >();
    c1 = config["MRAC"]["INTEGRAL_ADAPTATION_GAINS"]["c1"].as<std::vector<double> >(); 
    a2 = config["MRAC"]["PROPORTIONAL_ADAPTATION_GAINS"]["a2"].as<std::vector<double> >();
    b2 = config["MRAC"]["PROPORTIONAL_ADAPTATION_GAINS"]["b2"].as<std::vector<double> >();
    c2 = config["MRAC"]["PROPORTIONAL_ADAPTATION_GAINS"]["c2"].as<std::vector<double> >(); 
    sigma1 = config["MRAC"]["SIGMA_MODIFICATION_GAINS"]["sigma1"].as<std::vector<double> >();
    sigma2 = config["MRAC"]["SIGMA_MODIFICATION_GAINS"]["sigma2"].as<std::vector<double> >();
    sigma3 = config["MRAC"]["SIGMA_MODIFICATION_GAINS"]["sigma3"].as<std::vector<double> >(); 
    kp0 = config["MRAC"]["INITIAL_GAINS"]["Kp"].as<std::vector<double> >();
    kd0 = config["MRAC"]["INITIAL_GAINS"]["Kd"].as<std::vector<double> >(); 

    for (int i=0; i<6; i++)
    {
        M_[i] = M[i];
        B_[i] = B[i];
        K_[i] = K[i];

        omega_[i] = omega[i];
        zeta_[i] = zeta[i]; 

        wp_[i] = wp[i];
        wd_[i] = wd[i];

        a1_[i] = a1[i];
        b1_[i] = b1[i];
        c1_[i] = c1[i];
        a2_[i] = a2[i];
        b2_[i] = b2[i];
        c2_[i] = c2[i];

        sigma1_[i] = sigma1[i];
        sigma2_[i] = sigma2[i];
        sigma3_[i] = sigma3[i];

        kp0_[i] = kp0[i];
        kd0_[i] = kd0[i];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "impedance_control_node");
    ros::NodeHandle private_node_handle_("~");

    int rate, impedanceType, masn;
    std::string impedance_control_config_file;

    std::string path = ros::package::getPath("mmuav_control");

    private_node_handle_.param("rate", rate, int(1000));
    private_node_handle_.param("moving_average_sample_number", masn, int(10));
    private_node_handle_.param("impedance_control_params_file", impedance_control_config_file, std::string("/config/impedance_control_params.yaml"));
    private_node_handle_.param("ImpedanceType", impedanceType, int(1));

    ImpedanceControl impedance_control(rate, masn);

    impedance_control.LoadImpedanceControlParameters(path+impedance_control_config_file);
    impedance_control.setTargetImpedanceType(impedanceType);

    impedance_control.run();

    return 0;
}