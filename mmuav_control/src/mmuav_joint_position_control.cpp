#include <mmuav_control/mmuav_joint_position_control.h>
#include "yaml-cpp/yaml.h"
#include <mmuav_msgs/PIDController.h>
#include <ros/package.h>

#include <string>
#include <iostream>

JointControl::JointControl()
{
	ref_ = 0;
	config_start_ = false;
}

JointControl::~JointControl()
{
	delete dr_srv;
}

void JointControl::setReconfigure(ros::NodeHandle n)
{
	dr_srv = new dynamic_reconfigure::Server<mmuav_control::JointCtlParamsConfig>(n);

	cb = boost::bind(&JointControl::configCallback, this, _1, _2);
    dr_srv->setCallback(cb);
}

void JointControl::joint_ref_cb_ros(const std_msgs::Float64 &msg)
{
	ref_ = msg.data;
}

float JointControl::get_ref(void)
{
	return ref_;
}

void JointControl::set_kp(float kp)
{
	joint_pid_.set_kp(kp);
}

void JointControl::set_ki(float ki)
{
	joint_pid_.set_ki(ki);
}

void JointControl::set_kd(float kd)
{
	joint_pid_.set_kd(kd);
}

void JointControl::set_lim_low(float lim_low)
{
	joint_pid_.set_lim_low(lim_low);
}

void JointControl::set_lim_high(float lim_high)
{
	joint_pid_.set_lim_high(lim_high);
}

float JointControl::compute(float ref, float meas, float dt)
{
	return joint_pid_.compute(ref, meas, dt);
}

void JointControl::create_msg(mmuav_msgs::PIDController &msg)
{
	joint_pid_.create_msg(msg);
}

void JointControl::configCallback(mmuav_control::JointCtlParamsConfig &config, uint32_t level)
{
    /* Callback for dynamically reconfigurable parameters (P,I,D gains for each controller)*/

    if (!config_start_)
    {
        // callback is called for the first time. Use this to set the new params to the config server
        
        config.kp = joint_pid_.get_kp();
        config.ki = joint_pid_.get_ki();
        config.kd = joint_pid_.get_kd();

        config_start_ = true;

        dr_srv->updateConfig(config);
    }
    else
    {
        // The following code just sets up the P,I,D gains for all controllers
        
        joint_pid_.set_kp(config.kp);
        joint_pid_.set_ki(config.ki);
        joint_pid_.set_kd(config.kd);
    }
}



JointPositionControl::JointPositionControl(void)
{
	rate_ = 1000;
	sampling_reduction_ = 10;
	is_init_ = false;

	joint_states_sub_ros_ = n_.subscribe("joint_states", 1, &JointPositionControl::joint_state_cb_ros, this);
	clock_ros_sub_ = n_.subscribe("/clock", 1, &JointPositionControl::clock_cb, this);
}

JointPositionControl::~JointPositionControl(void)
{
	delete[] joint_control_;
}

void JointPositionControl::joint_state_cb_ros(const sensor_msgs::JointState &msg)
{
	int i, k;

	for (i = 0; i < msg.name.size(); i++)
	{
		for (k = 0; k < joint_name_.size(); k++)
		{
			if (msg.name[i] == joint_name_[k])
			{
				joint_meas_[k] = msg.position[i];
			}
		}
	}
}
void JointPositionControl::clock_cb(const rosgraph_msgs::Clock &msg)
{
    clock_ = msg;
}


void JointPositionControl::LoadParameters(std::string file, std::vector<std::string> &controllers)
{
	int i;
	YAML::Node config = YAML::LoadFile(file);
	joint_control_ = new JointControl[controllers.size()];
	std::string pub_topic;

	joint_name_.clear();
	joint_meas_.clear();

	for (i = 0; i < controllers.size(); i++)
	{
		pub_topic = config[controllers[i]]["command_topic"].as<std::string>();
		joint_name_.push_back(config[controllers[i]]["joint"].as<std::string>());
		joint_command_pub_ros_.push_back(n_.advertise<std_msgs::Float64>(pub_topic, 1));
		pid_state_pub_ros_.push_back(n_.advertise<mmuav_msgs::PIDController>(controllers[i]+"/state", 1));
		joint_control_[i].set_kp(config[controllers[i]]["pid"]["p"].as<double>());
		joint_control_[i].set_ki(config[controllers[i]]["pid"]["i"].as<double>());
		joint_control_[i].set_kd(config[controllers[i]]["pid"]["d"].as<double>());
		joint_control_[i].set_lim_high(config[controllers[i]]["pid"]["high_limit"].as<double>());
		joint_control_[i].set_lim_low(config[controllers[i]]["pid"]["low_limit"].as<double>());
		joint_meas_.push_back(0.0);
		joint_ref_ros_sub_.push_back(n_.subscribe(controllers[i] + "/command", 1, &JointControl::joint_ref_cb_ros, &joint_control_[i]));
		joint_control_[i].setReconfigure(ros::NodeHandle("~/"+controllers[i]));
	}

	is_init_ = true;
}

void JointPositionControl::run(void)
{
	ros::Rate loop_rate(rate_);
	int i;
	int counter = 1;
	float dt = 0, position_error;
	rosgraph_msgs::Clock clock_old;
	mmuav_msgs::PIDController pid_msg;
	std_msgs::Float64 velocity_ref;

	clock_old = clock_;

	while(ros::ok())
	{
		ros::spinOnce();

		if ((counter % sampling_reduction_) == 0)
		{
			dt = (clock_.clock.toSec() - clock_old.clock.toSec());
        	clock_old = clock_;

        	if (dt > 0.0)
        	{
        		for (i = 0; i < joint_name_.size(); i++)
        		{
        			position_error = joint_control_[i].get_ref() - joint_meas_[i];

        			if (fabs(position_error) > M_PI) 
        			{
                		if (position_error > 0)
                		{
                    		position_error = position_error - 2 * M_PI;
                		}
                		else 
                		{
                    		position_error = position_error + 2 * M_PI;
                		}
            		}
            		joint_meas_[i] = joint_control_[i].get_ref() - position_error;

        			velocity_ref.data = joint_control_[i].compute(joint_control_[i].get_ref(), joint_meas_[i], dt);

        			joint_control_[i].create_msg(pid_msg);
            		pid_state_pub_ros_[i].publish(pid_msg);

            		joint_command_pub_ros_[i].publish(velocity_ref);


        		}
        	}

        	counter = 0;

		}

		counter++;
		loop_rate.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joint_position_control_node");

	int i;

	ros::NodeHandle private_node_handle_("~");
	std::string path = ros::package::getPath("mmuav_control");
	std::string controller_params_file;

	if (argc < 2)
	{
		ROS_WARN("Missing argument controler name!");
		return 0;
	}

	std::vector<std::string> controllers;
	for (i = 1; i < argc; i++) controllers.push_back(std::string(argv[i])); 

	private_node_handle_.param("param_file", controller_params_file, std::string("/config/dual_arm_manipulator_position_control.yaml"));

	JointPositionControl joint_control;
	std::cout << path << controller_params_file << "\n";
	joint_control.LoadParameters(path+controller_params_file, controllers);
	joint_control.run();

	return 0;
}
