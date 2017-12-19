#include <mmuav_control/uav_force_control.h>

ForceControl::ForceControl(int rate)
{
	force_z_meas[10] = {0};

	start_flag_ = false;
	moving_average_sample_number_ = 10;
	rate_ = rate;

	force_ros_sub_ = n_.subscribe("/force_sensor/ft_sensor", 1, &ForceControl::force_measurement_cb, this);
	clock_ros_sub_ = n_.subscribe("/clock", 1, &ForceControl::clock_cb, this);

	force_filtered_pub_ = n_.advertise<geometry_msgs::WrenchStamped>("/force_sensor/filtered_ft_sensor", 1);
	d_force_filtered_pub_ = n_.advertise<geometry_msgs::WrenchStamped>("/force_sensor/d_filtered_ft_sensor", 1);
}

void ForceControl::force_measurement_cb(const geometry_msgs::WrenchStamped &msg)
{
	if (!start_flag_) start_flag_ = true;

	for (int i=0; i<(moving_average_sample_number_-1); i++)
		force_z_meas[i] = force_z_meas[i+1];

	force_z_meas[moving_average_sample_number_-1] = msg.wrench.force.z;
}

bool ForceControl::check_impact(void)
{
	return true;
}

void ForceControl::clock_cb(const rosgraph_msgs::Clock &msg)
{
    clock_ = msg;
}

float ForceControl::getFilteredForceZ(void)
{
	float sum = 0;
	float average;

	for (int i = 0; i<moving_average_sample_number_; i++)
		sum += force_z_meas[i];

	average = sum/moving_average_sample_number_;

	return average;
}

void ForceControl::run()
{
	float dt = 0;
	rosgraph_msgs::Clock clock_old;
	geometry_msgs::WrenchStamped filtered_ft_sensor_msg;
	geometry_msgs::WrenchStamped d_filtered_ft_sensor_msg;
	int counter = 1;

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

    ROS_INFO("Starting force control.");

    clock_old = clock_;

    while (ros::ok())
    {
        ros::spinOnce();

        dt = (clock_.clock.toSec() - clock_old.clock.toSec());
        clock_old = clock_;

        if ((counter % moving_average_sample_number_) == 0)
        {
        	if (dt > 0.0)
			{
				filtered_ft_sensor_msg.header.stamp = ros::Time::now();
				filtered_ft_sensor_msg.wrench.force.z = getFilteredForceZ();

				force_filtered_pub_.publish(filtered_ft_sensor_msg);
			}

        	counter = 0;
        }

        d_filtered_ft_sensor_msg.header.stamp = ros::Time::now();
		d_filtered_ft_sensor_msg.wrench.force.z = (force_z_meas[moving_average_sample_number_-1] - filtered_ft_sensor_msg.wrench.force.z)/dt;
		d_force_filtered_pub_.publish(d_filtered_ft_sensor_msg);

        counter++;

        loop_rate.sleep();
    } 

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_control_node");
    ros::NodeHandle private_node_handle_("~");
    int rate;

    private_node_handle_.param("rate", rate, int(1000));

    ForceControl force_control(rate);

    force_control.run();

    return 0;
}