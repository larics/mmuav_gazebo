#include <mmuav_control/uav_position_control.h>


/*
    Class implements MAV position control (x, y, z). Two PIDs in cascade are
    used for each degree of freedom.
    Subscribes to:
        odometry                  - used to extract position and velocity of the vehicle
        trajectory                - used to receive trajectory for the vehicle
        position_ref              - used to set the position referent (useful for testing controllers)
    Publishes:
        euler_ref           - referent euler angles sent to attitude controller
        mot_vel_ref         - referent value for thrust in terms of motor velocity (rad/s)
        pid_x               - publishes PID-x data - referent value, measured value, P, I, D and total component (useful for tuning params)
        pid_vx              - publishes PID-vx data - referent value, measured value, P, I, D and total component (useful for tuning params)
        pid_y               - publishes PID-y data - referent value, measured value, P, I, D and total component (useful for tuning params)
        pid_vy              - publishes PID-vy data - referent value, measured value, P, I, D and total component (useful for tuning params)
        pid_z               - publishes PID-z data - referent value, measured value, P, I, D and total component (useful for tuning params)
        pid_vz              - publishes PID-vz data - referent value, measured value, P, I, D and total component (useful for tuning params)

    Dynamic reconfigure is used to set controllers param online.
*/

PositionControl::PositionControl(int rate)
{
    /*
    Initialization of the class.
    */

    start_flag_ = false;                                //# flag indicates if the first measurement is received
    config_start_ = false;                              // flag indicates if the config callback is called for the first time

    yaw_sp_ = 0;
    position_sp_.vector.x = 0;
    position_sp_.vector.y = 0;
    position_sp_.vector.z = 0;

    orientation_mv_[0] = 0;
    orientation_mv_[1] = 0;
    orientation_mv_[2] = 0;

    /*////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////*/
    //  Add your PID params here

    pid_x_.set_kp(1.0);
    pid_x_.set_ki(0.0);
    pid_x_.set_kd(0.0);

    pid_vx_.set_kp(0.3);
    pid_vx_.set_ki(0.015);
    pid_vx_.set_kd(0.005);
    pid_vx_.set_lim_high(0.524);
    pid_vx_.set_lim_low(-0.524);

    pid_y_.set_kp(1.0);
    pid_y_.set_ki(0.0);
    pid_y_.set_kd(0.0);

    pid_vy_.set_kp(0.3);
    pid_vy_.set_ki(0.015);
    pid_vy_.set_kd(0.005);
    pid_vy_.set_lim_high(0.524);
    pid_vy_.set_lim_low(-0.524);

    pid_z_.set_kp(1.0);
    pid_z_.set_ki(0.0);
    pid_z_.set_kd(0.0);

    pid_vz_.set_kp(250.0);
    pid_vz_.set_ki(50.0);
    pid_vz_.set_kd(0.0);
    pid_vz_.set_lim_high(1475);
    pid_vz_.set_lim_low(-1475);

    /*//////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////*/

    rate_ = rate;
    
    odometry_ros_sub_ = n_.subscribe("odometry", 1, &PositionControl::odometry_cb, this);
    yaw_ros_sub_ = n_.subscribe("position_control/yaw_ref", 1, &PositionControl::yaw_ref_cb, this);
    position_ref_ros_sub_ = n_.subscribe("position_control/position_ref", 1, &PositionControl::position_ref_cb, this);
    clock_ros_sub_ = n_.subscribe("/clock", 1, &PositionControl::clock_cb, this);
    //imu_ros_sub_ = n_.subscribe("imu", 1, &AttitudeControl::ahrs_cb, this);

    euler_ref_pub_ros_ = n_.advertise<geometry_msgs::Vector3>("euler_ref", 1);
    height_pub_ros_ = n_.advertise<std_msgs::Float64>("mot_vel_ref", 1);
    pid_x_pub_ros_ = n_.advertise<mmuav_msgs::PIDController>("pid_x", 1);
    pid_vx_pub_ros_ = n_.advertise<mmuav_msgs::PIDController>("pid_vx", 1);
    pid_y_pub_ros_ = n_.advertise<mmuav_msgs::PIDController>("pid_y", 1);
    pid_vy_pub_ros_ = n_.advertise<mmuav_msgs::PIDController>("pid_vy", 1);
    pid_z_pub_ros_ = n_.advertise<mmuav_msgs::PIDController>("pid_z", 1);
    pid_vz_pub_ros_ = n_.advertise<mmuav_msgs::PIDController>("pid_vz", 1);
    
    cb = boost::bind(&PositionControl::configCallback, this, _1, _2);
    dr_srv.setCallback(cb);
}


void PositionControl::run()
{ 
    /* Runs ROS node - computes PID algorithms for cascade attitude control. */
    float dt = 0;
    float vx_sv, vx_output;
    float vy_sv, vy_output;
    float vz_sv, vz_output;
    float mot_speed_hover;
    float temp_x, temp_y;

    rosgraph_msgs::Clock clock_old;
    geometry_msgs::Vector3 euler_ref;
    mmuav_msgs::PIDController pid_msg;
    std_msgs::Float64 height_ref;


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

    ROS_INFO("Starting position control.");

    t_old_ = ros::Time::now();
    clock_old = clock_;

    while (ros::ok())
    {
        ros::spinOnce();

        dt = (clock_.clock.toSec() - clock_old.clock.toSec());
        clock_old = clock_;

        if (dt > 0.0)
        {    
            // Roll
            vx_sv = pid_x_.compute(position_sp_.vector.x, position_mv_.vector.x, dt);
            // roll rate pid compute
            vx_output = pid_vx_.compute(vx_sv, velocity_mv_.vector.x, dt);

            // Pitch
            vy_sv = pid_y_.compute(position_sp_.vector.y, position_mv_.vector.y, dt);
            // pitch rate pid compute
            vy_output = pid_vy_.compute(vy_sv, velocity_mv_.vector.y, dt);
           
            // Yaw
            vz_sv = pid_z_.compute(position_sp_.vector.z, position_mv_.vector.z, dt);
            // yaw rate pid compute

            mot_speed_hover = sqrt(9.81*(3)/(8.54858e-06*4.0));
            vz_output = mot_speed_hover + pid_vz_.compute(vz_sv, velocity_mv_.vector.z, dt);

            // Publish attitude
            euler_ref.x = vy_output;//- sin(orientation_mv_[2])*vx_output;//sin(orientation_mv_[2])*vx_output - cos(orientation_mv_[2])*vy_output;
            euler_ref.y = vx_output;//cos(orientation_mv_[2])*vx_output + sin(orientation_mv_[2])*vy_output;
            euler_ref.z = yaw_sp_;
            //euler_ref_pub_ros_.publish(euler_ref);

            height_ref.data = vz_output;
            height_pub_ros_.publish(height_ref);

            // Publish PID data - could be usefule for tuning
            pid_x_.create_msg(pid_msg);
            pid_x_pub_ros_.publish(pid_msg);

            pid_vx_.create_msg(pid_msg);
            pid_vx_pub_ros_.publish(pid_msg);

            pid_y_.create_msg(pid_msg);
            pid_y_pub_ros_.publish(pid_msg);

            pid_vy_.create_msg(pid_msg);
            pid_vy_pub_ros_.publish(pid_msg);

            pid_z_.create_msg(pid_msg);
            pid_z_pub_ros_.publish(pid_msg);

            pid_vz_.create_msg(pid_msg);
            pid_vz_pub_ros_.publish(pid_msg);
        }

        loop_rate.sleep();
    }   
}


void PositionControl::odometry_cb(const nav_msgs::Odometry &msg)
{        
    /*        
    AHRS callback. Used to extract roll, pitch, yaw and their rates.
    We used the following order of rotation - 1)yaw, 2) pitch, 3) roll
        :param msg: Type sensor_msgs/Imu
    */

    float q[4];

    if (!start_flag_) start_flag_ = true;

    position_mv_.vector.x = msg.pose.pose.position.x;
    position_mv_.vector.y = msg.pose.pose.position.y;
    position_mv_.vector.z = msg.pose.pose.position.z;

    velocity_mv_.vector.x = msg.twist.twist.linear.x;
    velocity_mv_.vector.y = msg.twist.twist.linear.y;
    velocity_mv_.vector.z = msg.twist.twist.linear.z;

    q[0] = msg.pose.pose.orientation.w;
    q[1] = msg.pose.pose.orientation.x;
    q[2] = msg.pose.pose.orientation.y;
    q[3] = msg.pose.pose.orientation.z;

    quaternion2euler(q, orientation_mv_);

}

/*void PositionControl::ahrs_cb(const sensor_msgs::Imu &msg)
{   
}*/

void PositionControl::quaternion2euler(float *quaternion, float *euler)
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

void PositionControl::position_ref_cb(const geometry_msgs::Vector3Stamped &msg)
{
    /*
    Euler ref values callback.
       :param msg: Type Vector3 (x-roll, y-pitch, z-yaw)
    */

    position_sp_ = msg;
}

void PositionControl::yaw_ref_cb(const std_msgs::Float64 &msg)
{
    /*
    Euler ref values callback.
       :param msg: Type Vector3 (x-roll, y-pitch, z-yaw)
    */

    yaw_sp_ = msg.data;
}

void PositionControl::clock_cb(const rosgraph_msgs::Clock &msg)
{
    clock_ = msg;
}

void PositionControl::configCallback(mmuav_control::UavPositionCtlParamsConfig &config, uint32_t level)
{
    /* Callback for dynamically reconfigurable parameters (P,I,D gains for each controller)*/

    if (!config_start_)
    {
        // callback is called for the first time. Use this to set the new params to the config server
        
        config.x_kp = pid_x_.get_kp();
        config.x_ki = pid_x_.get_ki();
        config.x_kd = pid_x_.get_kd();

        config.vx_kp = pid_vx_.get_kp();
        config.vx_ki = pid_vx_.get_ki();
        config.vx_kd = pid_vx_.get_kd();

        config.y_kp = pid_y_.get_kp();
        config.y_ki = pid_y_.get_ki();
        config.y_kd = pid_y_.get_kd();

        config.vy_kp = pid_vy_.get_kp();
        config.vy_ki = pid_vy_.get_ki();
        config.vy_kd = pid_vy_.get_kd();

        config.z_kp = pid_z_.get_kp();
        config.z_ki = pid_z_.get_ki();
        config.z_kd = pid_z_.get_kd();

        config.vz_kp = pid_vz_.get_kp();
        config.vz_ki = pid_vz_.get_ki();
        config.vz_kd = pid_vz_.get_kd();

        config_start_ = true;

        dr_srv.updateConfig(config);
    }
    else
    {
        // The following code just sets up the P,I,D gains for all controllers
        
        pid_x_.set_kp(config.x_kp);
        pid_x_.set_ki(config.x_ki);
        pid_x_.set_kd(config.x_kd);

        pid_vx_.set_kp(config.vx_kp);
        pid_vx_.set_ki(config.vx_ki);
        pid_vx_.set_kd(config.vx_kd);

        pid_y_.set_kp(config.y_kp);
        pid_y_.set_ki(config.y_ki);
        pid_y_.set_kd(config.y_kd);

        pid_vy_.set_kp(config.vy_kp);
        pid_vy_.set_ki(config.vy_ki);
        pid_vy_.set_kd(config.vy_kd);

        pid_z_.set_kp(config.z_kp);
        pid_z_.set_ki(config.z_ki);
        pid_z_.set_kd(config.z_kd);

        pid_vz_.set_kp(config.vz_kp);
        pid_vz_.set_ki(config.vz_ki);
        pid_vz_.set_kd(config.vz_kd);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_control_node");
    ros::NodeHandle private_node_handle_("~");
    int rate;

    private_node_handle_.param("rate", rate, int(100));

    PositionControl position_control(rate);

    position_control.run();

    return 0;
}