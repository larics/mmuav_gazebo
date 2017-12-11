#include <mmuav_control/uav_attitude_control.h>


/*
    Class implements MAV attitude control (roll, pitch, yaw). Two PIDs in cascade are
    used for each degree of freedom.
    Subscribes to:
        imu                - used to extract attitude and attitude rate of the vehicle
        mot_vel_ref        - used to receive referent motor velocity from the height controller
        euler_ref          - used to set the attitude referent (useful for testing controllers)
    Publishes:
        attitude_command   - referent motor velocities sent to each motor controller
        pid_roll           - publishes PID-roll data - referent value, measured value, P, I, D and total component (useful for tuning params)
        pid_roll_rate      - publishes PID-roll_rate data - referent value, measured value, P, I, D and total component (useful for tuning params)
        pid_pitch          - publishes PID-pitch data - referent value, measured value, P, I, D and total component (useful for tuning params)
        pid_pitch_rate     - publishes PID-pitch_rate data - referent value, measured value, P, I, D and total component (useful for tuning params)
        pid_yaw            - publishes PID-yaw data - referent value, measured value, P, I, D and total component (useful for tuning params)
        pid_yaw_rate       - publishes PID-yaw_rate data - referent value, measured value, P, I, D and total component (useful for tuning params)

    Dynamic reconfigure is used to set controllers param online.
*/

AttitudeControl::AttitudeControl(int rate)
{
    /*
    Initialization of the class.
    */

    start_flag_ = false;                                //# flag indicates if the first measurement is received
    config_start_ = false;                              // flag indicates if the config callback is called for the first time

    /*////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////*/
    //  Add your PID params here

    pid_roll_.set_kp(14.0);
    pid_roll_.set_ki(1.0);
    pid_roll_.set_kd(0.0);

    pid_roll_rate_.set_kp(200.0);
    pid_roll_rate_.set_ki(80.0);
    pid_roll_rate_.set_kd(0.1);
    pid_roll_rate_.set_lim_high(1475);
    pid_roll_rate_.set_lim_low(-1475);

    pid_pitch_.set_kp(14.0);
    pid_pitch_.set_ki(1.0);
    pid_pitch_.set_kd(0.0);

    pid_pitch_rate_.set_kp(200.0);
    pid_pitch_rate_.set_ki(80.0);
    pid_pitch_rate_.set_kd(0.1);
    pid_pitch_rate_.set_lim_high(1475);
    pid_pitch_rate_.set_lim_low(-1475);

    pid_yaw_.set_kp(8.0);
    pid_yaw_.set_ki(1.0);
    pid_yaw_.set_kd(0.0);

    pid_yaw_rate_.set_kp(100.0);
    pid_yaw_rate_.set_ki(20.0);
    pid_yaw_rate_.set_kd(0.0);
    pid_yaw_rate_.set_lim_high(1475);
    pid_yaw_rate_.set_lim_low(-1475);

    /*//////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////*/

    rate_ = rate;
    
    imu_ros_sub_ = n_.subscribe("imu", 1, &AttitudeControl::ahrs_cb, this);
    euler_ref_ros_sub_ = n_.subscribe("euler_ref", 1, &AttitudeControl::euler_ref_cb, this);
    clock_ros_sub_ = n_.subscribe("/clock", 1, &AttitudeControl::clock_cb, this);

    attitude_pub_ros_ = n_.advertise<geometry_msgs::Vector3Stamped>("attitude_command", 1);
    pid_roll_pub_ros_ = n_.advertise<mmuav_msgs::PIDController>("pid_roll", 1);
    pid_roll_rate_pub_ros_ = n_.advertise<mmuav_msgs::PIDController>("pid_roll_rate", 1);
    pid_pitch_pub_ros_ = n_.advertise<mmuav_msgs::PIDController>("pid_pitch", 1);
    pid_pitch_rate_pub_ros_ = n_.advertise<mmuav_msgs::PIDController>("pid_pitch_rate", 1);
    pid_yaw_pub_ros_ = n_.advertise<mmuav_msgs::PIDController>("pid_yaw", 1);
    pid_yaw_rate_pub_ros_ = n_.advertise<mmuav_msgs::PIDController>("pid_yaw_rate", 1);
    
    cb = boost::bind(&AttitudeControl::configCallback, this, _1, _2);
    dr_srv.setCallback(cb);
}


void AttitudeControl::run()
{ 
    /* Runs ROS node - computes PID algorithms for cascade attitude control. */
    float dt = 0;
    float roll_rate_sv, roll_rate_output;
    float pitch_rate_sv, pitch_rate_output;
    float yaw_rate_sv, yaw_rate_output;

    rosgraph_msgs::Clock clock_old;
    geometry_msgs::Vector3Stamped attitude_output;
    mmuav_msgs::PIDController pid_msg;


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

    ROS_INFO("Starting attitude control.");

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
            roll_rate_sv = pid_roll_.compute(euler_sp_.x, euler_mv_.x, dt);
            // roll rate pid compute
            roll_rate_output = pid_roll_rate_.compute(roll_rate_sv, euler_rate_mv_.x, dt);

            // Pitch
            pitch_rate_sv = pid_pitch_.compute(euler_sp_.y, euler_mv_.y, dt);
            // pitch rate pid compute
            pitch_rate_output = pid_pitch_rate_.compute(pitch_rate_sv, euler_rate_mv_.y, dt);
           
            // Yaw
            yaw_rate_sv = pid_yaw_.compute(euler_sp_.z, euler_mv_.z, dt);
            // yaw rate pid compute
            yaw_rate_output = pid_yaw_rate_.compute(yaw_rate_sv, euler_rate_mv_.z, dt);

            // Publish attitude
            attitude_output.vector.x = roll_rate_output;
            attitude_output.vector.y = pitch_rate_output;
            attitude_output.vector.z = yaw_rate_output;
            attitude_output.header.stamp = ros::Time::now();
            attitude_pub_ros_.publish(attitude_output);

            // Publish PID data - could be usefule for tuning
            pid_roll_.create_msg(pid_msg);
            pid_roll_pub_ros_.publish(pid_msg);

            pid_roll_rate_.create_msg(pid_msg);
            pid_roll_rate_pub_ros_.publish(pid_msg);

            pid_pitch_.create_msg(pid_msg);
            pid_pitch_pub_ros_.publish(pid_msg);

            pid_pitch_rate_.create_msg(pid_msg);
            pid_pitch_rate_pub_ros_.publish(pid_msg);

            pid_yaw_.create_msg(pid_msg);
            pid_yaw_pub_ros_.publish(pid_msg);

            pid_yaw_rate_.create_msg(pid_msg);
            pid_yaw_rate_pub_ros_.publish(pid_msg);
        }

        loop_rate.sleep();
    }   
}


void AttitudeControl::ahrs_cb(const sensor_msgs::Imu &msg)
{        
    /*        
    AHRS callback. Used to extract roll, pitch, yaw and their rates.
    We used the following order of rotation - 1)yaw, 2) pitch, 3) roll
        :param msg: Type sensor_msgs/Imu
    */

    float quaternion[4], euler[3];
    float p, q, r, sx, cx, cy, ty;

    if (!start_flag_) start_flag_ = true;

    quaternion[1] = msg.orientation.x;
    quaternion[2] = msg.orientation.y;
    quaternion[3] = msg.orientation.z;
    quaternion[0] = msg.orientation.w;

    quaternion2euler(quaternion, euler);
    euler_mv_.x = euler[0];
    euler_mv_.y = euler[1];
    euler_mv_.z = euler[2];

    // gyro measurements (p,q,r)
    p = msg.angular_velocity.x;
    q = msg.angular_velocity.y;
    r = msg.angular_velocity.z;

    sx = sin(euler_mv_.x);          // sin(roll)
    cx = cos(euler_mv_.x);     // cos(roll)
    cy = cos(euler_mv_.y);     // cos(pitch)
    ty = tan(euler_mv_.y);     // cos(pitch)

    // conversion gyro measurements to roll_rate, pitch_rate, yaw_rate
    euler_rate_mv_.x = p + sx * ty * q + cx * ty * r;
    euler_rate_mv_.y = cx * q - sx * r;
    euler_rate_mv_.z = sx / cy * q + cx / cy * r;
}

void AttitudeControl::quaternion2euler(float *quaternion, float *euler)
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

void AttitudeControl::euler_ref_cb(const geometry_msgs::Vector3 &msg)
{
    /*
    Euler ref values callback.
       :param msg: Type Vector3 (x-roll, y-pitch, z-yaw)
    */

    euler_sp_ = msg;
}

void AttitudeControl::clock_cb(const rosgraph_msgs::Clock &msg)
{
    clock_ = msg;
}

void AttitudeControl::configCallback(mmuav_control::UavAttitudeCtlParamsConfig &config, uint32_t level)
{
    /* Callback for dynamically reconfigurable parameters (P,I,D gains for each controller)*/

    if (!config_start_)
    {
        // callback is called for the first time. Use this to set the new params to the config server
        
        config.roll_kp = pid_roll_.get_kp();
        config.roll_ki = pid_roll_.get_ki();
        config.roll_kd = pid_roll_.get_kd();

        config.roll_r_kp = pid_roll_rate_.get_kp();
        config.roll_r_ki = pid_roll_rate_.get_ki();
        config.roll_r_kd = pid_roll_rate_.get_kd();

        config.pitch_kp = pid_pitch_.get_kp();
        config.pitch_ki = pid_pitch_.get_ki();
        config.pitch_kd = pid_pitch_.get_kd();

        config.pitch_r_kp = pid_pitch_rate_.get_kp();
        config.pitch_r_ki = pid_pitch_rate_.get_ki();
        config.pitch_r_kd = pid_pitch_rate_.get_kd();

        config.yaw_kp = pid_yaw_.get_kp();
        config.yaw_ki = pid_yaw_.get_ki();
        config.yaw_kd = pid_yaw_.get_kd();

        config.yaw_r_kp = pid_yaw_rate_.get_kp();
        config.yaw_r_ki = pid_yaw_rate_.get_ki();
        config.yaw_r_kd = pid_yaw_rate_.get_kd();

        config_start_ = true;

        dr_srv.updateConfig(config);
    }
    else
    {
        // The following code just sets up the P,I,D gains for all controllers
        
        pid_roll_.set_kp(config.roll_kp);
        pid_roll_.set_ki(config.roll_ki);
        pid_roll_.set_kd(config.roll_kd);

        pid_roll_rate_.set_kp(config.roll_r_kp);
        pid_roll_rate_.set_ki(config.roll_r_ki);
        pid_roll_rate_.set_kd(config.roll_r_kd);

        pid_pitch_.set_kp(config.pitch_kp);
        pid_pitch_.set_ki(config.pitch_ki);
        pid_pitch_.set_kd(config.pitch_kd);

        pid_pitch_rate_.set_kp(config.pitch_r_kp);
        pid_pitch_rate_.set_ki(config.pitch_r_ki);
        pid_pitch_rate_.set_kd(config.pitch_r_kd);

        pid_yaw_.set_kp(config.yaw_kp);
        pid_yaw_.set_kp(config.yaw_kp);
        pid_yaw_.set_ki(config.yaw_ki);
        pid_yaw_.set_kd(config.yaw_kd);

        pid_yaw_rate_.set_kp(config.yaw_r_kp);
        pid_yaw_rate_.set_ki(config.yaw_r_ki);
        pid_yaw_rate_.set_kd(config.yaw_r_kd);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "attitude_control_node");
    ros::NodeHandle private_node_handle_("~");
    int rate;

    private_node_handle_.param("rate", rate, int(100));

    AttitudeControl attitude_control(rate);

    attitude_control.run();

    return 0;
}