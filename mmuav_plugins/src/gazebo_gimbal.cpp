<<<<<<< HEAD
=======
#define _USE_MATH_DEFINES
#include <cmath>
>>>>>>> mbzirc-test
#include "mmuav_plugins/gazebo_gimbal.h"

namespace gazebo {

  enum {
    ROLL,
    PITCH,
    YAW,
};

<<<<<<< HEAD
=======
// double roll(double x, double y, double z, double w) {
//   double sinr_cosp = 2 * (w * x + y * z);
//   double cosr_cosp = 1 - 2 * (x * x + y * y);
//   return std::atan2(sinr_cosp, cosr_cosp);
// }

// double pitch(double x, double y, double z, double w) {  
//   double sinp = 2 * (w * y - z * x);
//   if (std::abs(sinp) >= 1)
//     return std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
//   else
//     return std::asin(sinp);
// }

// double yaw(double x, double y, double z, double w) {
//   double siny_cosp = 2 * (w * z + x * y);
//   double cosy_cosp = 1 - 2 * (y * y + z * z);
//   return std::atan2(siny_cosp, cosy_cosp);
// }


>>>>>>> mbzirc-test

  GazeboGimbal::GazeboGimbal() {}

  GazeboGimbal::~GazeboGimbal() {}

  void GazeboGimbal::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("Cannot load Gimbal plugin, ROS node for Gazebo not initialized");
      return;
    }

    joints_.resize(3);
    joints_[ROLL] = _model->GetJoint("gimbal_roll_joint");
    joints_[PITCH] = _model->GetJoint("gimbal_pitch_joint");
    joints_[YAW] = _model->GetJoint("gimbal_yaw_joint");

    link = _model->GetLink("base_link");
    world = _model;
    joints_[YAW]->SetPosition(0, 0);
    joints_[PITCH]->SetPosition(0, 0);
    joints_[ROLL]->SetPosition(0, 0);

    updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboGimbal::Update, this));
  }

  void GazeboGimbal::Update() {
    ignition::math::Pose3d pose = link->WorldPose();

    double x, y, z, w;
    x = pose.Rot().X();
    y = pose.Rot().Y();
    z = pose.Rot().Z();
    w = pose.Rot().W();


    double pitch_c = M_PI/2 - std::acos(2*x*z - 2*y*w);
    double roll_c = std::acos(2*y*z + 2*x*w) - M_PI/2;

    joints_[YAW]->SetPosition(0, 0);
    joints_[PITCH]->SetPosition(0, pitch_c);
    joints_[ROLL]->SetPosition(0, roll_c);
  }

GZ_REGISTER_MODEL_PLUGIN(GazeboGimbal);
}


