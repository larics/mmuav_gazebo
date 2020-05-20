#include "mmuav_plugins/gazebo_gimbal.h"

namespace gazebo {

  enum {
    ROLL,
    PITCH,
    YAW,
};


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
    count = 0;

    link = _model->GetLink("gimbal_link_1");
    world = _model;
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboGimbal::Update, this));
  }

  void GazeboGimbal::Update() {
    count++;
    ignition::math::Pose3d pose = link->WorldPose();
    ignition::math::Pose3d pose_inv = pose.Inverse();
    ignition::math::Vector3d rpy = pose_inv.Rot().Euler();

    joints_[ROLL]->SetPosition(0, rpy.X());
    joints_[PITCH]->SetPosition(0, rpy.Y());
    joints_[YAW]->SetPosition(0, 0);

  }

GZ_REGISTER_MODEL_PLUGIN(GazeboGimbal);
}


