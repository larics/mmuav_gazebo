#ifndef GAZEBO_GIMBAL_H
#define GAZEBO_GIMBAL_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>


namespace gazebo {


class GazeboGimbal : public ModelPlugin {
 public:
  GazeboGimbal();
  ~GazeboGimbal();
  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

protected:
  virtual void Update();

private:

 private:
  std::vector<physics::JointPtr> joints_;
  physics::LinkPtr link;
  physics::ModelPtr world;
  event::ConnectionPtr updateConnection_;
  int count;
};
}

#endif // GAZEBO_GIMBAL_H
