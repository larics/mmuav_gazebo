#!/usr/bin/env python3

__author__ = 'aivanovic'

import rospy, math
from math import sin, cos
import copy
from simple_filters import filterPT1
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped, PoseStamped, \
  TwistStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from mmuav_control.cfg import OdometryFilterParamsConfig
import tf
from rospkg import RosPack
import yaml
import time

class OdometryFilter:
  '''
  Class implements ROS node for cascade (z, vz) PID control for MAV height.
  Subscribes to:
    pose       - used to extract z-position of the vehicle
    /morus/velocity   - used to extract vz of the vehicle
    /morus/pos_ref    - used to set the reference for z-position
    /morus/vel_ref    - used to set the reference for vz-position (useful for testing velocity controller)

  Publishes:
    /morus/mot_vel_ref  - referent value for thrust in terms of motor velocity (rad/s)
    /morus/pid_z     start_flag   - publishes PID-z data - referent value, measured value, P, I, D and total component (useful for tuning params)
    /morus/pid_vz        - publishes PID-vz data - referent value, measured value, P, I, D and total component (useful for tuning params)

  Dynamic reconfigure is used to set controller params online.
  '''

  def __init__(self):
    '''
    Initialization of the class.
    '''

    # Flags for first pose and velocity
    self.first_pose_received = False
    self.first_velocity_received = False
    self.config_start = False       # flag indicates if the config callback is called for the first time

    # Params
    self.rate = rospy.get_param('~rate', 100)
    self.Ts = 1.0/float(self.rate)

    # Load parameters from yaml file
    file_name = rospy.get_param('~filename', 'OdometryFilter.yaml')
    file_name = RosPack().get_path('mmuav_control') + '/config/' + file_name
    initial_params = yaml.load(open(file_name, 'r'))


    #########################################################
    #########################################################

    # Add parameters for position
    self.filter_x_t = initial_params['x']['t']
    self.filter_x_k = initial_params['x']['k']
    self.filter_y_t = initial_params['y']['t']
    self.filter_y_k = initial_params['y']['k']
    self.filter_z_t = initial_params['z']['t']
    self.filter_z_k = initial_params['z']['k']

    # Add parameters for orientation
    self.filter_q_t = initial_params['q']['t']
    self.filter_q_k = initial_params['q']['k']

    # Add parameters for velocity
    self.filter_vx_t = initial_params['vx']['t']
    self.filter_vx_k = initial_params['vx']['k']
    self.filter_vy_t = initial_params['vy']['t']
    self.filter_vy_k = initial_params['vy']['k']
    self.filter_vz_t = initial_params['vz']['t']
    self.filter_vz_k = initial_params['vz']['k']

    #########################################################
    #########################################################

    # Pose and velocity messages
    self.pose = PoseStamped()
    self.pose_old = PoseStamped()
    self.pose_f = PoseStamped()
    self.pose_f_old = PoseStamped()
    self.velocity = TwistStamped()
    self.velocity_old = TwistStamped()
    self.velocity_f = TwistStamped()
    self.velocity_f_old = TwistStamped()

    self.cfg_server = Server(OdometryFilterParamsConfig, self.cfg_callback)
    self.ros_rate = rospy.Rate(self.rate) # attitude control at 100 Hz
    self.t_start = rospy.Time.now()

    # Publishers and subscribers
    self.pose_f_pub = rospy.Publisher('filtered/pose', PoseStamped, queue_size=1)
    self.velocity_f_pub = rospy.Publisher('filtered/velocity_relative', 
      TwistStamped, queue_size=1)

    rospy.Subscriber('pose', PoseStamped, self.pose_cb, queue_size=1)
    #rospy.Subscriber('odometry', Odometry, self.vel_cb, queue_size=1)
    rospy.Subscriber('velocity_relative', TwistStamped, self.vel_cb, queue_size=1)

  def run(self):
    '''
    Runs ROS node - computes PID algorithms for z and vz control.
    '''

    i = 0
    while (not self.first_velocity_received and not 
      self.first_pose_received and not rospy.is_shutdown()):
      if (i % 50) == 0:
        print('Waiting for pose and velocity measurements.')
      i = i+1
      rospy.sleep(0.01)
    print("Starting odometry filter.")

    # Set filtered pose and velocity to first obtained value.
    self.pose_f = copy.deepcopy(self.pose)
    self.pose_f_old = copy.deepcopy(self.pose)
    self.velocity_f = copy.deepcopy(self.velocity)
    self.velocity_f_old = copy.deepcopy(self.velocity)

    self.t_old = rospy.Time.now()
    #self.t_old = datetime.now()

    while not rospy.is_shutdown():
      # Odometry filter will rely on a simple PT1 filter with time constant T,
      # sampling time Ts and gain K.
      self.ros_rate.sleep()

      # Set filtered pose and velocity to current ones
      self.pose_f = copy.deepcopy(self.pose)
      self.velocity_f = copy.deepcopy(self.velocity)

      # Filter position
      self.pose_f.pose.position.x = filterPT1(
        self.pose_f_old.pose.position.x, self.pose.pose.position.x, 
        self.filter_x_t, self.Ts, self.filter_x_k)
      self.pose_f.pose.position.y = filterPT1(
        self.pose_f_old.pose.position.y, self.pose.pose.position.y, 
        self.filter_y_t, self.Ts, self.filter_y_k)
      self.pose_f.pose.position.z = filterPT1(
        self.pose_f_old.pose.position.z, self.pose.pose.position.z, 
        self.filter_z_t, self.Ts, self.filter_z_k)

      # Filter orientation. The proper way would be to use slerp, but for small changes
      # in orientation this works.
      self.pose_f.pose.orientation.x = filterPT1(
        self.pose_f_old.pose.orientation.x, self.pose.pose.orientation.x, 
        self.filter_q_t, self.Ts, self.filter_q_k)
      self.pose_f.pose.orientation.y = filterPT1(
        self.pose_f_old.pose.orientation.y, self.pose.pose.orientation.y, 
        self.filter_q_t, self.Ts, self.filter_q_k)
      self.pose_f.pose.orientation.z = filterPT1(
        self.pose_f_old.pose.orientation.z, self.pose.pose.orientation.z, 
        self.filter_q_t, self.Ts, self.filter_q_k)
      self.pose_f.pose.orientation.w = filterPT1(
        self.pose_f_old.pose.orientation.w, self.pose.pose.orientation.w, 
        self.filter_q_t, self.Ts, self.filter_q_k)
      # Normalize quaternion.
      magnitude = math.sqrt(self.pose_f.pose.orientation.x*self.pose_f.pose.orientation.x + 
        self.pose_f.pose.orientation.y*self.pose_f.pose.orientation.y + 
        self.pose_f.pose.orientation.z*self.pose_f.pose.orientation.z + 
        self.pose_f.pose.orientation.w*self.pose_f.pose.orientation.w)
      self.pose_f.pose.orientation.x = self.pose_f.pose.orientation.x/magnitude
      self.pose_f.pose.orientation.y = self.pose_f.pose.orientation.y/magnitude
      self.pose_f.pose.orientation.z = self.pose_f.pose.orientation.z/magnitude
      self.pose_f.pose.orientation.w = self.pose_f.pose.orientation.w/magnitude

      # Filter velocity
      self.velocity_f.twist.linear.x = filterPT1(
        self.velocity_f_old.twist.linear.x, self.velocity.twist.linear.x, 
        self.filter_vx_t, self.Ts, self.filter_vx_k)
      self.velocity_f.twist.linear.y = filterPT1(
        self.velocity_f_old.twist.linear.y, self.velocity.twist.linear.y, 
        self.filter_vy_t, self.Ts, self.filter_vy_k)
      self.velocity_f.twist.linear.z = filterPT1(
        self.velocity_f_old.twist.linear.z, self.velocity.twist.linear.z, 
        self.filter_vz_t, self.Ts, self.filter_vz_k)

      # Publish filtered values
      self.pose_f_pub.publish(self.pose_f)
      self.velocity_f_pub.publish(self.velocity_f)

      # Set old values
      self.pose_f_old = copy.deepcopy(self.pose_f)
      self.velocity_f_old = copy.deepcopy(self.velocity_f)


  def pose_cb(self, msg):
    self.first_pose_received = True
    self.pose_old = copy.deepcopy(self.pose)
    self.pose = msg

  def vel_cb(self, msg):
    self.first_velocity_received = True
    self.velocity_old = copy.deepcopy(self.velocity)
    self.velocity = msg

  def cfg_callback(self, config, level):
    """
    Callback for dynamically reconfigurable parameters
    """

    if not self.config_start:
      # callback is called for the first time. Use this to set the new params to the config server
      config.x_t = self.filter_x_t
      config.x_k = self.filter_x_k
      config.y_t = self.filter_y_t
      config.y_k = self.filter_y_k
      config.z_t = self.filter_z_t
      config.z_k = self.filter_z_k

      config.q_t = self.filter_q_t
      config.q_k = self.filter_q_k

      config.vx_t = self.filter_vx_t
      config.vx_k = self.filter_vx_k
      config.vy_t = self.filter_vy_t
      config.vy_k = self.filter_vy_k
      config.vz_t = self.filter_vz_t
      config.vz_k = self.filter_vz_k

      self.config_start = True
    else:
      # The following code just sets up filter parameters
      self.filter_x_t = config.x_t
      self.filter_x_k = config.x_k
      self.filter_y_t = config.y_t
      self.filter_y_k = config.y_k
      self.filter_z_t = config.z_t
      self.filter_z_k = config.z_k

      self.filter_q_t = config.q_t
      self.filter_q_k = config.q_k

      self.filter_vx_t = config.vx_t
      self.filter_vx_k = config.vx_k
      self.filter_vy_t = config.vy_t
      self.filter_vy_k = config.vy_k
      self.filter_vz_t = config.vz_t
      self.filter_vz_k = config.vz_k

    # this callback should return config data back to server
    return config

if __name__ == '__main__':

  rospy.init_node('odometry_filter')
  odometry_filter = OdometryFilter()
  odometry_filter.run()
