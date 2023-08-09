/**---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
 * This file is part of the Neurorobotics Platform software
 * Copyright (C) 2014,2015,2016,2017 Human Brain Project
 * https://www.humanbrainproject.eu
 *
 * The Human Brain Project is a European Commission funded project
 * in the frame of the Horizon2020 FET Flagship plan.
 * http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 * ---LICENSE-END**/
#ifndef GAZEBO_ROS_CREATE_H
#define GAZEBO_ROS_CREATE_H

#include "gazebo/physics/physics.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include "husky_gazebo_plugins/WheelSpeeds.h"

#include <husky_gazebo_plugins/SetHuskyCmdVel.h>
#include <husky_gazebo_plugins/SetHuskyWheelSpeeds.h>
#include <husky_gazebo_plugins/GetHuskyJointStates.h>
#include <husky_gazebo_plugins/GetHuskyOdometry.h>

#include <tf/transform_broadcaster.h>
#include <ros/ros.h>


/*
 * Desc: Gazebo 1.x plugin for a Clearpath Robotics Husky A200
 * Adapted from the TurtleBot plugin
 * Author: Ryan Gariepy
 */

namespace gazebo
{
  class HuskyPlugin : public ModelPlugin
  {
    public:
      HuskyPlugin();
      virtual ~HuskyPlugin();

      virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf );

      virtual void UpdateChild();

    private:

      void OnContact(const std::string &name, const physics::Contact &contact);
      void OnCmdVel( const geometry_msgs::TwistConstPtr &msg);
      void OnWheelSpeeds( const husky_gazebo_plugins::WheelSpeeds::ConstPtr &msg);

      void OnCmdVel(const geometry_msgs::Twist &msg);
      void OnWheelSpeeds( const husky_gazebo_plugins::WheelSpeeds &msg);

      bool OnCmdVelService(const husky_gazebo_plugins::SetHuskyCmdVelRequest &req, husky_gazebo_plugins::SetHuskyCmdVelResponse &resp);
      bool OnWheelSpeedsService(const husky_gazebo_plugins::SetHuskyWheelSpeedsRequest &req, husky_gazebo_plugins::SetHuskyWheelSpeedsResponse &resp);

      bool OnOdometryService(const husky_gazebo_plugins::GetHuskyOdometryRequest &req, husky_gazebo_plugins::GetHuskyOdometryResponse &resp);
      bool OnJointStateService(const husky_gazebo_plugins::GetHuskyJointStatesRequest &req, husky_gazebo_plugins::GetHuskyJointStatesResponse &resp);

      /// Parameters
      std::string node_namespace_;
      std::string bl_joint_name_;
      std::string br_joint_name_;
      std::string fl_joint_name_;
      std::string fr_joint_name_;
      std::string base_geom_name_;

      /// Separation between the wheels
      float wheel_sep_;

      /// Diameter of the wheels
      float wheel_diam_;

      ///Torque applied to the wheels
      float torque_;

      ros::NodeHandle *rosnode_;

      ros::Publisher sensor_state_pub_;
      ros::Publisher odom_pub_;
      ros::Publisher joint_state_pub_;

      ros::Subscriber cmd_vel_sub_;
      ros::Subscriber wheel_speeds_sub_;

      std::mutex update_lock_;
      ros::ServiceServer odom_service_;
      ros::ServiceServer joint_state_service_;
      ros::ServiceServer cmd_vel_service_;
      ros::ServiceServer wheel_speeds_service_;

      physics::WorldPtr world_;
      physics::ModelPtr model_;
      sensors::SensorPtr parent_sensor_;

      /// Speeds of the wheels
      float *wheel_speed_;

      // Simulation time of the last update
      common::Time prev_update_time_;
      common::Time last_cmd_vel_time_;

      float odom_pose_[3];
      float odom_vel_[3];

      bool set_joints_[4];
      physics::JointPtr joints_[4];
      physics::CollisionPtr base_geom_;

      tf::TransformBroadcaster transform_broadcaster_;
      sensor_msgs::JointState js_;

      nav_msgs::Odometry odom_;

      void spin();
      boost::thread *spinner_thread_;

      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;

      bool kill_sim;
  };
}
#endif
