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
/*
 * Copyright (c) 2012, Clearpath Robotics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Desc: Gazebo 1.x plugin for a Clearpath Robotics Husky A200
 * Adapted from the TurtleBot plugin
 * Author: Ryan Gariepy
 */

#include <boost/thread.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <husky_gazebo_plugins/WheelSpeeds.h>

#include <husky_plugin/husky_plugin.h>

#include <ros/time.h>

#include <mutex>

using namespace gazebo;

enum {BL= 0, BR=1, FL=2, FR=3};

HuskyPlugin::HuskyPlugin()
{
  kill_sim = false;
  this->spinner_thread_ = new boost::thread( boost::bind( &HuskyPlugin::spin, this) );

  wheel_speed_ = new float[4];
  wheel_speed_[BL] = 0.0;
  wheel_speed_[BR] = 0.0;
  wheel_speed_[FL] = 0.0;
  wheel_speed_[FR] = 0.0;

  set_joints_[0] = false;
  set_joints_[1] = false;
  set_joints_[2] = false;
  set_joints_[3] = false;

  joints_[0].reset();
  joints_[1].reset();
  joints_[2].reset();
  joints_[3].reset();
}

HuskyPlugin::~HuskyPlugin()
{
  this->updateConnection.reset();

  rosnode_->shutdown();
  kill_sim = true;
  this->spinner_thread_->join();
  delete this->spinner_thread_;
  delete [] wheel_speed_;
  delete rosnode_;
}

void HuskyPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  this->model_ = _parent;
  this->world_ = this->model_->GetWorld();

  this->node_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->node_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";


  bl_joint_name_ = "backLeftJoint";
  if (_sdf->HasElement("backLeftJoint"))
    bl_joint_name_ = _sdf->GetElement("backLeftJoint")->Get<std::string>();

  br_joint_name_ = "backRightJoint";
  if (_sdf->HasElement("backRightJoint"))
    br_joint_name_ = _sdf->GetElement("backRightJoint")->Get<std::string>();

  fl_joint_name_ = "frontLeftJoint";
  if (_sdf->HasElement("frontLeftJoint"))
    fl_joint_name_ = _sdf->GetElement("frontLeftJoint")->Get<std::string>();

  fr_joint_name_ = "frontRightJoint";
  if (_sdf->HasElement("frontRightJoint"))
    fr_joint_name_ = _sdf->GetElement("frontRightJoint")->Get<std::string>();

  wheel_sep_ = 0.55;
  if (_sdf->HasElement("wheelSeparation"))
    wheel_sep_ = _sdf->GetElement("wheelSeparation")->Get<double>();

  wheel_diam_ = 0.30;
  if (_sdf->HasElement("wheelDiameter"))
    wheel_diam_ = _sdf->GetElement("wheelDiameter")->Get<double>();

  torque_ = 15.0;
  if (_sdf->HasElement("torque"))
    torque_ = _sdf->GetElement("torque")->Get<double>();

  base_geom_name_ = "base_link";
  if (_sdf->HasElement("baseGeom"))
    base_geom_name_ = _sdf->GetElement("baseGeom")->Get<std::string>();
  base_geom_ = model_->GetChildCollision(base_geom_name_);

  //base_geom_->SetContactsEnabled(true);

  // Get the name of the parent model
  std::string modelName = _sdf->GetParent()->Get<std::string>("name");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&HuskyPlugin::UpdateChild, this));
  gzdbg << "Plugin model name: " << modelName << "\n";

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_husky", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  rosnode_ = new ros::NodeHandle( node_namespace_ );

  cmd_vel_sub_ = rosnode_->subscribe("husky/cmd_vel", 1, &HuskyPlugin::OnCmdVel, this );
  wheel_speeds_sub_ = rosnode_->subscribe("husky/wheel_speeds", 1, &HuskyPlugin::OnWheelSpeeds, this);

  //model_->SaveControllerActuatorRosTopics("husky/cmd_vel", "geometry_msgs/Twist");
  //model_->SaveControllerActuatorRosTopics("husky/wheel_speeds", "gazebo_msgs/WheelSpeeds");

  odom_pub_ = rosnode_->advertise<nav_msgs::Odometry>("odom", 1);

  joint_state_pub_ = rosnode_->advertise<sensor_msgs::JointState>("joint_states", 1);

  this->cmd_vel_service_ = rosnode_->advertiseService<husky_gazebo_plugins::SetHuskyCmdVelRequest, husky_gazebo_plugins::SetHuskyCmdVelResponse>("husky/cmd_vel_service", boost::bind(&HuskyPlugin::OnCmdVelService, this, boost::placeholders::_1, boost::placeholders::_2));
  this->wheel_speeds_service_ = rosnode_->advertiseService<husky_gazebo_plugins::SetHuskyWheelSpeedsRequest, husky_gazebo_plugins::SetHuskyWheelSpeedsResponse>("husky/wheel_speeds_service", boost::bind(&HuskyPlugin::OnWheelSpeedsService, this, boost::placeholders::_1, boost::placeholders::_2));
  this->odom_service_ = rosnode_->advertiseService<husky_gazebo_plugins::GetHuskyOdometryRequest, husky_gazebo_plugins::GetHuskyOdometryResponse>("odom_service", boost::bind(&HuskyPlugin::OnOdometryService, this, boost::placeholders::_1, boost::placeholders::_2));
  this->joint_state_service_ = rosnode_->advertiseService<husky_gazebo_plugins::GetHuskyJointStatesRequest, husky_gazebo_plugins::GetHuskyJointStatesResponse>("joint_states_service", boost::bind(&HuskyPlugin::OnJointStateService, this, boost::placeholders::_1, boost::placeholders::_2));

  js_.name.push_back( bl_joint_name_ );
  js_.position.push_back(0);
  js_.velocity.push_back(0);
  js_.effort.push_back(0);

  js_.name.push_back( br_joint_name_ );
  js_.position.push_back(0);
  js_.velocity.push_back(0);
  js_.effort.push_back(0);

  js_.name.push_back( fl_joint_name_ );
  js_.position.push_back(0);
  js_.velocity.push_back(0);
  js_.effort.push_back(0);

  js_.name.push_back( fr_joint_name_ );
  js_.position.push_back(0);
  js_.velocity.push_back(0);
  js_.effort.push_back(0);

  prev_update_time_ = 0;
  last_cmd_vel_time_ = 0;

  joints_[BL] = model_->GetJoint(bl_joint_name_);
  joints_[BR] = model_->GetJoint(br_joint_name_);
  joints_[FL] = model_->GetJoint(fl_joint_name_);
  joints_[FR] = model_->GetJoint(fr_joint_name_);

  if (joints_[BL]) set_joints_[BL] = true;
  if (joints_[BR]) set_joints_[BR] = true;
  if (joints_[FL]) set_joints_[FL] = true;
  if (joints_[FR]) set_joints_[FR] = true;

  //initialize time and odometry position
  prev_update_time_ = last_cmd_vel_time_ = this->world_->SimTime();
  odom_pose_[0] = 0.0;
  odom_pose_[1] = 0.0;
  odom_pose_[2] = 0.0;
}


void HuskyPlugin::UpdateChild()
{
  std::lock_guard<std::mutex> lock(this->update_lock_);

  common::Time time_now = this->world_->SimTime();
  common::Time step_time = time_now - prev_update_time_;
  prev_update_time_ = time_now;

  double wd, ws;
  double d_bl, d_br, d_fl, d_fr;
  double dr, da;

  wd = wheel_diam_;
  ws = wheel_sep_;

  d_bl = d_br = d_fl = d_fr = 0;
  dr = da = 0;

  // Distance travelled by front wheels
  if (set_joints_[BL])
    d_bl = step_time.Double() * (wd / 2) * joints_[BL]->GetVelocity(0);
  if (set_joints_[BR])
    d_br = step_time.Double() * (wd / 2) * joints_[BR]->GetVelocity(0);
  if (set_joints_[FL])
    d_fl = step_time.Double() * (wd / 2) * joints_[FL]->GetVelocity(0);
  if (set_joints_[FR])
    d_fr = step_time.Double() * (wd / 2) * joints_[FR]->GetVelocity(0);

  // Can see NaN values here, just zero them out if needed
  if (std::isnan(d_bl)) {
    ROS_WARN_THROTTLE(0.1, "Gazebo ROS Husky plugin. NaN in d_bl. Step time: %.2f. WD: %.2f. Velocity: %.2f", step_time.Double(), wd, joints_[BL]->GetVelocity(0));
    d_bl = 0;
  }
  if (std::isnan(d_br)) {
    ROS_WARN_THROTTLE(0.1, "Gazebo ROS Husky plugin. NaN in d_br. Step time: %.2f. WD: %.2f. Velocity: %.2f", step_time.Double(), wd, joints_[BR]->GetVelocity(0));
    d_br = 0;
  }
  if (std::isnan(d_fl)) {
    ROS_WARN_THROTTLE(0.1, "Gazebo ROS Husky plugin. NaN in d_fl. Step time: %.2f. WD: %.2f. Velocity: %.2f", step_time.Double(), wd, joints_[FL]->GetVelocity(0));
    d_fl = 0;
  }
  if (std::isnan(d_fr)) {
    ROS_WARN_THROTTLE(0.1, "Gazebo ROS Husky plugin. NaN in d_fr. Step time: %.2f. WD: %.2f. Velocity: %.2f", step_time.Double(), wd, joints_[FR]->GetVelocity(0));
    d_fr = 0;
  }

  dr = (d_bl + d_br + d_fl + d_fr) / 4;
  da = ((d_br+d_fr)/2 - (d_bl+d_fl)/2) / ws;

  // Compute odometric pose
  odom_pose_[0] += dr * cos( odom_pose_[2] );
  odom_pose_[1] += dr * sin( odom_pose_[2] );
  odom_pose_[2] += da;

  // Compute odometric instantaneous velocity
  odom_vel_[0] = dr / step_time.Double();
  odom_vel_[1] = 0.0;
  odom_vel_[2] = da / step_time.Double();


  if (set_joints_[BL])
  {
    joints_[BL]->SetVelocity( 0, wheel_speed_[BL] / (wd/2.0) );
#if GAZEBO_MAJOR_VERSION <= 6
    joints_[BL]->SetMaxForce( 0, torque_ );
#else
    joints_[BL]->SetEffortLimit( 0, torque_ );
#endif
  }
  if (set_joints_[BR])
  {
    joints_[BR]->SetVelocity( 0, wheel_speed_[BR] / (wd / 2.0) );
#if GAZEBO_MAJOR_VERSION <= 6
    joints_[BR]->SetMaxForce( 0, torque_ );
#else
    joints_[BR]->SetEffortLimit( 0, torque_ );
#endif
  }
  if (set_joints_[FL])
  {
    joints_[FL]->SetVelocity( 0, wheel_speed_[FL] / (wd / 2.0) );
#if GAZEBO_MAJOR_VERSION <= 6
    joints_[FL]->SetMaxForce( 0, torque_ );
#else
    joints_[FL]->SetEffortLimit( 0, torque_ );
#endif
  }
  if (set_joints_[FR])
  {
    joints_[FR]->SetVelocity( 0, wheel_speed_[FR] / (wd / 2.0) );
#if GAZEBO_MAJOR_VERSION <= 6
    joints_[FR]->SetMaxForce( 0, torque_ );
#else
    joints_[FR]->SetEffortLimit( 0, torque_ );
#endif
  }

  odom_.header.stamp.sec = time_now.sec;
  odom_.header.stamp.nsec = time_now.nsec;
  odom_.header.frame_id = "odom";
  odom_.child_frame_id = "base_footprint";
  odom_.pose.pose.position.x = odom_pose_[0];
  odom_.pose.pose.position.y = odom_pose_[1];
  odom_.pose.pose.position.z = 0;

  tf::Quaternion qt;
  qt.setRPY(0,0,odom_pose_[2]);

  odom_.pose.pose.orientation.x = qt.getX();
  odom_.pose.pose.orientation.y = qt.getY();
  odom_.pose.pose.orientation.z = qt.getZ();
  odom_.pose.pose.orientation.w = qt.getW();

  double pose_cov[36] = { 1e-3, 0, 0, 0, 0, 0,
                          0, 1e-3, 0, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e3};

  memcpy( &odom_.pose.covariance[0], pose_cov, sizeof(double)*36 );
  memcpy( &odom_.twist.covariance[0], pose_cov, sizeof(double)*36 );

  odom_.twist.twist.linear.x = 0;
  odom_.twist.twist.linear.y = 0;
  odom_.twist.twist.linear.z = 0;

  odom_.twist.twist.angular.x = 0;
  odom_.twist.twist.angular.y = 0;
  odom_.twist.twist.angular.z = 0;

  odom_pub_.publish( odom_ );

  js_.header.stamp.sec = time_now.sec;
  js_.header.stamp.nsec = time_now.nsec;
  if (this->set_joints_[BL])
  {
    js_.position[0] = joints_[BL]->Position();
    js_.velocity[0] = joints_[BL]->GetVelocity(0);
    js_.effort[0] = joints_[BL]->GetForce(0); //GetForce not implemented yet!
  }

  if (this->set_joints_[BR])
  {
    js_.position[1] = joints_[BR]->Position();
    js_.velocity[1] = joints_[BR]->GetVelocity(0);
    js_.effort[1] = joints_[BR]->GetForce(0);
  }

  if (this->set_joints_[FL])
  {
    js_.position[2] = joints_[FL]->Position();
    js_.velocity[2] = joints_[FL]->GetVelocity(0);
    js_.effort[2] = joints_[FL]->GetForce(0);
  }

  if (this->set_joints_[FR])
  {
    js_.position[3] = joints_[FR]->Position();
    js_.velocity[3] = joints_[FR]->GetVelocity(0);
    js_.effort[3] = joints_[FR]->GetForce(0);
  }

  joint_state_pub_.publish( js_ );

  // Timeout if we haven't received a cmd in <0.1 s
  common::Time time_since_last_cmd = time_now - last_cmd_vel_time_;
  if (time_since_last_cmd.Double() > 0.1)
  {
    wheel_speed_[BL] = 0;
    wheel_speed_[BR] = 0;
    wheel_speed_[FL] = 0;
    wheel_speed_[FR] = 0;
  }
}

void HuskyPlugin::OnCmdVel( const geometry_msgs::TwistConstPtr &msg)
{
  return this->OnCmdVel(*msg);
}

void HuskyPlugin::OnWheelSpeeds( const husky_gazebo_plugins::WheelSpeeds::ConstPtr &msg )
{
    return this->OnWheelSpeeds(*msg);
}

void HuskyPlugin::OnCmdVel(const geometry_msgs::Twist &msg)
{
  last_cmd_vel_time_ = this->world_->SimTime();
  double vr, va;
  vr = msg.linear.x;
  va = msg.angular.z;

  wheel_speed_[BL] = vr - va * (wheel_sep_) / 2;
  wheel_speed_[BR] = vr + va * (wheel_sep_) / 2;
  wheel_speed_[FL] = vr - va * (wheel_sep_) / 2;
  wheel_speed_[FR] = vr + va * (wheel_sep_) / 2;
}

void HuskyPlugin::OnWheelSpeeds(const husky_gazebo_plugins::WheelSpeeds &msg)
{
  last_cmd_vel_time_ = this->world_->SimTime();
  double back_left, back_right, front_left, front_right;
  back_left = msg.back_left_wheel;
  back_right = msg.back_right_wheel;
  front_left = msg.front_left_wheel;
  front_right = msg.front_right_wheel;

  wheel_speed_[BL] = back_left;
  wheel_speed_[BR] = back_right;
  wheel_speed_[FL] = front_left;
  wheel_speed_[FR] = front_right;
}

bool HuskyPlugin::OnCmdVelService(const husky_gazebo_plugins::SetHuskyCmdVelRequest &req, husky_gazebo_plugins::SetHuskyCmdVelResponse &resp)
{
  this->OnCmdVel(req.cmd_vel);

  resp.status_message = "";
  resp.success = true;

  return resp.success;
}

bool HuskyPlugin::OnWheelSpeedsService(const husky_gazebo_plugins::SetHuskyWheelSpeedsRequest &req, husky_gazebo_plugins::SetHuskyWheelSpeedsResponse &resp)
{
  this->OnWheelSpeeds(req.wheel_speeds);

  resp.status_message = "";
  resp.success = true;

  return resp.success;
}

bool HuskyPlugin::OnOdometryService(const husky_gazebo_plugins::GetHuskyOdometryRequest &req, husky_gazebo_plugins::GetHuskyOdometryResponse &resp)
{
  std::lock_guard<std::mutex> lock(this->update_lock_);

  resp.odometry = this->odom_;

  resp.status_message = "";
  resp.success = true;

  return resp.success;
}

bool HuskyPlugin::OnJointStateService(const husky_gazebo_plugins::GetHuskyJointStatesRequest &req, husky_gazebo_plugins::GetHuskyJointStatesResponse &resp)
{
  std::lock_guard<std::mutex> lock(this->update_lock_);

  resp.joint_states = this->js_;

  resp.status_message = "";
  resp.success = true;

  return resp.success;
}

void HuskyPlugin::spin()
{
  while(ros::ok() && !kill_sim) ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN(HuskyPlugin);

