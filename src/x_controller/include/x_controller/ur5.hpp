/**
  Copyright (C) 2021 Filippo Rossi

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once
#include <queue>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>

#include "x_controller/params.hpp"

#define UR5_SHOULDER_PAN_TOPIC "/shoulder_pan_joint_position_controller"
#define UR5_SHOULDER_LIFT_TOPIC "/shoulder_lift_joint_position_controller"
#define UR5_ELBOW_TOPIC "/elbow_joint_position_controller"
#define UR5_WRIST_1_TOPIC "/wrist_1_joint_position_controller"
#define UR5_WRIST_2_TOPIC "/wrist_2_joint_position_controller"
#define UR5_WRIST_3_TOPIC "/wrist_3_joint_position_controller"
#define UR5_TARGET "/command"
#define UR5_STATE "/state"

class UR5 {
public:
  enum Joint {
    UR5_SHOULDER_PAN,
    UR5_SHOULDER_LIFT,
    UR5_ELBOW,
    UR5_WRIST_1,
    UR5_WRIST_2,
    UR5_WRIST_3,
    UR5_JOINT_COUNT
  };

  typedef control_msgs::JointControllerState JointState;
  typedef GenericJointParams<double, Joint, UR5_JOINT_COUNT> JointParams;

  UR5(ros::NodeHandle &n);
  ~UR5();

  void tick();
  void publish();
  void push(JointParams params);
  const JointParams &current() const;
  JointParams pop();
  size_t remaining() const;

private:
  bool reached();
  bool close_enough(Joint joint, double target, double state);

private:
  std::queue<JointParams> m_jobs;
  JointParams m_state = JointParams();
  ros::Publisher m_publishers[UR5_JOINT_COUNT];
  ros::Subscriber m_subscribers[UR5_JOINT_COUNT];
};