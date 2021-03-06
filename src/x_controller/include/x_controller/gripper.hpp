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

#include <atomic>

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>

#define GRIPPER_TOPIC "/gripper_controller/gripper_cmd"

#define GRIPPER_MODEL "robot"
#define GRIPPER_LINK "robotiq_2f_base_link"

class Gripper {
public:
  typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> ActionClient;
  typedef control_msgs::GripperCommandGoal Goal;
  typedef actionlib::SimpleClientGoalState GoalState;

  Gripper(ros::NodeHandle &n);
  ~Gripper();

  bool push(float position, bool deferred = false);
  size_t remaining() const;

  bool attach(const std::string &model, const std::string &link);
  bool detach();

  bool enable_collisions(std::string model = GRIPPER_MODEL);
  bool disable_collisions(std::string model = GRIPPER_MODEL);

protected:
  void done(const actionlib::SimpleClientGoalState &state,
            const control_msgs::GripperCommandResultConstPtr &result);

private:
  std::atomic<size_t> m_jobs{0};
  std::atomic<bool> m_attached{false};
  std::string m_attached_model;
  std::string m_attached_link;
  ros::ServiceClient m_attach_client;
  ros::ServiceClient m_detach_client;
  ros::ServiceClient m_set_collision_client;
  ActionClient *m_action_client;
};