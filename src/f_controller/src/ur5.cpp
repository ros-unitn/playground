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

#include <iostream>

#include "f_controller/ur5.hpp"

static constexpr const char *PubTopics[] = {
    UR5_SHOULDER_PAN_TOPIC TARGET,
    UR5_SHOULDER_LIFT_TOPIC TARGET,
    UR5_ELBOW_TOPIC TARGET,
    UR5_WRIST_1_TOPIC TARGET,
    UR5_WRIST_2_TOPIC TARGET,
    UR5_WRIST_3_TOPIC TARGET,
};
static constexpr const char *SubTopics[] = {
    UR5_SHOULDER_PAN_TOPIC STATE,
    UR5_SHOULDER_LIFT_TOPIC STATE,
    UR5_ELBOW_TOPIC STATE,
    UR5_WRIST_1_TOPIC STATE,
    UR5_WRIST_2_TOPIC STATE,
    UR5_WRIST_3_TOPIC STATE,
};

UR5::UR5(ros::NodeHandle &n) {
  for (int i = 0; i < UR5_JOINT_COUNT; i++) {
    m_publishers[i] = n.advertise<std_msgs::Float64>(PubTopics[i], 1000);
    m_subscribers[i] = n.subscribe<JointState>(
        SubTopics[i], 1000, [this, i](const JointState::ConstPtr &msg) { m_state[i] = msg->process_value; });
  }
}

UR5::~UR5() {
  for (int i = 0; i < UR5_JOINT_COUNT; i++) {
    m_publishers[i].shutdown();
    m_subscribers[i].shutdown();
  }
}

void UR5::tick() {
  if (m_jobs.empty())
    return;
  if (reached()) {
    //std::cout << "job finished" << std::endl;
    m_jobs.pop();
  } else {
    //std::cout << "executing job in queue" << std::endl;
  }
}

void UR5::publish() {
  if (m_jobs.empty())
    return;
  JointParams &target = m_jobs.front();
  for (int i = 0; i < UR5_JOINT_COUNT; i++) {
    std_msgs::Float64 msg;
    msg.data = target[i];
    m_publishers[i].publish(msg);
  }
}

void UR5::push(JointParams params) {
  m_jobs.push(params);
}

const UR5::JointParams &UR5::current() const {
  return m_jobs.front();
}

UR5::JointParams UR5::pop() {
  JointParams &front = m_jobs.front();
  m_jobs.pop();
  return front;
}

size_t UR5::remaining() const {
  return m_jobs.size();
}

bool UR5::reached() {
  if (m_jobs.empty())
    return true;
  //std::cout << "checking if reached" << std::endl;
  JointParams &target = m_jobs.front();
  for (int i = 0; i < UR5_JOINT_COUNT; i++) {
    if (target.is_ignored((Joint)i))
      continue;
    //std::cout << i << "\t" << target[i] << "\t" << m_state[i] << std::endl;
    if (!close_enough((Joint)i, target[i], m_state[i]))
      return false;
  }
  return true;
}

bool UR5::close_enough(Joint joint, double target, double state) {
  return abs(target - state) <= 0.1; // TODO: error
}