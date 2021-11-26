#include <iostream>

#include "f_controller/ur5.hpp"

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
    std::cout << "job finished" << std::endl;
    m_jobs.pop();
  } else {
    std::cout << "executing job in queue" << std::endl;
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

bool UR5::reached() {
  if (m_jobs.empty())
    return true;
  std::cout << "checking if reached" << std::endl;
  JointParams &target = m_jobs.front();
  for (int i = 0; i < UR5_JOINT_COUNT; i++) {
    if (target.is_ignored((Joint)i))
      continue;
    std::cout << i << "\t" << target[i] << "\t" << m_state[i] << std::endl;
    if (!close_enough((Joint)i, target[i], m_state[i]))
      return false;
  }
  return true;
}

bool UR5::close_enough(Joint joint, float target, float state) {
  return abs(target - state) <= 0.1; // TODO: error
}