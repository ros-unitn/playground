#include "f_controller/ur5.hpp"

UR5::UR5(ros::NodeHandle& n) {
  for (int i = 0; i < UR5_JOINT_COUNT; i++) {
    m_publishers[i] = n.advertise<std_msgs::Float64>(PubTopics[i], 1000);
    m_subscribers[UR5_SHOULDER_PAN] =
        n.subscribe<JointState>(UR5_SHOULDER_PAN_TOPIC STATE, 1000,
                                [&](const JointState::ConstPtr& ctr_msg) {
                                  m_state[i] = ctr_msg->process_value;
                                });
  }
}

UR5::~UR5() {
  for (int i = 0; i < UR5_JOINT_COUNT; i++) {
    m_subscribers[i].shutdown();
    m_publishers[i].shutdown();
  }
}

void UR5::tick() {
  if (m_jobs.empty());
  if (reached()) {
    m_jobs.pop();
  }
}

void UR5::publish() {
  if (m_jobs.empty()) return;
  auto targets = m_jobs.front();
  for (int i = 0; i < UR5_JOINT_COUNT; i++) {
    std_msgs::Float64 msg;
    msg.data = targets[i];
    m_publishers[i].publish(msg);
  }
}

bool UR5::reached() {
  if (m_jobs.empty()) return true;
  auto targets = m_jobs.front();
  for (int i = 0; i < UR5_JOINT_COUNT; i++) {
    if (!closeEnough((Joint) i, targets[i], m_state[i])) return false;
  }
  return true;
}

bool UR5::closeEnough(Joint joint, float target, float state) {
  return target == state; // TODO: error
}