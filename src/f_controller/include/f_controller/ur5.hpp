#pragma once
#include <queue>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>

#include "f_controller/params.hpp"

#define UR5_SHOULDER_PAN_TOPIC "/shoulder_lift_joint_position_controller"
#define UR5_SHOULDER_LIFT_TOPIC "/shoulder_lift_joint_position_controller"
#define UR5_ELBOW_TOPIC "/elbow_joint_position_controller"
#define UR5_WRIST_1_TOPIC "/wrist_1_joint_position_controller"
#define UR5_WRIST_2_TOPIC "/wrist_2_joint_position_controller"
#define UR5_WRIST_3_TOPIC "/wrist_3_joint_position_controller"
#define TARGET "/command"
#define STATE "/state"

class UR5 {
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
  typedef GenericJointParams<float, Joint, UR5_JOINT_COUNT> JointParams;

  UR5(ros::NodeHandle &n);
  ~UR5();

  void tick();
  void publish();
  void push(JointParams params);
  const JointParams &current() const;
  JointParams pop();

private:
  bool reached();
  bool close_enough(Joint joint, float target, float state);

private:
  std::queue<JointParams> m_jobs;
  JointParams m_state = JointParams();
  ros::Publisher m_publishers[UR5_JOINT_COUNT];
  ros::Subscriber m_subscribers[UR5_JOINT_COUNT];
};