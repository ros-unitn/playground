#include "x_controller/gripper.hpp"

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>

Gripper::Gripper(ros::NodeHandle &n) {
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> ac("fibonacci", true);
}

Gripper::~Gripper() {

}