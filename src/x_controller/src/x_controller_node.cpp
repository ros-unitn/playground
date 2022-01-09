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

#define _USE_MATH_DEFINES

#include <cmath>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <map>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>

#include "x_controller/gripper.hpp"
#include "x_controller/kinematics.hpp"
#include "x_controller/ur5.hpp"
#include "x_msgs/Block.h"
#include "x_msgs/Blocks.h"

std::map<std::string, const Eigen::Vector3d> drop_points = {
  {"X1-Y1-Z2", Eigen::Vector3d(-0.3, -0.45, 0.3)},
  {"X1-Y2-Z2", Eigen::Vector3d(-0.12, -0.45, 0.3)},
  {"X1-Y3-Z2", Eigen::Vector3d(0.09, -0.45, 0.3)},
  {"X1-Y4-Z2", Eigen::Vector3d(0.3, -0.45, 0.3)},

  {"X1-Y2-Z1", Eigen::Vector3d(-0.3, -0.61, 0.3)},
  {"X1-Y2-Z2-TWINFILLET", Eigen::Vector3d(-0.12, -0.61, 0.3)},
  {"X1-Y3-Z2-FILLET", Eigen::Vector3d(0.09, -0.61, 0.3)},
  {"X2-Y2-Z2", Eigen::Vector3d(0.3, -0.61, 0.3)},

  {"X1-Y4-Z1", Eigen::Vector3d(-0.3, -0.79, 0.3)},
  {"X1-Y2-Z2-CHAMFER", Eigen::Vector3d(-0.12, -0.79, 0.3)},
  {"X2-Y2-Z2-FILLET", Eigen::Vector3d(0.3, -0.79, 0.3)},
};

bool execute_motion(ros::Rate &rate, UR5 &ur5, const Eigen::Vector3d &pos, const Eigen::Vector3d &rot, const Eigen::VectorXd &qEs, double max_t) {
  Eigen::MatrixXd dest_angles = Kinematics::ik(Kinematics::create_homogeneous_matrix(pos, Kinematics::eul2rotm(rot)));
  Eigen::VectorXd qEf = Kinematics::best_angles(qEs, dest_angles);

  Eigen::MatrixXd points = Kinematics::p2p(qEs, qEf, max_t);

  for (int i = 0; i < points.rows(); i++) {
    UR5::JointParams params;
    params.set(UR5::Joint::UR5_SHOULDER_PAN, points(i, 0));
    params.set(UR5::Joint::UR5_SHOULDER_LIFT, points(i, 1));
    params.set(UR5::Joint::UR5_ELBOW, points(i, 2));
    params.set(UR5::Joint::UR5_WRIST_1, points(i, 3));
    params.set(UR5::Joint::UR5_WRIST_2, points(i, 4));
    params.set(UR5::Joint::UR5_WRIST_3, points(i, 5));

    ur5.push(params);
  }

  while (ros::ok() && ur5.remaining() > 0) {
    ur5.tick();
    ur5.publish();
    ros::spinOnce();
    rate.sleep();
  }

  return true;
}

Eigen::VectorXd refresh_theta() {
  sensor_msgs::JointStateConstPtr msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");

  if (msg != nullptr) {
    throw std::runtime_error("Joint state message is null.");
  }

  Eigen::VectorXd theta(6);

  if (msg->name.size() == 7) {
    theta << msg->position[3], msg->position[2], msg->position[0], msg->position[4], msg->position[5], msg->position[6];
  } else if (msg->name.size() == 12) {
    theta << msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], msg->position[5];
  } else {
    throw std::runtime_error("Joint state message has wrong size.");
  }

  return theta;
}

const bool working_position(ros::Rate &rate, UR5 &ur5, const Eigen::VectorXd &qEs, double max_t, bool new_table, std::string block_name) {
  Eigen::Vector3d pos;
  Eigen::Vector3d rot;

  if (new_table) {
    pos = drop_points[block_name];
    rot << 0.0, -M_PI, 0.0;

  } else {
    pos << -0.75, 0.0, 0.3;
    rot << -M_PI_2, -M_PI, 0.0;
  }

  ROS_INFO_STREAM("pos: " << pos.transpose());
  ROS_INFO_STREAM("rot: " << rot.transpose());

  execute_motion(rate, ur5, pos, rot, refresh_theta(), max_t);

  return true;
}

bool objects_position(ros::Rate &rate, UR5 &ur5, Gripper &gripper, Eigen::VectorXd qEs, Eigen::Vector3d &pos, std::string block_name, double block_rotation) {
  Eigen::Vector3d over_pos = pos + Eigen::Vector3d(0.0, 0.0, 0.2);
  Eigen::Vector3d rot = (Eigen::Vector3d() << block_rotation, -M_PI, 0.0).finished();
  ROS_INFO_STREAM("pos: " << pos.transpose());
  ROS_INFO_STREAM("rot: " << rot.transpose());

  execute_motion(rate, ur5, over_pos, rot, refresh_theta(), 1);
  execute_motion(rate, ur5, pos, rot, refresh_theta(), 0.5);

  gripper.push(0.2);
  gripper.attach_below();

  working_position(rate, ur5, refresh_theta(), 2, true, block_name); // new_table

  gripper.push(0.0);
  gripper.detach();

  return true;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "x_controller");
  ros::NodeHandle n;
  
  UR5 ur5(n);
  Gripper gripper(n);
  
  ros::Rate rate(100);

  ros::ServiceClient client = n.serviceClient<x_msgs::Blocks>("blocks");
  x_msgs::Blocks srv;
  
  if (client.call(srv)) {
    ROS_INFO_STREAM("Found " << srv.response.list.size() << " blocks");
  } else {
    ROS_INFO_STREAM("Failed to call service");
    return 1;
  }

  // test_position(rate, ur5, refresh_theta(),1);
  // working_position(rate, ur5, refresh_theta(), 2, true, argv);

  for (x_msgs::Block b : srv.response.list) {
    geometry_msgs::Point curr = b.obj;
    std::string block_name = b.label;
    double block_rotation = b.angle;
    Eigen::Vector3d pos = (Eigen::Vector3d() << curr.x, curr.y, curr.z).finished();
    objects_position(rate, ur5, gripper, refresh_theta(), pos, block_name, block_rotation);
  }

  working_position(rate, ur5, refresh_theta(), 2, false, "");

  ROS_INFO_STREAM("Success!");

  return 0;
}