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

#include <geometry_msgs/Point.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>

#include "x_controller/gripper.hpp"
#include "x_controller/kinematics.hpp"
#include "x_controller/ur5.hpp"
#include "x_msgs/Block.h"
#include "x_msgs/Blocks.h"

bool execute_motion(ros::Rate &loop_rate, UR5 &ur5, const Eigen::Vector3d &pos, const Eigen::Vector3d &rot, const Eigen::VectorXd &qEs, double maxT) {
  Eigen::MatrixXd dest_angles = Kinematics::ik(Kinematics::create_homogeneous_matrix(pos, Kinematics::eul2rotm(rot)));
  Eigen::VectorXd qEf = Kinematics::best_angles(qEs, dest_angles);

  Eigen::MatrixXd points = Kinematics::p2p(qEs, qEf, maxT);

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

  while (ros::ok() && ur5.remaining() != 0) {
    ur5.tick();
    ur5.publish();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return true;
}

Eigen::VectorXd refresh_theta() {
  sensor_msgs::JointState msg = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states"));
  Eigen::VectorXd theta(6);

  if (msg.name.size() == 7) {
    theta << msg.position[3], msg.position[2], msg.position[0], msg.position[4], msg.position[5], msg.position[6];
  } else if (msg.name.size() == 12) {
    theta << msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5];
  } else {
    throw std::runtime_error("Joint state message has wrong size");
  }

  return theta;
}

const bool working_position(ros::Rate &loop_rate, UR5 &ur5, const Eigen::VectorXd &qEs, double maxT, bool newTable) {

  Eigen::Vector3d pos = (Eigen::Vector3d() << -0.678645, -0.00214728, 0.507122).finished();
  Eigen::Vector3d rot = (Eigen::Vector3d() << -1.60254, -3.15141, 0.0257321).finished();

  if(newTable){
    Eigen::Matrix3d rotate_z_left = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pos=rotate_z_left*pos;
    std::cout << "pos:" << pos.transpose() << std::endl;
    rot=rot+Eigen::Vector3d(M_PI_2, 0.0, 0.0);
    std::cout << "rot: " << rot.transpose() << std::endl;
    
    
  } else {
    std::cout << "pos:" << pos.transpose() << std::endl;
    std::cout << "rot: " << rot.transpose() << std::endl;
  }

  execute_motion(loop_rate, ur5, pos, rot, refresh_theta(), maxT);

  return true;

}

bool objects_position(ros::Rate &loop_rate, UR5 &ur5, Gripper &gripper, Eigen::VectorXd qEs, Eigen::Vector3d &pos) {
  
  Eigen::Vector3d over_pos = pos + Eigen::Vector3d(0.0, 0.0, 0.2);
  std::cout << "pos: " << pos.transpose() << std::endl;
  Eigen::Vector3d rot = (Eigen::Vector3d() << -1.60254, -3.15141, 0.0257321).finished();
  std::cout << "rot: " << rot.transpose() << std::endl;
  execute_motion(loop_rate, ur5, over_pos, rot, refresh_theta(), 1);
  execute_motion(loop_rate, ur5, pos, rot, refresh_theta(), 0.5);
  gripper.attach("X2-Y2-Z2", "X2-Y2-Z2::link");
  //execute_motion(loop_rate, ur5, over_pos, rot, refresh_theta(), 0.5);
  //working_position(loop_rate, ur5, refresh_theta(), 1, false);
  working_position(loop_rate, ur5, refresh_theta(), 2, true); //newTable
  //execute_motion(loop_rate, ur5, pos, rot, refresh_theta(), 0.5);
  gripper.detach();

  return true;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "x_controller");
  ros::NodeHandle n;
  UR5 ur5(n);
  Gripper gripper(n);
  ros::Rate loop_rate(100);

  ros::ServiceClient client = n.serviceClient<x_msgs::Blocks>("blocks");
  x_msgs::Blocks srv;
  if (client.call(srv)) {
    std::cout << "Called service" << std::endl;
  } else {
    std::cout << "Failed to call service" << std::endl;
    return 1;
  }
  //test_position(loop_rate, ur5, refresh_theta(),1);
  //working_position(loop_rate, ur5, refresh_theta(), 1);

  for (x_msgs::Block b : srv.response.list) {
    geometry_msgs::Point curr = b.obj;
    Eigen::Vector3d pos = (Eigen::Vector3d() << curr.x, curr.y, curr.z).finished();
    objects_position(loop_rate, ur5, gripper, refresh_theta(), pos);
    //gripper.push(0.1);
    //gripper.attach("X2-Y2-Z2", "X2-Y2-Z2::link");
  }

  working_position(loop_rate, ur5, refresh_theta(), 2, false);

  std::cout << "Success!" << std::endl;

  return 0;
}