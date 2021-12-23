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
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>

#include "x_controller/gripper.hpp"
#include "x_controller/kinematics.hpp"
#include "x_controller/ur5.hpp"

// temp
const bool working_position(ros::Rate &loop_rate, UR5 &ur5, const Eigen::VectorXd &qEs, int argc, char **argv) {

  // changing relative position/orientation based on working position

  // double x = std::atof(argv[1]);

  // double y = std::atof(argv[2]);

  // double z = std::atof(argv[3]);

  // double eul0 = std::atof(argv[4]);

  // double eul1 = std::atof(argv[5]);

  // double eul2 = std::atof(argv[6]);

  // Eigen::Vector3d pos = (Eigen::Vector3d() << -0.678645, -0.00214728, 0.507122).finished() + Eigen::Vector3d(x, y, z);
  // Eigen::Vector3d rot = (Eigen::Vector3d() << -1.60254, -3.15141, -0.0042679).finished() + Eigen::Vector3d(eul0, eul1, eul2);

  Eigen::Vector3d pos = (Eigen::Vector3d() << -0.678645, -0.00214728, 0.507122).finished();
  Eigen::Vector3d rot = (Eigen::Vector3d() << -1.60254, -3.15141, 0.0257321).finished();
  
  ROS_INFO_STREAM("pos: " << pos.transpose());
  ROS_INFO_STREAM("rot: " << rot.transpose());

  Eigen::MatrixXd dest_angles = Kinematics::ik(Kinematics::create_homogeneous_matrix(pos, Kinematics::eul2rotm(rot)));
  Eigen::VectorXd qEf = Kinematics::best_angles(qEs, dest_angles);

  Eigen::MatrixXd points = Kinematics::p2p(qEs, qEf);

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

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "f_controller");
  ros::NodeHandle n;
  UR5 ur5(n);
  Gripper gripper(n);
  ros::Rate loop_rate(100);

  gripper.push(0.4);
  gripper.attach("cube1", "link");

  sensor_msgs::JointState msg = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states"));
  Eigen::VectorXd theta(6);

  if (msg.name.size() == 7) {
    theta << msg.position[3], msg.position[2], msg.position[0], msg.position[4], msg.position[5], msg.position[6];
  } else if (msg.name.size() == 12) {
    theta << msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5];
  } else {
    throw std::runtime_error("Joint state message has wrong size");
  }

  if (working_position(loop_rate, ur5, theta, argc, argv)) {
    ROS_INFO("Success");
  }

  return 0;
}