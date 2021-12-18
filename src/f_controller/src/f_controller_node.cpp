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

#include "f_controller/m_kinematics.hpp"
#include "f_controller/ur5.hpp"
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "f_controller");
  ros::NodeHandle n;
  UR5 ur5(n);
  ros::Rate loop_rate(100);

  sensor_msgs::JointState msg = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states"));
  Eigen::VectorXd theta(6);
  if (msg.name.size() == 7) {
    theta << msg.position[3], msg.position[2], msg.position[0], msg.position[4], msg.position[5], msg.position[6];
  } else if (msg.name.size() == 12) {
    theta << msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5];
  } else {
    throw std::runtime_error("Joint state message has wrong size");
  }

  double x=std::atof(argv[1]);

  double y=std::atof(argv[2]);

  double z=std::atof(argv[3]);

  double eul0=std::atof(argv[4]);

  double eul1=std::atof(argv[5]);

  double eul2=std::atof(argv[6]);

  ROS_INFO("theta: %f %f %f %f %f %f", theta(0), theta(1), theta(2), theta(3),
           theta(4), theta(5));
  //std::cout << "Actual position =\t" << KIN::get_position(mat).transpose() << std::endl;
  //std::cout << "Actual orientation =\t" << KIN::rotm2eul(KIN::get_orientation(mat)).transpose() << std::endl;
  Eigen::Vector3d pos = (Eigen::Vector3d() << -0.678645, -0.00214728, 0.507122).finished()+Eigen::Vector3d(x,y,z);
  Eigen::Vector3d rot = (Eigen::Vector3d() << -1.60254, -3.13141, 0.0257321).finished()+Eigen::Vector3d(eul0,eul1,eul2); //un po' storto
  std::cout << "Desired position =\t" << pos.transpose() << std::endl;
  std::cout << "Desired orientation =\t" << rot.transpose() << std::endl;
  Eigen::MatrixXd res = KIN::p2p(theta, pos, rot);
  //std::cout << res << std::endl;

  /*//Eigen::Vector3d pos=(Eigen::Vector3d() << 0.4572, 0.4572, 0.7747).finished();
  Eigen::Vector3d pos=(Eigen::Vector3d() << x,y,z).finished();
  Eigen::Vector3d eul=(Eigen::Vector3d() << eul0,eul1,eul2).finished();
  Eigen::Matrix3d rot=KIN::eul2rotm(eul);

  Eigen::Matrix4d mat=KIN::createHomogeneousMatrix(pos,rot);
  Eigen::MatrixXd thetas=KIN::ik(mat);*/

  for (int i = 0; i < res.rows(); i++) {
    UR5::JointParams params;
    params.set(UR5::Joint::UR5_SHOULDER_PAN, res(i, 0));
    params.set(UR5::Joint::UR5_SHOULDER_LIFT, res(i, 1));
    params.set(UR5::Joint::UR5_ELBOW, res(i, 2));
    params.set(UR5::Joint::UR5_WRIST_1, res(i, 3));
    params.set(UR5::Joint::UR5_WRIST_2, res(i, 4));
    params.set(UR5::Joint::UR5_WRIST_3, res(i, 5));

    ur5.push(params);
  }

  while (ros::ok() && ur5.remaining()!=0) {
    ur5.tick();
    ur5.publish();
    ros::spinOnce();
    loop_rate.sleep();
  }

/*msg = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states"));
  if (msg.name.size() == 7) {
    theta << msg.position[3], msg.position[2], msg.position[0], msg.position[4], msg.position[5], msg.position[6];
  } else if (msg.name.size() == 12) {
    theta << msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5];
  } else {
    throw std::runtime_error("Joint state message has wrong size");
  }


  Eigen::Matrix4d fin=KIN::fk(theta);
  std::cout << "Final position =\t" << KIN::get_position(fin).transpose() << std::endl;
  std::cout << "Final orientation =\t" << KIN::rotm2eul(KIN::get_orientation(fin)).transpose() << std::endl;*/
  return 0;
}