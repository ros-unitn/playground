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
#include <exception>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <map>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>

#include "x_controller/vendor/linenoise.hpp"

#include "x_controller/gripper.hpp"
#include "x_controller/kinematics.hpp"
#include "x_controller/ur5.hpp"
#include "x_msgs/Block.h"
#include "x_msgs/Blocks.h"

struct Brick {
  x_msgs::Block block;
  int key;

  Brick(x_msgs::Block b) {
    block = b;

    if (block.label == "X1-Y1-Z2")
      key = 1;
    else if (block.label == "X1-Y2-Z1")
      key = 2;
    else if (block.label == "X1-Y4-Z1")
      key = 3;
    else if (block.label == "X1-Y2-Z2")
      key = 4;
    else if (block.label == "X1-Y2-Z2-TWINFILLET")
      key = 5;
    else if (block.label == "X1-Y2-Z2-CHAMFER")
      key = 6;
    else if (block.label == "X1-Y3-Z2")
      key = 7;
    else if (block.label == "X1-Y3-Z2-FILLET")
      key = 8;
    else if (block.label == "X1-Y4-Z2")
      key = 9;
    else if (block.label == "X2-Y2-Z2")
      key = 10;
    else if (block.label == "X2-Y2-Z2-FILLET")
      key = 11;
  }

  bool operator<(const Brick &b) const {
    return key < b.key;
  }
};

bool is_castle = false;
int count = 0;

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

void execute_motion(ros::Rate &rate, UR5 &ur5, const Eigen::Vector3d &pos, const Eigen::Vector3d &rot, const Eigen::VectorXd &qEs, double max_t, Eigen::VectorXd qEf = Eigen::VectorXd::Zero(6)) {
  if (qEf == Eigen::VectorXd::Zero(6)) {
    Eigen::MatrixXd dest_angles = Kinematics::ik(Kinematics::create_homogeneous_matrix(pos, Kinematics::eul2rotm(rot)));
    qEf = Kinematics::best_angles(qEs, dest_angles);
  }

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
}

Eigen::VectorXd refresh_theta() {
  sensor_msgs::JointStateConstPtr msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");

  if (msg == nullptr) {
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

void working_position(ros::Rate &rate, UR5 &ur5, const Eigen::VectorXd &qEs, Eigen::Vector3d rot, double max_t, std::string block_name, double inclination) {
  Eigen::Vector3d pos = drop_points[block_name];

  double x;
  if (block_name == "X1-Y1-Z2" || block_name == "X1-Y2-Z1" || block_name == "X1-Y4-Z1")
    x = -0.1 * sin(inclination);
  else if (block_name == "X1-Y4-Z2" || block_name == "X2-Y2-Z2" || block_name == "X2-Y2-Z2-FILLET")
    x = 0.1 * sin(inclination);
  else
    x = 0;

  // flip x if blocks are on right side

  if (inclination < 0)
    pos -= Eigen::Vector3d(x, 0.025 * sin(inclination), 0);
  else if (inclination > 0)
    pos += Eigen::Vector3d(-x, 0.025 * sin(inclination), 0);

  ROS_INFO_STREAM("Position: " << pos.transpose());
  ROS_INFO_STREAM("Rotation: " << rot.transpose());

  execute_motion(rate, ur5, pos, rot, refresh_theta(), max_t);
}

void castle(ros::Rate &rate, UR5 &ur5, const Eigen::VectorXd &qEs, Eigen::Vector3d rot, double max_t, std::string block_name, double inclination) {

  Eigen::Vector3d pos;

  switch (count) {
  case 0:
    pos << -0.07, -0.53, 0.25;
    break;
  case 1:
    pos << 0, -0.53, 0.25;
    break;
  case 2:
    pos << 0.07, -0.53, 0.25;
    break;
  case 3:
    pos << -0.07, -0.6, 0.25;
    break;
  case 4:
    pos << 0, -0.6, 0.25;
    break;
  case 5:
    pos << 0.07, -0.6, 0.25;
    break;
  case 6:
    pos << -0.07, -0.67, 0.25;
    break;
  case 7:
    pos << 0, -0.67, 0.25;
    break;
  case 8:
    pos << 0.07, -0.67, 0.25;
    break;
  case 9:
    pos << -0.035, -0.565, 0.3;
    break;
  case 10:
    pos << 0.035, -0.565, 0.3;
    break;
  case 11:
    pos << -0.035, -0.635, 0.3;
    break;
  case 12:
    pos << 0.035, -0.635, 0.3;
    break;
  case 13:
    pos << 0, -0.6, 0.35;
    break;
  case 14:
    pos << 0, -0.6, 0.4;
    break;
  }

  count++;

  double x;
  if (block_name == "X1-Y1-Z2" || block_name == "X1-Y2-Z1" || block_name == "X1-Y4-Z1")
    x = -0.2 * sin(inclination);
  else if (block_name == "X1-Y4-Z2" || block_name == "X2-Y2-Z2" || block_name == "X2-Y2-Z2-FILLET")
    x = 0.2 * sin(inclination);
  else
    x = 0;

  // flip x if blocks are on right side

  if (inclination < 0)
    pos -= Eigen::Vector3d(x, 0.1 * sin(inclination), 0);
  else if (inclination > 0)
    pos += Eigen::Vector3d(-x, 0.1 * sin(inclination), 0);

  ROS_INFO_STREAM("Position: " << pos.transpose());
  ROS_INFO_STREAM("Rotation: " << rot.transpose());

  execute_motion(rate, ur5, pos, rot, refresh_theta(), max_t);
}

void object_position(ros::Rate &rate, UR5 &ur5, Gripper &gripper, Eigen::VectorXd qEs, Eigen::Vector3d &pos, std::string block_name, double rotation, double inclination) {

  Eigen::Vector3d over_pos = pos + Eigen::Vector3d(0.0, 0.0, 0.1);
  if (rotation > 0) {
    rotation = (abs(rotation - M_PI) < abs(rotation)) ? rotation - M_PI : rotation;
  } else {
    rotation = (abs(rotation + M_PI) < abs(rotation)) ? rotation + M_PI : rotation;
  }
  Eigen::Vector3d rot = (Eigen::Vector3d() << -rotation, -M_PI, 0).finished();

  ROS_INFO_STREAM("Position: " << pos.transpose());
  ROS_INFO_STREAM("Rotation: " << rot.transpose());

  execute_motion(rate, ur5, over_pos, rot, refresh_theta(), 2);

  execute_motion(rate, ur5, pos, rot, refresh_theta(), 0.5);

  if (inclination == 0) {
    if (block_name.find("X1") != std::string::npos)
      gripper.push(0.5);
    else
      gripper.push(0.4);
  } else {
    if (block_name.find("Z1") != std::string::npos)
      gripper.push(0.5);
    else
      gripper.push(0.4);
  }

  gripper.attach(block_name, "link");

  execute_motion(rate, ur5, over_pos, rot, refresh_theta(), 0.5);

  rot = rot + Eigen::Vector3d(0.0, inclination, -inclination);

  if (!is_castle)
    working_position(rate, ur5, refresh_theta(), rot, 2, block_name, inclination);
  else
    castle(rate, ur5, refresh_theta(), rot, 2, block_name, inclination);

  gripper.push(0.0, true);
  gripper.detach();
}

void reset(ros::NodeHandle &, UR5 &ur5, Gripper &, ros::Rate &rate, Eigen::VectorXd &original) {
  execute_motion(rate, ur5, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), refresh_theta(), 1, original);
}

void classify(ros::NodeHandle &n, UR5 &ur5, Gripper &gripper, ros::Rate &rate, Eigen::VectorXd &original) {
  ros::ServiceClient client = n.serviceClient<x_msgs::Blocks>("blocks");
  x_msgs::Blocks blocks;

  if (!client.call(blocks)) {
    throw std::runtime_error("Failed to call service");
  }

  gripper.disable_collisions();

  if (blocks.response.list.size() == 0) {
    ROS_INFO_STREAM("No blocks detected");
    return;
  }

  if (blocks.response.list.size() == 15) {
    is_castle = true;
  }

  while (blocks.response.list.size() > 0) {

    ROS_INFO_STREAM("Found " << blocks.response.list.size() << " bricks");

    std::vector<Brick> bricks;

    for (x_msgs::Block block : blocks.response.list) {
      bricks.push_back(Brick(block));
    }

    std::sort(bricks.begin(), bricks.end(), std::less<Brick>());

    geometry_msgs::Point curr = bricks[0].block.obj;
    std::string block_name = bricks[0].block.label;
    ROS_INFO_STREAM("Current block: " << block_name);
    double rotation = bricks[0].block.angle;
    double inclination = bricks[0].block.inclination;
    Eigen::Vector3d pos = (Eigen::Vector3d() << curr.x, curr.y, curr.z).finished();

    object_position(rate, ur5, gripper, refresh_theta(), pos, block_name, rotation, inclination);

    if (!client.call(blocks)) {
      throw std::runtime_error("Failed to call service");
    }
  }

  reset(n, ur5, gripper, rate, original);
  gripper.enable_collisions();
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "x_controller");
  ros::NodeHandle n;

  UR5 ur5(n);
  Gripper gripper(n);

  ros::Rate rate(100);

  Eigen::VectorXd original = refresh_theta();

  linenoise::SetCompletionCallback([](const char *buf, std::vector<std::string> &completions) {
    if (buf[0] == 'c') {
      completions.push_back("classify");
    } else if (buf[0] == 'r') {
      completions.push_back("reset");
    }
  });

  while (true) {
    std::string line;
    auto quit = linenoise::Readline("ur5 > ", line);

    if (quit) {
      break;
    }

    try {
      if (line == "classify") {
        classify(n, ur5, gripper, rate, original);
      } else if (line == "reset") {
        reset(n, ur5, gripper, rate, original);
      } else {
        ROS_ERROR_STREAM("Command \"" << line << "\" not found!");
      }
    } catch (const std::exception &exception) {
      ROS_ERROR_STREAM("Encountered an error: " << exception.what());
    }

    linenoise::AddHistory(line.c_str());
  }

  return 0;
}