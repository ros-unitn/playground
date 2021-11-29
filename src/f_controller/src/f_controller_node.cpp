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

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "f_controller/ur5.hpp"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "f_controller");
  ros::NodeHandle n;

  UR5 ur5(n);

  ros::Rate loop_rate(1); // Hz

  UR5::JointParams params;
  params.set(UR5::Joint::UR5_SHOULDER_LIFT, -0.5);
  params.set(UR5::Joint::UR5_SHOULDER_PAN, -0.5);

  std::cout << "push" << std::endl;

  ur5.push(params);

  while (ros::ok()) {
    std::cout << "tick" << std::endl;
    ur5.tick();

    std::cout << "publish" << std::endl;
    ur5.publish();

    ros::spinOnce();

    std::cout << "sleep" << std::endl;
    loop_rate.sleep();
  }

  return 0;
}