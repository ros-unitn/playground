#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "f_controller/ur5.hpp"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "f_controller");
  ros::NodeHandle n;

  UR5 ur5(n);

  ros::Rate loop_rate(0.5); // Hz

  while (ros::ok()) {
    ur5.tick();

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}