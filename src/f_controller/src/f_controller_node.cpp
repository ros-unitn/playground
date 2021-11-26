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