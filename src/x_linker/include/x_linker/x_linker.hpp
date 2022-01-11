#pragma once

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

#include "x_linker/Link.h"
#include "x_linker/LinkRequest.h"
#include "x_linker/LinkResponse.h"

namespace gazebo {

class LinkerPlugin : public WorldPlugin {
public:
  struct FixedJoint {
    std::string model_name_1;
    physics::ModelPtr model_1;
    std::string link_name_1;
    physics::LinkPtr link_1;
    std::string model_name_2;
    physics::ModelPtr model_2;
    std::string link_name_2;
    physics::LinkPtr link_2;
    physics::JointPtr joint;
  };

  LinkerPlugin();
  void Load(physics::WorldPtr _world, sdf::ElementPtr);

  bool attach(std::string model_name_1, std::string link_name_1,
              std::string model_name_2, std::string link_name_2, FixedJoint &fixed);

  bool detach(std::string model_name_1, std::string link_name_1,
              std::string model_name_2, std::string link_name_2, FixedJoint &fixed);

  bool get_joint(std::string model_name_1, std::string link_name_1,
                std::string model_name_2, std::string link_name_2, FixedJoint &fixed);

protected:
  bool attach_callback(x_linker::Link::Request &req,
                       x_linker::Link::Response &res);
  bool detach_callback(x_linker::Link::Request &req,
                       x_linker::Link::Response &res);

private:
  ros::NodeHandle m_n;
  ros::ServiceServer m_attach_server;
  ros::ServiceServer m_detach_server;
  ros::ServiceServer m_attach_below_server;
  std::vector<FixedJoint> m_joints;
  physics::PhysicsEnginePtr m_physics;
  physics::WorldPtr m_world;
};

GZ_REGISTER_WORLD_PLUGIN(LinkerPlugin)

} // namespace gazebo