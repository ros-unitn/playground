#pragma once

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

#include <x_linker/Link.h>
#include <x_linker/LinkRequest.h>
#include <x_linker/LinkResponse.h>

namespace gazebo {

class LinkerPlugin : public WorldPlugin {
public:
  struct FixedJoint {
    std::string model1;
    physics::ModelPtr m1;
    std::string link1;
    physics::LinkPtr l1;
    std::string model2;
    physics::ModelPtr m2;
    std::string link2;
    physics::LinkPtr l2;
    physics::JointPtr joint;
  };

  LinkerPlugin();
  void Load(physics::WorldPtr _world, sdf::ElementPtr);

  bool attach(std::string model1, std::string link1,
              std::string model2, std::string link2);

  bool detach(std::string model1, std::string link1,
              std::string model2, std::string link2);

  bool get_joint(std::string model1, std::string link1,
                std::string model2, std::string link2, FixedJoint &joint);

protected:
  bool attach_callback(x_linker::Link::Request &req,
                       x_linker::Link::Response &res);
  bool detach_callback(x_linker::Link::Request &req,
                       x_linker::Link::Response &res);

private:
  ros::NodeHandle m_n;
  ros::ServiceServer m_attach_server;
  ros::ServiceServer m_detach_server;
  std::vector<FixedJoint> m_joints;
  physics::PhysicsEnginePtr m_physics;
  physics::WorldPtr m_world;
};

GZ_REGISTER_WORLD_PLUGIN(LinkerPlugin)

} // namespace gazebo