#pragma once

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

#include "x_linker/Link.h"
#include "x_linker/LinkRequest.h"
#include "x_linker/LinkResponse.h"
#include "x_linker/SetCollision.h"
#include "x_linker/SetCollisionRequest.h"
#include "x_linker/SetCollisionResponse.h"

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

  void update();

  bool attach(std::string model_name_1, std::string link_name_1,
              std::string model_name_2, std::string link_name_2, FixedJoint &fixed);

  bool detach(std::string model_name_1, std::string link_name_1,
              std::string model_name_2, std::string link_name_2, FixedJoint &fixed);

  bool set_collision(std::string model_name, std::string mode);

  bool get_joint(std::string model_name_1, std::string link_name_1,
                 std::string model_name_2, std::string link_name_2, FixedJoint &fixed);

protected:
  bool attach_callback(x_linker::Link::Request &req,
                       x_linker::Link::Response &res);
  bool detach_callback(x_linker::Link::Request &req,
                       x_linker::Link::Response &res);
                      
  bool set_collision_callback(x_linker::SetCollision::Request &req,
                              x_linker::SetCollision::Response &res);

private:
  ros::NodeHandle m_n;
  ros::ServiceServer m_attach_server;
  ros::ServiceServer m_detach_server;
  ros::ServiceServer m_set_collision_server;
  std::vector<FixedJoint> m_joints;
  physics::PhysicsEnginePtr m_physics;
  physics::WorldPtr m_world;
  event::ConnectionPtr m_update_conn;
  std::queue<FixedJoint> m_attach_queue;
  std::mutex m_attach_queue_mutex;
  std::queue<FixedJoint> m_detach_queue;
  std::mutex m_detach_queue_mutex;
};

GZ_REGISTER_WORLD_PLUGIN(LinkerPlugin)

} // namespace gazebo