#include "x_controller/gripper.hpp"

#include <x_linker/Link.h>
#include <x_linker/SetCollision.h>

#define GRIPPER_SERVICE_ATTACH "/x_linker_node/attach"
#define GRIPPER_SERVICE_DETACH "/x_linker_node/detach"
#define GRIPPER_SERVICE_SET_COLLISION "/x_linker_node/set_collision"

Gripper::Gripper(ros::NodeHandle &n) {
  m_client = new Gripper::Client(n, GRIPPER_TOPIC);
  m_client->waitForServer();
  m_attach_client = n.serviceClient<x_linker::Link>(GRIPPER_SERVICE_ATTACH);
  m_detach_client = n.serviceClient<x_linker::Link>(GRIPPER_SERVICE_DETACH);
  m_set_collision = n.serviceClient<x_linker::SetCollision>(GRIPPER_SERVICE_SET_COLLISION);
}

Gripper::~Gripper() {
  delete m_client;
}

void Gripper::done(const actionlib::SimpleClientGoalState &state,
                   const control_msgs::GripperCommandResultConstPtr &result) {
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Effort: %i", result->effort);
  ROS_INFO("Position: %i", result->position);
  ROS_INFO("Reached: %i", result->reached_goal);
  ROS_INFO("Stalled: %i", result->stalled);
  m_jobs--;
}

void Gripper::push(float position) {
  Gripper::Goal goal;
  goal.command.position = position;
  goal.command.max_effort = -1;
  m_client->sendGoal(goal, boost::bind(&Gripper::done, this, _1, _2), Client::SimpleActiveCallback(), Client::SimpleFeedbackCallback());
  m_jobs++;
}

size_t Gripper::remaining() const {
  return m_jobs;
}

bool Gripper::attach(const std::string &model, const std::string &link) {
  if (m_attached) {
    return false;
  }
  
  m_attached = true;

  x_linker::Link srv;
  srv.request.model_name_1 = GRIPPER_MODEL;
  srv.request.link_name_1 = GRIPPER_LINK;
  srv.request.model_name_2 = model;
  srv.request.link_name_2 = link;

  bool call = m_attach_client.call(srv);
  if (!call) {
    ROS_ERROR("attach service call failed");
    return false;
  }

  m_attached = srv.response.ok;
  m_attached_model = srv.response.model_name_2;
  m_attached_link = srv.response.link_name_2;

  return srv.response.ok;
}

bool Gripper::detach() {
  if (!m_attached) {
    return true;
  }

  x_linker::Link srv;
  srv.request.model_name_1 = GRIPPER_MODEL;
  srv.request.link_name_1 = GRIPPER_LINK;
  srv.request.model_name_2 = m_attached_model;
  srv.request.link_name_2 = m_attached_link;

  bool call = m_detach_client.call(srv);
  if (!call) {
    ROS_ERROR("attach service call failed");
    return false;
  }

  m_attached = !srv.response.ok;

  return srv.response.ok;
}

bool Gripper::enable_collisions(std::string model) {
  x_linker::SetCollision srv;
  srv.request.model_name = GRIPPER_MODEL;
  srv.request.mode = "all";
  bool call = m_set_collision.call(srv);
  if (!call) {
    ROS_ERROR("enable_collisions service call failed");
    return false;
  }

  return srv.response.ok;
}

bool Gripper::disable_collisions(std::string model) {
  x_linker::SetCollision srv;
  srv.request.model_name = GRIPPER_MODEL;
  srv.request.mode = "none";
  bool call = m_set_collision.call(srv);
  if (!call) {
    ROS_ERROR("disable_collisions service call failed");
    return false;
  }
  
  return srv.response.ok;
}