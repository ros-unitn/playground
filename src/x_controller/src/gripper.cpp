#include "x_controller/gripper.hpp"

#include <x_linker/Link.h>
#include <x_linker/SetCollision.h>

#define GRIPPER_SERVICE_ATTACH "/x_linker_node/attach"
#define GRIPPER_SERVICE_DETACH "/x_linker_node/detach"
#define GRIPPER_SERVICE_SET_COLLISION "/x_linker_node/set_collision"

Gripper::Gripper(ros::NodeHandle &n) {
  m_action_client = new Gripper::ActionClient(n, GRIPPER_TOPIC);
  m_action_client->waitForServer();
  m_attach_client = n.serviceClient<x_linker::Link>(GRIPPER_SERVICE_ATTACH);
  m_detach_client = n.serviceClient<x_linker::Link>(GRIPPER_SERVICE_DETACH);
  m_set_collision_client = n.serviceClient<x_linker::SetCollision>(GRIPPER_SERVICE_SET_COLLISION);
}

Gripper::~Gripper() {
  delete m_action_client;
}

void Gripper::done(const actionlib::SimpleClientGoalState &state,
                   const control_msgs::GripperCommandResultConstPtr &result) {
  m_jobs--;
}

bool Gripper::push(float position, bool deferred) {
  Gripper::Goal goal;
  goal.command.position = position;
  goal.command.max_effort = -1;
  if (deferred) {
    m_action_client->sendGoal(goal, boost::bind(&Gripper::done, this, _1, _2), ActionClient::SimpleActiveCallback(), ActionClient::SimpleFeedbackCallback());
    m_jobs++;
    return true;
  }
  return m_action_client->sendGoalAndWait(goal).isDone();
}

size_t Gripper::remaining() const {
  return m_jobs;
}

bool Gripper::attach(const std::string &model_name, const std::string &link_name) {
  if (m_attached) {
    return false;
  }

  m_attached = true;

  x_linker::Link link;
  link.request.model_name_1 = GRIPPER_MODEL;
  link.request.link_name_1 = GRIPPER_LINK;
  link.request.model_name_2 = model_name;
  link.request.link_name_2 = link_name;

  bool call = m_attach_client.call(link);
  if (!call) {
    ROS_ERROR("attach service call failed");
    return false;
  }

  m_attached = link.response.ok;
  m_attached_model = link.response.model_name_2;
  m_attached_link = link.response.link_name_2;

  return link.response.ok;
}

bool Gripper::detach() {
  if (!m_attached) {
    return true;
  }

  x_linker::Link link;
  link.request.model_name_1 = GRIPPER_MODEL;
  link.request.link_name_1 = GRIPPER_LINK;
  link.request.model_name_2 = m_attached_model;
  link.request.link_name_2 = m_attached_link;

  bool call = m_detach_client.call(link);
  if (!call) {
    ROS_ERROR("detach service call failed");
    return false;
  }

  m_attached = !link.response.ok;

  return link.response.ok;
}

bool Gripper::enable_collisions(std::string model) {
  x_linker::SetCollision set_collision;
  set_collision.request.model_name = GRIPPER_MODEL;
  set_collision.request.mode = "all";
  bool call = m_set_collision_client.call(set_collision);
  if (!call) {
    ROS_ERROR("enable_collisions service call failed");
    return false;
  }

  return set_collision.response.ok;
}

bool Gripper::disable_collisions(std::string model) {
  x_linker::SetCollision set_collision;
  set_collision.request.model_name = GRIPPER_MODEL;
  set_collision.request.mode = "none";
  bool call = m_set_collision_client.call(set_collision);
  if (!call) {
    ROS_ERROR("disable_collisions service call failed");
    return false;
  }

  return set_collision.response.ok;
}