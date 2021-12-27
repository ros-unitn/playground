#include "x_linker/x_linker.hpp"

#include <gazebo/physics/physics.hh>

namespace gazebo {

LinkerPlugin::LinkerPlugin() : WorldPlugin(), m_n("x_linker_node") {}

void LinkerPlugin::Load(physics::WorldPtr world, sdf::ElementPtr) {
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libx_linker.so' in the gazebo_ros package)");
    return;
  }

  m_world = world;
  m_attach_server = m_n.advertiseService("attach", &LinkerPlugin::attach_callback, this);
  m_detach_server = m_n.advertiseService("detach", &LinkerPlugin::detach_callback, this);
  ROS_INFO_STREAM("Attach service at: " << m_n.resolveName("attach"));
  ROS_INFO_STREAM("Detach service at: " << m_n.resolveName("detach"));
  ROS_INFO("Link attacher node initialized.");
}

bool LinkerPlugin::attach(std::string model1, std::string link1,
                          std::string model2, std::string link2) {

  // look for any previous instance of the joint first.
  // if we try to create a joint in between two links
  // more than once (even deleting any reference to the first one)
  // gazebo hangs/crashes
  FixedJoint j;
  if (get_joint(model1, link1, model2, link2, j)) {
    ROS_INFO_STREAM("Joint already existed, reusing it.");
    j.joint->Attach(j.l1, j.l2);
    return true;
  } else {
    ROS_INFO_STREAM("Creating new joint.");
  }
  j.model1 = model1;
  j.link1 = link1;
  j.model2 = model2;
  j.link2 = link2;
  ROS_DEBUG_STREAM("Getting BasePtr of " << model1);
  physics::BasePtr b1 = m_world->ModelByName(model1);

  if (b1 == NULL) {
    ROS_ERROR_STREAM(model1 << " model was not found");
    return false;
  }
  ROS_DEBUG_STREAM("Getting BasePtr of " << model2);
  physics::BasePtr b2 = m_world->ModelByName(model2);
  if (b2 == NULL) {
    ROS_ERROR_STREAM(model2 << " model was not found");
    return false;
  }

  ROS_DEBUG_STREAM("Casting into ModelPtr");
  physics::ModelPtr m1(dynamic_cast<physics::Model *>(b1.get()));
  j.m1 = m1;
  physics::ModelPtr m2(dynamic_cast<physics::Model *>(b2.get()));
  j.m2 = m2;

  ROS_DEBUG_STREAM("Getting link: '" << link1 << "' from model: '" << model1 << "'");
  physics::LinkPtr l1 = m1->GetLink(link1);
  if (l1 == NULL) {
    ROS_ERROR_STREAM(link1 << " link was not found");
    return false;
  }
  if (l1->GetInertial() == NULL) {
    ROS_ERROR_STREAM("link1 inertia is NULL!");
  } else
    ROS_DEBUG_STREAM("link1 inertia is not NULL, for example, mass is: " << l1->GetInertial()->Mass());
  j.l1 = l1;
  ROS_DEBUG_STREAM("Getting link: '" << link2 << "' from model: '" << model2 << "'");
  physics::LinkPtr l2 = m2->GetLink(link2);
  if (l2 == NULL) {
    ROS_ERROR_STREAM(link2 << " link was not found");
    return false;
  }
  if (l2->GetInertial() == NULL) {
    ROS_ERROR_STREAM("link2 inertia is NULL!");
  } else
    ROS_DEBUG_STREAM("link2 inertia is not NULL, for example, mass is: " << l2->GetInertial()->Mass());
  j.l2 = l2;

  ROS_DEBUG_STREAM("Links are: " << l1->GetName() << " and " << l2->GetName());

  ROS_DEBUG_STREAM("Creating revolute joint on model: '" << model1 << "'");
  j.joint = m_world->Physics()->CreateJoint("revolute", m1);
  m_joints.push_back(j);

  j.joint->Attach(l1, l2);
  j.joint->Load(l1, l2, ignition::math::Pose3d());
  j.joint->SetModel(m2);

  j.joint->SetUpperLimit(0, 0);
  j.joint->SetLowerLimit(0, 0);
  j.joint->Init();

  return true;
}

bool LinkerPlugin::detach(std::string model1, std::string link1,
                          std::string model2, std::string link2) {
  // search for the instance of joint and do detach
  FixedJoint j;
  if (get_joint(model1, link1, model2, link2, j)) {
    j.joint->Detach();
    return true;
  }

  return false;
}

bool LinkerPlugin::get_joint(std::string model1, std::string link1,
                             std::string model2, std::string link2,
                             FixedJoint &joint) {
  FixedJoint j;
  for (std::vector<FixedJoint>::iterator it = m_joints.begin(); it != m_joints.end(); ++it) {
    j = *it;
    if ((j.model1.compare(model1) == 0) && (j.model2.compare(model2) == 0) && (j.link1.compare(link1) == 0) && (j.link2.compare(link2) == 0)) {
      joint = j;
      return true;
    }
  }
  return false;
}

bool LinkerPlugin::attach_callback(x_linker::Link::Request &req,
                                   x_linker::Link::Response &res) {
  ROS_INFO_STREAM("Received request to attach model: '"
                  << req.model_name_1
                  << "' using link: '" << req.link_name_1
                  << "' with model: '" << req.model_name_2
                  << "' using link: '" << req.link_name_2 << "'");
  if (!attach(req.model_name_1, req.link_name_1,
              req.model_name_2, req.link_name_2)) {
    ROS_ERROR_STREAM("Could not attach.");
    res.ok = false;
  } else {
    ROS_INFO_STREAM("Attach was succesful");
    res.ok = true;
  }
  return true;
}

bool LinkerPlugin::detach_callback(x_linker::Link::Request &req,
                                   x_linker::Link::Response &res) {
  ROS_INFO_STREAM("Received request to detach model: '"
                  << req.model_name_1
                  << "' using link: '" << req.link_name_1
                  << "' with model: '" << req.model_name_2
                  << "' using link: '" << req.link_name_2 << "'");
  if (!detach(req.model_name_1, req.link_name_1,
              req.model_name_2, req.link_name_2)) {
    ROS_ERROR_STREAM("Could not detach.");
    res.ok = false;
  } else {
    ROS_INFO_STREAM("Detach was successful");
    res.ok = true;
  }
  return true;
}

} // namespace gazebo