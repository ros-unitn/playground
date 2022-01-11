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

bool LinkerPlugin::attach(std::string model_name_1, std::string link_name_1,
                          std::string model_name_2, std::string link_name_2,
                             FixedJoint &fixed) {

  ROS_INFO_STREAM("Getting BasePtr of " << model_name_1);
  physics::BasePtr base_1 = m_world->ModelByName(model_name_1);
  if (base_1 == nullptr) {
    ROS_ERROR_STREAM(model_name_1 << " model was not found");
    return false;
  }

  physics::ModelPtr model_1(dynamic_cast<physics::Model *>(base_1.get()));
  fixed.model_1 = model_1;

  ROS_INFO_STREAM("Getting link: '" << link_name_1 << "' from model: '" << model_1->GetName() << "'");
  physics::LinkPtr link_1 = model_1->GetLink(link_name_1);
  if (link_1 == nullptr) {
    ROS_ERROR_STREAM(link_name_1 << " link was not found");
    return false;
  }

  fixed.link_1 = link_1;

  ROS_INFO_STREAM("Getting BasePtr of " << model_name_2);
  physics::BasePtr base_2 = m_world->ModelByName(model_name_2);
  if (base_2 == nullptr) {
    ROS_ERROR_STREAM(model_name_2 << " model was not found. Searching for a subset.");

    physics::Model_V haystack = m_world->Models();
    physics::ModelPtr needle = nullptr;
    ignition::math::Vector3d lhs = link_1->BoundingBox().Center();
    ROS_ERROR_STREAM("Link position " << lhs);
    double best = std::numeric_limits<double>::infinity();
    for (physics::ModelPtr model : haystack) {
      ROS_ERROR_STREAM("Evaluating " << model->GetName());
      if (model->GetName().find(model_name_2) != std::string::npos) {
        // name contains the string we are searching for
        ignition::math::Vector3d rhs = model->WorldPose().Pos();
        ROS_ERROR_STREAM("Distance " << lhs.Distance(rhs));
        double distance = lhs.Distance(rhs);
        if (distance < best) {
          needle = model;
          best = distance;
        }
      }
    }

    if (needle == nullptr) {
      ROS_ERROR_STREAM(model_name_2 << " model was not found. No maching model found.");
      return false;
    }

    base_2 = needle;
  }

  physics::ModelPtr model_2(dynamic_cast<physics::Model *>(base_2.get()));
  fixed.model_2 = model_2;

  ROS_INFO_STREAM("Getting link: '" << link_name_2 << "' from model: '" << model_2->GetName() << "'");
  physics::LinkPtr link_2 = model_2->GetLink(link_name_2);
  if (link_2 == nullptr) {
    ROS_ERROR_STREAM(link_name_2 << " link was not found");
    return false;
  }

  fixed.link_2 = link_2;

  // look for any previous instance of the joint.
  // if we try to create a joint in between two links
  // more than once (even deleting any reference to the first one)
  // gazebo hangs/crashes.
  FixedJoint previous;
  if (get_joint(model_1->GetName(), link_1->GetName(), model_2->GetName(), link_2->GetName(), previous)) {
    ROS_INFO_STREAM("Joint already existed, reusing it.");
    previous.joint->Attach(previous.link_1, previous.link_2);
    return true;
  } else {
    ROS_INFO_STREAM("Creating new joint.");
  }

  ROS_INFO_STREAM("Links are: " << link_1->GetName() << " and " << link_2->GetName());

  ROS_INFO_STREAM("Creating revolute joint on model: '" << model_name_1 << "'");
  fixed.joint = m_world->Physics()->CreateJoint("revolute", model_1);

  fixed.joint->Attach(link_1, link_2);
  fixed.joint->Load(link_1, link_2, ignition::math::Pose3d());
  fixed.joint->SetModel(model_2);

  fixed.joint->SetUpperLimit(0, 0);
  fixed.joint->SetLowerLimit(0, 0);
  fixed.joint->Init();

  fixed.model_1->SetCollideMode("none");
  fixed.model_2->SetCollideMode("none");

  fixed.model_name_1 = model_1->GetName();
  fixed.link_name_1 = link_1->GetName();
  fixed.model_name_2 = model_2->GetName();
  fixed.link_name_2 = link_2->GetName();

  m_joints.push_back(fixed);

  return true;
}

bool LinkerPlugin::detach(std::string model_name_1, std::string link_name_1,
                          std::string model_name_2, std::string link_name_2,
                             FixedJoint &fixed) {
  // search for the instance of joint and do detach
  if (get_joint(model_name_1, link_name_1, model_name_2, link_name_2, fixed)) {
    fixed.joint->Detach();
    fixed.model_1->SetCollideMode("all");
    fixed.model_2->SetCollideMode("all");
    return true;
  }

  return false;
}

bool LinkerPlugin::get_joint(std::string model_name_1, std::string link_name_1,
                             std::string model_name_2, std::string link_name_2,
                             FixedJoint &fixed) {
  FixedJoint j;
  for (std::vector<FixedJoint>::iterator it = m_joints.begin(); it != m_joints.end(); ++it) {
    j = *it;
    if ((j.model_name_1.compare(model_name_1) == 0) && (j.model_name_2.compare(model_name_2) == 0) && (j.link_name_1.compare(link_name_1) == 0) && (j.link_name_2.compare(link_name_2) == 0)) {
      fixed = j;
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
  FixedJoint fixed;
  if (!attach(req.model_name_1, req.link_name_1,
              req.model_name_2, req.link_name_2, fixed)) {
    ROS_ERROR_STREAM("Could not attach.");
    res.ok = false;
  } else {
    ROS_INFO_STREAM("Attach was succesful");
    res.ok = true;
    res.model_name_1 = fixed.model_name_1;
    res.link_name_1 = fixed.link_name_1;
    res.model_name_2 = fixed.model_name_2;
    res.link_name_2 = fixed.link_name_2;
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
  FixedJoint fixed;
  if (!detach(req.model_name_1, req.link_name_1,
              req.model_name_2, req.link_name_2, fixed)) {
    ROS_ERROR_STREAM("Could not detach.");
    res.ok = false;
  } else {
    ROS_INFO_STREAM("Detach was successful");
    res.ok = true;
    res.model_name_1 = fixed.model_name_1;
    res.link_name_1 = fixed.link_name_1;
    res.model_name_2 = fixed.model_name_2;
    res.link_name_2 = fixed.link_name_2;
  }
  return true;
}

} // namespace gazebo