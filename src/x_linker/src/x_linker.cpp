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
  m_update_conn = event::Events::ConnectBeforePhysicsUpdate(std::bind(&LinkerPlugin::before_physics_update, this));

  m_attach_server = m_n.advertiseService("attach", &LinkerPlugin::attach_callback, this);
  m_detach_server = m_n.advertiseService("detach", &LinkerPlugin::detach_callback, this);
  m_set_collision_server = m_n.advertiseService("set_collision", &LinkerPlugin::set_collision_callback, this);

  ROS_INFO_STREAM("Attach service at: " << m_n.resolveName("attach"));
  ROS_INFO_STREAM("Detach service at: " << m_n.resolveName("detach"));
  ROS_INFO_STREAM("Set Collision service at: " << m_n.resolveName("set_collision"));
  ROS_INFO("Link attacher node initialized.");
}

void LinkerPlugin::before_physics_update() {
  m_safe_drop_vector_mutex.lock();
  auto it = m_safe_drop_vector.begin();
  auto now = std::chrono::high_resolution_clock::now();
  while (it != m_safe_drop_vector.end()) {
    if (now > it->expiration) {
      if (it->link->WorldLinearVel().Abs().Z() < 1e6) {
        ROS_INFO_STREAM("Safe drop finished for model " << it->model->GetName());
        it->link->GetInertial()->SetCoG(it->value);
        it->link->UpdateMass();
        it = m_safe_drop_vector.erase(it);
        continue;
      }
    }
    ++it;
  }
  m_safe_drop_vector_mutex.unlock();

  m_attach_queue_mutex.lock();
  while (!m_attach_queue.empty()) {
    FixedJoint &fixed = m_attach_queue.front();
    fixed.joint->Attach(fixed.link_1, fixed.link_2);
    fixed.joint->Load(fixed.link_1, fixed.link_2, ignition::math::Pose3d());
    fixed.joint->SetModel(fixed.model_2);
    fixed.joint->SetUpperLimit(0, 0);
    fixed.joint->SetLowerLimit(0, 0);
    fixed.joint->Init();
    ROS_INFO_STREAM("Attach finished for model " << fixed.model_name_2);
    m_attach_queue.pop();
  }
  m_attach_queue_mutex.unlock();

  m_detach_queue_mutex.lock();

  while (!m_detach_queue.empty()) {
    FixedJoint &fixed = m_detach_queue.front();
    fixed.joint->Detach();

    DeferredValueChange<ignition::math::Pose3d> change;
    change.model = fixed.model_2;
    change.link = fixed.link_2;
    change.value = fixed.link_2->GetInertial()->Pose();
    change.expiration = std::chrono::high_resolution_clock::now() + std::chrono::seconds(4);

    fixed.link_2->GetInertial()->SetCoG(ignition::math::Pose3d::Zero);
    fixed.link_2->UpdateMass();

    m_safe_drop_vector_mutex.lock();
    m_safe_drop_vector.push_back(change);
    m_safe_drop_vector_mutex.unlock();

    ROS_INFO_STREAM("Detach finished for model " << fixed.model_name_2);
    m_detach_queue.pop();
  }
  m_detach_queue_mutex.unlock();
}

bool LinkerPlugin::attach(std::string model_name_1, std::string link_name_1,
                          std::string model_name_2, std::string link_name_2,
                          FixedJoint &fixed) {

  ROS_INFO_STREAM("Getting model of " << model_name_1);
  fixed.model_1 = m_world->ModelByName(model_name_1);
  if (fixed.model_1 == nullptr) {
    ROS_ERROR_STREAM(model_name_1 << " model was not found");
    return false;
  }

  ROS_INFO_STREAM("Getting link: '" << link_name_1 << "' from model: '" << fixed.model_1->GetName() << "'");
  fixed.link_1 = fixed.model_1->GetLink(link_name_1);
  if (fixed.link_1 == nullptr) {
    ROS_ERROR_STREAM(link_name_1 << " link was not found");
    return false;
  }

  ROS_INFO_STREAM("Getting model of " << model_name_2);
  fixed.model_2 = m_world->ModelByName(model_name_2);
  if (fixed.model_2 == nullptr) {
    ROS_ERROR_STREAM(model_name_2 << " model was not found. Searching for a subset.");

    physics::Model_V haystack = m_world->Models();
    physics::ModelPtr needle = nullptr;
    ignition::math::Vector3d lhs = fixed.link_1->BoundingBox().Center();

    double best = std::numeric_limits<double>::infinity();
    unsigned int best_string_distance = std::numeric_limits<unsigned int>::max();

    for (physics::ModelPtr model : haystack) {
      ROS_INFO_STREAM("Evaluating " << model->GetName());
      if (model->GetName().find(model_name_2) != std::string::npos) {
        ROS_INFO_STREAM(model->GetName() << " is a superset of " << model_name_2);
        best_string_distance = 0;
        // name contains the string we are searching for
        ignition::math::Vector3d rhs = model->WorldPose().Pos();
        ROS_INFO_STREAM("Distance " << lhs.Distance(rhs));
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

    fixed.model_2 = needle;
  }

  ROS_INFO_STREAM("Getting link: '" << link_name_2 << "' from model: '" << fixed.model_2->GetName() << "'");
  fixed.link_2 = fixed.model_2->GetLink(link_name_2);
  if (fixed.link_2 == nullptr) {
    ROS_ERROR_STREAM(link_name_2 << " link was not found. Searching for a subset.");

    physics::Link_V haystack = fixed.model_2->GetLinks();
    physics::LinkPtr needle = nullptr;
    for (physics::LinkPtr link : haystack) {
      ROS_INFO_STREAM("Evaluating " << link->GetName());
      if (link->GetName().find(link_name_2) != std::string::npos) {
        needle = link;
        break;
      }
    }

    if (needle == nullptr) {
      ROS_ERROR_STREAM(link_name_2 << " model was not found. No maching model found.");
      return false;
    }

    fixed.link_2 = needle;
  }

  // look for any previous instance of the joint.
  // if we try to create a joint in between two links
  // more than once (even deleting any reference to the first one)
  // gazebo hangs/crashes.
  FixedJoint previous;
  if (get_joint(fixed.model_1->GetName(), fixed.link_1->GetName(), fixed.model_2->GetName(), fixed.link_2->GetName(), previous)) {
    ROS_INFO_STREAM("Joint already existed, reusing it.");
    previous.joint->Attach(previous.link_1, previous.link_2);
    fixed = previous;
    return true;
  } else {
    ROS_INFO_STREAM("Creating new joint.");
  }

  ROS_INFO_STREAM("Links are: " << fixed.link_1->GetName() << " and " << fixed.link_2->GetName());

  ROS_INFO_STREAM("Creating revolute joint on model: '" << model_name_1 << "'");
  fixed.joint = m_world->Physics()->CreateJoint("revolute", fixed.model_1);

  fixed.model_name_1 = fixed.model_1->GetName();
  fixed.link_name_1 = fixed.link_1->GetName();
  fixed.model_name_2 = fixed.model_2->GetName();
  fixed.link_name_2 = fixed.link_2->GetName();

  m_attach_queue_mutex.lock();
  m_attach_queue.push(fixed);
  m_attach_queue_mutex.unlock();

  m_joints.push_back(fixed);

  return true;
}

bool LinkerPlugin::detach(std::string model_name_1, std::string link_name_1,
                          std::string model_name_2, std::string link_name_2,
                          FixedJoint &fixed) {
  // search for the instance of joint and do detach
  if (get_joint(model_name_1, link_name_1, model_name_2, link_name_2, fixed)) {
    m_detach_queue_mutex.lock();
    m_detach_queue.push(fixed);
    m_detach_queue_mutex.unlock();
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

bool LinkerPlugin::set_collision(std::string model_name, std::string mode) {
  physics::ModelPtr model = m_world->ModelByName(model_name);
  if (model == nullptr) {
    ROS_ERROR_STREAM(model_name << " model was not found");
    return false;
  }
  model->SetCollideMode(mode);
  return true;
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

bool LinkerPlugin::set_collision_callback(x_linker::SetCollision::Request &req,
                                          x_linker::SetCollision::Response &res) {
  ROS_INFO_STREAM("Received request to set collision of model: '"
                  << req.model_name
                  << "' to: '" << req.mode << "'");
  if (!set_collision(req.model_name, req.mode)) {
    ROS_ERROR_STREAM("Could not set collision.");
    res.ok = false;
  } else {
    ROS_INFO_STREAM("Set collision was successful");
    res.ok = true;
  }
  return true;
}

} // namespace gazebo