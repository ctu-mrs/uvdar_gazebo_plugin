// clang: MatousFormat
#include "ObjectMgr.h"

ObjectMgr::ObjectMgr(ros::NodeHandle& nh, const std::string& model_name, const std::string& attach_link_name, bool control_own_movement)
  : m_model_name(model_name), m_attach_link_name(attach_link_name), m_pose_initialized(false), m_control_own_movement(control_own_movement)
{
  if (m_control_own_movement)
    m_model_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 100);
}

void ObjectMgr::update_link_pose(const std::string& link_name, const geometry_msgs::Pose& n_pose)
{
  // if this object controls its own movement and the pose has
  // already been initialized, do not update the pose
  if (!(m_control_own_movement && m_pose_initialized))
  {
    m_pose = n_pose;
    m_pose_initialized = true;
  }
}

ObjectMgr::~ObjectMgr()
{}
