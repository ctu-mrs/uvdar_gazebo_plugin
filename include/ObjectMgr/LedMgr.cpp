// clang: MatousFormat
#include "LedMgr.h"

LedMgr::LedMgr(
    ros::NodeHandle& nh,
    mrs_lib::ParamLoader& pl,
    const std::string& name
    )
  : ObjectMgr(nh, name, "led_link")
{
  return;
}

void LedMgr::update_link_pose(const std::string& link_name, const geometry_msgs::Pose& i_pose)
{
  ObjectMgr::update_link_pose(link_name, i_pose);
}

void LedMgr::update_frequency(double i_frequency){
  frequency = i_frequency;
}

void LedMgr::update_all(const std::string& link_name, const geometry_msgs::Pose& i_pose, double i_frequency)
{
  if (!m_pose_initialized)
    return;

  update_link_pose(link_name, i_pose);
  update_frequency(i_frequency);
}

bool LedMgr::get_pose(geometry_msgs::Pose &output)
{
  if (!m_pose_initialized)
    return false;

  output = m_pose;
  return true;
}
