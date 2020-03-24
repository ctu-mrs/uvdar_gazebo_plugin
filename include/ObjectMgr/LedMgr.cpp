// clang: MatousFormat
#include "LedMgr.h"

LedMgr::LedMgr(
    ros::NodeHandle& nh,
    const std::string& name
    )
  : ObjectMgr(nh, name, "led_link") {
  frequency_initialized = false;
  return;
}

void LedMgr::update_link_pose(const std::string& link_name, const geometry_msgs::Pose& i_pose) {
  ObjectMgr::update_link_pose(link_name, i_pose);
}

void LedMgr::update_frequency(double i_frequency){ /* std::cout << "setting frequency to " << i_frequency << std::endl; */
  std::cout << "Updating frequency to " << i_frequency << std::endl;
  f = i_frequency;
  T  = 1.0 / f;
  Th = T / 2.0;
  frequency_initialized = true;
}

void LedMgr::update_all(const std::string& link_name, const geometry_msgs::Pose& i_pose, double i_frequency) {
  if (!m_pose_initialized)
    return;

  update_link_pose(link_name, i_pose);
  update_frequency(i_frequency);
}

bool LedMgr::get_pose(geometry_msgs::Pose &output, double nowTime) {
  if (!m_pose_initialized)
    return false;

  if (!frequency_initialized)
    return false;

  if (f < 0.001){ //p.m. equals zero = always on
    output = m_pose;
    return true;
  }

  if (fmod(nowTime, T) > Th){
    output = m_pose;
    return true;
  }

  return false;
  
}

