// clang: MatousFormat
#include "LedMgr.h"

LedMgr::LedMgr(
    ros::NodeHandle& nh,
    const std::string& name
    )
  : ObjectMgr(nh, name, "led_link") {

  mode = 1; //set special sequence mode
  return;
}

void LedMgr::update_link_pose(const std::string& link_name, const geometry_msgs::Pose& i_pose) {
  ObjectMgr::update_link_pose(link_name, i_pose);
}

void LedMgr::update_frequency(double i_frequency){ /* std::cout << "setting frequency to " << i_frequency << std::endl; */
  /* std::cout << "Updating frequency to " << i_frequency << std::endl; */
  f = i_frequency;
  T  = 1.0 / f;
  Th = T / 2.0;
  frequency_initialized = true;
}

void LedMgr::update_sequence(std::vector<bool> i_sequence, double i_bit_rate){ /* std::cout << "setting frequency to " << i_frequency << std::endl; */
  /* std::cout << "Updating frequency to " << i_frequency << std::endl; */
  sequence = i_sequence;
  bit_rate = i_bit_rate;
  seq_duration = (double)(sequence.size())/bit_rate;
  time_scaler = (double)(sequence.size())/seq_duration;
  sequence_initialized = true;
}

/* void LedMgr::update_all(const std::string& link_name, const geometry_msgs::Pose& i_pose, double i_frequency=0.0, std::vector<bool> i_sequence={}) { */
/*   if (!m_pose_initialized) */
/*     return; */

/*   update_link_pose(link_name, i_pose); */
/*   if (mode == 0){ */
/*     update_frequency(i_frequency); */
/*   } */
/*   else if (mode == 1){ */
/*     update_sequence(i_sequence); */
/*   } */
/* } */

bool LedMgr::get_pose(geometry_msgs::Pose &output, double nowTime) {
  if (!m_pose_initialized)
    return false;

  if (mode == 0){
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
  }
  else if (mode == 1){
    if (!sequence_initialized)
      return false;

    int seq_index = (int)(fmod(nowTime, seq_duration)*time_scaler);
    seq_index = std::min((int)(sequence.size())-1,seq_index);
    if (sequence[seq_index]){
      output = m_pose;
      return true;
    }
  }

  return false;
  
}

