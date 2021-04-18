// clang: MatousFormat
#include "LedMgr.h"

LedMgr::LedMgr(
    ros::NodeHandle& nh,
    const std::string& name
    )
  : ObjectMgr(nh, name, "led_link") {


  diag_signal[DIAG_SIGNAL_LENGTH] = '\0';
  diag_order[DIAG_SIGNAL_LENGTH] = '\0';
  mode = 0; //set special sequence mode
  return;
}

void LedMgr::update_link_pose(const std::string& link_name, const geometry_msgs::Pose& i_pose) {
  ObjectMgr::update_link_pose(link_name, i_pose);
}

/* void LedMgr::update_frequency(double i_frequency){ /1* std::cout << "setting frequency to " << i_frequency << std::endl; *1/ */
/*   /1* std::cout << "Updating frequency to " << i_frequency << std::endl; *1/ */
/*   f = i_frequency; */
/*   T  = 1.0 / f; */
/*   Th = T / 2.0; */
/*   frequency_initialized = true; */
/* } */

void LedMgr::update_sequence(std::vector<bool> i_sequence, double i_bit_rate){ /* std::cout << "setting frequency to " << i_frequency << std::endl; */
  /* std::cout << "Updating frequency to " << i_frequency << std::endl; */
  sequence = i_sequence;
  /* if (diag_seq.length() < (int)(sequence.size())) */
  diag_seq = "";
  for (auto s: sequence){
    diag_seq += (s?'1':'0');
  }

  bit_rate = i_bit_rate;
  seq_duration = (double)(sequence.size())/bit_rate;
  /* time_scaler = (double)(sequence.size())/seq_duration; */
  sequence_initialized = true;
}

void LedMgr::update_message(std::vector<bool> i_message, double i_bit_rate){
  if (i_bit_rate > 0){
    bit_rate = i_bit_rate;
  }
  else {
    bit_rate = 60;
  }
  message = i_message;
  mes_duration = (double)(message.size())/bit_rate;
  message_initialized = true;
}

void LedMgr::set_mode(int i_mode){
  mode = i_mode;
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

char toHex(int input){
  if (input < 16){
    if (input < 10){
      return char(input+48);
    }
    else {
      return char(input+55);
    }
  }
  else {
    return '*';
  }
}

bool LedMgr::get_pose(geometry_msgs::Pose &output, double nowTime) {
  if (!m_pose_initialized)
    return false;

  if (mode == -2){ //deprecated
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
  else if (mode == 0){
    dsi++;
    if (dsi>=DIAG_SIGNAL_LENGTH){
      dsi = 0;
      /* std::cout << "Signal from LED s:" << diag_seq << " was:\n" << diag_signal << "\n" << diag_order << std::endl; */
    }
    if (!sequence_initialized){
      diag_signal[dsi] = '0';
      diag_order[dsi] = '0';
      return false;
    }

    int seq_index = (int)(fmod(nowTime, seq_duration)*bit_rate);
    seq_index = std::min((int)(sequence.size())-1,seq_index);//sanitization
    if (sequence[seq_index]){
      output = m_pose;
      diag_signal[dsi] = '1';
      diag_order[dsi] = toHex(seq_index);
      return true;
    }
    else{
      diag_signal[dsi] = '0';
      diag_order[dsi] = toHex(seq_index);
      return false;
    }
  }
  else if (mode == 1){
    if (!message_initialized){
      return false;
    }

    int mes_index = (int)(fmod(nowTime, mes_duration)*bit_rate);
    mes_index = std::min((int)(message.size())-1,mes_index);//sanitization
    if (message[mes_index]){
      output = m_pose;
      return true;
    }
    else{
      return false;
    }
  }

  return false;
  
}

