// clang: MatousFormat
#ifndef LEDMGR_H
#define LEDMGR_H

#include <ros/ros.h>
#include <string>
#include <memory>
#include <geometry_msgs/Point32.h>
#include "ObjectMgr.h"
#include "LedMgr.h"

#define DIAG_SIGNAL_LENGTH 50

class LedMgr : public ObjectMgr
{
  public:
    LedMgr(
        ros::NodeHandle& nh,
        const std::string& name);
    void update_link_pose(const std::string& link_name, const geometry_msgs::Pose& i_pose);
    /* void update_frequency(double i_frequency); */
    void update_data(std::vector<bool> i_sequence, double i_seq_bit_rate, double i_mes_bit_rate);
    void update_bitrate(double i_seq_bit_rate, double i_mes_bit_rate);
    void update_message(std::vector<bool> i_message, double i_bit_rate = -1);
    void set_mode(int i_mode);
    void set_active(bool i_active);

    /* void update_all(const std::string& link_name, const geometry_msgs::Pose& i_pose, double i_frequency); */
    bool get_pose(geometry_msgs::Pose &output, double nowTime = 0.0);
    virtual ~LedMgr() {};
  private:
    void update_timing(double i_seq_bit_rate, double i_mes_bit_rate);

    bool active;

    int mode; //0 - static frequency, 1 - selected sequences

    std::vector<bool> sequence;
    std::vector<bool> message;
    double seq_duration;
    double mes_duration;
    double seq_bit_rate;
    double mes_bit_rate;
    /* double time_scaler; */

    double f;
    double T;
    double Th;
    bool frequency_initialized = false;
    bool sequence_initialized = false;
    bool message_initialized = false;



    char diag_signal[DIAG_SIGNAL_LENGTH+1];
    char diag_order[DIAG_SIGNAL_LENGTH+1];
    int dsi = -1;
    std::string diag_seq;
};

#endif //LEDMGR_H
