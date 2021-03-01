// clang: MatousFormat
#ifndef LEDMGR_H
#define LEDMGR_H

#include <ros/ros.h>
#include <string>
#include <memory>
#include <geometry_msgs/Point32.h>
#include "ObjectMgr.h"
#include "LedMgr.h"

class LedMgr : public ObjectMgr
{
  public:
    LedMgr(
        ros::NodeHandle& nh,
        const std::string& name);
    void update_link_pose(const std::string& link_name, const geometry_msgs::Pose& i_pose);
    void update_frequency(double i_frequency);
    
    void update_sequence(std::vector<bool> i_sequence, double i_bit_rate);
    /* void update_all(const std::string& link_name, const geometry_msgs::Pose& i_pose, double i_frequency); */
    bool get_pose(geometry_msgs::Pose &output, double nowTime = 0.0);
    virtual ~LedMgr() {};
  private:
    int mode; //0 - static frequency, 1 - selected sequences

    std::vector<bool> sequence;
    double seq_duration;
    double bit_rate;
    /* double time_scaler; */

    double f;
    double T;
    double Th;
    bool frequency_initialized = false;
    bool sequence_initialized = false;
};

#endif //LEDMGR_H
