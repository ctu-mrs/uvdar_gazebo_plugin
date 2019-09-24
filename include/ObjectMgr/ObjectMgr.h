// clang: MatousFormat
#ifndef OBJECTMGR_H
#define OBJECTMGR_H

#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>


class ObjectMgr
{
  public:
    virtual void update_link_pose(const std::string& link_name, const geometry_msgs::Pose& n_pose);
    bool is_pose_initialized() {return m_pose_initialized;};
    geometry_msgs::Pose get_pose() {return m_pose;};
    std::string get_name() {return m_model_name;};
    std::string get_attach_link_name() {return m_attach_link_name;};

  protected:
    ObjectMgr(ros::NodeHandle& nh, const std::string& model_name, const std::string& attach_link_name, bool control_own_movement = false);
    virtual ~ObjectMgr();

  protected:
    ros::Publisher m_model_state_pub;

  protected:
    std::string m_model_name;
    std::string m_attach_link_name;
    geometry_msgs::Pose m_pose;
    bool m_pose_initialized;
    bool m_control_own_movement;
};

#endif //OBJECTMGR_H

