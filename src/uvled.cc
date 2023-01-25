#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
/* #include <gazebo/rendering/rendering.hh> */
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math/Vector3.hh>
#include <mutex>
#include <thread>
#include <uvdar_gazebo_plugin/LedInfo.h>
#include <uvdar_gazebo_plugin/LedMessage.h>
#include <uvdar_core/SetLedMessage.h>
#include <mrs_msgs/SetInt.h>
#include <mrs_msgs/Float64Srv.h>

namespace uuid {
    static std::random_device              rd;
    static std::mt19937                    gen(rd());
    static std::uniform_int_distribution<> dis(0, 15);
    static std::uniform_int_distribution<> dis2(8, 11);

    std::string generate_uuid_v4() {
        std::stringstream ss;
        int i;
        ss << std::hex;
        for (i = 0; i < 8; i++) {
            ss << dis(gen);
        }
        ss << "_";
        for (i = 0; i < 4; i++) {
            ss << dis(gen);
        }
        ss << "_4";
        for (i = 0; i < 3; i++) {
            ss << dis(gen);
        }
        ss << "_";
        ss << dis2(gen);
        for (i = 0; i < 3; i++) {
            ss << dis(gen);
        }
        ss << "_";
        for (i = 0; i < 12; i++) {
            ss << dis(gen);
        };
        return ss.str();
    }
}

namespace gazebo
{
class UvLed : public SensorPlugin {
private:
  ros::NodeHandle nh;
  std::string     device_id;
  std::string     link_name;
  /* double          updatePeriod; */
  /* float           f; */

  double          fs;
  double          fm;

  int             mode = 0;
  int             ID;

  bool active = true;

  std::string unique_ID;

  /* transport::PublisherPtr posePub; */
  ros::Publisher led_info_pub;
  ros::Publisher led_message_pub;
  ros::Publisher led_mode_pub;
  /* transport::PublisherPtr statePub ; */
  gazebo::physics::WorldPtr world;
  /* physics::EntityPtr        parent_link; */
  std::thread               pub_thread;
  sensors::SensorPtr        sensor;
  /* ignition::math::Pose3d                pose; */
  /* msgs::Pose                poseMsg; */
  uvdar_gazebo_plugin::LedInfo      led_info;
  uvdar_gazebo_plugin::LedMessage   led_msg;
  std::mutex                   pubMutex;

  ros::ServiceServer sequence_setter_;
  ros::ServiceServer frequency_setter_;
  ros::ServiceServer mode_setter_;
  ros::ServiceServer message_sender_;
  ros::ServiceServer active_setter_;

public:
  void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {

    std::cout << "Loading UV LED" << std::endl;

    unique_ID = uuid::generate_uuid_v4(); // to ensure that each name is unique in the simulation

    this->sensor           = _sensor;
    world                  = physics::get_world("default");
    link_name = sensor->ParentName();
    /* parent_link                 = world->EntityByName(parentName); */

    if (_sdf->HasElement("device_id")) {
      device_id = _sdf->GetElement("device_id")->Get<std::string>();
      std::cout << "LED device_id is " << device_id << std::endl;
    } else {

      device_id = uuid::generate_uuid_v4();  
      std::cout << "LED device_id defaulting to " + device_id << std::endl;
    }

    led_info_pub    = nh.advertise<uvdar_gazebo_plugin::LedInfo>("/gazebo/ledProperties", 1, true);
    led_message_pub    = nh.advertise<uvdar_gazebo_plugin::LedMessage>("/gazebo/ledMessage/" + device_id, 1, true);
    led_mode_pub = nh.advertise<std_msgs::Int32>("/gazebo/ledMode/" + device_id, 1, true);

    if (_sdf->HasElement("signal_id")) {
      ID = _sdf->GetElement("signal_id")->Get<int>();
      std::cout << "LED signal ID is " << ID << std::endl;
    } else {
      std::cout << "LED signal_id is not set" << std::endl;
      ID = -1;
    }

    if (_sdf->HasElement("frequency")) {
      fs = _sdf->GetElement("frequency")->Get<double>();
      fm = fs;
      std::cout << "Initial LED bitrate is " << fs << "Hz" << std::endl;
    } else {
      std::cout << "Initial LED bitrate defaulting to 60Hz." << std::endl;
      fs = 60.0;  // camera framerate
      fm = fs;
    }


    /* updatePeriod = 1.0; */
    /* if (_sdf->HasElement("updateRate")) { */
    /*   updatePeriod = 1.0 / _sdf->GetElement("updateRate")->Get<double>(); */
    /*   std::cout << "Update rate is " << 1.0 / updatePeriod << "Hz" << std::endl; */
    /* } else */
    /*   std::cout << "Update rate defaulting to 1 Hz" << std::endl; */


    transport::NodePtr node(new transport::Node());
    node->Init();


    char poseTopicName[30];
    std::sprintf(poseTopicName, "~/uvleds/pose");
    sequence_setter_ = nh.advertiseService(("/gazebo/ledSignalSetter/" + device_id).c_str(), &UvLed::callbackSetSequence, this);
    frequency_setter_ = nh.advertiseService(("/gazebo/ledFrequencySetter/" + device_id).c_str(), &UvLed::callbackSetFrequency, this);
    mode_setter_ = nh.advertiseService(("/gazebo/ledModeSetter/" + device_id).c_str(), &UvLed::callbackSetMode, this);
    message_sender_ = nh.advertiseService(("/gazebo/ledMessageSender/" + device_id).c_str(), &UvLed::callbackSendMessage, this);
    active_setter_ = nh.advertiseService(("/gazebo/ledActiveSetter/" + device_id).c_str(), &UvLed::callbackSetActive, this);
  }

public:
  void Init() {
    std::cout << "Initializing UV LED " << device_id << std::endl;
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&UvLed::OnUpdate, this));
    std::cout << "Sending LED data" << std::endl;
    publishData();
    /* pub_thread = std::thread(&UvLed::PubThread, this); */
  }
  // Called by the world update start event
public:
  void OnUpdate() {
  }

  // Pointer to the sensor
private:
  bool callbackSetFrequency(mrs_msgs::Float64Srv::Request &req, mrs_msgs::Float64Srv::Response &res) {
    if (mode == 0){
      fs                     = req.value;
      publishData();
      res.message = "Setting the sequence bitrate to ";
      res.message += std::to_string(fs);
    }
    else if (mode == 1){
      fm                     = req.value;
      publishData();
      res.message = "Setting the message bitrate to ";
      res.message += std::to_string(fm);
    }
    ROS_INFO_STREAM(res.message);
    res.success = true;
    return true;
  }

  bool callbackSetSequence(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res) {
    ID                     = req.value;
    publishData();
    res.message = "Setting the signal ID to ";
    res.message += std::to_string(ID);
    ROS_INFO_STREAM(res.message);
    res.success = true;
    return true;
  }

  bool callbackSetMode(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res) {
    mode                     = req.value;

    std_msgs::Int32 mode_msg;
    mode_msg.data = mode;
    led_mode_pub.publish(mode_msg);
    publishData();
    res.message = "Setting the mode to ";
    res.message += std::to_string(mode);
    ROS_INFO_STREAM(res.message);
    res.success = true;
    return true;
  }

  bool callbackSendMessage(uvdar_core::SetLedMessage::Request &req, uvdar_core::SetLedMessage::Response &res) {
    if (mode == 1){
      res.message = "Sending message";

      /* std::string message_text; */
      /* for (auto b : req.data_frame){ */
      /*   if (b == 0) */
      /*     message_text += '0'; */
      /*   else */ 
      /*     message_text += '1'; */
      /* } */
      /* ROS_INFO_STREAM(res.message << " :" << message_text); */

      led_msg.data_frame = req.data_frame;
      led_message_pub.publish(led_msg);


      res.success = true;
      return true;
    }
    else {
      res.message = "Will not send message - the appropriate mode is not set!";
      ROS_INFO_STREAM(res.message);
      res.success = false;
      return true;
    }
  }

  bool callbackSetActive(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    active = req.data;
    if  (active)
      res.message = "Activating LED";
    else
      res.message = "Deactivating LED";
    
    publishData();

    res.success = true;
    return true;
  }
  /* void PubThread() { */
  /*   ros::Rate r(2); */
  /*   while (true) { */

  /*     if (!true) { */
  /*       publishData() */
  /*       // std::cout << "Publishing base frequency, UVDAR in localization mode" << std::endl; */
  /*     } else { */
  /*       // std::cout << "Ignoring base frequency, UVDAR in RXTX mode" << device_id << std::endl; */
  /*     } */
  /*     r.sleep(); */
  /*   } */
  /* } */

  void publishData(){
        led_info.seq_bitrate.data = fs;
        led_info.mes_bitrate.data = fm;
        led_info.ID.data = ID;
        led_info.active.data = active;
        led_info.device_id.data = device_id;
        led_info.link_name.data = link_name;
        led_info_pub.publish(led_info);
  }

private:
  event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(UvLed)
}  // namespace gazebo
