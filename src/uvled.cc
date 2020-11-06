#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
/* #include <gazebo/rendering/rendering.hh> */
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math/Vector3.hh>
#include <mutex>
#include <thread>
#include <uvdar_gazebo_plugin/LedInfo.h>
#include <mrs_msgs/SetInt.h>


namespace gazebo
{
class UvLed : public SensorPlugin {
private:
  ros::NodeHandle nh;
  int             n;
  double          updatePeriod;
  float           f;
  bool            leds_info = false;
  /* transport::PublisherPtr posePub; */
  ros::Publisher ledPub;
  /* transport::PublisherPtr statePub ; */
  gazebo::physics::WorldPtr world;
  physics::EntityPtr        parent;
  std::thread               pub_thread;
  sensors::SensorPtr        sensor;
  /* ignition::math::Pose3d                pose; */
  /* msgs::Pose                poseMsg; */
  uvdar_gazebo_plugin::LedInfo ledMsg;
  std::mutex                   pubMutex;
  std::string                  link_name;

  ros::ServiceServer frequency_setter_;

public:
  void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {

    std::cout << "Loading UV LED" << std::endl;

    this->sensor           = _sensor;
    world                  = physics::get_world("default");
    std::string parentName = sensor->ParentName();
    parent                 = world->EntityByName(parentName);

    if (_sdf->HasElement("link_name")) {
      link_name = _sdf->GetElement("link_name")->Get<std::string>();
      ledPub    = nh.advertise<uvdar_gazebo_plugin::LedInfo>("/gazebo/ledProperties/" + link_name, 1, true);
    } else {
      std::cout << "Could not find the link name of the LED" << std::endl;
    }

    if (_sdf->HasElement("frequency")) {
      f = _sdf->GetElement("frequency")->Get<double>();
      std::cout << "LED frequency is " << f << "Hz" << std::endl;
    } else {
      std::cout << "LED frequency defaulting to 10Hz." << std::endl;
      f = 10.0;  // camera framerate
    }
    if (_sdf->HasElement("number")) {
      n = _sdf->GetElement("number")->Get<int>();
      std::cout << "LED number is " << n << std::endl;
    } else {
      std::cout << "LED number defaulting to -1" << std::endl;
      n = -1;  // camera framerate
    }

    updatePeriod = 1.0;
    if (_sdf->HasElement("updateRate")) {
      updatePeriod = 1.0 / _sdf->GetElement("updateRate")->Get<double>();
      std::cout << "Update rate is " << 1.0 / updatePeriod << "Hz" << std::endl;
    } else
      std::cout << "Update rate defaulting to 1 Hz" << std::endl;


    transport::NodePtr node(new transport::Node());
    node->Init();


    char poseTopicName[30];
    std::sprintf(poseTopicName, "~/uvleds/pose");
    frequency_setter_ = nh.advertiseService(("/gazebo/ledFrequencySetter/" + link_name).c_str(), &UvLed::callbackSetFrequency, this);
  }

public:
  void Init() {
    std::cout << "Initializing UV LED" << n << std::endl;
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&UvLed::OnUpdate, this));
    ledMsg.frequency.data  = f;
    ledMsg.isOff.data      = false;
    std::cout << "Sending message" << std::endl;
    ledPub.publish(ledMsg);
    pub_thread = std::thread(&UvLed::PubThread, this);
  }
  // Called by the world update start event
public:
  void OnUpdate() {
  }

  // Pointer to the sensor
private:
    bool callbackSetFrequency(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res) {
      f = req.value;
      if(f==INT_MAX) {
        ledMsg.frequency.data = std::numeric_limits<_Float64>::max();
      }
      else{
        ledMsg.frequency.data = f;
      }
      res.message = "Setting the frequency to ";
      res.message += std::to_string(f);
      ROS_INFO_STREAM(res.message);
      res.success = true;
      return true;
    }

  void PubThread() {
    ros::Rate r(2);
    while (true) {

    ledMsg.frequency.data = f;
    ledMsg.isOff.data     = false;
    ledPub.publish(ledMsg);
    r.sleep();
    }
  }

private:
  event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(UvLed)
}  // namespace gazebo
