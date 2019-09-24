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


namespace gazebo
{
class UvLed : public SensorPlugin {
private:
  ros::NodeHandle         nh;
  int                     n;
  double                  updatePeriod;
  float                   f;
  float                   T;
  float                   Th;
  /* transport::PublisherPtr posePub; */
  ros::Publisher frequencyPub;
  /* transport::PublisherPtr statePub ; */
  gazebo::physics::WorldPtr world;
  physics::EntityPtr        parent;
  std::thread               pub_thread;
  sensors::SensorPtr        sensor;
  /* ignition::math::Pose3d                pose; */
  /* msgs::Pose                poseMsg; */
  std_msgs::Float64         freqnencyMsg;
  std::mutex                pubMutex;
  std::string               link_name;

public:
  void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {


    std::cout << "Loading UV LED" << std::endl;

    /* nh = ros::NodeHandle("/"); */

    // Store the pointer to the model
    this->sensor           = _sensor;
    world                  = physics::get_world("default");
    std::string parentName = sensor->ParentName();
    parent                 = world->EntityByName(parentName);
    /* std::cout << _sdf->GetParent()->GetName() << std::endl; */
    /* std::cout << _sdf->GetParent()->GetElement("plugin")->Get< std::string >("name") << std::endl; */
    /* std::cout << "LED parent name: " << _sdf->GetParent()->GetElement("visual")->GetElement("Pose")->GetValue() << std::endl; */
    /* std::cout << "LED plugin : " << _sdf->GetAttribute("name")->GetAsString() << std::endl; */
    /* std::cout << _sdf->GetParent()->GetElement("geometry")->GetElement("sphere")->GetElement("radius")->GetValue()->GetAsString() << std::endl; */
    /* std::cout << _sdf->GetParent()->GetElement("meta")->GetAttribute("frequency")->GetAsString() << std::endl; */
    /* std::cout << _sdf->Get<std::string>("name") << std::endl; */
    /* std::cout << _sdf->GetElement("freq")->GetValue() << std::endl; */


    if (_sdf->HasElement("link_name")) {
      link_name = _sdf->GetElement("link_name")->Get< std::string >();
      frequencyPub = nh.advertise<std_msgs::Float64>("/gazebo/ledProperties/"+link_name+"/frequency",1);
    }
    else{
      std::cout << "Could not find the link name of the LED" << std::endl;
    }

    if (_sdf->HasElement("frequency")) {
      f = _sdf->GetElement("frequency")->Get< double >();
      std::cout << "LED frequency is " << f << "Hz" << std::endl;
    } else {
      std::cout << "LED frequency defaulting to 10Hz." << std::endl;
      f = 10.0;  // camera framerate
    }
    if (_sdf->HasElement("number")) {
      n = _sdf->GetElement("number")->Get< int >();
      std::cout << "LED number is " << n << std::endl;
    } else {
      std::cout << "LED number defaulting to -1" << std::endl;
      n = -1;  // camera framerate
    }

    T  = 1.0 / f;
    Th = T / 2.0;

    updatePeriod = 1.0;
    if (_sdf->HasElement("updateRate")) {
      updatePeriod = 1.0/_sdf->GetElement("updateRate")->Get< double >();
      std::cout << "Update rate is " << 1.0/updatePeriod << "Hz" << std::endl;
    }
    else
      std::cout << "Update rate defaulting to 1 Hz" << std::endl;


    transport::NodePtr node(new transport::Node());
    node->Init();


    char poseTopicName[30];
    std::sprintf(poseTopicName, "~/uvleds/pose");
    /* char stateTopicName[30]; */
    /* std::sprintf(stateTopicName, "~/uvleds/state", id); */


    /* posePub = node->Advertise< msgs::Pose >(poseTopicName); */
    /* statePub = node->Advertise<msgs::Int>(stateTopicName); */
    /* statePub->WaitForConnection(); */
    /* posePub->WaitForConnection(); */


    /* transport::fini(); */
    // Listen to the update event. This event is broadcast every
    /* shutdown(); */

    /* while(true){ */
    /*   common::Time::MSleep(1); */
    /* } */
  }

public:
  void Init() {
    std::cout << "Initializing UV LED" << n << std::endl;
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&UvLed::OnUpdate, this));
    currTime               = ros::Time::now();
    pub_thread             = std::thread(&UvLed::PubThread, this);
  }
  // Called by the world update start event
public:
  void OnUpdate() {
    /* currTime = ros::Time::now(); */
    /* if ((currTime-prevTime).toSec()<(updatePeriod)){ */
    /*   /1* std::cout << "HAPPENING" << std::endl; *1/ */
    /*   return; */
    /* } */

    /* bool state = (fmod(ros::Time::now().toSec(), T) > Th); */
    /* if ((!state) && (f > 0.0)) { */
    /*   return; */
    /* } */

    /* pose = sensor->Pose() + parent->WorldPose(); */
    /* msgs::Set(&poseMsg, pose); */
    /* posePub->Publish(poseMsg); */
  }

  // Pointer to the sensor
private:
  void PubThread() {
    ros::Rate r(1);  // 10 h
    /* r.reset(); */
    while (true) {
      /* pubMutex.lock(); */
      freqnencyMsg.data = f;
      frequencyPub.publish(freqnencyMsg);
      /* pubMutex.unlock(); */
      r.sleep();
    }
  }

private:
  ros::Time            currTime, prevTime;
  event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(UvLed)
}
