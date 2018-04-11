#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
/* #include <gazebo/rendering/rendering.hh> */
#include <gazebo/sensors/sensors.hh>
#include <ignition/math/Vector3.hh>


namespace gazebo
{
class UvLed : public SensorPlugin {
private:
  int                     n;
  float                   f;
  float                   T;
  float                   Th;
  transport::PublisherPtr posePub;
  /* transport::PublisherPtr statePub ; */
  gazebo::physics::WorldPtr world;
  physics::EntityPtr        parent;

public:
  void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {


    std::cout << "Loading UV LED" << std::endl;

    // Store the pointer to the model
    this->sensor = _sensor;
    world                  = physics::get_world("default");
    std::string parentName = sensor->ParentName();
    parent                 = world->GetEntity(parentName);
    std::cout << _sdf->GetParent()->GetName() << std::endl;
    std::cout << _sdf->GetParent()->GetElement("plugin")->Get<std::string>("name") << std::endl;
    /* std::cout << "LED parent name: " << _sdf->GetParent()->GetElement("visual")->GetElement("Pose")->GetValue() << std::endl; */
    /* std::cout << "LED plugin : " << _sdf->GetAttribute("name")->GetAsString() << std::endl; */
    /* std::cout << _sdf->GetParent()->GetElement("geometry")->GetElement("sphere")->GetElement("radius")->GetValue()->GetAsString() << std::endl; */
    /* std::cout << _sdf->GetParent()->GetElement("meta")->GetAttribute("frequency")->GetAsString() << std::endl; */
    /* std::cout << _sdf->Get<std::string>("name") << std::endl; */
    /* std::cout << _sdf->GetElement("freq")->GetValue() << std::endl; */


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

    T  = 1.0 / f;
    Th = T / 2.0;


    transport::NodePtr node(new transport::Node());
    node->Init();


    char poseTopicName[30];
    std::sprintf(poseTopicName, "~/uvleds/pose");
    /* char stateTopicName[30]; */
    /* std::sprintf(stateTopicName, "~/uvleds/state", id); */


    posePub = node->Advertise<msgs::Pose>(poseTopicName);
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
  }
  // Called by the world update start event
public:
  void OnUpdate() {
    /* std::cout << "sending" << std::endl; */
    // Apply a small linear velocity to the sensor.
    bool state = (fmod(common::Time::GetWallTime().Double(), T) > Th);

    /* std::cout << "T: "<< T << std::endl; */
    /* std::cout << "LED" << n << ": " << state << std::endl; */
    if ((!state) && (f > 0.0)) {
      return;
    }
    /* math::Pose pose = sensor->Pose();  // sensor->GetPose(); */
    /* if (n==0) { */
    /* std::cout << "LED: parent name: " << parent->GetName()<< std::endl; */
    /* std::cout << "LED: parent: " << parent->GetWorldPose().Ign() << std::endl; */
    /* std::cout << "LED: local: " << sensor->Pose() << std::endl; */
    /* std::cout << "LED: combined: " << sensor->Pose() + parent->GetWorldPose().Ign() << std::endl; */
    /* std::cout << "LED: combinedI: " << parent->GetWorldPose().Ign() + sensor->Pose() << std::endl; */
    /* std::cout << "LED: combined-: " << sensor->Pose() - parent->GetWorldPose().Ign() << std::endl; */
    /* std::cout << "LED: combinedI-: " << parent->GetWorldPose().Ign() - sensor->Pose() << std::endl; */
    /* } */
    math::Pose pose = sensor->Pose() + parent->GetWorldPose().Ign();
    /* std::cout << "LED frequency: " << f << std::endl; */
    msgs::Pose poseMsg;
    msgs::Set(&poseMsg, pose.Ign());
    /* std::cout << "Publishing" << std::endl; */
    posePub->Publish(poseMsg);
    /* msgs::Int stateMsg; */
    /* stateMsg.set_data(state); */
    /* statePub->Publish(stateMsg); */
  }

  // Pointer to the sensor
private:
  sensors::SensorPtr sensor;

  // Pointer to the update event connection
private:
  event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(UvLed)
}
