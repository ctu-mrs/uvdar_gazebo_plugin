#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include <ignition/math/Vector3.hh>


namespace gazebo
{
class UvLed : public VisualPlugin {
private:
  float f;
  float T;
  float Th;
    transport::PublisherPtr posePub  ;
    /* transport::PublisherPtr statePub ; */

public:
  void Load(const rendering::VisualPtr _visual, sdf::ElementPtr _sdf) {

    
    std::cout << "Loading UV LED" << std::endl;

    // Store the pointer to the model
    this->model = _visual;
    std::cout << "LED parent name: " << this->model->GetMeshName()  << std::endl;
    
    std::cout << "LED element : " << _sdf->GetAttribute("name")->GetAsString() << std::endl;

    if (_sdf->HasElement("frequency")) {
    std::cout << "LED element frequency exists." << std::endl;
    
      f = _sdf->GetElement("frequency")->Get<double>();
    } else {
    std::cout << "LED defaulting to 10Hz." << std::endl;
      f = 10.0;  // camera framerate
    }        

    std::cout << "LED frequency : "<< f << std::endl;

    T  = 1.0 / f;
    Th = T / 2.0;




    transport::NodePtr node(new transport::Node());
    node->Init();


    char poseTopicName[30];
    std::sprintf(poseTopicName, "~/uvleds/pose");
    /* char stateTopicName[30]; */
    /* std::sprintf(stateTopicName, "~/uvleds/state", id); */


    posePub  = node->Advertise<msgs::Pose>(poseTopicName);
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
  void Init(){
    std::cout << "Initializing UV LED" << std::endl;
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&UvLed::OnUpdate, this));
  }
  // Called by the world update start event
public:
  void OnUpdate() {
    /* std::cout << "sending" << std::endl; */
    // Apply a small linear velocity to the model.
    bool       state = (fmod(common::Time::GetWallTime().Double(), T) > Th);

    /* std::cout << "T: "<< T << std::endl; */
    /* std::cout << state << std::endl; */
    if ((!state) && (f>0.0)){
      return;
    }
    math::Pose pose  = model->GetPose();
    std::cout << "LED frequency: " << f << std::endl;
    msgs::Pose poseMsg;
    msgs::Set(&poseMsg, pose.Ign());
    posePub->Publish(poseMsg);
    /* msgs::Int stateMsg; */
    /* stateMsg.set_data(state); */
    /* statePub->Publish(stateMsg); */
  }

  // Pointer to the model
private:
  rendering::VisualPtr model;

  // Pointer to the update event connection
private:
  event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(UvLed)
}
