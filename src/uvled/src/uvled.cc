#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>


namespace gazebo
{
class UvLed : public ModelPlugin {
private:
  int   id;
  float f;
  float T;
  float Th;
    transport::PublisherPtr posePub  ;
    transport::PublisherPtr statePub ;

public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {

    id = 1;
    
    std::cout << "Initializing LED n. " << id << std::endl;

    // Store the pointer to the model
    this->model = _parent;
    /* std::cout << this->model->GetName()  << std::endl; */
    

    f  = 10;
    T  = 1.0 / f;
    Th = T / 2.0;




    transport::NodePtr node(new transport::Node());
    node->Init();


    char poseTopicName[30];
    std::sprintf(poseTopicName, "~/uvleds/%d/pose", id);
    char stateTopicName[30];
    std::sprintf(stateTopicName, "~/uvleds/%d/state", id);


    posePub  = node->Advertise<msgs::Pose>(poseTopicName);
    statePub = node->Advertise<msgs::Int>(stateTopicName);
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
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&UvLed::OnUpdate, this));
  }
  // Called by the world update start event
public:
  void OnUpdate() {
    /* std::cout << "sending" << std::endl; */
    // Apply a small linear velocity to the model.
    math::Pose pose  = model->GetWorldPose();
    bool       state = (fmod(common::Time::GetWallTime().Double(), T) > Th);

    /* std::cout << "f: "<< f << std::endl; */
    /* std::cout << "T: "<< T << std::endl; */
    /* std::cout << state << std::endl; */
    msgs::Pose poseMsg;
    msgs::Int stateMsg;
    msgs::Set(&poseMsg, pose.Ign());
    stateMsg.set_data(state);
    posePub->Publish(poseMsg);
    statePub->Publish(stateMsg);
  }

  // Pointer to the model
private:
  physics::ModelPtr model;

  // Pointer to the update event connection
private:
  event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(UvLed)
}
