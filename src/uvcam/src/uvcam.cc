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
    transport::SubscriberPtr poseSub  ;
    transport::SubscriberPtr stateSub ;

public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
    
    std::cout << "Initializing" << std::endl;

    // Store the pointer to the model
    this->model = _parent;

    id = 1;
    f  = 10;
    T  = 1.0 / f;
    Th = T / 2.0;




    transport::NodePtr node(new transport::Node());
    node->Init();


    char poseTopicName[30];
    std::sprintf(poseTopicName, "~/uvleds/%d/pose", id);
    char stateTopicName[30];
    std::sprintf(stateTopicName, "~/uvleds/%d/state", id);


    poseSub  = node->Subscribe<msgs::Pose>(poseTopicName, poseCB );
    stateSub = node->Subscribe(stateTopicName, stateCB);
    /* statePub->WaitForConnection(); */
    /* posePub->WaitForConnection(); */


    /* transport::fini(); */
    // Listen to the update event. This event is broadcast every
  }

  // Called by the world update start event
public:
  void static poseCB(ConstPosePtr &i_pose) {
    std::cout << "here" << std::endl;
  }
  void static stateCB(ConstIntPtr &i_state) {
    std::cout << i_state->data() << std::endl;
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
