#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>


namespace gazebo
{
class UvCam : public ModelPlugin {
private:
  int                      id;
  float                    f;
  float                    T;
  float                    Th;
  transport::SubscriberPtr poseSub;
  transport::SubscriberPtr stateSub;
  math::Pose               pose;
  bool                     ledState[20];
  math::Pose               ledPose[20];

public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {

    std::cout << "Initializing UV camera" << std::endl;

    // Store the pointer to the model
    this->model = _parent;

    id = 1;
    f  = 70;  // camera framerate
    T  = 1.0 / f;
    Th = T / 2.0;


    transport::NodePtr node(new transport::Node());
    node->Init();


    char poseTopicName[30];
    std::sprintf(poseTopicName, "~/uvleds/%d/pose", id);
    char stateTopicName[30];
    std::sprintf(stateTopicName, "~/uvleds/%d/state", id);


    poseSub  = node->Subscribe(poseTopicName, &UvCam::poseCB, this, false);
    stateSub = node->Subscribe(stateTopicName, &UvCam::stateCB, this, false);
    /* statePub->WaitForConnection(); */
    /* posePub->WaitForConnection(); */


    /* transport::fini(); */
    // Listen to the update event. This event is broadcast every
    /* shutdown(); */
    /* while (true){ */
    /*   common::Time::MSleep(1000*T); */

    /* } */
  }

  void Init() {
    for (int i             = 0; i++; i < 20)
      ledState[i]          = false;
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&UvCam::OnUpdate, this));
  }

public:
  void OnUpdate() {
    /* std::cout << "sending" << std::endl; */
    // Apply a small linear velocity to the model.
    pose = model->GetWorldPose();
    std::cout << ledPose[1] - pose << std::endl;
  }
  // Called by the world update start event
public:
  void poseCB(ConstPosePtr &i_pose) {
    /* math::Pose poseDiff = msgs::ConvertIgn(*i_pose) - pose; */
    /* std::cout << poseDiff.pos.x << std::endl; */
    ledPose[1] = msgs::ConvertIgn(*i_pose);
  }
  void stateCB(ConstIntPtr &i_state) {
    ledState[1] = (bool)(i_state->data());
    /* std::cout << i_state->data() << std::endl; */
  }

  // Pointer to the model
private:
  physics::ModelPtr model;

  // Pointer to the update event connection
private:
  event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(UvCam)
}
