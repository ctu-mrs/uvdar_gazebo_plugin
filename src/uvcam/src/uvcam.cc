#include <undistortFunctions/ocam_functions.h>
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


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
  double                   ledProj[20][2];
  double                   ledIntensity[20];
  struct ocam_model        oc_model;
  cv::Mat                  currImage;
  double                   coef[3] = {1.3398, 31.4704, 0.0154};

public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {

    std::cout << "Initializing UV camera" << std::endl;

    // Store the pointer to the model
    this->model = _parent;

    id = 1;
    f  = 70;  // camera framerate
    T  = 1.0 / f;
    Th = T * 0.9;


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
    for (int i    = 0; i++; i < 20)
      ledState[i] = false;
    get_ocam_model(&oc_model, (char *)"/home/viktor/gazebo_plugins/src/uvcam/config/calib_results.txt");
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&UvCam::OnUpdate, this));
  }

public:
  void DrawCurrImage() {
    currImage = cv::Mat(oc_model.height, oc_model.width, CV_8UC1, cv::Scalar(0));
    cv::circle(currImage, cv::Point2i(ledProj[1][1], ledProj[1][0]), ledIntensity[1], cv::Scalar(255), -1);
    cv::GaussianBlur(currImage, currImage, cv::Size(3,3), 0,0);
  }

public:
  void OnUpdate() {
    /* std::cout << "sending" << std::endl; */
    // Apply a small linear velocity to the model.
    pose = model->GetWorldPose();
    for (int i = 0; i < 20; i++) {
      if (ledState[i] == true) {
        auto   diffPose = ledPose[i] - pose;
        double input[3] = {diffPose.pos.y, diffPose.pos.x, -(diffPose.pos.z)};

        std::cout << oc_model.height << std::endl;
        world2cam(ledProj[i], input, &oc_model);
        gazebo::math::Pose a        = math::Pose(0, 0, 1, 0, 0, 0).RotatePositionAboutOrigin(ledPose[i].rot);
        gazebo::math::Pose b        = math::Pose((pose.pos) - (ledPose[i].pos), math::Quaternion(0, 0, 0));
        double             distance = b.pos.GetLength();
        double             cosAngle = a.pos.Dot(b.pos) / (distance);
        ledIntensity[i]             = round(std::max(.0, cosAngle) * (coef[0] + (coef[1] / ((distance + coef[2]) * (distance + coef[2])))));
        std::cout << "LED n. " << i << ": [" << distance << "]" << std::endl;
        std::cout << "LED n. " << i << ": [" << b << "]" << std::endl;
        std::cout << "LED n. " << i << ": [" << cosAngle << "]" << std::endl;
        std::cout << "LED n. " << i << ": [" << ledIntensity[i] << "]" << std::endl;
        std::cout << "LED n. " << i << ": [" << input[0] << ":" << input[1] << ":" << input[2] << "]" << std::endl;
        std::cout << "LED n. " << i << ": [" << ledProj[1][1] << ":" << ledProj[1][0] << "]" << std::endl;
      }
    }
    if (fmod(common::Time::GetWallTime().Double(), T) > Th) {
      DrawCurrImage();
      cv::imshow("cv_fl_test", currImage);
      cv::waitKey(20);
    }
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
