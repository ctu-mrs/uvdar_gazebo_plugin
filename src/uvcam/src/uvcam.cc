#include <undistortFunctions/ocam_functions.h>
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>


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
  /* bool                     ledState[20]; */
  math::Pose        ledPose;
  double            ledProj[2];
  double            ledIntensity;
  struct ocam_model oc_model;
  cv::Mat           currImage;
  double            coef[3] = {1.3398, 31.4704, 0.0154};
  std::mutex        imgMtx;
  std::thread       draw_thread;

public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

    std::cout << "Initializing UV camera" << std::endl;

    // Store the pointer to the model
    this->model = _parent;

    std::cout << "Camera parent name: " << this->model->GetName()  << std::endl;

    /* id = 1; */
    if (_sdf->HasElement("framerate")) {
      f = _sdf->GetElement("framerate")->Get<double>();
    } else {
      f = 70.0;  // camera framerate
    }        

    T  = 1.0 / f;
    Th = T * 0.9;


    transport::NodePtr node(new transport::Node());
    node->Init();


    char poseTopicName[30];
    std::sprintf(poseTopicName, "~/uvleds/pose");
    /* char stateTopicName[30]; */
    /* std::sprintf(stateTopicName, "~/uvleds/state", id); */


    poseSub = node->Subscribe(poseTopicName, &UvCam::poseCB, this, false);
    /* stateSub = node->Subscribe(stateTopicName, &UvCam::stateCB, this, false); */
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
    /* for (int i    = 0; i++; i < 20) */
    /* ledState[i] = false; */
    get_ocam_model(&oc_model, (char *)"/home/viktor/gazebo_plugins/src/uvcam/config/calib_results.txt");
    ResetCurrImage();
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&UvCam::OnUpdate, this));
    draw_thread            = std::thread(&UvCam::DrawThread, this);
  }

private:
  void DrawThread() {
    while (true) {
      imgMtx.lock();
      cv::GaussianBlur(currImage, currImage, cv::Size(3, 3), 0, 0);
      cv::imshow("cv_fl_test", currImage);
      ResetCurrImage();
      imgMtx.unlock();
      cv::waitKey((int)(T * 1000));
      /* cv::waitKey((int)(10)); */
    }
  }

public:
  void ResetCurrImage() {
    currImage = cv::Mat(oc_model.height, oc_model.width, CV_8UC1, cv::Scalar(0));
  }

public:
  void OnUpdate() {
    /* std::cout << "sending" << std::endl; */
    // Apply a small linear velocity to the model.
    pose = model->GetWorldPose();
    /* for (int i = 0; i < 20; i++) { */
    /* if (ledState[i] == true) { */
  }
  // Called by the world update start event
public:
  void poseCB(ConstPosePtr &i_pose) {
    /* math::Pose poseDiff = msgs::ConvertIgn(*i_pose) - pose; */
    /* std::cout << poseDiff.pos.x << std::endl; */
    ledPose         = msgs::ConvertIgn(*i_pose);
    auto   diffPose = ledPose - pose;
    double input[3] = {diffPose.pos.y, diffPose.pos.x, -(diffPose.pos.z)};

    world2cam(ledProj, input, &oc_model);
    gazebo::math::Pose a        = math::Pose(0, 0, 1, 0, 0, 0).RotatePositionAboutOrigin(ledPose.rot);
    gazebo::math::Pose b        = math::Pose((pose.pos) - (ledPose.pos), math::Quaternion(0, 0, 0));
    double             distance = b.pos.GetLength();
    double             cosAngle = a.pos.Dot(b.pos) / (distance);
    ledIntensity                = round(std::max(.0, cosAngle) * (coef[0] + (coef[1] / ((distance + coef[2]) * (distance + coef[2])))));
    std::cout << "CAM: "
              << ": [" << distance << "]" << std::endl;
    std::cout << "CAM: "
              << ": [" << b << "]" << std::endl;
    std::cout << "CAM: "
              << ": [" << cosAngle << "]" << std::endl;
    std::cout << "CAM: "
              << ": [" << ledIntensity << "]" << std::endl;
    std::cout << "CAM: "
              << ": [" << input[0] << ":" << input[1] << ":" << input[2] << "]" << std::endl;
    std::cout << "CAM: "
              << ": [" << ledProj[1] << ":" << ledProj[0] << "]" << std::endl;
    /* } */
    /* } */
    imgMtx.lock();
    cv::circle(currImage, cv::Point2i(ledProj[1], ledProj[0]), ledIntensity, cv::Scalar(255), -1);
    imgMtx.unlock();
  }
  /* void stateCB(ConstIntPtr &i_state) { */
  /*   ledState[1] = (bool)(i_state->data()); */
  /*   /1* std::cout << i_state->data() << std::endl; *1/ */
  /* } */

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
