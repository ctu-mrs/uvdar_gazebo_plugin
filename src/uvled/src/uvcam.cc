#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <undistortFunctions/ocam_functions.h>
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math/Vector3.hh>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>

#define exprate 0.001


namespace gazebo
{
class UvCam : public SensorPlugin {
private:
  int                       id;
  float                     f;
  float                     T;
  float                     Th;
  uchar                     background;
  transport::SubscriberPtr  poseSub;
  transport::SubscriberPtr  stateSub;
  math::Pose                pose;
  gazebo::physics::WorldPtr world;
  physics::EntityPtr        parent;
  /* bool                     ledState[20]; */
  math::Pose                       ledPose;
  double                           ledProj[2];
  double                           ledIntensity;
  struct ocam_model                oc_model;
  cv::Mat                          currImage;
  double                           coef[3] = {1.3398, 31.4704, 0.0154};
  std::mutex                       imgMtx;
  std::thread                      draw_thread;
  ros::NodeHandle                  nh;
  image_transport::ImageTransport *it;
  image_transport::Publisher       pub;

public:
  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {

    std::cout << "Initializing UV camera" << std::endl;

    // Store the pointer to the model
    this->sensor           = _parent;
    world                  = physics::get_world("default");
    std::string parentName = sensor->ParentName();
    parent                 = world->GetEntity(parentName);
    std::cout << "Camera parent name: " << this->sensor->ScopedName() << std::endl;

    /* id = 1; */
    if (_sdf->HasElement("framerate")) {
      f = _sdf->GetElement("framerate")->Get<double>();
      std::cout << "LED framerate is " << f << "Hz" << std::endl;
    } else {
      std::cout << "LED framerate defaulting to 70Hz." << std::endl;
      f = 70.0;  // camera framerat
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
    char *dummy = NULL;
    int   zero  = 0;
    ros::init(zero, &dummy, "uv_bluefox_emulator");
    it = new image_transport::ImageTransport(nh);
    char camTopicName[30];
    sprintf(camTopicName, "%s/bluefox/image_raw", parentName.substr(0, parentName.find(":")).c_str());
    pub = it->advertise(camTopicName, 1);

    background = std::rand() % 100;
  }

  void Init() {
    /* for (int i    = 0; i++; i < 20) */
    /* ledState[i] = false; */
    get_ocam_model(&oc_model, (char *)"/home/viktor/gazebo_plugins/src/uvled/config/calib_results.txt");
    ResetCurrImage();
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&UvCam::OnUpdate, this));
    draw_thread            = std::thread(&UvCam::DrawThread, this);
  }

private:
  void DrawThread() {
    ros::Rate r(f);  // 10 h
    while (ros::ok()) {
      imgMtx.lock();
      /* ros::Duration((1.0 / f) - exprate).sleep(); */
      /* cv::GaussianBlur(currImage, currImage, cv::Size(3, 3), 0, 0); */
      /* cv::imshow("cv_fl_test", currImage); */
      /* cv::waitKey((int)(T * 1000)); */

      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", currImage).toImageMsg();

      /* ros::Rate loop_rate(5); */
      /* while (nh.ok()) { */
      /* std::cout << "publishing image" << std::endl; */
      pub.publish(msg);
      ResetCurrImage();
      imgMtx.unlock();
      r.sleep();
      /* cv::waitKey((int)(T * 1000)); */
    }
  }

public:
  void ResetCurrImage() {
    currImage = cv::Mat(oc_model.height, oc_model.width, CV_8UC1, cv::Scalar(background));
  }

public:
  void OnUpdate() {
    /* std::cout << "sending" << std::endl; */
    // Apply a small linear velocity to the model.
    pose = sensor->Pose() + parent->GetWorldPose().Ign();
  }
  // Called by the world update start event
public:
  void poseCB(ConstPosePtr &i_pose) {
    /* math::Pose poseDiff = msgs::ConvertIgn(*i_pose) - pose; */
    /* std::cout << poseDiff.pos.x << std::endl; */
    ledPose                    = msgs::ConvertIgn(*i_pose);
    math::Quaternion invOrient = ledPose.rot;
    invOrient.Invert();
    auto   diffPose = (ledPose - pose);
    double input[3] = {-(diffPose.pos.z), -(diffPose.pos.y), -(diffPose.pos.x)};

    world2cam(ledProj, input, &oc_model);
    gazebo::math::Pose a        = math::Pose(0, 0, 1, 0, 0, 0).RotatePositionAboutOrigin(invOrient);
    gazebo::math::Pose b        = math::Pose((pose.pos) - (ledPose.pos), math::Quaternion(0, 0, 0));
    double             distance = b.pos.GetLength();
    double             cosAngle = a.pos.Dot(b.pos) / (distance);
    ledIntensity                = round(std::max(.0, cosAngle) * (coef[0] + (coef[1] / ((distance + coef[2]) * (distance + coef[2])))));

    /* if (ledIntensity>0){ */
    /* std::cout << "CAM: " */
    /*           << "local: [" << sensor->Pose() << "]" << std::endl; */
    /* std::cout << "CAM: " */
    /*           << "pose: [" << pose << "]" << std::endl; */
    /* std::cout << "CAM: " */
    /*           << "ledPose: [" << ledPose << "]" << std::endl; */
    /* std::cout << "CAM: " */
    /*           << "dostance: [" << distance << "]" << std::endl; */
    /* std::cout << "CAM: " */
    /*           << "ledRot: [" << ledPose.rot << "]" << std::endl; */
    /* std::cout << "CAM: " */
    /*           << "a: [" << a << "]" << std::endl; */
    /* std::cout << "CAM: " */
    /*           << "b: [" << b << "]" << std::endl; */
    /* std::cout << "CAM: " */
    /*           << "cosAngle: [" << cosAngle << "]" << std::endl; */
    /* std::cout << "CAM: " */
    /*           << "intensity: [" << ledIntensity << "]" << std::endl; */
    /* std::cout << "CAM: " */
    /*           << "input: [" << input[0] << ":" << input[1] << ":" << input[2] << "]" << std::endl; */
    /* std::cout << "CAM: " */
    /*           << "proj: [" << ledProj[1] << ":" << ledProj[0] << "]" << std::endl; */
    /* } */
    if (imgMtx.try_lock()) {
      if (ledIntensity > 0) {
        cv::circle(currImage, cv::Point2i(ledProj[1], ledProj[0]), ledIntensity, cv::Scalar(255), -1);
      }
      imgMtx.unlock();
    }
  }
  /*   ledState[1] = (bool)(i_state->data()); */
  /*   /1* std::cout << i_state->data() << std::endl; *1/ */

  // Pointer to the sensor
private:
  sensors::SensorPtr sensor;

  // Pointer to the update event connection
private:
  event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(UvCam)
}
