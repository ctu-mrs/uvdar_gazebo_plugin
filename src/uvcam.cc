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
#include <ignition/math/Pose3.hh>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>

/* #define exprate 0.001 */

#define index2d(X, Y) (oc_model.width * (Y) + (X))

/* using namespace ignition; */
namespace gazebo
{
class UvCam : public SensorPlugin {
private:
  std::mutex mtx_buffer;
  std::vector<std::pair<ignition::math::Pose3d,ignition::math::Pose3d>> buffer;
  int                       id;
  float                     f;
  float                     T;
  float                     Th;
  uchar                     background;
  transport::SubscriberPtr  poseSub;
  transport::SubscriberPtr  stateSub;
  ignition::math::Pose3d                pose;
  gazebo::physics::WorldPtr world;
  physics::EntityPtr        parent;
  /* bool                     ledState[20]; */
  ignition::math::Pose3d       ledPose;
  ignition::math::Quaternion<double> invOrient;
  ignition::math::Pose3d       diffPose;

  ignition::math::Pose3d a;
  ignition::math::Pose3d b;
  double     distance;
  double     cosAngle;

  double radius;

  double            input[3];
  double            ledProj[2];
  double            ledIntensity;
  struct ocam_model oc_model;
  /* cv::Mat                          currImage; */
  cv_bridge::CvImage               cvimg;
  sensor_msgs::ImagePtr            msg;
  double                           coef[3] = {1.3398, 31.4704, 0.0154};
  /* std::mutex                       imgMtx; */
  std::thread                      draw_thread;
  ros::NodeHandle                  nh;
  image_transport::ImageTransport *it;
  image_transport::Publisher       pub;

  /* int  count; */
  bool shutterOpen;
  bool shutterOpenPrev;
  std::string filename;

  ros::Duration exposure;

public:
  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {

    std::cout << "Initializing UV camera" << std::endl;

    // Store the pointer to the model
    this->sensor           = _parent;
    world                  = physics::get_world("default");
    std::string parentName = sensor->ParentName();
    parent                 = world->EntityByName(parentName);
    std::cout << "Camera parent name: " << this->sensor->ScopedName() << std::endl;

    if (_sdf->HasElement("calibration_file")) {
      filename = _sdf->GetElement("calibration_file")->Get< std::string >();
      std::cout << "Calibration file is " << filename << std::endl;
    }
    else {
      std::cerr << "No calibration file provided. Exiting" << std::endl;
      return;
      }
    /* id = 1; */
    if (_sdf->HasElement("framerate")) {
      f = _sdf->GetElement("framerate")->Get< double >();
      std::cout << "LED framerate is " << f << "Hz" << std::endl;
    } else {
      std::cout << "LED framerate defaulting to 70Hz." << std::endl;
      f = 70.0;  // camera framerat
    }

    T        = 1.0 / f;
    Th       = T * 0.5;
    exposure = ros::Duration(Th);


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

    /* } */
    char *dummy = NULL;
    int   zero  = 0;
    ros::init(zero, &dummy, "uvdar_bluefox_emulator");
    it = new image_transport::ImageTransport(nh);
    char camTopicName[30];
    sprintf(camTopicName, "%s/uvdar_bluefox/image_raw", parentName.substr(0, parentName.find(":")).c_str());
    pub = it->advertise(camTopicName, 1);

    background = std::rand() % 100;
  }

  void Init() {
    /* for (int i    = 0; i++; i < 20) */
    /* ledState[i] = false; */

    get_ocam_model(&oc_model, (char*)(filename.c_str()));
    cvimg                  = cv_bridge::CvImage(std_msgs::Header(), "mono8", cv::Mat(oc_model.height, oc_model.width, CV_8UC1, cv::Scalar(0)));
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&UvCam::OnUpdate, this));
      /* std::cout << "Calling draw thread..." << std::endl; */
    draw_thread            = std::thread(&UvCam::DrawThread, this);
  }

private:
  void DrawThread() {
    ros::Rate rt(f);
    ros::Duration hd((rt.expectedCycleTime().toSec()*0.75));
    while (ros::ok()){
      shutterOpen =true;
      hd.sleep();
      shutterOpen = false;
      for (int j = 0; j < cvimg.image.rows; j++) {
        for (int i = 0; i < cvimg.image.cols; i++) {
          if (cvimg.image.data[index2d(i, j)] != background)
            (cvimg.image.data[index2d(i, j)] = background);
        }
      }
      /* cvimg.image = cv::Scalar(background); */
      /* imgMtx.unlock(); */
      /* std::cout << "DT: buffer size: " << buffer.size() << std::endl; */
      mtx_buffer.lock();
      for (std::pair<ignition::math::Pose3d,ignition::math::Pose3d>& i : buffer){
        /* std::cout << "Drawing..." << std::endl; */
        drawPose(i);
      }
      /* std::cout << "Finished drawing; clearing buffer..." << std::endl; */
      buffer.clear();
      /* std::cout << "Buffer cleared" << std::endl; */
      /* std::cout << "DT: Unlocking mutex..." << std::endl; */
      mtx_buffer.unlock();

      msg = cvimg.toImageMsg();
      /* std::cout << "Publishing..." << std::endl; */
      pub.publish(msg);
      /* std::cout << "Count: " << count << std::endl; */
      /* count = 0; */
      /* if (!(r.sleep())) */
      /*   std::cout << "LATE! " << r.cycleTime() << " s" << std::endl; */
      rt.sleep();
    }
  }


public:
  void OnUpdate() {
    /* std::cout << "sending" << std::endl; */
    // Apply a small linear velocity to the model.
    pose = sensor->Pose() + parent->WorldPose();

    /* shutterOpenPrev = shutterOpen; */
    /* shutterOpen = (fmod(ros::Time::now().toSec(), T) > Th); */

    /* if ((!shutterOpen) && (shutterOpenPrev)) { */
      /* /1* std::cout << "Joining draw thread..." << std::endl; *1/ */
      /* /1* draw_thread.join(); *1/ */
      /* } */
      /* ros::Rate r(f);  // 10 h */
      /* r.reset(); */
        /* imgMtx.lock(); */
        /* std::scoped_lock lock(mtx_buffer); */
  }
  // Called by the world update start event
public:
  void poseCB(ConstPosePtr &i_pose) {
    /* std::cout << "Got pose..." << std::endl; */
    if (!shutterOpen)
      return;

    /* std::cout << "Shutter is opened" << std::endl; */

    /* ignition::math::Pose3d poseDiff = msgs::ConvertIgn(*i_pose) - pose; */
    /* std::cout << poseDiff.Pos().X() << std::endl; */
    /* crcMtx.lock(); */
    ledPose   = msgs::ConvertIgn(*i_pose);
    /* std::cout << "CB: Locking mutex..." << std::endl; */
    mtx_buffer.lock();
    /* std::cout << "Pushing poses to buffer..." << std::endl; */
    buffer.push_back(std::pair<ignition::math::Pose3d,ignition::math::Pose3d>(ledPose,pose));
    /* std::cout << "CB: Unlocking mutex..." << std::endl; */
    mtx_buffer.unlock();
  }
  void drawPose(std::pair<ignition::math::Pose3d,ignition::math::Pose3d> input_poses){
    /* std::cout << "A" << std::endl; */
    ignition::math::Pose3d ledPose = input_poses.first;
    ignition::math::Pose3d pose = input_poses.second;
    invOrient = ledPose.Rot();
    invOrient.Invert();
    diffPose = (ledPose - pose);
    input[0] = -(diffPose.Pos().Z());
    input[1] = -(diffPose.Pos().Y());
    input[2] = -(diffPose.Pos().X());

    /* std::cout << "B" << std::endl; */
    world2cam(ledProj, input, &oc_model);
    a            = ignition::math::Pose3d(0, 0, 1, 0, 0, 0).RotatePositionAboutOrigin(invOrient);
    b            = ignition::math::Pose3d((pose.Pos()) - (ledPose.Pos()), ignition::math::Quaternion<double>(0, 0, 0));
    distance     = b.Pos().Length();
    cosAngle     = a.Pos().Dot(b.Pos()) / (distance);
    ledIntensity = round(std::max(.0, cosAngle) * (coef[0] + (coef[1] / ((distance + coef[2]) * (distance + coef[2])))));

    radius = sqrt(ledIntensity / M_PI);

    /* std::cout << "C" << std::endl; */

    /* count++; */
    if (ledIntensity > 0) {
      /* if (imgMtx.try_lock()) { */
      /* imgMtx.lock(); */
      /* if (shutterOpen) { */
    /* std::cout << "D" << std::endl; */
        cv::circle(cvimg.image, cv::Point2i(ledProj[1], ledProj[0]), radius, cv::Scalar(255), -1);
    /* std::cout << "E" << std::endl; */
      /* } */
      /* imgMtx.unlock(); */
    }
  }
  /* crcMtx.unlock(); */
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
