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
#include <thread>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <unordered_map>
#include <ObjectMgr/LedMgr.h>
#include <std_msgs/Float64.h>
#include <uvdar_gazebo_plugin/LedInfo.h>
/* #define exprate 0.001 */

#define index2d(X, Y) (oc_model.width * (Y) + (X))

/* using namespace ignition; */
namespace gazebo
{
class UvCam : public SensorPlugin {
private:
  /* variables //{ */
  /* std::mutex mtx_buffer; */
  boost::mutex mtx_leds;
  /* std::vector<std::pair<ignition::math::Pose3d,ignition::math::Pose3d>> buffer; */
  int                       id;
  float                     f;
  uchar                     background;
  transport::SubscriberPtr  poseSub;
  transport::SubscriberPtr  stateSub;
  ros::Subscriber           linkSub;
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
  cv_bridge::CvImage               cvimg;
  sensor_msgs::ImagePtr            msg;
  double                           coef[3] = {1.3398, 31.4704, 0.0154};
  std::thread                      draw_thread;
  ros::NodeHandle                  nh;
  image_transport::ImageTransport *it;
  image_transport::Publisher       pub;
  std::vector<ros::Subscriber> ledInfoSubscribers;

  std::string filename;

  std::unordered_map<std::string, std::shared_ptr<LedMgr> > _leds_by_name_;

  sensors::SensorPtr sensor;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;

  //}

public:

/* Load //{ */
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

    transport::NodePtr node(new transport::Node());
    nh = ros::NodeHandle("~");

    node->Init();


    char poseTopicName[30];
    std::sprintf(poseTopicName, "~/uvleds/pose");
    /* char stateTopicName[30]; */
    /* std::sprintf(stateTopicName, "~/uvleds/state", id); */


    /* poseSub = node->Subscribe(poseTopicName, &UvCam::poseCB, this, false); */
    /* stateSub = node->Subscribe(stateTopicName, &UvCam::stateCB, this, false); */
    /* statePub->WaitForConnection(); */
    /* posePub->WaitForConnection(); */

    linkSub = nh.subscribe("/gazebo/link_states", 1, &UvCam::linkCallback, this);


    /* transport::fini(); */
    // Listen to the update event. This event is broadcast every
    /* shutdown(); */
    /* while (true){ */

    /* } */
    char *dummy = NULL;
    int   zero  = 0;
    ros::init(zero, &dummy, "uvdar_bluefox_emulator");
    it = new image_transport::ImageTransport(nh);
    std::string publish_topic;
    if (_sdf->HasElement("camera_publish_topic")) {
      publish_topic = _sdf->GetElement("camera_publish_topic")->Get<std::string>();
    } else {
      publish_topic = parentName.substr(0, parentName.find(":")) + "/uvdar_bluefox/image_raw";
    }
    std::cout << "Publishing UV Camera to topic: \"" << publish_topic << "\"" << std::endl;
    pub = it->advertise(publish_topic, 1);

    background = std::rand() % 100;
  }
  //}

  /* Init //{ */
  void Init() {
    /* for (int i    = 0; i++; i < 20) */
    /* ledState[i] = false; */

    get_ocam_model(&oc_model, (char*)(filename.c_str()));
    cvimg                  = cv_bridge::CvImage(std_msgs::Header(), "mono8", cv::Mat(oc_model.height, oc_model.width, CV_8UC1, cv::Scalar(0)));
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&UvCam::OnUpdate, this));

    draw_thread            = std::thread(&UvCam::DrawThread, this);
  }
  //}

private:
  /* DrawThread //{ */
  void DrawThread() {
    ros::Rate rt(f);
    geometry_msgs::Pose cur_pose;
    while (ros::ok()){
      boost::mutex::scoped_lock lock(mtx_leds);
        //CHECK: Optimize the following
        for (int j = 0; j < cvimg.image.rows; j++) {
          for (int i = 0; i < cvimg.image.cols; i++) {
            if (cvimg.image.data[index2d(i, j)] != background)
              (cvimg.image.data[index2d(i, j)] = background);
          }
        }
      /* for (std::pair<ignition::math::Pose3d,ignition::math::Pose3d>& i : buffer){ */
    for (auto led : _leds_by_name_){
      if (led.second->get_pose(cur_pose, ros::Time::now().toSec())){
        drawPose({cur_pose,pose});
      }
    }
      msg = cvimg.toImageMsg();
      msg->header.stamp = ros::Time::now();
      pub.publish(msg);
      rt.sleep();
    }
  }
  //}

public:
/* OnUpdate //{ */
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
    /* std::cout << "Found objects: " << std::endl; */
    /* for (auto object : _leds_by_name_){ */
    /*   std::cout << object.first << std::endl; */
    /* } */
    /* std::cout << "---" << std::endl; */
  }
  // Called by the world update start event
  //}
  
public:

/* linkCallback //{ */
void linkCallback(const gazebo_msgs::LinkStates link_states)
{
	const std::vector<std::string>& names = link_states.name;
	const std::vector<geometry_msgs::Pose>& poses = link_states.pose;
  
	for (size_t it = 0; it < names.size(); ++it) 
  {
    const std::string& cur_name = names.at(it);
    const size_t prependix_pos = cur_name.find_first_of("::");
		const std::string cur_model_name = cur_name.substr(0, prependix_pos);
		const std::string cur_link_name = cur_name.substr(prependix_pos+2);


    /* std::cout << "pose of " << cur_name << ": " << std::endl; */
    /* std::cout << poses.at(it) << std::endl; */

    if (cur_link_name.find("_uvled_") != std::string::npos){
      if (_leds_by_name_.count(cur_link_name) != 0){
        boost::mutex::scoped_lock lock(mtx_leds);
        _leds_by_name_.at(cur_link_name)->update_link_pose(cur_link_name, poses.at(it));
      }
      else{
        boost::mutex::scoped_lock lock(mtx_leds);
        std::shared_ptr<LedMgr> led = std::make_shared<LedMgr>(nh, cur_name);
        _leds_by_name_.insert({cur_link_name, led});
        /* std::cout << "Subscribing to LED info of " << "/gazebo/ledProperties/"+cur_link_name << std::endl; */
        ledInfoSubscribers.push_back(nh.subscribe("/gazebo/ledProperties/"+cur_link_name, 1, &UvCam::ledCallback,this));
      }
    }
	}

}
//}

/* void ledCallback //{ */
void ledCallback(const ros::MessageEvent<uvdar_gazebo_plugin::LedInfo const>& event){
  /* std::cout << "Getting message" << std::endl; */
    ros::M_string mhdr = event.getConnectionHeader();
    std::string topic = mhdr["topic"];
    std::string link_name = topic.substr(std::string("/gazebo/ledProperties/").length());
    const uvdar_gazebo_plugin::LedInfoConstPtr& led_info = event.getMessage();
    std::cout << "UV CAM: receiving frequency of " << led_info->frequency.data << " for link  " << link_name << std::endl;
    _leds_by_name_.at(link_name)->update_frequency(led_info->frequency.data);
}
//}

/* poseCB //{ */
  /* void poseCB(ConstPosePtr &i_pose) { */
  /*   /1* std::cout << "Got pose..." << std::endl; *1/ */
  /*   if (!shutterOpen) */
  /*     return; */

  /*   /1* std::cout << "Shutter is opened" << std::endl; *1/ */

  /*   /1* ignition::math::Pose3d poseDiff = msgs::ConvertIgn(*i_pose) - pose; *1/ */
  /*   /1* std::cout << poseDiff.Pos().X() << std::endl; *1/ */
  /*   /1* crcMtx.lock(); *1/ */
  /*   ledPose   = msgs::ConvertIgn(*i_pose); */
  /*   /1* std::cout << "CB: Locking mutex..." << std::endl; *1/ */
  /*   mtx_buffer.lock(); */
  /*   /1* std::cout << "Pushing poses to buffer..." << std::endl; *1/ */
  /*   buffer.push_back(std::pair<ignition::math::Pose3d,ignition::math::Pose3d>(ledPose,pose)); */
  /*   /1* std::cout << "CB: Unlocking mutex..." << std::endl; *1/ */
  /*   mtx_buffer.unlock(); */
  /* } */
//}

/* drawPose //{ */
  void drawPose(std::pair<geometry_msgs::Pose,ignition::math::Pose3d> input_poses){
    /* std::cout << "A" << std::endl; */
    ignition::math::Pose3d ledPose(
        input_poses.first.position.x,
        input_poses.first.position.y,
        input_poses.first.position.z,
        input_poses.first.orientation.w,
        input_poses.first.orientation.x,
        input_poses.first.orientation.y,
        input_poses.first.orientation.z
        );

    ignition::math::Pose3d pose = input_poses.second;
    invOrient = ledPose.Rot();
    invOrient.Invert();
    diffPose = (ledPose - pose);
    input[0] = -(diffPose.Pos().Z());
    input[1] = -(diffPose.Pos().Y());
    input[2] = -(diffPose.Pos().X());

    /* std::cout << "input: " << std::endl; */
    /* std::cout << input_poses.first << std::endl; */
    /* std::cout << "converted: " << std::endl; */
    /* std::cout << ledPose << std::endl; */
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
//}

/* drawPose_legacy //{ */
  void drawPose_legacy(std::pair<ignition::math::Pose3d,ignition::math::Pose3d> input_poses){
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
    //}

};

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(UvCam)
}
