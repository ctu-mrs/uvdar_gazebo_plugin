#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <undistortFunctions/ocam_functions.h>
#include <functional>
#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ode/ODEPhysics.hh>
#include <gazebo/physics/ode/ODECollision.hh>
#include <gazebo/physics/ode/ODETypes.hh>
#include <gazebo/physics/ode/ODERayShape.hh>
#include "collisions/collision_kernel.h"
#include "collisions/collision_std.h"
#include "collisions/intersection.hpp"
#include <gazebo/sensors/sensors.hh>
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/rendering.hh"
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
#include <sensor_msgs/PointCloud.h>
/* #define exprate 0.001 */

#define index2d(X, Y) (oc_model.width * (Y) + (X))

/* using namespace ignition; */
namespace gazebo
{
class UvCam : public SensorPlugin {
private:
  /* variables //{ */
  /* std::mutex mtx_buffer; */
  boost::mutex mtx_leds, mtx_occlusion;
  /* std::vector<std::pair<ignition::math::Pose3d,ignition::math::Pose3d>> buffer; */
  int                       id;
  double                    f,T;
  bool                      _use_occlusions_;
  uchar                     background;
  transport::SubscriberPtr  poseSub;
  transport::SubscriberPtr  stateSub;
  ros::Subscriber           linkSub;
  ignition::math::Pose3d                pose;
  gazebo::physics::WorldPtr world;
  gazebo::physics::PhysicsEnginePtr pengine;
  physics::RayShapePtr curr_ray;
  physics::EntityPtr        parent;
  /* bool                     ledState[20]; */
  ignition::math::Pose3d       ledPose;
  ignition::math::Quaternion<double> invOrient;
  ignition::math::Pose3d       diffPose;

  ignition::math::Pose3d a;
  ignition::math::Pose3d b;
  ignition::math::Pose3d c;
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
  std::thread                      transfer_thread;
  std::thread                      link_callback_thread;
  ros::NodeHandle                  nh_;
  std::vector<ros::Subscriber> ledInfoSubscribers;

  std::string filename;

  std::unordered_map<std::string, std::shared_ptr<LedMgr> > _leds_by_name_;

  sensors::SensorPtr sensor;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;

  /* ros::Time last_link_received_; */

  ros::CallbackQueue link_queue_;

  ros::Publisher virtual_points_publisher;

  rendering::ScenePtr world_scene_;
  rendering::VisualPtr visual_current_;
  std::vector<rendering::VisualPtr> visuals_serialized;

  /* bool occlusion_initialized_; */
  //}

public:

/* Load //{ */
  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {

    /* occlusion_initialized_ = false; */

    gazebo::rendering::Events::createScene("default");


    std::cout << "Initializing UV camera" << std::endl;

    /* CameraPlugin::Load(_parent, _sdf); */
    // copying from CameraPlugin into GazeboRosCameraUtils
    // Store the pointer to the model
    this->sensor           = _parent;

    /* this->parentSensor->SetActive(false); */
    world      = physics::get_world("default");
    pengine      = world->Physics();
    curr_ray = boost::dynamic_pointer_cast<physics::RayShape>(
        pengine->CreateShape("ray", physics::CollisionPtr()));
    std::string parentName = _parent->ParentName();
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

    T = 1.0/f;

    if (_sdf->HasElement("occlusion")) {
      _use_occlusions_ = _sdf->GetElement("occlusion")->Get< bool >();
    }
    else
      _use_occlusions_ = false;


    if (_use_occlusions_) {
      std::cout << "Occlusions in UV camera enabled!" << std::endl;
    }
    else {
      std::cout << "Occlusions in UV camera disabled!" << std::endl;
    }

    /* transport::NodePtr node(new transport::Node()); */
    nh_ = ros::NodeHandle("~");



    char poseTopicName[30];
    std::sprintf(poseTopicName, "~/uvleds/pose");


    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<gazebo_msgs::LinkStates>(
          "/gazebo/link_states",
          1,
          boost::bind(&UvCam::linkCallback, this, _1),
          ros::VoidPtr(), &link_queue_);

    linkSub = nh_.subscribe(so);
    /* linkSub = nh.subscribe("/gazebo/link_states", 1, &UvCam::linkCallback, this); */


    /* node->Init(); */

    char *dummy = NULL;
    int   zero  = 0;
    ros::init(zero, &dummy, "uvdar_bluefox_emulator");
    std::string publish_topic;
    if (_sdf->HasElement("camera_publish_topic")) {
      publish_topic = _sdf->GetElement("camera_publish_topic")->Get<std::string>();
    } else {
      publish_topic = parentName.substr(0, parentName.find(":")) + "/uvdar_bluefox/image_raw";
    }
    std::cout << "Publishing UV Camera data to topic: /gazebo" << publish_topic << "\"" << std::endl;

    virtual_points_publisher = nh_.advertise<sensor_msgs::PointCloud>("/gazebo"+publish_topic, 1);

  }
  //}

  /* Init //{ */
  void Init() {
    /* for (int i    = 0; i++; i < 20) */
    /* ledState[i] = false; */

    get_ocam_model(&oc_model, (char*)(filename.c_str()));
    /* cvimg                  = cv_bridge::CvImage(std_msgs::Header(), "mono8", cv::Mat(oc_model.height, oc_model.width, CV_8UC1, cv::Scalar(0))); */
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&UvCam::OnUpdate, this));

    transfer_thread            = std::thread(&UvCam::TransferThread, this);
    link_callback_thread   = std::thread(&UvCam::LinkQueueThread, this);
  }
  //}

private:
  /* TransferThread //{ */
  void TransferThread() {
    /* clock_t begin, end; */
    /* clock_t begin_p, end_p; */
    /* double  elapsedTime; */
    pengine->InitForThread();

    ros::Rate rt(f);
    geometry_msgs::Pose cur_pose;
    cv::Point3d output;
    geometry_msgs::Point32 pt;
    sensor_msgs::PointCloud msg_ptcl;
    std::pair<std::string, std::shared_ptr<LedMgr> > led;
    while (ros::ok()){
      /* begin         = std::clock(); */
      //CHECK: Optimize the following
      /* for (int j = 0; j < cvimg.image.rows; j++) { */
      /*   for (int i = 0; i < cvimg.image.cols; i++) { */
      /*     if (cvimg.image.data[index2d(i, j)] != background) */
      /*       (cvimg.image.data[index2d(i, j)] = background); */
      /*   } */
      /* } */
      /* end_p         = std::clock(); */
      /* elapsedTime = double(end_p - begin) / CLOCKS_PER_SEC; */
      /* std::cout << "UV CAM: Wiping took : " << elapsedTime << " s" << std::endl; */
      /* for (std::pair<ignition::math::Pose3d,ignition::math::Pose3d>& i : buffer){ */
      msg_ptcl.header.stamp = ros::Time::now();
      msg_ptcl.points.clear();
      double sec_time = ros::Time::now().toSec();
      for (auto& ledr : _leds_by_name_){
        {
          boost::mutex::scoped_lock lock(mtx_leds);
          led = ledr;
        }
        if (led.second->get_pose(cur_pose, sec_time)){
          if (drawPose_virtual({cur_pose,pose}, output)){
            pt.x = output.x;
            pt.y = output.y;
            pt.z = output.z;
            msg_ptcl.points.push_back(pt);
          }
        }
      }
      virtual_points_publisher.publish(msg_ptcl);
      /* end         = std::clock(); */
      /* elapsedTime = double(end - begin) / CLOCKS_PER_SEC; */
      /* std::cout << "UV CAM: Drawing took : " << elapsedTime << " s" << std::endl; */
      rt.sleep();
    }
  }
  //}
  
  /* LinkQueueThread //{ */
  void LinkQueueThread() {

    static const double timeout = T;
    ros::Rate rt(f);
    while (nh_.ok())
    {
      link_queue_.callAvailable(ros::WallDuration(timeout));
      rt.sleep();
    }
  }
  //}

public:
/* OnUpdate //{ */
  void OnUpdate() {
    /* ros::spinOnce(); */
    /* std::cout << "sending" << std::endl; */
    // Apply a small linear velocity to the model.
    pose = sensor->Pose() + parent->WorldPose();

    
    /* occlusion_initialized_ = true; */
    /* if (!occlusion_initialized_){ */
    /*   std::cout << "Initializing world" << std::endl; */

    /*   /1* rendering::RenderEngine::Instance()->Init(); *1/ */
    /*   /1* rendering::fini(); *1/ */
    /*   rendering::load(); */
    /*   /1* rendering::init(); *1/ */
    /*   /1* rendering::load(); *1/ */
    /*   /1* rendering::init(); *1/ */
    /*   rendering::create_scene("occlusion", true, true); */

    /*   /1* std::cout << "renderPathType: " << rendering::RenderEngine::Instance()->GetRenderPathType() << std::endl; *1/ */
    /*   /1* std::cout << "Scene count: " << rendering::RenderEngine::Instance()->SceneCount() << std::endl; *1/ */
      
    /*   world_scene_ = rendering::get_scene(); */    
    /*   std::cout << "Scene: " << world_scene_->Name() << std::endl; */
    /*   /1* world_scene_->Init(); *1/ */
    /*   if (!(world_scene_)){ */
    /*     std::cout << "Scene was not found." << std::endl; */
    /*     return; */
    /*   } */
    /*   if (!(world_scene_->Initialized())){ */
    /*     std::cout << "Scene " << world_scene_->Name() <<" is not initialized." << std::endl; */
    /*     return; */
    /*   } */

    /*   if (!world_scene_ || !(world_scene_->Initialized())) */
    /*     return; */

    /*   std::cout << "Retrieving visual scene objects." << std::endl; */
    /*   visual_current_ = world_scene_->WorldVisual(); */
    /*   std::cout << "Serializing occluding objects." << std::endl; */
    /*   meshVisuals(visual_current_, visuals_serialized); */

    /*   occlusion_initialized_ = true; */

    /*   std::cout << "Initialization completed" << std::endl; */

    /* } */
    /* else{ */
    /*   boost::mutex::scoped_lock lock(mtx_occlusion); */
    /* } */

    /* shutterOpenPrev = shutterOpen; */
    /* shutterOpen = (fmod(ros::Time::now().toSec(), T) > Th); */

    /* if ((!shutterOpen) && (shutterOpenPrev)) { */
      /* /1* std::cout << "Joining draw thread..." << std::endl; *1/ */
      /* /1* transfer_thread.join(); *1/ */
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
void linkCallback(const gazebo_msgs::LinkStatesConstPtr &link_states)
{
      /* std::cout << " HERE " << std::endl; */

  /* ros::Time temp_time = ros::Time::now(); */
  /* if ((temp_time-last_link_received_).toSec()>=T){ */
  if (true){
    /* last_link_received_ = temp_time; */

    const std::vector<std::string>& names = link_states->name;
    const std::vector<geometry_msgs::Pose>& poses = link_states->pose;

    for (size_t it = 0; it < names.size(); ++it) 
    {
      const std::string& cur_name = names.at(it);
      if (cur_name.find("_uvled_") == std::string::npos){
        continue;
      }

      const size_t prependix_pos = cur_name.find_first_of("::");
      const std::string cur_model_name = cur_name.substr(0, prependix_pos);
      const std::string cur_link_name = cur_name.substr(prependix_pos+2);


      /* std::cout << "pose of " << cur_name << ": " << std::endl; */
      /* std::cout << poses.at(it) << std::endl; */

      if (_leds_by_name_.count(cur_link_name) != 0){
        boost::mutex::scoped_lock lock(mtx_leds);
        _leds_by_name_.at(cur_link_name)->update_link_pose(cur_link_name, poses.at(it));
      }
      else{
        boost::mutex::scoped_lock lock(mtx_leds);
        std::shared_ptr<LedMgr> led = std::make_shared<LedMgr>(nh_, cur_name);
        _leds_by_name_.insert({cur_link_name, led});
        /* std::cout << "Subscribing to LED info of " << "/gazebo/ledProperties/"+cur_link_name << std::endl; */
        ledInfoSubscribers.push_back(nh_.subscribe("/gazebo/ledProperties/"+cur_link_name, 1, &UvCam::ledCallback,this));
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
    /* std::cout << "UV CAM: receiving frequency of " << led_info->frequency.data << " for link  " << link_name << std::endl; */
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

/* drawPose_virtual //{ */
  bool drawPose_virtual(std::pair<geometry_msgs::Pose,ignition::math::Pose3d> input_poses, cv::Point3d &output){

    /* if (!occlusion_initialized_) */
    /*   return false; */
    /* return false; */
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
    output.x = ledProj[1];
    output.y = ledProj[0];
    output.z = radius;
    if (ledIntensity > 0.1) {
      
    /* std::cout << "Here A" << std::endl; */
    /*   visual_current_ = world_scene_->GetVisual("tree_*"); */
    /* std::cout << "Here B" << std::endl; */
      /* if (visual_current_ != NULL){ */
        /* auto diff = ignition::math::Pose3d((ledPose.Pos()) - (pose.Pos()), ignition::math::Quaternion<double>(0, 0, 0)); */
        if (getObstacle_granular( pose, ledPose))
          return false;
      /* } */
      /* if (getObstacle( pose, ledPose)){ */
        /* std::cout << "Hitting obstacle " << std::endl; */
        /* return false; */
      /* } */
      else{
        /* std::cout << "Line of sight " << std::endl; */
        return true;
      }
    }
    else
      return false;
        /* cv::circle(cvimg.image, cv::Point2i(ledProj[1], ledProj[0]), radius, cv::Scalar(255), -1); */
  }
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
    

/* getObstacle //{ */
bool getObstacle(ignition::math::Pose3d camera, ignition::math::Pose3d led){
  /* std::chrono::steady_clock::time_point begin_t = std::chrono::steady_clock::now(); */

  if (!_use_occlusions_)
    return false;
  double led_distance = (camera.Pos() - led.Pos()).Length();

  /* std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now(); */
  curr_ray->SetPoints(camera.Pos(),led.Pos());
  /* std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now(); */
  /* std::chrono::duration<double> elapsed_1 = end - begin; */

  std::string intersection_entity;
  double intersection_distance;

  /* begin = std::chrono::steady_clock::now(); */
  curr_ray->GetIntersection(intersection_distance, intersection_entity);
  /* end = std::chrono::steady_clock::now(); */
  /* std::chrono::duration<double> elapsed_2 = end - begin; */

  bool hitting_obstacle;
  hitting_obstacle =  ((intersection_distance*1.01) < led_distance);
  hitting_obstacle &= (intersection_distance > 0.0001);
  hitting_obstacle &= (intersection_entity.find("inertia_collision")==std::string::npos);

  /* if (! (intersection_entity.find("inertia_collision")==std::string::npos)) */
  /*     std::cout << "Hitting " << intersection_entity << " at " << intersection_distance << " m away." << std::endl; */
  /* if (hitting_obstacle) */
    /* std::cout << "Hitting " << intersection_entity << " at " << intersection_distance << " m away." << std::endl; */
  /* std::cout << "LED should be " << led_distance << " m away." << std::endl; */



  /* std::chrono::steady_clock::time_point finish_t = std::chrono::steady_clock::now(); */
  /* std::chrono::duration<double> elapsed_t = finish_t - begin_t; */

  /* std::cout << "Elapsed time: setup: " << elapsed_1.count() << " s\n"; */
  /* std::cout << "Elapsed time: intersection: " << elapsed_2.count() << " s\n"; */
  /* std::cout << "Elapsed time: total: " << elapsed_t.count() << " s\n"; */

  return (hitting_obstacle) ;
}
//}


/* getObstacle_granular //{ */
bool getObstacle_granular(ignition::math::Pose3d camera, ignition::math::Pose3d led)
{
  if (!_use_occlusions_)
    return false;

  if (pengine->GetType() != "ode"){
    std::cerr << "[UVDAR camera]: Fast occulsions are only implemented for ODE physics engine. Returning." << std::endl;
    return false;
  }


  double led_distance = (camera.Pos() - led.Pos()).Length();
  /* std::cout << "Cam. pose: " << camera.Pos() << "; Diff. vector: " << diff << std::endl; */

  boost::recursive_mutex::scoped_lock lock(*(pengine->GetPhysicsUpdateMutex()));

      // Do collision detection
      /* dSpaceCollide2(this->geomId, */
          /* (dGeomID)(this->physicsEngine->GetSpaceId()), */
          /* &intersection, &UpdateCallback); */
    /* } */

  curr_ray->SetPoints(camera.Pos(),led.Pos());
  /* std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now(); */

  auto geom = (dGeomID)((boost::static_pointer_cast<gazebo::physics::ODEPhysics>(pengine))->GetSpaceId());
  if (!geom){
    std::cerr << "[UVDAR camera]: Could not access space for occlusion retrieval. Returning." << std::endl;
    return false;
  }

  /* lock_count++; */

  /* ((dxSpace*)geom)->cleanGeoms(); */

  auto ode_ray = boost::static_pointer_cast<gazebo::physics::ODERayShape>(curr_ray);
  auto ray_geomid = (dxSpace*)(ode_ray)->ODEGeomId();
  auto first = (ray_geomid)->first;

	/* if (IS_SPACE(geom)) */ 
    /* std::cout << "[UVDAR camera]: WORLD is space." << std::endl; */
  /* else */
    /* std::cout << "[UVDAR camera]: WORLD is NOT space." << std::endl; */

	/* if (IS_SPACE(ray_geomid)) */ 
    /* std::cout << "[UVDAR camera]: RAY is space." << std::endl; */
  /* else */
    /* std::cout << "[UVDAR camera]: RAY is NOT space." << std::endl; */

  ODERayHack::recomputeAABB(geom);

  ODERayHack::Intersection intersection;
  intersection.depth = 1000;

  // intersect bounding boxes
  for (dxGeom *g=first; g; g=g->next) {
    /* if (GEOM_ENABLED(g)){ */
/* ode_ray->UpdateCallback */

    dNearCallback* dummy;
    ODERayHack::collideAABBs (g,geom,&intersection,dummy);
    /* } */
  }

  std::string intersection_entity = intersection.name;
  double intersection_distance = intersection.depth;


  bool hitting_obstacle;
  hitting_obstacle =  ((intersection_distance*1.01) < led_distance);
  hitting_obstacle &= (intersection_distance > 0.0001);
  hitting_obstacle &= (intersection_entity.find("inertia_collision")==std::string::npos);


  return (hitting_obstacle) ;
}


//}
/* meshVisuals //{ */
void meshVisuals(const rendering::VisualPtr _visual,
    std::vector<rendering::VisualPtr> &_visuals) const {
  std::size_t found = _visual->Name().find("trunk");
  if (found!=std::string::npos)
    if (!_visual->GetMeshName().empty() &&
        (_visual->GetVisibilityFlags() & GZ_VISIBILITY_SELECTABLE)){
      /* std::cout << "Mesh name: " <<  _visual->GetMeshName() << std::endl; */
      _visuals.push_back(_visual);
  }

  for (unsigned int i = 0; i < _visual->GetChildCount(); ++i)
    meshVisuals(_visual->GetChild(i), _visuals);
}
//}

};

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(UvCam)
}
