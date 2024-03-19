#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/package.h>
#include <ros/master.h>
#include <undistortFunctions/ocam_functions.h>
#include <functional>
#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
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
#include <uvdar_gazebo_plugin/CamInfo.h>
#include <uvdar_gazebo_plugin/LedMessage.h>
#include <sensor_msgs/PointCloud.h>
/* #define exprate 0.001 */

#define CAM_EXISTENCE_HEADER "/gazebo/uvcam_"
#define CAM_YIELD_TOPIC "/gazebo/uvcam_yield"

#define index2d(X, Y) (oc_model.width * (Y) + (X))

/* using namespace ignition; */
namespace gazebo
{

  struct CameraProps {
      std::string scoped_name;
      std::string parent_name;
      unsigned int sensor_id;
      std::string publish_topic;
      double f;
      bool occlusions;
  };

  struct CameraData {
      CameraProps props;
      physics::EntityPtr parent_entity;
      sensors::SensorPtr sensor;
      ignition::math::Pose3d pose;
      ros::Publisher virtual_points_publisher;
  };

class UvCam : public SensorPlugin {
private:
  /* variables //{ */
  /* std::mutex mtx_buffer; */
  std::mutex mtx_leds, mtx_cameras;
  /* std::vector<std::pair<ignition::math::Pose3d,ignition::math::Pose3d>> buffer; */
  int                       id;
  uchar                     background;
  transport::SubscriberPtr  poseSub;
  transport::SubscriberPtr  stateSub;
  ros::Subscriber           linkSub;

  sensors::SensorPtr local_sensor;

  ros::Subscriber           yield_sub;

  gazebo::physics::WorldPtr world;
  gazebo::physics::PhysicsEnginePtr pengine;
  physics::RayShapePtr curr_ray;
  boost::shared_ptr<ODERayHack::RayIntersectorHack> ray_int_hack;
  /* physics::EntityPtr        parent; */
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
  bool                             finishing_program = false;
  ros::NodeHandle                  nh_;
  ros::Subscriber ledInfoSubscriber;
  std::vector<ros::Subscriber> ledMessageSubscribers;
  std::vector<ros::Subscriber> ledModeSubscribers;

  std::string filename;
  std::string publish_topic;

  std::unordered_map<std::string, std::shared_ptr<LedMgr> > _leds_by_name_;

  std::vector<CameraData> cameras;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;

  CameraProps camera_props;

  /* ros::Time last_link_received_; */

  ros::CallbackQueue link_queue_;

  ros::Publisher yield_publisher;
  bool yielded_rendering = false;

  rendering::ScenePtr world_scene_;
  rendering::VisualPtr visual_current_;
  std::vector<rendering::VisualPtr> visuals_serialized;

  std::vector<std::vector<bool>> sequences_;
  /* bool occlusion_initialized_; */
  //}

public:

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  /* ~UvCam() destructor //{ */
  ~UvCam()
  {
    finishing_program = true;
    transfer_thread.join();
    link_callback_thread.join();
    ROS_DEBUG_STREAM_NAMED("UvCam", "Unloaded");
  }
    //}

/* Load //{ */
  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {

    /* occlusion_initialized_ = false; */

    gazebo::rendering::Events::createScene("default");


    std::cout << "Initializing UV camera" << std::endl;

    /* CameraPlugin::Load(_parent, _sdf); */
    // copying from CameraPlugin into GazeboRosCameraUtils
    // Store the pointer to the model
    local_sensor           = _parent;


    /* this->parentSensor->SetActive(false); */
    world      = physics::get_world("default");
    /* world->PrintEntityTree(); */
    pengine      = world->Physics();

    curr_ray = boost::dynamic_pointer_cast<physics::RayShape>(
        pengine->CreateShape("ray", physics::CollisionPtr()));
    std::string parentName = _parent->ParentName();
    /* parent                 = world->EntityByName(parentName); */
    std::cout << "Camera scoped name: " << _parent->ScopedName() << std::endl;
    ray_int_hack = boost::make_shared<ODERayHack::RayIntersectorHack>(pengine, _parent->ScopedName());

    if (_sdf->HasElement("calibration_file")) {
      filename = _sdf->GetElement("calibration_file")->Get< std::string >();
      filename.erase(remove( filename.begin(), filename.end(), '\"' ),filename.end());
      std::cout << "Calibration file is " << filename << std::endl;
    }
    else {
      std::cerr << "No calibration file provided. Exiting" << std::endl;
      return;
    }
    /* id = 1; */

    double                    f;
    bool                      _use_occlusions;

    if (_sdf->HasElement("framerate")) {
      f = _sdf->GetElement("framerate")->Get< double >();
      std::cout << "Camera framerate is " << f << "Hz" << std::endl;
    } else {
      std::cout << "Camera framerate defaulting to 60Hz." << std::endl;
      f = 60.0;  // camera framerat
    }

    if (_sdf->HasElement("occlusion")) {
      _use_occlusions = _sdf->GetElement("occlusion")->Get< bool >();
    }
    else
      _use_occlusions = false;


    if (_use_occlusions) {
      std::cout << "Occlusions in UV camera enabled!" << std::endl;
    }
    else {
      std::cout << "Occlusions in UV camera disabled!" << std::endl;
    }

    /* transport::NodePtr node(new transport::Node()); */
    nh_ = ros::NodeHandle("~");


    if (get_ocam_model(&oc_model, (char*)(filename.c_str())) < 0){
      std::cerr << "UVCAM " << " failed to open calibration file " << filename << std::endl;
      return;
    }

    /* parseSequenceFile("/home/viktor/mrs_workspace/src/uav_modules/ros_packages/uvdar_meta/uvdar_core/config/BlinkingSequence-8-3-3-2-8.txt"); */
    parseSequenceFile(ros::package::getPath("uvdar_core")+"/config/selected.txt");
    /* cvimg                  = cv_bridge::CvImage(std_msgs::Header(), "mono8", cv::Mat(oc_model.height, oc_model.width, CV_8UC1, cv::Scalar(0))); */
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&UvCam::OnUpdate, this));

    /* char poseTopicName[30]; */
    /* std::sprintf(poseTopicName, "~/uvleds/pose"); */

    if (_sdf->HasElement("camera_publish_topic")) {
      publish_topic = _sdf->GetElement("camera_publish_topic")->Get<std::string>();
    } else {
      publish_topic = parentName.substr(0, parentName.find(":")) + "/uvdar_bluefox/image_raw";
    }

    ledInfoSubscriber = nh_.subscribe("/gazebo/ledProperties", 20, &UvCam::ledInfoCallback,this);

    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<gazebo_msgs::LinkStates>(
          "/gazebo/link_states",
          1,
          boost::bind(&UvCam::linkCallback, this, _1),
          ros::VoidPtr(), &link_queue_);

    linkSub = nh_.subscribe(so);
    /* linkSub = nh.subscribe("/gazebo/link_states", 1, &UvCam::linkCallback, this); */

    camera_props = {.scoped_name=_parent->ScopedName(), .parent_name=_parent->ParentName(), .sensor_id=_parent->Id(), .publish_topic=publish_topic, .f=f, .occlusions=_use_occlusions};
    auto preferredCamera = findExistingEquivalentCamera(camera_props);
    if (preferredCamera) {
      std::cout << "UVCAM " << _parent->Name() << " will yield rendering to " << preferredCamera.value().scoped_name << "." << std::endl;
      if (!yieldRendering(camera_props)){
      std::cout << "UVCAM " << _parent->Name() << " failed to yield rendering!" << std::endl;
      }
    } else{
      /* nh_.setParam(CAM_EXISTENCE_HEADER+_parent->Name()+"/name", _parent->Name()); */
      nh_.setParam(CAM_EXISTENCE_HEADER+_parent->Name()+"/scoped_name", _parent->ScopedName());
      nh_.setParam(CAM_EXISTENCE_HEADER+_parent->Name()+"/publish_topic", publish_topic);
      nh_.setParam(CAM_EXISTENCE_HEADER+_parent->Name()+"/f", f);
      nh_.setParam(CAM_EXISTENCE_HEADER+_parent->Name()+"/occlusions", _use_occlusions);
      /* CameraProps camera_props = {.name=_parent->Name(), .scoped_name=_parent->ScopedName(), .publish_topic=publish_topic, .f=f, .occlusions=_use_occlusions}; */

      char *dummy = NULL;
      int   zero  = 0;
      ros::init(zero, &dummy, "uvdar_bluefox_emulator");
      addCamera(camera_props, true);

      yield_sub = nh_.subscribe(CAM_YIELD_TOPIC, 10, &UvCam::yieldCallback, this);

    }
  }
  //}

  /* Init //{ */
  void Init() {
    if (yielded_rendering){
      return;
    }

    /* for (int i    = 0; i++; i < 20) */
    /* ledState[i] = false; */


    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Starting transfer thread");
    transfer_thread            = std::thread(&UvCam::TransferThread, this);
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Starting callback thread");
    link_callback_thread   = std::thread(&UvCam::LinkQueueThread, this);



    /* node->Init(); */


  }
  //}

private:
  /* TransferThread //{ */
  void TransferThread() {
    /* clock_t begin, end; */
    /* clock_t begin_p, end_p; */
    /* double  elapsedTime; */
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: InitForThread");
    pengine->InitForThread();

    /* ros::Rate rt(f + ((double(rand()%100)/50.0)-1.0)); //to simulate the possible difference between the camera framerate and the blinking generator bit rate. This has to be done here, since LEDs on a single UAV are always synchronized w.r.t. each other. */
    ros::Rate rt(cameras.front().props.f);
    geometry_msgs::Pose cur_led_pose;
    cv::Point3d output;
    geometry_msgs::Point32 pt;
    sensor_msgs::PointCloud msg_ptcl;
    std::unordered_map<std::string, std::shared_ptr<LedMgr>> leds_by_name_local;
    /* std::pair<std::string, std::shared_ptr<LedMgr> > led; */
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: loop start");
    while (ros::ok() && !finishing_program){
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
      std::vector<CameraData> cameras_local;
      {
        std::scoped_lock lock(mtx_cameras);
        cameras_local = cameras;
      }

      for (auto &cam : cameras_local)
      {
        {
          std::scoped_lock lock(mtx_leds);
          leds_by_name_local = _leds_by_name_;
          /* led = ledr; */
        }
        msg_ptcl.header.stamp = ros::Time::now();
        msg_ptcl.points.clear();
        double sec_time = msg_ptcl.header.stamp.toSec();
        for (auto& led : leds_by_name_local){
          if (led.second->get_pose(cur_led_pose, sec_time)){
            if (drawPose_virtual({cur_led_pose,cam.pose}, output)){
              pt.x = output.x;
              pt.y = output.y;
              pt.z = output.z;
              msg_ptcl.points.push_back(pt);
            }
          }
        }
    /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: publishing"); */
        cam.virtual_points_publisher.publish(msg_ptcl);
        /* end         = std::clock(); */
        /* elapsedTime = double(end - begin) / CLOCKS_PER_SEC; */
        /* std::cout << "UV CAM: Drawing took : " << elapsedTime << " s" << std::endl; */
    /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: loop end"); */
      }
      rt.sleep();
    }
  }
  //}
  
  /* LinkQueueThread //{ */
  void LinkQueueThread() {

    double timeout;
    ros::Rate rt(1);
    {
      std::scoped_lock lock(mtx_cameras);
      timeout= 1.0/(cameras.front().props.f);
      rt = ros::Rate(cameras.front().props.f);
    }
    while (nh_.ok() && ros::ok() && !finishing_program)
    {
      link_queue_.callAvailable(ros::WallDuration(timeout));
      rt.sleep();
    }
  }
  //}

public:
/* OnUpdate //{ */
  void OnUpdate() {
    std::scoped_lock lock(mtx_cameras);

    /* ros::spinOnce(); */
    /* std::cout << "sending" << std::endl; */
    // Apply a small linear velocity to the model.
    for (auto &c : cameras){
      c.pose = c.sensor->Pose() + c.parent_entity->WorldPose();
    }

    
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
    /*   std::scoped_lock lock(mtx_occlusion); */
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
      const std::string& link_name = names.at(it);
      if (link_name.find("uvled_") == std::string::npos){
        continue;
      }

      /* const size_t prependix_pos = cur_name.find_first_of("::"); */
      /* const std::string cur_model_name = cur_name.substr(0, prependix_pos); */
      /* const std::string link_name = cur_name.substr(prependix_pos+2); */


      /* std::cout << "pose of " << cur_name << ": " << std::endl; */
      /* std::cout << poses.at(it) << std::endl; */

      if (!(_leds_by_name_.find(link_name) == _leds_by_name_.end())){
        std::scoped_lock lock(mtx_leds);
        /* std::cout << "Updating " << link_name << std::endl; */
        _leds_by_name_.at(link_name)->update_link_pose(link_name, poses.at(it));
      }
      else
      {
        std::scoped_lock lock(mtx_leds);
        std::shared_ptr<LedMgr> led = std::make_shared<LedMgr>(nh_, link_name);
        std::cout << "Adding " << link_name << std::endl;
        _leds_by_name_.insert({link_name, led});
        _leds_by_name_.at(link_name)->set_active(false);
      }
      

    }
  }

}
//}

/* void ledInfoCallback //{ */
void ledInfoCallback(const ros::MessageEvent<uvdar_gazebo_plugin::LedInfo const>& event){
  std::scoped_lock lock(mtx_leds);
  /* std::cout << "Getting message" << std::endl; */
  ros::M_string mhdr = event.getConnectionHeader();
  std::string topic = mhdr["topic"];
  const uvdar_gazebo_plugin::LedInfoConstPtr& led_info = event.getMessage();
  std::string link_name = led_info->link_name.data;
  std::string device_id = led_info->device_id.data;
  /* std::cout << "UV CAM: receiving frequency of " << led_info->frequency.data << " for link  " << link_name << std::endl; */

  if (_leds_by_name_.find(link_name) == _leds_by_name_.end()){
    std::shared_ptr<LedMgr> led = std::make_shared<LedMgr>(nh_, link_name);
    _leds_by_name_.insert({link_name,led});
  }

  if ((led_info->ID.data >= 0) && (led_info->ID.data < (int)(sequences_.size())))
    _leds_by_name_.at(link_name)->update_data(link_name, sequences_[led_info->ID.data],led_info->seq_bitrate.data, led_info->mes_bitrate.data);
  else
    std::cerr << "[UVDAR camera]: Invalid sequence ID: " << led_info->ID.data << std::endl;

  if (_leds_by_name_.at(link_name)->get_device_id() == ""){

    _leds_by_name_.at(link_name)->set_device_id(device_id);
  std::cout << "Subscribing to LED info of " << "/gazebo/ledMessage/"+device_id << std::endl;
    ledMessageSubscribers.push_back(nh_.subscribe("/gazebo/ledMessage/"+device_id, 1, &UvCam::ledMessageCallback,this));
    ledModeSubscribers.push_back(nh_.subscribe("/gazebo/ledMode/"+device_id, 1, &UvCam::ledModeCallback,this));

  }

  std::cout << "Setting the state of LED " << link_name << " to " << (led_info->active.data?"ON":"OFF") << std::endl;
  _leds_by_name_.at(link_name)->set_active(led_info->active.data);
}
//}

/* void ledMessageCallback //{ */
void ledMessageCallback(const ros::MessageEvent<uvdar_gazebo_plugin::LedMessage const>& event){
  std::scoped_lock lock(mtx_leds);
  /* std::cout << "Getting message" << std::endl; */
  ros::M_string mhdr = event.getConnectionHeader();
  std::string topic = mhdr["topic"];
  std::string link_name = topic.substr(std::string("/gazebo/ledMessage/").length());
  const uvdar_gazebo_plugin::LedMessageConstPtr& led_message = event.getMessage();
  /* std::cout << "UV CAM: receiving message of length " << led_message->data_frame.size() << std::endl; */

  if (led_message->data_frame.size() >0){
    std::vector<bool> data_frame;
    for (auto d : led_message->data_frame){
      data_frame.push_back(d>0);
    }
    /* std::cout << "UV CAM: will transmit with length of  " << data_frame.size() << std::endl; */
    _leds_by_name_.at(link_name)->update_message(data_frame);
  }
}
//}
//
/* void ledModeCallback //{ */
void ledModeCallback(const ros::MessageEvent<std_msgs::Int32 const>& event){
  std::scoped_lock lock(mtx_leds);
  /* std::cout << "Getting message" << std::endl; */
  ros::M_string mhdr = event.getConnectionHeader();
  std::string topic = mhdr["topic"];
  std::string link_name = topic.substr(std::string("/gazebo/ledMode/").length());
  auto led_mode = event.getMessage();
  /* std::cout << "UV CAM: receiving mode of " << led_mode->data << std::endl; */
  _leds_by_name_.at(link_name)->set_mode(led_mode->data);
}
//}

/* yieldCallback //{ */
void yieldCallback(const uvdar_gazebo_plugin::CamInfoConstPtr &cam_info)
{
  CameraProps cam_props;
  cam_props.scoped_name = cam_info->scoped_name.data;
  cam_props.parent_name = cam_info->parent_name.data;
  cam_props.sensor_id = cam_info->sensor_id.data;
  cam_props.publish_topic = cam_info->publish_topic.data;
  cam_props.f = cam_info->f.data;
  cam_props.occlusions = cam_info->occlusions.data;

  ros::Duration(1.0).sleep();
  addCamera(cam_props);
}
//}

bool addCamera(CameraProps cam_props, bool is_local=false)
{
  ros::Rate retry_rate(1.0);
  while (ros::ok()){
    sensors::SensorPtr sensor;
    if (!is_local){
      sensor = getSensor(cam_props.sensor_id);
    }
    else {
      sensor = local_sensor;
    }

    if (sensor != nullptr){
      physics::EntityPtr entity = world->EntityByName(sensor->ParentName());

      if (entity != nullptr){
        std::scoped_lock lock(mtx_cameras);
        cameras.push_back({
            .props=cam_props,
            .parent_entity = entity,
            .sensor = sensor,
            .pose=ignition::math::Pose3d(),
            .virtual_points_publisher = nh_.advertise<sensor_msgs::PointCloud>("/gazebo"+cam_props.publish_topic, 1)});
        std::cout << "Adding UV Camera " << cam_props.scoped_name << " data to topic: /gazebo" << cam_props.publish_topic << "\"" << std::endl;
        return true;
      }
      else 
      {
        std::cerr << "Could not find entity with name " << cam_props.scoped_name << ", ignoring UV camera " << cam_props.scoped_name << std::endl;
        /* return false; */
      }
    }
    else
    {
      std::cerr << "Could not find sensor with ID " << id << ", ignoring UV camera " << cam_props.scoped_name << std::endl;
      /* return false; */
    }
    std::cerr << "try again" << std::endl;
    if (!is_local){
      retry_rate.sleep();
    }
    else {
      break;
    }
  }
  return false;
}

sensors::SensorPtr getSensor(unsigned int id)
{
  /* std::cout << "Model count: " << world->Models().size() << '\n'; */
  for (const auto& model : world->Models()) {
  /* std::cout << "Model: " << model->GetName() << '\n'; */
    /* std::cout << "Link count: " << model->GetLinks().size() << '\n'; */
    for (const auto &link : model->GetLinks()) {
    /* std::cout << "Link: " << link->GetName() << '\n'; */
      /* std::cout << "Sensor count: " << link->GetSensorCount() << '\n'; */
      for (unsigned int j = 0; j < link->GetSensorCount(); j++) {
        const sensors::SensorPtr candidate = sensors::get_sensor(link->GetSensorName(j));
        /* std::cout << "Considering sensor: " << candidate->Name() << '\n'; */
        if (candidate->Id() == id){
          /* std::cout << "Accepting sensor: " << candidate->Name() << '\n'; */
          return candidate;
        }
      }
    }
  }
  return nullptr;
}

std::optional<CameraProps> findExistingEquivalentCamera(CameraProps cam_props_current){

  std::vector<std::string> paramlist;
  if (nh_.getParamNames(paramlist)){
    for (auto p : paramlist){
      /* std::cout << "UVCAM sees param: " << p << std::endl; */
      std::size_t found = p.find(CAM_EXISTENCE_HEADER);
      if (found!=std::string::npos){
        std::string cam_name = p;
        cam_name = cam_name.erase(cam_name.rfind('/'));
        std::cout << "CAM found: " << cam_name << '\n';
        CameraProps cam_props_candidate;
        nh_.getParam(cam_name+"/scoped_name",cam_props_candidate.scoped_name);
        nh_.getParam(cam_name+"/publish_topic",cam_props_candidate.publish_topic);
        nh_.getParam(cam_name+"/f",cam_props_candidate.f);
        nh_.getParam(cam_name+"/occlusions",cam_props_candidate.occlusions);

        if ((cam_props_candidate.f == cam_props_current.f) && (cam_props_candidate.occlusions == cam_props_current.occlusions)){
          return std::optional<CameraProps>{cam_props_candidate};
        }
      }
    }
  }
  return std::nullopt;

}

bool yieldRendering(CameraProps cam_props){
  yield_publisher =  nh_.advertise<uvdar_gazebo_plugin::CamInfo>(CAM_YIELD_TOPIC, 10, true);
  uvdar_gazebo_plugin::CamInfo cam_info;
  cam_info.scoped_name.data = cam_props.scoped_name;
  cam_info.parent_name.data = cam_props.parent_name;
  cam_info.sensor_id.data = cam_props.sensor_id;
  cam_info.publish_topic.data = cam_props.publish_topic;
  cam_info.f.data = cam_props.f;
  cam_info.occlusions.data = cam_props.occlusions;
  yield_publisher.publish(cam_info);
  yielded_rendering = true;
  return true;
}

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

    ignition::math::Pose3d cam_pose = input_poses.second;
    invOrient = ledPose.Rot();
    invOrient.Invert();
    diffPose = (ledPose - cam_pose);
    input[0] = -(diffPose.Pos().Z());
    input[1] = -(diffPose.Pos().Y());
    input[2] = -(diffPose.Pos().X());

    /* std::cout << "input: " << std::endl; */
    /* std::cout << "Input: " << input_poses.first << std::endl; */
    /* std::cout << "converted: " << std::endl; */
    /* std::cout << "LED pose: " <<  ledPose << std::endl; */
    /* std::cout << "CAM pose: " << cam_pose << std::endl; */
    /* std::cout << "CAM res: " << oc_model.width << "x" << oc_model.height << std::endl; */
    world2cam(ledProj, input, &oc_model);
    a            = ignition::math::Pose3d(0, 0, 1, 0, 0, 0).RotatePositionAboutOrigin(invOrient);
    b            = ignition::math::Pose3d((cam_pose.Pos()) - (ledPose.Pos()), ignition::math::Quaternion<double>(0, 0, 0));


    distance     = b.Pos().Length();
    cosAngle     = a.Pos().Dot(b.Pos()) / (distance);
    ledIntensity = round(std::max(.0, cosAngle) * (coef[0] + (coef[1] / ((distance + coef[2]) * (distance + coef[2])))));

    radius = sqrt(ledIntensity / M_PI);

    /* std::cout << "C" << std::endl; */

    /* count++; */
    output.x = ledProj[1];
    output.y = ledProj[0];
    output.z = radius;
    /* std::cout << "Output: " << output << std::endl; */
    if (ledIntensity > 0.1) {
      
    /* std::cout << "Here A" << std::endl; */
    /*   visual_current_ = world_scene_->GetVisual("tree_*"); */
    /* std::cout << "Here B" << std::endl; */
      /* if (visual_current_ != NULL){ */
        /* auto diff = ignition::math::Pose3d((ledPose.Pos()) - (cam_pose.Pos()), ignition::math::Quaternion<double>(0, 0, 0)); */
        if (getObstacle_granular( cam_pose, ledPose))
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

  if (!cameras.front().props.occlusions)
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
  if (!cameras.front().props.occlusions)
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

  /* std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now(); */

  ODERayHack::Intersection intersection;
  ray_int_hack->getIntersection(camera, led, intersection);

  std::string intersection_entity = intersection.name;
  double intersection_distance = intersection.depth;


  bool hitting_obstacle;
  hitting_obstacle =  ((intersection_distance*1.01) < led_distance);

  /* std::cout << "Hitting " << intersection_entity << " at " << intersection_distance << " m away." << std::endl; */
  /* hitting_obstacle &= (intersection_distance > 0.0001); */
  /* hitting_obstacle &= (intersection_entity.find("inertia_collision")==std::string::npos); */


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

  /**
   * @brief Loads the file with lines describing useful blinking singals
   *
   * @param sequence_file The input file name
   *
   * @return Success status
   */
  /* parseSequenceFile //{ */
  bool parseSequenceFile(std::string sequence_file){
    std::cout << "[UVDAR camera]: Add sanitation - sequences must be of equal, non-zero length" << std::endl;
    std::cout << "[UVDAR camera]: Loading sequence from file: [ " + sequence_file + " ]" << std::endl;
    std::ifstream ifs;
    ifs.open(sequence_file);
    std::string word;
    std::string line;

    std::vector<std::vector<bool>> sequences;
    if (ifs.good()) {
      std::cout << "[UVDAR camera]: Loaded Sequences: [: " << std::endl;
      while (getline( ifs, line )){
        if (line[0] == '#'){
          continue;
        }
        std::string show_string = "";
        std::vector<bool> sequence;
        std::stringstream iss(line); 
        std::string token;
        while(std::getline(iss, token, ',')) {
          sequence.push_back(token=="1");
          if (sequence.back()){
            show_string += "1,";
          }
          else {
            show_string += "0,";
          }
        }
        sequences.push_back(sequence);
        std::cout << "[UVDAR camera]:   [" << show_string << "]" << std::endl;
      }
      std::cout << "[UVDAR camera]: ]" << std::endl;
      ifs.close();

      sequences_ = sequences;
    }
    else {
      std::cout << "[UVDAR camera]: Failed to load sequence file " << sequence_file << "! Returning." << std::endl;
      ifs.close();
      return false;
    }
    return true;
  }
  //}
};

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(UvCam)
}
