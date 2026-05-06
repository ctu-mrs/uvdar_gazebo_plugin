#include <functional>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh> 
#include <gz/sim/components/Name.hh>
#include <gz/common/Console.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Util.hh>   
#include <gz/sim/World.hh>
#include <gz/plugin/Loader.hh>
//#include <gz/sim/components/PhysicsEnginePlugin.hh> 
#include <gz/sim/components/RaycastData.hh>
#include <gz/sim/components/Physics.hh>   


// ROS 2 Headers
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <mutex>
#include <uvdar_gazebo_plugin/msg/led_info.hpp>
#include <uvdar_gazebo_plugin/msg/cam_info.hpp>
#include <uvdar_gazebo_plugin/msg/led_message.hpp>
#include <uvdar_core_msgs/msg/point2_d_with_float.hpp>
#include <uvdar_core_msgs/srv/set_led_message.hpp>
#include <uvdar_gazebo_plugin/msg/led_info.hpp>
#include <mrs_msgs/srv/set_int.hpp>
#include <mrs_msgs/srv/float64_srv.hpp>
#include <random>
#include <sstream>

namespace uvdar_gazebo_plugin {



  class OcclusionCheck: public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemUpdate,
  public gz::sim::ISystemPreUpdate

  {

    private:

      rclcpp::Node::SharedPtr nh;
      std::string device_id = "-1";
      bool use_occlusions = false;
      std::string entity_name = "-1";
      int link_entity;
      std::string link_name;
      std::string led_link_name = "led";
      bool active = true;
      uint16_t counter = 0;
      const uint16_t refresh_interval = 5000; 
      gz::sim::Entity rayEntity{gz::sim::kNullEntity};
      gz::sim::Entity world_entity;


      std::mutex mtx_leds, mtx_cameras;
      gz::math::Pose3d camPose; 

      // Publishers
      rclcpp::Publisher<uvdar_core_msgs::msg::Point2DWithFloat>::SharedPtr pub_visible_leds;
      rclcpp::Subscription<uvdar_gazebo_plugin::msg::LedInfo>::SharedPtr sub_visible_leds;


      gz::sim::Entity entity_id;
      std::map<std::string, gz::math::Pose3d> ledEntities;
      std::mutex pubMutex;

      uvdar_core_msgs::msg::Point2DWithFloat msg_visible_leds;

      void scan_for_LED_entities(std::string led_ns, gz::sim::EntityComponentManager &ecm){
        ecm.Each<gz::sim::components::Name>(
            [&](const gz::sim::Entity &e,
              const gz::sim::components::Name *n)-> bool

            {
            if (!n)
            return true;

            const std::string &name = n->Data();

            if (name.find(led_ns) == std::string::npos) 
            return true;

            if (name.find("joint") != std::string::npos) 
            return true;

            if (name.find("_" + this->entity_name + "_") != std::string::npos)
            return true;


            gzmsg << name << " and entity name " << this->entity_name << std::endl;
            gz::math::Pose3d led_pose = gz::sim::worldPose(e, ecm);
            this->ledEntities[name] = led_pose;

            return true;
            });
      }

    public:
      OcclusionCheck() = default;
      ~OcclusionCheck() override = default;

      // --------------------------------------------------------------------------
      // Configure: Replaces Load()
      // Called once when the plugin is loaded into the simulation
      // --------------------------------------------------------------------------
      void Configure(const gz::sim::Entity &_entity,
          const std::shared_ptr<const sdf::Element> &_sdf,
          gz::sim::EntityComponentManager &_ecm,
          gz::sim::EventManager &) override
      {
        this->entity_id = _entity;
        this->world_entity = gz::sim::worldEntity(_ecm);


        if (_sdf->HasElement("device_id")) {
          auto elem = _sdf->FindElement("device_id");
          if (elem) {
            this->device_id = elem->Get<std::string>();
          }
        }

        if (_sdf->HasElement("use_occlusion")) {
          auto elem = _sdf->FindElement("use_occlusion");
          if (elem) {
            this->use_occlusions = elem->Get<bool>();
          }
        }
        if (!this->use_occlusions){
          gzmsg << "[OcclusionCheck] Plugin disabled! For model=" << entity_name
            << " device_id=" << this->device_id
            << " link_name=" << this->link_name << std::endl;
          return;
        }

        
//        if (!rclcpp::ok()) {
//          rclcpp::init(0, nullptr);
//        }

        std::string entity_name = "unknown";
        auto nameComp = _ecm.Component<gz::sim::components::Name>(_entity);
        if (nameComp) {
          this->entity_name = nameComp->Data();
        }

        // Append device_id so multiple instances on the same model are unique
        std::string node_name = "uvdar_cam_" + entity_name + "_" + this->device_id;
        this->nh = rclcpp::Node::make_shared(node_name);

        gzmsg << "[OcclusionCheck] Plugin started! model=" << entity_name
          << " device_id=" << this->device_id
          << " link_name=" << this->link_name << std::endl;

        this->ledEntities.clear();
        this->scan_for_LED_entities(led_link_name, _ecm); 

        _ecm.CreateComponent(this->world_entity,
            gz::sim::components::PhysicsCollisionDetector("bullet"));

        this->rayEntity = _ecm.CreateEntity();
        _ecm.CreateComponent(this->rayEntity, gz::sim::components::RaycastData());
        _ecm.CreateComponent(this->rayEntity,
            gz::sim::components::Pose(gz::math::Pose3d::Zero));

        // We'll update the actual ray direction in PreUpdate once we
        // know the live poses. For now, push a placeholder ray.
//        auto *rayData = _ecm.Component<gz::sim::components::RaycastData>(this->rayEntity);
//        gz::sim::components::RayInfo ray;
//        ray.start = gz::math::Vector3d::Zero;
//        ray.end   = gz::math::Vector3d::Zero;
//        rayData->Data().rays.push_back(ray); 
        // In Configure() — start with 0 rays, resize dynamically in PreUpdate
        auto *rayData = _ecm.Component<gz::sim::components::RaycastData>(this->rayEntity);
        rayData->Data().rays.clear();  
        // --- 4. Publishers (same as before) ---
        //  this->pub_visible_leds = this->nh->create_publisher<uvdar_core_msgs::msg::Point2DWithFloat>(
        //      "/" + device_id + "/gazebo/uvcam/visble_leds",  10);

        //  this->sub_visible_leds = this->nh->create_subscription<uvdar_gazebo_plugin::msg::LedInfo>(
        //          "/gazebo/ledProperties", // TODO: THIS needs to be changed to the correct one 
        //          20,
        //          std::bind(&UvCam::ledInfoCallback, this, std::placeholders::_1)
        //          );
        //
        gzdbg << "[OcclusionCheck] Plugin configured successfully." << std::endl;

      }

      void PreUpdate(const gz::sim::UpdateInfo &/*_info*/,
          gz::sim::EntityComponentManager &_ecm) override
      {
        if(!this->use_occlusions){
          return;
        }

        auto *rayData = _ecm.Component<gz::sim::components::RaycastData>(this->rayEntity);
        auto &rays    = rayData->Data().rays;
        auto &results = rayData->Data().results;

        const size_t nLeds = this->ledEntities.size();

        // --- Resize ray buffer to match current LED count ---
        if (rays.size() != nLeds)
        {
          rays.resize(nLeds);
          // Results will be resized by Physics to match next step
        }

        // Write all rays — iterate map with index
        size_t i = 0;
        for (const auto &[name, ledPose] : this->ledEntities)
        {
          gz::math::Vector3d camPos = this->camPose.Pos();
          gz::math::Vector3d ledPos = ledPose.Pos();
          gz::math::Vector3d dir    = (ledPos - camPos).Normalized();

          rays[i].start = camPos + dir * 0.15; // offset to avoid self-hit
          rays[i].end   = ledPos;
          ++i;
        }

        _ecm.SetChanged(this->rayEntity,
            gz::sim::components::RaycastData::typeId,
            gz::sim::ComponentState::OneTimeChange);

        // Read results (1-step lag — only valid when sizes match)
        if (results.size() != nLeds) return;

        
        i = 0;
        for (const auto &[name, ledPose] : this->ledEntities)
        {
          gz::math::Vector3d camPos = this->camPose.Pos();
          gz::math::Vector3d ledPos = ledPose.Pos();

          double ledDist     = (ledPos - camPos).Length();
          double hitFraction = results[i].fraction;
          double hitDist     = hitFraction * ledDist;

          constexpr double kEpsilon = 0.05; // cm
          bool occluded = (!std::isnan(hitFraction) &&
              hitFraction > 1e-6 &&
              hitDist < (ledDist - kEpsilon));

          if (occluded)
          {
            gzmsg << "[OcclusionChecker] LED=" << name
              << " hitDist="  << hitDist
              << " ledDist="  << ledDist
              << " LEDPosition="  << ledPos
              << " hitPoint=" << results[i].point
              << " OCCLUDED\n";
          }

          ++i;
        }
      }

      void Update(const gz::sim::UpdateInfo &_info,
          gz::sim::EntityComponentManager &_ecm) override
      {
        if(!this->use_occlusions){
          return;
        }

        //TBD:
        //rclcpp::spin_some(this->nh);

        counter = (counter + 1) % refresh_interval; 
        if (counter == 0){
          this->ledEntities.clear();
          this->scan_for_LED_entities(led_link_name, _ecm); 
          gzdbg << "[OcclusionCheck]: Rescanning for new LED entities." << std::endl;
        }


        this->camPose = gz::sim::worldPose(this->entity_id, _ecm);


      }
    private:

      // likely remove it 
      void ledInfoCallback(
          const uvdar_gazebo_plugin::msg::LedInfo::SharedPtr msg)
      {
        std::scoped_lock lock(mtx_leds);

        gzmsg << "Sending LED info message..." << std::endl;
      }

  };
}
GZ_ADD_PLUGIN(uvdar_gazebo_plugin::OcclusionCheck, 
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemUpdate,
    gz::sim::ISystemPreUpdate)
