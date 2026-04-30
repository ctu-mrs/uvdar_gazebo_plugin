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


class UvCam: public gz::sim::System,
              public gz::sim::ISystemConfigure,
              public gz::sim::ISystemUpdate
{

private:

  rclcpp::Node::SharedPtr nh;
  std::string device_id;
  int link_entity;
  std::string link_name;
  bool active = true;

  std::mutex mtx_leds, mtx_cameras;

  // Publishers
  rclcpp::Publisher<uvdar_core_msgs::msg::Point2DWithFloat>::SharedPtr pub_visible_leds;
  rclcpp::Subscription<uvdar_gazebo_plugin::msg::LedInfo>::SharedPtr sub_visible_leds;

  // Services

  gz::sim::Entity entity_id;
  std::vector<gz::sim::Entity> ledEntities;
  std::mutex pubMutex;

  uvdar_core_msgs::msg::Point2DWithFloat msg_visible_leds;

public:
  UvCam() = default;
  ~UvCam() override = default;

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

  // --- 1. Read all SDF parameters FIRST ---
  if (_sdf->HasElement("device_id")) {
    auto elem = _sdf->FindElement("device_id");
    if (elem) {
      this->device_id = elem->Get<std::string>();
  }else{
    this->device_id = -1;
  }
  }

  // --- 3. Create a uniquely named ROS node ---
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  std::string entity_name = "unknown";
  auto nameComp = _ecm.Component<gz::sim::components::Name>(_entity);
  if (nameComp) {
    entity_name = nameComp->Data();
  }
  gzmsg << "[UvCam] Entity Name"  << entity_name << std::endl;

  // Append device_id so multiple instances on the same model are unique
  std::string node_name = "uvdar_led_" + entity_name + "_" + this->device_id;
  this->nh = rclcpp::Node::make_shared(node_name);

  gzmsg << "[UvCam] Plugin started! model=" << entity_name
        << " device_id=" << this->device_id
        << " link_name=" << this->link_name << std::endl;

  _ecm.Each<gz::sim::components::Name>(
          [&](const gz::sim::Entity &e,
              const gz::sim::components::Name *n) -> bool
          {
          if (n->Data().find("led") != std::string::npos)
          this->ledEntities.push_back(e);
          return true;
          });
  
  // --- 4. Publishers (same as before) ---
  this->pub_visible_leds = this->nh->create_publisher<uvdar_core_msgs::msg::Point2DWithFloat>(
      "/" + device_id + "/gazebo/uvcam/visble_leds",  10);

  this->sub_visible_leds = this->nh->create_subscription<uvdar_gazebo_plugin::msg::LedInfo>(
          "/gazebo/ledProperties", // TODO: THIS needs to be changed to the correct one 
          20,
          std::bind(&UvCam::ledInfoCallback, this, std::placeholders::_1)
          );

  gzdbg << "UV Cam Plugin configured successfully." << std::endl;

}

void Update(const gz::sim::UpdateInfo &_info,
            gz::sim::EntityComponentManager &_ecm) override
{
  rclcpp::spin_some(this->nh);

    for (auto e : this->ledEntities)
    {
      auto pose = _ecm.Component<gz::sim::components::WorldPose>(e);
      if (!pose) continue;
    
      gzdbg << pose->Data().Pos() << std::endl;
    
    }

  }
private:

    void ledInfoCallback(
            const uvdar_gazebo_plugin::msg::LedInfo::SharedPtr msg)
    {
        std::scoped_lock lock(mtx_leds);
    
        gzdbg << "Sending LED info message..." << std::endl;
    }

};
}
GZ_ADD_PLUGIN(uvdar_gazebo_plugin::UvCam, 
        gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemUpdate)
