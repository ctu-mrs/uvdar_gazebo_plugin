#include <functional>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh> // For Gazebo Transport (optional but recommended)
#include <gz/sim/components/Name.hh>
#include <gz/common/Console.hh>
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
#include <uvdar_core_msgs/srv/set_led_message.hpp>
#include <mrs_msgs/srv/set_int.hpp>
#include <mrs_msgs/srv/float64_srv.hpp>
#include <random>
#include <sstream>

namespace uvdar_gazebo_plugin {

// Helper class for UUID generation to keep it local
class UuidGenerator {
public:
    static std::string generate_v4() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_int_distribution<> dis(0, 15);
        static std::uniform_int_distribution<> dis2(8, 11);

        std::stringstream ss;
        ss << std::hex;
        
        // 8 chars
        for (int i = 0; i < 8; i++) ss << dis(gen);
        ss << "_";
        // 4 chars
        for (int i = 0; i < 4; i++) ss << dis(gen);
        ss << "_4";
        // 3 chars
        for (int i = 0; i < 3; i++) ss << dis(gen);
        ss << "_";
        ss << dis2(gen);
        // 3 chars
        for (int i = 0; i < 3; i++) ss << dis(gen);
        ss << "_";
        // 12 chars
        for (int i = 0; i < 12; i++) ss << dis(gen);
        
        return ss.str();
    }
};


class UvLed : public gz::sim::System,
              public gz::sim::ISystemConfigure,
              public gz::sim::ISystemUpdate
{
//class UvLed : public SensorPlugin {
private:

  rclcpp::Node::SharedPtr nh;
  std::string device_id;
  int link_entity;
  std::string link_name;
  double fs = 60.0; // Default frequency
  double fm = 60.0;
  int mode = 0;
  int id = -1;
  bool active = true;
  std::string unique_ID;

  // Publishers
  rclcpp::Publisher<uvdar_gazebo_plugin::msg::LedInfo>::SharedPtr led_info_pub;
  rclcpp::Publisher<uvdar_gazebo_plugin::msg::LedMessage>::SharedPtr led_message_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr led_mode_pub;

  // Services
  rclcpp::Service<mrs_msgs::srv::SetInt>::SharedPtr srv_mode_setter;
  rclcpp::Service<mrs_msgs::srv::Float64Srv>::SharedPtr srv_freq_setter;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_active_setter;
  rclcpp::Service<uvdar_core_msgs::srv::SetLedMessage>::SharedPtr srv_msg_sender;
  rclcpp::Service<mrs_msgs::srv::SetInt>::SharedPtr srv_seq_setter; // Assuming SetInt for sequence


  gz::sim::Entity entity_id;
  std::mutex pubMutex;

  // Message objects (reused to avoid allocation if needed, though shared_ptr is preferred)
  uvdar_gazebo_plugin::msg::LedInfo led_info_msg;
  uvdar_gazebo_plugin::msg::LedMessage led_msg_msg;


public:
  UvLed() = default;
  ~UvLed() override = default;

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
    }
  }
  if (this->device_id.empty()) {
    this->device_id = UuidGenerator::generate_v4();
  }

 std::cerr << "I am herei 2" << this->device_id<< std::endl;
  if (_sdf->HasElement("signal_id")) {
    auto elem = _sdf->FindElement("signal_id");
    if (elem) {
      this->id = elem->Get<int>();
    }
  } else {
    this->id = -1;
  }

  if (_sdf->HasElement("frequency")) {
    auto elem = _sdf->FindElement("frequency");
    if (elem) {
      this->fs = elem->Get<double>();
      this->fm = this->fs;
    }
  } else {
    this->fs = 60.0;
    this->fm = 60.0;
  }

  // Read the link name explicitly from SDF
  if (_sdf->HasElement("link_name")) {
    auto elem = _sdf->FindElement("link_name");
    if (elem) {
      this->link_name = elem->Get<std::string>();
    }
  }
 std::cerr << "I am herei 3" << this->link_name<< std::endl;
  // --- 2. Validate model and resolve the target link entity ---
  auto model = gz::sim::Model(_entity);
  if (!model.Valid(_ecm)) {
    gzerr << "[UvLed] Plugin must be attached to a model!" << std::endl;
    return;
  }

  this->link_entity = model.LinkByName(_ecm, this->link_name);
  if (this->link_entity == gz::sim::kNullEntity) {
    gzerr << "[UvLed] Specified link not found: " << this->link_name << std::endl;
    return;
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

  // Append device_id so multiple instances on the same model are unique
  std::string node_name = "uvdar_led_" + entity_name + "_" + this->device_id;
  this->nh = rclcpp::Node::make_shared(node_name);

  gzmsg << "[UvLed] Plugin started! model=" << entity_name
        << " device_id=" << this->device_id
        << " link_name=" << this->link_name << std::endl;

  // --- 4. Publishers (same as before) ---
  this->led_info_pub = this->nh->create_publisher<uvdar_gazebo_plugin::msg::LedInfo>(
      "/gazebo/ledProperties" + device_id, 10);

  this->led_message_pub = this->nh->create_publisher<uvdar_gazebo_plugin::msg::LedMessage>(
      "/gazebo/ledMessage/" + device_id, 10);

  this->led_mode_pub = this->nh->create_publisher<std_msgs::msg::Int32>(
      "/gazebo/ledMode/" + device_id, 10);

  // --- 5. Services (same as before) ---
  this->srv_mode_setter = this->nh->create_service<mrs_msgs::srv::SetInt>(
      "/gazebo/ledModeSetter/" + device_id,
      std::bind(&UvLed::callbackSetMode, this, std::placeholders::_1, std::placeholders::_2));

  this->srv_freq_setter = this->nh->create_service<mrs_msgs::srv::Float64Srv>(
      "/gazebo/ledFrequencySetter/" + device_id,
      std::bind(&UvLed::callbackSetFrequency, this, std::placeholders::_1, std::placeholders::_2));

  this->srv_active_setter = this->nh->create_service<std_srvs::srv::SetBool>(
      "/gazebo/ledActiveSetter/" + device_id,
      std::bind(&UvLed::callbackSetActive, this, std::placeholders::_1, std::placeholders::_2));

  this->srv_msg_sender = this->nh->create_service<uvdar_core_msgs::srv::SetLedMessage>(
      "/gazebo/ledMessageSender/" + device_id,
      std::bind(&UvLed::callbackSendMessage, this, std::placeholders::_1, std::placeholders::_2));

  this->srv_seq_setter = this->nh->create_service<mrs_msgs::srv::SetInt>(
      "/gazebo/ledSignalSetter/" + device_id,
      std::bind(&UvLed::callbackSetSequence, this, std::placeholders::_1, std::placeholders::_2));

  gzdbg << "UV LED Plugin configured successfully." << std::endl;

  publishData();
}

  // --------------------------------------------------------------------------
  // Update: Replaces OnUpdate()
  // Called every simulation step
  // --------------------------------------------------------------------------
  void Update(const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override
  {
    // Optional: Spin the node to process incoming service requests immediately
    // Note: In a real high-frequency loop, you might want to spin less often or use a timer
    rclcpp::spin_some(this->nh);

    // Logic to publish data based on frequency could go here
    // For now, we rely on the service callbacks or external triggers to call publishData()
    // Or you can implement a time-based check:
    /*
    double currentTime = _info.simTime.Double();
    if (currentTime - lastPublishTime >= 1.0 / fs) {
        publishData();
        lastPublishTime = currentTime;
    }
    */
  }
private:
  void publishData() {
    // Lock mutex if accessing shared data from multiple threads (though ROS 2 callbacks are usually single-threaded per executor)
    std::lock_guard<std::mutex> lock(pubMutex);

    led_info_msg.seq_bitrate.data = fs;
    led_info_msg.mes_bitrate.data = fm;
    led_info_msg.id.data = id;
    led_info_msg.active.data = active;
    led_info_msg.mode.data = mode;
    led_info_msg.device_id.data = device_id;
    led_info_msg.link_name.data = link_name;

    gzdbg << "Sending LED info message..." << std::endl;
    this->led_info_pub->publish(led_info_msg);
  }

  // Service Callbacks
  bool callbackSetFrequency(
      const std::shared_ptr<mrs_msgs::srv::Float64Srv::Request> req,
      std::shared_ptr<mrs_msgs::srv::Float64Srv::Response> res)
  {
    if (mode == 0){
      fs = req->value;
      publishData();
      res->message = "Setting the sequence bitrate to " + std::to_string(fs);
    }
    else if (mode == 1){
      fm = req->value;
      publishData();
      res->message = "Setting the message bitrate to " + std::to_string(fm);
    }
    RCLCPP_INFO(this->nh->get_logger(), "%s", res->message.c_str());
//    gzinfo << res->message << std::endl;
    res->success = true;
    return true;
  }

  bool callbackSetSequence(
      const std::shared_ptr<mrs_msgs::srv::SetInt::Request> req,
      std::shared_ptr<mrs_msgs::srv::SetInt::Response> res)
  {
    id= req->value;
    publishData();
    res->message = "Setting the signal ID to " + std::to_string(id);
    RCLCPP_INFO(this->nh->get_logger(), "%s", res->message.c_str());
    //gzinfo << res->message << std::endl;
    res->success = true;
    return true;
  }

  bool callbackSetMode(
      const std::shared_ptr<mrs_msgs::srv::SetInt::Request> req,
      std::shared_ptr<mrs_msgs::srv::SetInt::Response> res)
  {
    mode = req->value;
    publishData();
    res->message = "Setting the mode to " + std::to_string(mode);
    //gzinfo << res->message << std::endl;
    RCLCPP_INFO(this->nh->get_logger(), "%s", res->message.c_str());
    res->success = true;
    return true;
  }

  bool callbackSendMessage(
      const std::shared_ptr<uvdar_core_msgs::srv::SetLedMessage::Request> req,
      std::shared_ptr<uvdar_core_msgs::srv::SetLedMessage::Response> res)
  {
      if (mode == 1){
          res->message = "Sending message";
          led_msg_msg.link_name.data = link_name;
          led_msg_msg.data_frame = req->data_frame;

          this->led_message_pub->publish(led_msg_msg);
          res->success = true;
          return true;
      }
      else {
          res->message = "Will not send message - the appropriate mode is not set!";
//          gzwarn << res->message << std::endl;
          RCLCPP_WARN(this->nh->get_logger(), "%s", res->message.c_str());
          res->success = false;
          return true;
      }
  }

  bool callbackSetActive(
          const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
          std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
      active = req->data;
      if (active)
          res->message = "Activating LED";
      else
          res->message = "Deactivating LED";

      publishData();
      res->success = true;
      return true;
  }
};
}
GZ_ADD_PLUGIN(uvdar_gazebo_plugin::UvLed, 
        gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemUpdate)
