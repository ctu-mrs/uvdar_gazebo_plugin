#include <functional>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/sim/components/Name.hh>
#include <gz/common/Console.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/RaycastData.hh>
#include <gz/sim/components/Physics.hh>

// ROS 2 Headers
#include "rclcpp/rclcpp.hpp"
#include <uvdar_gazebo_plugin/msg/led_info.hpp>
#include <uvdar_gazebo_plugin/msg/cam_info.hpp>
#include <uvdar_gazebo_plugin/msg/led_message.hpp>
#include <uvdar_gazebo_plugin/components/led_blink.hpp>
#include <uvdar_core/msg/image_points_with_covariances_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <opencv2/imgproc.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <undistortFunctions/ocam_functions.h>

#include <map>
#include <mutex>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <chrono>

namespace uvdar_gazebo_plugin {

  // Casts one ray per known LED to the physics engine each step to determine
  // occlusion, projects visible LEDs through the OCamCalib omnidirectional
  // model, and publishes them directly as detector output
  class OcclusionCheck: public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemUpdate,
  public gz::sim::ISystemPreUpdate

  {

    private:

      rclcpp::Node::SharedPtr nh;
      std::string device_id = "-1";
      bool use_occlusions = false;
      std::string entity_name = "unknown";
      std::string led_link_name = "led";
      uint16_t counter = 0;
      const uint16_t refresh_interval = 5000;

      // AMI's tracker (uvdar_core) buffers one detection message per blink
      // bit and reads back the trailing N messages as the N-bit sequence
      // (see uvdar_core's sequence_buffer.hpp trailingSamples) -- it does not
      // resample by timestamp. Update() runs every physics step (hundreds of
      // Hz), so without throttling here, "N trailing messages" covers only a
      // fraction of a real bit period and the tracker never sees a coherent
      // sequence. Publishing must therefore be rate-limited to match the
      // LEDs' blink bitrate (uvled.cc's default `fs`, 60Hz) so one message
      // really does correspond to one bit.
      double points_rate = 60.0;
      // Index of the last published frame on the absolute sim-time grid; see
      // the rationale in Update().
      long long last_frame_index = -1;
      bool has_published_once = false;

      gz::sim::Entity rayEntity{gz::sim::kNullEntity};
      gz::sim::Entity world_entity{gz::sim::kNullEntity};
      // The model this plugin instance is attached to (see Configure).
      gz::sim::Entity entity_id{gz::sim::kNullEntity};
      // This camera's own link within that model -- the actual pose source.
      gz::sim::Entity cam_entity{gz::sim::kNullEntity};

      gz::math::Pose3d camPose;
      std::map<std::string, gz::sim::Entity> ledEntities;

      struct ocam_model oc_model;
      bool calibration_loaded = false;

      // LED apparent-intensity model: intensity = cosAngle * (c0 + c1/(d+c2)^2)
      // Coefficients are ROS1's (uvcam.cc:102), fitted to the real bluefox +
      // UV LED. Intensity is in *lit pixels*: the drawn blob has area
      // pi*r^2 == intensity, so it falls off as 1/d^2 and is only ~3 px at 5 m.
      double coef[3] = {1.3398, 31.4704, 0.0154};
      double min_intensity = 0.1;
      // Multiplies the modelled intensity, i.e. blob AREA. 1.0 reproduces ROS1
      // exactly; raise it to emulate a brighter LED / longer exposure (a real
      // saturated LED blooms across more pixels than the ideal point-source
      // model predicts). Radius scales as sqrt(gain).
      double led_gain = 1.0;
      // Only used when covariance_model == "constant".
      double pixel_covariance = 1.0;
      // "blob"     -- derive the covariance from the LED's apparent blob, so the
      //               direct points path reports what uvdar_core's detector
      //               would have measured off the emulated image. Default.
      // "constant" -- legacy flat isotropic pixel_covariance, identical for
      //               every LED at every range.
      std::string covariance_model = "blob";

      rclcpp::Publisher<uvdar_core::msg::ImagePointsWithCovariancesStamped>::SharedPtr pub_visible_leds;
      std::mutex pubMutex;

      // Optional "bluefox emulator" path: render the projected LEDs into a
      // synthetic mono8 image so uvdar_core's real detector_node can be
      // exercised, instead of feeding it detector-format points directly.
      bool publish_image = false;
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image;

      std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster;
      bool tf_published = false;
      // Parent of the published frames. Defaults to "<model>/fcu", the MRS body
      // frame, which is rigid with the SDF root link the cameras hang off.
      std::string parent_frame;
      // Grey level the emulated frame is filled with before LEDs are drawn
      int background_setting = 30;
      uint8_t background_level = 30;

      // Resolves this instance's camera link ("uvcam_<device_id>_link", per
      // cam.sdf.jinja) inside the model the plugin is attached to.
      bool resolveCamEntity(gz::sim::EntityComponentManager &ecm) {
        if (this->cam_entity != gz::sim::kNullEntity) {
          return true;
        }
        const std::string link_name = "uvcam_" + this->device_id + "_link";
        this->cam_entity = gz::sim::Model(this->entity_id).LinkByName(ecm, link_name);
        if (this->cam_entity == gz::sim::kNullEntity) {
          return false;
        }
        gzmsg << "[OcclusionCheck] Resolved camera link '" << link_name
              << "' for device_id=" << this->device_id << std::endl;
        return true;
      }

      // Covariance of the LED's image position, matching what uvdar_core's FIMD
      // detector would report for the same blob, so switching
      // UVDAR_SIM_PUBLISH_IMAGE does not silently change the noise model.
      //
      void blobCovariance(double cx, double cy, double radius,
          double &c00, double &c01, double &c11) const {
        constexpr double kPixelDiscretizationVariance = 1.0 / 12.0;

        const int r = static_cast<int>(std::lround(radius));
        const int cxi = static_cast<int>(std::lround(cx));
        const int cyi = static_cast<int>(std::lround(cy));

        // Welford's online algorithm, matching the detector's accumulator
        // (postprocess.hpp) 
        double n = 0.0;
        double mean_x = 0.0, mean_y = 0.0;
        double m2xx = 0.0, m2xy = 0.0, m2yy = 0.0;
        if (r > 0) {
          const long long r2 = static_cast<long long>(r) * r;
          for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
              if (static_cast<long long>(dx) * dx + static_cast<long long>(dy) * dy > r2) {
                continue;
              }
              const int px = cxi + dx;
              const int py = cyi + dy;
              // cv::circle clips, so pixels outside the sensor are never lit
              // and must not contribute 
              if (px < 0 || px >= oc_model.width || py < 0 || py >= oc_model.height) {
                continue;
              }
              const double x = static_cast<double>(px);
              const double y = static_cast<double>(py);

              n += 1.0;
              // Deviation from the mean BEFORE this sample is folded in...
              const double dx_old = x - mean_x;
              const double dy_old = y - mean_y;
              mean_x += dx_old / n;
              mean_y += dy_old / n;
              // ...multiplied by the deviation from the mean AFTER. The two
              // differ by exactly the amount the mean just moved, and that
              // product is what makes the running sum come out exact.
              m2xx += dx_old * (x - mean_x);
              m2yy += dy_old * (y - mean_y);
              m2xy += dx_old * (y - mean_y);
            }
          }
        }

        if (n <= 1.0) {
          // Single lit pixel: no measurable spread, so the grid floor is the
          // entire uncertainty. Same branch the detector takes for count == 1.
          c00 = kPixelDiscretizationVariance;
          c01 = 0.0;
          c11 = kPixelDiscretizationVariance;
          return;
        }

        const double inv = 1.0 / (n - 1.0);   // detector uses sum_weights - 1, weight 1/px
        c00 = m2xx * inv + kPixelDiscretizationVariance;
        c01 = m2xy * inv;
        c11 = m2yy * inv + kPixelDiscretizationVariance;
      }

      void publish_camera_tf(gz::sim::EntityComponentManager &ecm) {
        if (this->tf_published || !this->tf_broadcaster
            || this->cam_entity == gz::sim::kNullEntity) {
          return;
        }

        const gz::math::Pose3d model_world = gz::sim::worldPose(this->entity_id, ecm);
        const gz::math::Pose3d cam_world = gz::sim::worldPose(this->cam_entity, ecm);
        const gz::math::Pose3d cam_rel = model_world.Inverse() * cam_world;

        // Rz(-90) * Rx(-90): body (X fwd, Y left, Z up) -> optical.
        const gz::math::Pose3d body_to_optical(
            gz::math::Vector3d::Zero,
            gz::math::Quaterniond(-M_PI / 2.0, 0.0, -M_PI / 2.0));
        const gz::math::Pose3d optical_rel = cam_rel * body_to_optical;

        const std::string link_frame = this->entity_name + "/uvcam_" + this->device_id + "_link";
        const std::string optical_frame = this->entity_name + "/uvcam_" + this->device_id + "_optical";

        auto make = [&](const std::string &child, const gz::math::Pose3d &p) {
          geometry_msgs::msg::TransformStamped t;
          t.header.stamp = this->nh->now();
          t.header.frame_id = this->parent_frame;
          t.child_frame_id = child;
          t.transform.translation.x = p.Pos().X();
          t.transform.translation.y = p.Pos().Y();
          t.transform.translation.z = p.Pos().Z();
          t.transform.rotation.w = p.Rot().W();
          t.transform.rotation.x = p.Rot().X();
          t.transform.rotation.y = p.Rot().Y();
          t.transform.rotation.z = p.Rot().Z();
          return t;
        };

        this->tf_broadcaster->sendTransform(
            {make(link_frame, cam_rel), make(optical_frame, optical_rel)});
        this->tf_published = true;

        gzmsg << "[OcclusionCheck] Published static TF " << this->parent_frame
              << " -> " << link_frame << " and -> " << optical_frame
              << " (xyz " << cam_rel.Pos().X() << " " << cam_rel.Pos().Y()
              << " " << cam_rel.Pos().Z() << ")" << std::endl;
      }

      void scan_for_LED_entities(const std::string &led_ns, gz::sim::EntityComponentManager &ecm){
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

              this->ledEntities[name] = e;

              return true;
            });
        gzdbg << "[OcclusionCheck] Tracking " << this->ledEntities.size() << " LED entities." << std::endl;
      }

      std::string resolveCalibrationPath(const std::string &filename) {
        if (!filename.empty() && filename.front() == '/') {
          return filename;
        }
        try {
          return ament_index_cpp::get_package_share_directory("uvdar_gazebo_plugin") +
                 "/config/ocamcalib/" + filename;
        } catch (const std::exception &e) {
          gzerr << "[OcclusionCheck] Could not locate uvdar_gazebo_plugin share directory: "
                << e.what() << std::endl;
          return filename;
        }
      }

      bool projectLed(const gz::math::Pose3d &ledPose, double point2D[2],
          double *radius_out = nullptr) const {
        const gz::math::Vector3d ledInCam = ledPose.CoordPositionSub(camPose);
        double input[3] = {
          -ledInCam.Z(),
          -ledInCam.Y(),
          -ledInCam.X()
        };
        world2cam(point2D, input, const_cast<struct ocam_model *>(&oc_model));

        if (!std::isfinite(point2D[0]) || !std::isfinite(point2D[1])) {
          return false;
        }
        // point2D is [row, col] in OCamCalib convention.
        if (point2D[0] < 0.0 || point2D[0] >= static_cast<double>(oc_model.height) ||
            point2D[1] < 0.0 || point2D[1] >= static_cast<double>(oc_model.width)) {
          return false;
        }

        double reprojected[3];
        cam2world(reprojected, point2D, const_cast<struct ocam_model *>(&oc_model));
        const double input_norm =
            std::sqrt(input[0] * input[0] + input[1] * input[1] + input[2] * input[2]);
        if (input_norm < 1e-9) {
          return false;
        }
        const double alignment =
            (reprojected[0] * input[0] + reprojected[1] * input[1] + reprojected[2] * input[2]) /
            input_norm;
        if (alignment <= 0.0) {
          return false;
        }

        gz::math::Quaterniond invOrient = ledPose.Rot().Inverse();
        gz::math::Pose3d ledForward =
            gz::math::Pose3d(0, 0, 1, 0, 0, 0).RotatePositionAboutOrigin(invOrient);
        gz::math::Vector3d toCam = camPose.Pos() - ledPose.Pos();

        double distance = toCam.Length();
        if (distance < 1e-6) {
          return false;
        }
        double cosAngle = ledForward.Pos().Dot(toCam) / distance;
        double intensity = std::round(this->led_gain * std::max(0.0, cosAngle) *
            (coef[0] + (coef[1] / ((distance + coef[2]) * (distance + coef[2])))));

        if (radius_out != nullptr) {
          *radius_out = std::sqrt(std::max(0.0, intensity) / M_PI);
        }

        return intensity > min_intensity;
      }

    public:
      OcclusionCheck() = default;
      ~OcclusionCheck() override = default;

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

        if (_sdf->HasElement("use_occlusions")) {
          auto elem = _sdf->FindElement("use_occlusions");
          if (elem) {
            this->use_occlusions = elem->Get<bool>();
          }
        }

        if (_sdf->HasElement("pixel_covariance")) {
          auto elem = _sdf->FindElement("pixel_covariance");
          if (elem) {
            this->pixel_covariance = elem->Get<double>();
          }
        }

        if (_sdf->HasElement("min_intensity")) {
          auto elem = _sdf->FindElement("min_intensity");
          if (elem) {
            this->min_intensity = elem->Get<double>();
          }
        }

        if (_sdf->HasElement("points_rate")) {
          auto elem = _sdf->FindElement("points_rate");
          if (elem) {
            this->points_rate = elem->Get<double>();
          }
        }

        if (_sdf->HasElement("publish_image")) {
          auto elem = _sdf->FindElement("publish_image");
          if (elem) {
            this->publish_image = elem->Get<bool>();
          }
        }

        if (_sdf->HasElement("covariance_model")) {
          auto elem = _sdf->FindElement("covariance_model");
          if (elem) {
            this->covariance_model = elem->Get<std::string>();
          }
        }
        if (const char *env = std::getenv("UVDAR_SIM_COV_MODEL")) {
          this->covariance_model = env;
        }
        if (this->covariance_model != "blob" && this->covariance_model != "constant") {
          gzerr << "[OcclusionCheck] Unknown covariance_model '" << this->covariance_model
                << "', falling back to 'blob'." << std::endl;
          this->covariance_model = "blob";
        }

        if (_sdf->HasElement("parent_frame")) {
          auto elem = _sdf->FindElement("parent_frame");
          if (elem) {
            this->parent_frame = elem->Get<std::string>();
          }
        }

        if (_sdf->HasElement("background_level")) {
          auto elem = _sdf->FindElement("background_level");
          if (elem) {
            this->background_setting = elem->Get<int>();
          }
        }
        if (const char *env = std::getenv("UVDAR_SIM_BACKGROUND")) {
          try {
            this->background_setting = std::stoi(env);
          } catch (const std::exception &) {
            gzerr << "[OcclusionCheck] Ignoring malformed UVDAR_SIM_BACKGROUND='"
                  << env << "'" << std::endl;
          }
        }

        if (_sdf->HasElement("led_gain")) {
          auto elem = _sdf->FindElement("led_gain");
          if (elem) {
            this->led_gain = elem->Get<double>();
          }
        }
        if (const char *env = std::getenv("UVDAR_SIM_LED_GAIN")) {
          try {
            const double gain = std::stod(env);
            if (std::isfinite(gain) && gain > 0.0) {
              this->led_gain = gain;
            } else {
              gzerr << "[OcclusionCheck] Ignoring non-positive UVDAR_SIM_LED_GAIN='"
                    << env << "'" << std::endl;
            }
          } catch (const std::exception &) {
            gzerr << "[OcclusionCheck] Ignoring malformed UVDAR_SIM_LED_GAIN='"
                  << env << "'" << std::endl;
          }
        }
        if (const char *env = std::getenv("UVDAR_SIM_PUBLISH_IMAGE")) {
          const std::string value(env);
          this->publish_image = (value == "1" || value == "true" || value == "True");
        }

        if (const char *env = std::getenv("UVDAR_SIM_BITRATE")) {
          try {
            const double rate = std::stod(env);
            if (std::isfinite(rate) && rate > 0.0) {
              this->points_rate = rate;
              gzmsg << "[OcclusionCheck] Sampling rate overridden by "
                    << "UVDAR_SIM_BITRATE=" << rate << " Hz" << std::endl;
            } else {
              gzerr << "[OcclusionCheck] Ignoring non-positive "
                    << "UVDAR_SIM_BITRATE='" << env << "'" << std::endl;
            }
          } catch (const std::exception &) {
            gzerr << "[OcclusionCheck] Ignoring malformed UVDAR_SIM_BITRATE='"
                  << env << "'" << std::endl;
          }
        }

        auto nameComp = _ecm.Component<gz::sim::components::Name>(_entity);
        if (nameComp) {
          this->entity_name = nameComp->Data();
        }

        std::string calibration_file = "calib_results_bf_uv_fe.txt";
        if (_sdf->HasElement("calibration_file")) {
          auto elem = _sdf->FindElement("calibration_file");
          if (elem) {
            calibration_file = elem->Get<std::string>();
          }
        }
        std::string calib_path = resolveCalibrationPath(calibration_file);
        if (get_ocam_model(&this->oc_model, const_cast<char *>(calib_path.c_str())) == 0) {
          this->calibration_loaded = true;
          gzmsg << "[OcclusionCheck] Loaded camera calibration from " << calib_path << std::endl;
        } else {
          gzerr << "[OcclusionCheck] Failed to load camera calibration from " << calib_path
                << " -- no points will be published." << std::endl;
        }

        if (!rclcpp::ok()) {
          rclcpp::init(0, nullptr);
        }

        std::string node_name = "uvdar_cam_" + this->entity_name + "_" + this->device_id;
        this->nh = rclcpp::Node::make_shared(node_name);

        if (this->parent_frame.empty()) {
          this->parent_frame = this->entity_name + "/fcu";
        }
        this->tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->nh);

        std::string points_topic = "/" + this->device_id + "/uvdar_bluefox/points";
        if (_sdf->HasElement("points_topic")) {
          auto elem = _sdf->FindElement("points_topic");
          if (elem) {
            points_topic = elem->Get<std::string>();
          }
        }
        this->pub_visible_leds = this->nh->create_publisher<uvdar_core::msg::ImagePointsWithCovariancesStamped>(
            points_topic, 10);

        std::string image_topic = "/" + this->device_id + "/uvdar_bluefox/image_raw";
        if (_sdf->HasElement("image_topic")) {
          auto elem = _sdf->FindElement("image_topic");
          if (elem) {
            image_topic = elem->Get<std::string>();
          }
        }
        if (this->publish_image) {
          this->pub_image = this->nh->create_publisher<sensor_msgs::msg::Image>(image_topic, 10);
          const int level = (this->background_setting < 0)
              ? (std::rand() % 100)
              : this->background_setting;
          this->background_level = static_cast<uint8_t>(std::clamp(level, 0, 100));
        }

        gzmsg << "[OcclusionCheck] Plugin started! model=" << this->entity_name
          << " device_id=" << this->device_id
          << " occlusions=" << (this->use_occlusions ? "on" : "off")
          << " covariance=" << this->covariance_model
          << " points_topic=" << points_topic;
        if (this->publish_image) {
          gzmsg << " image_topic=" << image_topic
                << " (background=" << static_cast<int>(this->background_level)
                << ", led_gain=" << this->led_gain << ")";
        }
        gzmsg << std::endl;

        this->ledEntities.clear();
        this->scan_for_LED_entities(led_link_name, _ecm);

        if (this->use_occlusions) {
          _ecm.CreateComponent(this->world_entity,
              gz::sim::components::PhysicsCollisionDetector("bullet"));

          this->rayEntity = _ecm.CreateEntity();
          _ecm.CreateComponent(this->rayEntity, gz::sim::components::RaycastData());
          _ecm.CreateComponent(this->rayEntity,
              gz::sim::components::Pose(gz::math::Pose3d::Zero));

          // Ray count is resized dynamically in PreUpdate as LEDs come and go.
          auto *rayData = _ecm.Component<gz::sim::components::RaycastData>(this->rayEntity);
          rayData->Data().rays.clear();
        }

        gzdbg << "[OcclusionCheck] Plugin configured successfully." << std::endl;
      }

      void PreUpdate(const gz::sim::UpdateInfo &/*_info*/,
          gz::sim::EntityComponentManager &_ecm) override
      {
        if (!this->use_occlusions || this->rayEntity == gz::sim::kNullEntity) {
          return;
        }

        auto *rayData = _ecm.Component<gz::sim::components::RaycastData>(this->rayEntity);
        auto &rays = rayData->Data().rays;

        const size_t nLeds = this->ledEntities.size();
        if (rays.size() != nLeds) {
          rays.resize(nLeds);
          // Results will be resized by the physics system to match next step.
        }

        size_t i = 0;
        gz::math::Vector3d camPos = this->camPose.Pos();
        for (const auto &[name, ledEntity] : this->ledEntities)
        {
          gz::math::Vector3d ledPos = gz::sim::worldPose(ledEntity, _ecm).Pos();
          gz::math::Vector3d dir = ledPos - camPos;
          if (dir.Length() > 1e-6) {
            dir.Normalize();
          }

          rays[i].start = camPos + dir * 0.15; // offset to avoid self-hit
          rays[i].end   = ledPos;
          ++i;
        }

        _ecm.SetChanged(this->rayEntity,
            gz::sim::components::RaycastData::typeId,
            gz::sim::ComponentState::OneTimeChange);
      }

      void Update(const gz::sim::UpdateInfo &_info,
          gz::sim::EntityComponentManager &_ecm) override
      {
        if (!resolveCamEntity(_ecm)) {
          return;
        }
        publish_camera_tf(_ecm);

        this->camPose = gz::sim::worldPose(this->cam_entity, _ecm);

        const double sim_sec = std::chrono::duration<double>(_info.simTime).count();
        const long long frame_index = (this->points_rate > 0.0)
            ? static_cast<long long>(std::floor(sim_sec * this->points_rate + 1.0e-9))
            : 0;
        const bool due = !this->has_published_once
            || this->points_rate <= 0.0
            || frame_index != this->last_frame_index;
        if (this->calibration_loaded && due) {
          publishVisibleLeds(_info, _ecm);
          this->last_frame_index = frame_index;
          this->has_published_once = true;
        }

        // Rescan for added/removed LED entities *after* publishing, so this
        // step's occlusion results (read against the LED set PreUpdate cast
        // rays for) stay aligned with the LED set used here.
        counter = (counter + 1) % refresh_interval;
        if (counter == 0){
          this->ledEntities.clear();
          this->scan_for_LED_entities(led_link_name, _ecm);
        }
      }

    private:

      void publishVisibleLeds(const gz::sim::UpdateInfo &_info,
          gz::sim::EntityComponentManager &_ecm)
      {
        std::vector<bool> occluded(this->ledEntities.size(), false);
        if (this->use_occlusions && this->rayEntity != gz::sim::kNullEntity) {
          auto *rayData = _ecm.Component<gz::sim::components::RaycastData>(this->rayEntity);
          if (rayData) {
            const auto &rays = rayData->Data().rays;
            const auto &results = rayData->Data().results;
            if (results.size() == rays.size() && results.size() == this->ledEntities.size()) {
              gz::math::Vector3d camPos = this->camPose.Pos();
              size_t i = 0;
              for (const auto &[name, ledEntity] : this->ledEntities) {
                gz::math::Vector3d ledPos = gz::sim::worldPose(ledEntity, _ecm).Pos();
                double ledDist = (ledPos - camPos).Length();
                double hitFraction = results[i].fraction;
                double hitDist = hitFraction * ledDist;

                constexpr double kEpsilon = 0.05; // metres
                occluded[i] = (!std::isnan(hitFraction) &&
                    hitFraction > 1e-6 &&
                    hitDist < (ledDist - kEpsilon));
                ++i;
              }
            }
          }
        }

        uvdar_core::msg::ImagePointsWithCovariancesStamped msg;
        auto simSec = std::chrono::duration_cast<std::chrono::seconds>(_info.simTime);
        auto simNsec = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime - simSec);
        msg.stamp.sec = static_cast<int32_t>(simSec.count());
        msg.stamp.nanosec = static_cast<uint32_t>(simNsec.count());
        msg.image_height = static_cast<uint32_t>(this->oc_model.height);
        msg.image_width = static_cast<uint32_t>(this->oc_model.width);

        // Emulated camera frame, drawn from the same projections as the points
        // below so the two topics are guaranteed consistent for a given stamp.
        cv::Mat frame;
        if (this->publish_image && this->pub_image) {
          frame = cv::Mat(this->oc_model.height, this->oc_model.width, CV_8UC1,
              cv::Scalar(this->background_level));
        }

        size_t i = 0;
        for (const auto &[name, ledEntity] : this->ledEntities) {
          bool led_occluded = (i < occluded.size()) ? occluded[i] : false;
          ++i;
          if (led_occluded) {
            continue;
          }

          auto *blink = _ecm.Component<uvdar_gazebo_plugin::components::LedBlinkState>(ledEntity);
          if (!blink || !blink->Data().on) {
            continue;
          }

          gz::math::Pose3d ledPose = gz::sim::worldPose(ledEntity, _ecm);
          double point2D[2];
          double radius = 0.0;
          if (!projectLed(ledPose, point2D, &radius)) {
            continue;
          }

          uvdar_core::msg::Point2DWithCovariance p;
          p.x = point2D[1];
          p.y = point2D[0];
          if (this->covariance_model == "constant") {
            p.covariance_00 = this->pixel_covariance;
            p.covariance_01 = 0.0;
            p.covariance_10 = 0.0;
            p.covariance_11 = this->pixel_covariance;
          } else {
            double c00 = 0.0, c01 = 0.0, c11 = 0.0;
            blobCovariance(p.x, p.y, radius, c00, c01, c11);
            p.covariance_00 = c00;
            p.covariance_01 = c01;
            p.covariance_10 = c01;   // symmetric
            p.covariance_11 = c11;
          }
          msg.points.push_back(p);

          if (!frame.empty()) {
            cv::circle(frame,
                cv::Point2i(static_cast<int>(std::lround(p.x)),
                            static_cast<int>(std::lround(p.y))),
                static_cast<int>(std::lround(radius)),
                cv::Scalar(255), -1);
          }
        }

        std::lock_guard<std::mutex> lock(pubMutex);
        this->pub_visible_leds->publish(msg);

        if (!frame.empty()) {
          sensor_msgs::msg::Image image_msg;
          image_msg.header.stamp.sec = msg.stamp.sec;
          image_msg.header.stamp.nanosec = msg.stamp.nanosec;
          image_msg.header.frame_id = this->device_id;
          image_msg.height = static_cast<uint32_t>(frame.rows);
          image_msg.width = static_cast<uint32_t>(frame.cols);
          image_msg.encoding = "mono8";
          image_msg.is_bigendian = 0;
          image_msg.step = static_cast<uint32_t>(frame.cols);
          image_msg.data.assign(frame.datastart, frame.dataend);
          this->pub_image->publish(image_msg);
        }
      }

  };
}
GZ_ADD_PLUGIN(uvdar_gazebo_plugin::OcclusionCheck,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemUpdate,
    gz::sim::ISystemPreUpdate)
