#include <gtest/gtest.h>

#include <mrs_uav_testing/test_generic.h>
#include <uvdar_core/ImagePointsWithFloatStamped.h>

#define DEF_IMG_WIDTH 752
#define DEF_IMG_HEIGHT 480

class Tester : public mrs_uav_testing::TestGeneric {

private:
  std::string _uav_name_1_, _uav_name_2_;
  std::string _gazebo_spawner_params_1_, _gazebo_spawner_params_2_;

  mrs_lib::SubscribeHandler<uvdar_core::ImagePointsWithFloatStamped> sh_detector_handler_left_, sh_detector_handler_right_, sh_detector_handler_back_;
public:
  Tester();
  std::tuple<bool, std::string>  checkImageSize(mrs_lib::SubscribeHandler<uvdar_core::ImagePointsWithFloatStamped> &sh);
  std::tuple<bool, std::string> checkObservedPoints(mrs_lib::SubscribeHandler<uvdar_core::ImagePointsWithFloatStamped> &sh);

  bool test();
};

bool Tester::test() {

  auto [uh1o, uh1_message] = makeUAV(_uav_name_1_, false);
  if (uh1o == std::nullopt){
   ROS_ERROR("[%s]: Failed to create uav %s: %s", ros::this_node::getName().c_str(), _uav_name_.c_str(), uh1_message.c_str());
    return false;
  }
  auto uh1 = *std::move(uh1o);

  auto [uh2o, uh2_message] = makeUAV(_uav_name_2_, false);
  if (uh2o == std::nullopt){
    ROS_ERROR("[%s]: Failed to create uav %s: %s", ros::this_node::getName().c_str(), _uav_name_.c_str(), uh2_message.c_str());
    return false;
  }
  auto uh2 = *std::move(uh2o);

  ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: HERE A");
  if (isGazeboSimulation()){
    uh1.spawn(_gazebo_spawner_params_1_);
    sleep(5.0);
  ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: HERE B");
    uh2.spawn(_gazebo_spawner_params_2_);
  }


  ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: HERE C");
  // wait for the MRS system
  /* while (true) { */

  /*   ROS_INFO_THROTTLE(1.0, "[%s]: waiting for the MRS system", ros::this_node::getName().c_str()); */

  /*   if (!ros::ok()) { */
  /*     ROS_ERROR("[%s]: killed from outside", ros::this_node::getName().c_str()); */
  /*     return false; */
  /*   } */

  /*   if (uh1.mrsSystemReady()) { */
  /*     break; */
  /*   } */
  /* } */

  /* { */
  /*   auto [success, message] = uh1.takeoff(); */

  /*   if (!success) { */
  /*     ROS_ERROR("[%s]: takeoff failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str()); */
  /*     return false; */
  /*   } */
  /* } */

  {
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: TESTING RES");
    auto [success_l, message_l] = checkImageSize(sh_detector_handler_left_);
    if (!success_l){
      ROS_ERROR("[%s]: Resolution check failed for left camera!: %s", ros::this_node::getName().c_str(), message_l.c_str());
      return false;
    }
    auto [success_r, message_r] = checkImageSize(sh_detector_handler_right_);
    if (!success_r){
      ROS_ERROR("[%s]: Resolution check failed for right camera!: %s", ros::this_node::getName().c_str(), message_r.c_str());
      return false;
    }
    auto [success_b, message_b] = checkImageSize(sh_detector_handler_back_);
    if (!success_b){
      ROS_ERROR("[%s]: Resolution check failed for back camera!: %s", ros::this_node::getName().c_str(), message_b.c_str());
      return false;
    }

    return (success_l && success_r && success_b);
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: TESTING RES DONE");
  }

  {
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: TESTING POINTS");
    auto [success, message] = checkObservedPoints(sh_detector_handler_left_);
    if (!success){
      ROS_ERROR("[%s]: No points!: %s", ros::this_node::getName().c_str(), message.c_str());
    }
    sleep(1);
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: TESTING POINTS DONE");
  }




  ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: ");

  sleep(5.0);

  if (uh1.isFlyingNormally()) {
    return true;
  } else {
    ROS_ERROR("[%s]: not flying normally", ros::this_node::getName().c_str());
    return false;
  }


  return true;
}
Tester::Tester() : mrs_uav_testing::TestGeneric() {


  pl_ = std::make_shared<mrs_lib::ParamLoader>(nh_, "Test");
  pl_->loadParam("gazebo_spawner_params_1", _gazebo_spawner_params_1_, std::string());
  pl_->loadParam("gazebo_spawner_params_2", _gazebo_spawner_params_2_, std::string());
  pl_->loadParam("uav_name_1", _uav_name_1_, std::string());
  pl_->loadParam("uav_name_2", _uav_name_2_, std::string());

  sh_detector_handler_left_ = mrs_lib::SubscribeHandler<uvdar_core::ImagePointsWithFloatStamped>(shopts_, "/"+_uav_name_1_+"/uvdar/points_seen_left");
  sh_detector_handler_right_ = mrs_lib::SubscribeHandler<uvdar_core::ImagePointsWithFloatStamped>(shopts_, "/"+_uav_name_1_+"/uvdar/points_seen_right");
  sh_detector_handler_back_ = mrs_lib::SubscribeHandler<uvdar_core::ImagePointsWithFloatStamped>(shopts_, "/"+_uav_name_1_+"/uvdar/points_seen_back");
}

std::tuple<bool, std::string>  Tester::checkImageSize(mrs_lib::SubscribeHandler<uvdar_core::ImagePointsWithFloatStamped> &sh){
    if (sh.hasMsg()) {
      if ( sh.getMsg()->image_width != DEF_IMG_WIDTH){ 
        ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: The UV image width on topic " << sh.topicName() << " does not match the expected!");
        return {false, "The UV image width on topic " + sh.topicName() + " does not match the expected!"};
      }
      if ( sh.getMsg()->image_height != DEF_IMG_HEIGHT){ 
        ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: The UV image height on topic " << sh.topicName() << " does not match the expected!");
        return {false, "The UV image height on topic " + sh.topicName() + " does not match the expected!"};
      }
    }
    else
      return {false, "No messages received on topic " + sh.topicName()};

    return {true, "Resolution of UV image on topic " + sh.topicName() + " is correct."};
}

std::tuple<bool, std::string> Tester::checkObservedPoints(mrs_lib::SubscribeHandler<uvdar_core::ImagePointsWithFloatStamped> &sh){
  if (sh.hasMsg()) {
    for (auto pt : sh.getMsg()->points){
      ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: X: " << pt.x << "; Y: " << pt.y << "; Val: " << pt.value);
    }
  }
  else
    return {false, "No messages received on topic "+sh.topicName()};


  return {true, "TBD"};
}


// --------------------------------------------------------------
// |                     DO NOT MODIFY BELOW                    |
// --------------------------------------------------------------

TEST(TESTSuite, test) {

  Tester tester;

  bool result = tester.test();

  if (result) {
    GTEST_SUCCEED();
  } else {
    GTEST_FAIL();
  }
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  ros::init(argc, argv, "test");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
