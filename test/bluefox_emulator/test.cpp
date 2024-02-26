#include <gtest/gtest.h>

#include <mrs_uav_gazebo_testing/test_generic.h>
#include <sensor_msgs/Image.h>

#define DEF_IMG_WIDTH 752
#define DEF_IMG_HEIGHT 480

class Tester : public mrs_uav_gazebo_testing::TestGeneric {

private:
  std::string _uav_name_1_, _uav_name_2_;
  std::string _gazebo_spawner_params_1_, _gazebo_spawner_params_2_;

  mrs_lib::SubscribeHandler<sensor_msgs::Image> sh_image_handler_left_, sh_image_handler_right_, sh_image_handler_back_;
public:
  Tester();


  std::tuple<bool, std::string>  checkImageSize(mrs_lib::SubscribeHandler<sensor_msgs::Image> &sh);
  std::vector<std::vector<ros::Time>> gatherImages(std::vector<std::reference_wrapper<mrs_lib::SubscribeHandler<sensor_msgs::Image>>> shs, double duration_seconds);
  std::tuple<bool, std::string> checkTimings(std::vector<std::vector<ros::Time>> test_sample, double expected_framerate);


  bool test();
};

bool Tester::test() {

  auto [uh1o, uh1_message] = getUAVHandler(_uav_name_1_, false);
  if (uh1o == std::nullopt){
   ROS_ERROR("[%s]: Failed to create uav %s: %s", ros::this_node::getName().c_str(), _uav_name_.c_str(), uh1_message.c_str());
    return false;
  }
  auto uh1 = *std::move(uh1o);

  {
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Spawning " << _uav_name_1_);
    uh1->spawn(_gazebo_spawner_params_1_);
  }

  sleep(10);

  {
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Testing resolution...");
    auto [success_l, message_l] = checkImageSize(sh_image_handler_left_);
    if (!success_l){
      ROS_ERROR("[%s]: Resolution check failed for left camera!: %s", ros::this_node::getName().c_str(), message_l.c_str());
      return false;
    }
    auto [success_r, message_r] = checkImageSize(sh_image_handler_right_);
    if (!success_r){
      ROS_ERROR("[%s]: Resolution check failed for right camera!: %s", ros::this_node::getName().c_str(), message_r.c_str());
      return false;
    }
    auto [success_b, message_b] = checkImageSize(sh_image_handler_back_);
    if (!success_b){
      ROS_ERROR("[%s]: Resolution check failed for back camera!: %s", ros::this_node::getName().c_str(), message_b.c_str());
      return false;
    }

    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Testing resolution done.");
    if (!(success_l && success_r && success_b))
      return false;
  }


  sleep(1);
  {
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Testing image retrieval...");
    std::vector<std::vector<ros::Time>> test_sample = gatherImages(
        {
        sh_image_handler_left_,
        sh_image_handler_right_,
        sh_image_handler_back_
        },
        1.0);
    auto [success, message] = checkTimings(test_sample, 60.0);
    if (!success){
      ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: " << message);
      return false;
    }
  }

  ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Testing image retrieval done.");




  return true;
}
Tester::Tester() : mrs_uav_gazebo_testing::TestGeneric() {


  pl_ = std::make_shared<mrs_lib::ParamLoader>(nh_, "Test");
  pl_->loadParam("gazebo_spawner_params_1", _gazebo_spawner_params_1_, std::string());
  pl_->loadParam("gazebo_spawner_params_2", _gazebo_spawner_params_2_, std::string());
  pl_->loadParam("uav_name_1", _uav_name_1_, std::string());
  pl_->loadParam("uav_name_2", _uav_name_2_, std::string());

  sh_image_handler_left_ = mrs_lib::SubscribeHandler<sensor_msgs::Image>(shopts_, "/"+_uav_name_1_+"/uvdar_bluefox_left/image_raw");
  sh_image_handler_right_ = mrs_lib::SubscribeHandler<sensor_msgs::Image>(shopts_, "/"+_uav_name_1_+"/uvdar_bluefox_right/image_raw");
  sh_image_handler_back_ = mrs_lib::SubscribeHandler<sensor_msgs::Image>(shopts_, "/"+_uav_name_1_+"/uvdar_bluefox_back/image_raw");
}

std::tuple<bool, std::string>  Tester::checkImageSize(mrs_lib::SubscribeHandler<sensor_msgs::Image> &sh){
    if (sh.hasMsg()) {
      auto msg = sh.getMsg();
      if ( msg->width != DEF_IMG_WIDTH){ 
        ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: The UV image width on topic " << sh.topicName() << " does not match the expected!");
        return {false, "The UV image width on topic " + sh.topicName() + " does not match the expected!"};
      }
      if ( msg->height != DEF_IMG_HEIGHT){ 
        ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: The UV image height on topic " << sh.topicName() << " does not match the expected!");
        return {false, "The UV image height on topic " + sh.topicName() + " does not match the expected!"};
      }
    }
    else
      return {false, "No messages received on topic " + sh.topicName()};

    return {true, "Resolution of UV image on topic " + sh.topicName() + " is correct."};
}

std::vector<std::vector<ros::Time>> Tester::gatherImages(std::vector<std::reference_wrapper<mrs_lib::SubscribeHandler<sensor_msgs::Image>>> shs, double duration_seconds){
  std::vector<std::vector<ros::Time>>  output;
  for ([[maybe_unused]] auto& sh : shs){
    output.push_back({});
  }
  ros::Time init_time = ros::Time::now();
  while ((ros::Time::now() - init_time).toSec() < duration_seconds) {
    int i = 0; 
    for (auto &sh : shs){
      if (sh.get().newMsg()) {
        {
          auto msg = sh.get().getMsg();
          output.at(i).push_back(msg->header.stamp);
          /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: C: " << i << " - stamp: " << output.at(i).back().toSec()); */
        }
      }

      i++;
    }
  }

  return output;
}

std::tuple<bool, std::string> Tester::checkTimings(std::vector<std::vector<ros::Time>> test_sample, double expected_framerate){
    static_assert(std::numeric_limits<float>::is_iec559, "IEEE 754 required");

  if ((int)(test_sample.size()) < 1 ){
    return {false, "No samples were retrieved! The image topic is not producing data!"};
  }

  if ((int)(test_sample.size()) < 2 ){
    return {false, "Only a single image received! Cannot perform a check of framerate."};
  }

  for (int c = 0; c < (int)(test_sample.size()); c++){ 
      int sample_count = (int)(test_sample.at(c).size());
      double maxtime = -INFINITY;
      double mintime = INFINITY;
      double sumtime = 0;

      for ( int i = 1; i < sample_count; i++){
        double dt = test_sample.at(c).at(i).toSec() - test_sample.at(c).at(i-1).toSec();
        sumtime += dt;

        if (dt > maxtime)
          maxtime = dt;

        if (dt < mintime)
          mintime = dt;
      }

      double avg_period = (sumtime/(sample_count-1));
      double avg_framerate = 1.0/avg_period;

      if (abs(avg_framerate - expected_framerate) > 0.5){
        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Sumtime: " << sumtime << ", sample_count: " << sample_count << ", mintime: " << mintime << ", maxtime: " << maxtime);
        return {false, "Camera " + std::to_string(c) + " has a framerate of " + std::to_string(avg_framerate) + " that differs significantly from the expected framerate of " + std::to_string(expected_framerate) + "!"};
      }

      if ((abs(avg_period - maxtime) > (0.5*avg_period)) || (abs(avg_period - mintime) > (0.5*avg_period))){
        return {false, "Camera " + std::to_string(c) + " exhibits inconsistent framerate; the average period is " + std::to_string(avg_period) + "s, while the minimum and maximum periods were" + std::to_string(mintime) + " and " + std::to_string(maxtime) + " respectively!"};
      }


  }

  return {true, "Success!"};
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
