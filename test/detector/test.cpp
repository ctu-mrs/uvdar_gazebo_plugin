#include <gtest/gtest.h>

#include <mrs_uav_gazebo_testing/test_generic.h>
#include <uvdar_core/ImagePointsWithFloatStamped.h>

#define DEF_IMG_WIDTH 752
#define DEF_IMG_HEIGHT 480

struct point {
  double x;
  double y;

  std::string to_string(){
    return "[" + std::to_string(x) +":"+ std::to_string(y)+"]";
  }
};

class Tester : public mrs_uav_gazebo_testing::TestGeneric {

private:
  std::string _uav_name_1_, _uav_name_2_;
  std::string _gazebo_spawner_params_1_, _gazebo_spawner_params_2_;

  mrs_lib::SubscribeHandler<uvdar_core::ImagePointsWithFloatStamped> sh_detector_handler_left_, sh_detector_handler_right_, sh_detector_handler_back_;
public:
  Tester();


  std::tuple<bool, std::string>  checkImageSize(mrs_lib::SubscribeHandler<uvdar_core::ImagePointsWithFloatStamped> &sh);
  std::vector<std::vector<point>> gatherObservedPoints(std::vector<std::reference_wrapper<mrs_lib::SubscribeHandler<uvdar_core::ImagePointsWithFloatStamped>>> shs, double duration_seconds);
  std::tuple<bool, std::string> checkObservedPoints(std::vector<std::vector<point>> test_sample, std::vector<std::vector<point>> comparison_templates);

  double pointDistance(point a, point b);
  

  bool test();
};

bool Tester::test() {

  auto [uh1o, uh1_message] = getUAVHandler(_uav_name_1_, false);
  if (uh1o == std::nullopt){
   ROS_ERROR("[%s]: Failed to create uav %s: %s", ros::this_node::getName().c_str(), _uav_name_.c_str(), uh1_message.c_str());
    return false;
  }
  auto uh1 = *std::move(uh1o);

  auto [uh2o, uh2_message] = getUAVHandler(_uav_name_2_, false);
  if (uh2o == std::nullopt){
    ROS_ERROR("[%s]: Failed to create uav %s: %s", ros::this_node::getName().c_str(), _uav_name_.c_str(), uh2_message.c_str());
    return false;
  }
  auto uh2 = *std::move(uh2o);

  {
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Spawning " << _uav_name_1_);
    uh1->spawn(_gazebo_spawner_params_1_);
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Spawning " << _uav_name_2_);
    uh2->spawn(_gazebo_spawner_params_2_);
  }

  sleep(10);

  {
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Testing resolution...");
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

    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Testing resolution done.");
    if (!(success_l && success_r && success_b))
      return false;
  }


  {
    auto [success, message] = uh1->moveTo(0,0,0,0);
    if (!success)
      return false;
  }
  {
    auto [success, message] = uh2->moveTo(5,0,0,0);
    if (!success)
      return false;
  }

  sleep(1);
  {
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Testing point retrieval at 1. position...");
    std::vector<std::vector<point>> test_sample = gatherObservedPoints(
        {
        sh_detector_handler_left_,
        sh_detector_handler_right_,
        sh_detector_handler_back_
        },
        1.0); //gather for # seconds
    auto [success, message] = checkObservedPoints(test_sample, {
        {{669,233},{688,234}},
        {{59,233},{39,234}},
        {}
        });
    if (!success){
      ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: " << message);
      return false;
    }
  }


  {
    auto [success, message] = uh2->moveTo(-5,0,0,0);
    if (!success)
      return false;
  }
  sleep(1);
  {
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Testing point retrieval at 2. position...");
    std::vector<std::vector<point>> test_sample = gatherObservedPoints(
        {
        sh_detector_handler_left_,
        sh_detector_handler_right_,
        sh_detector_handler_back_
        },
        1.0);
    auto [success, message] = checkObservedPoints(test_sample, {
        {},
        {},
        {{367,240},{367,218}}
        });
    if (!success){
      ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: " << message);
      return false;
    }
  }


  {
    auto [success, message] = uh2->moveTo(3,-4,0,0);
    if (!success)
      return false;
  }
  sleep(1);
  {
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Testing point retrieval at 3. position...");
    std::vector<std::vector<point>> test_sample = gatherObservedPoints(
        {
        sh_detector_handler_left_,
        sh_detector_handler_right_,
        sh_detector_handler_back_
        },
        1.0);
    auto [success, message] = checkObservedPoints(test_sample, {
        {},
        {{271,233},{288,233},{300,233}},
        {}
        });
    if (!success){
      ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: " << message);
      return false;
    }
  }

  ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Testing point retrieval done.");




  return true;
}
Tester::Tester() : mrs_uav_gazebo_testing::TestGeneric() {


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
      auto msg = sh.getMsg();
      if ( msg->image_width != DEF_IMG_WIDTH){ 
        ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: The UV image width on topic " << sh.topicName() << " does not match the expected!");
        return {false, "The UV image width on topic " + sh.topicName() + " does not match the expected!"};
      }
      if ( msg->image_height != DEF_IMG_HEIGHT){ 
        ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: The UV image height on topic " << sh.topicName() << " does not match the expected!");
        return {false, "The UV image height on topic " + sh.topicName() + " does not match the expected!"};
      }
    }
    else
      return {false, "No messages received on topic " + sh.topicName()};

    return {true, "Resolution of UV image on topic " + sh.topicName() + " is correct."};
}

std::vector<std::vector<point>> Tester::gatherObservedPoints(std::vector<std::reference_wrapper<mrs_lib::SubscribeHandler<uvdar_core::ImagePointsWithFloatStamped>>> shs, double duration_seconds){
  std::vector<std::vector<point>>  output;
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
          for (auto pt : msg->points){
            /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: X: " << pt.x << "; Y: " << pt.y << "; Val: " << pt.value); */
            output.at(i).push_back({pt.x,pt.y});

          }
        }
      }

      i++;
    }
  }

  return output;
}

std::tuple<bool, std::string> Tester::checkObservedPoints(std::vector<std::vector<point>> test_sample, std::vector<std::vector<point>> comparison_templates){
  if (test_sample.size() != comparison_templates.size()){
    return {false, "The comparison template and sensor output sample presume different number of cameras!"};
  }


  for (int i = 0; i < (int)(comparison_templates.size()); i++){ //iterate though the template and sample sets for each camera
    std::vector<bool> template_filled(comparison_templates.at(i).size(),false); //flags checking if the current template has at least one point matching it

    if ((comparison_templates.at(i).size() > 0) && (test_sample.at(i).size() == 0))
      return {false, "Template expects points for this camera, but no points were obtained!"};
    if ((test_sample.at(i).size() > 0) && (comparison_templates.at(i).size() == 0))
      return {false, "Template expects no points for this camera, but some points were obtained!"};

    for (auto pt : test_sample.at(i)){ //iterate though the retrieved points for the current camera
      bool match_found = false; // assume the point was not matched yet
      int j = 0;
      for (auto tmpl : comparison_templates.at(i)){ //iterate through all templates for the current camera
        if (pointDistance(pt, tmpl) <= 2.0){ // point is close to the current template
          match_found = true;
          template_filled.at(j) = true;
        }

        j++;
      }

      if (!match_found)
        return {false, "Found a point " + pt.to_string() + " not matching any template!"};

    }

    int k = 0;
    for (auto tmfl : template_filled){
      if (!tmfl)
        return {false, "One of the templates " + comparison_templates.at(i).at(k).to_string() + " for the current camera matched with no point in the sample!"};

      k++;
    }

  }


  return {true, "Success!"};
}

double Tester:: pointDistance(point a, point b){
  point diff;
  diff.x = b.x-a.x;
  diff.y = b.y-a.y;
  return sqrt(diff.x*diff.x + diff.y*diff.y);
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
