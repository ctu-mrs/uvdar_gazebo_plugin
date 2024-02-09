#include <gtest/gtest.h>

#include <mrs_uav_testing/test_generic.h>
#include <uvdar_core/ImagePointsWithFloatStamped.h>

#define DEF_IMG_WIDTH 752
#define DEF_IMG_HEIGHT 480

struct point {
  double x;
  double y;
  int ID;

  std::string to_string(){
    return "[" + std::to_string(x) +":"+ std::to_string(y)+"-ID:"+std::to_string(ID)+"]";
  }
};

class Tester : public mrs_uav_testing::TestGeneric {

private:
  std::string _uav_name_1_, _uav_name_2_;
  std::string _gazebo_spawner_params_1_, _gazebo_spawner_params_2_;

  mrs_lib::SubscribeHandler<uvdar_core::ImagePointsWithFloatStamped> sh_blink_handler_left_, sh_blink_handler_right_, sh_blink_handler_back_;
public:
  Tester();


  std::tuple<bool, std::string>  checkImageSize(mrs_lib::SubscribeHandler<uvdar_core::ImagePointsWithFloatStamped> &sh);
  std::vector<std::vector<point>> gatherObservedPoints(std::vector<std::reference_wrapper<mrs_lib::SubscribeHandler<uvdar_core::ImagePointsWithFloatStamped>>> shs, double duration_seconds);
  std::tuple<bool, std::string> checkObservedPoints(std::vector<std::vector<point>> test_sample, std::vector<std::vector<point>> comparison_templates);

  double pointDistance(point a, point b);
  

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

  if (isGazeboSimulation()){
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Spawning " << _uav_name_1_);
    uh1.spawn(_gazebo_spawner_params_1_);
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Spawning " << _uav_name_2_);
    uh2.spawn(_gazebo_spawner_params_2_);
  }

  /* sleep(3.0); */
  {
    auto [success, message] = uh1.moveTo(0,0,0,0);
    if (!success)
      return false;
  }
  {
    /* auto [success, message] = uh2.moveTo(3,-4,0,3.14); */
    auto [success, message] = uh2.moveTo(3,-4,0,0);
    if (!success)
      return false;
  }
  sleep(3);
  {
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Testing blinker retrieval...");
    std::vector<std::vector<point>> test_sample = gatherObservedPoints(
        {
        sh_blink_handler_left_,
        sh_blink_handler_right_,
        sh_blink_handler_back_
        },
        1.0);
    auto [success, message] = checkObservedPoints(test_sample, {
        {},
        {{271,233,2},{288,233,1},{300,233,3}},
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
Tester::Tester() : mrs_uav_testing::TestGeneric() {


  pl_ = std::make_shared<mrs_lib::ParamLoader>(nh_, "Test");
  pl_->loadParam("gazebo_spawner_params_1", _gazebo_spawner_params_1_, std::string());
  pl_->loadParam("gazebo_spawner_params_2", _gazebo_spawner_params_2_, std::string());
  pl_->loadParam("uav_name_1", _uav_name_1_, std::string());
  pl_->loadParam("uav_name_2", _uav_name_2_, std::string());

  sh_blink_handler_left_ = mrs_lib::SubscribeHandler<uvdar_core::ImagePointsWithFloatStamped>(shopts_, "/"+_uav_name_1_+"/uvdar/blinkers_seen_left");
  sh_blink_handler_right_ = mrs_lib::SubscribeHandler<uvdar_core::ImagePointsWithFloatStamped>(shopts_, "/"+_uav_name_1_+"/uvdar/blinkers_seen_right");
  sh_blink_handler_back_ = mrs_lib::SubscribeHandler<uvdar_core::ImagePointsWithFloatStamped>(shopts_, "/"+_uav_name_1_+"/uvdar/blinkers_seen_back");
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

std::vector<std::vector<point>> Tester::gatherObservedPoints(std::vector<std::reference_wrapper<mrs_lib::SubscribeHandler<uvdar_core::ImagePointsWithFloatStamped>>> shs, double duration_seconds){
  std::vector<std::vector<point>>  output;
  for ([[maybe_unused]] auto& sh : shs){
    output.push_back({});
  }
  ros::Time init_time = ros::Time::now();
  while ((ros::Time::now() - init_time).toSec() < duration_seconds) {
    int i = 0; 
    for (auto &sh : shs){
      if (sh.get().hasMsg()) {
        {
          for (auto pt : sh.get().getMsg()->points){
            /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: X: " << pt.x << "; Y: " << pt.y << "; Val: " << pt.value); */
            output.at(i).push_back({pt.x,pt.y,(int)(pt.value)});

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

  int total_point_count = 0;
  int unidentified_point_count = 0;

  for (int i = 0; i < (int)(comparison_templates.size()); i++){ //iterate though the template and sample sets for each camera
    std::vector<bool> template_filled(comparison_templates.at(i).size(),false); //flags checking if the current template has at least one point matching it

    if ((comparison_templates.at(i).size() > 0) && (test_sample.at(i).size() == 0))
      return {false, "Template expects points for this camera, but no points were obtained!"};
    if ((test_sample.at(i).size() > 0) && (comparison_templates.at(i).size() == 0))
      return {false, "Template expects no points for this camera, but some points were obtained!"};

    for (auto pt : test_sample.at(i)){ //iterate though the retrieved points for the current camera
      total_point_count++;
      if (pt.ID < 0){
        unidentified_point_count++;
        continue;
      }


      bool match_found = false; // assume the point was not matched yet
      int j = 0;
      for (auto tmpl : comparison_templates.at(i)){ //iterate through all templates for the current camera
        if ((pointDistance(pt, tmpl) <= 2.0) && (pt.ID == tmpl.ID)){ // point is close to the current template
          match_found = true;
          template_filled.at(j) = true;
        }

        j++;
      }

      if ((!match_found) && (pt.ID >= 0))
        return {false, "Found a point " + pt.to_string() + " not matching any template!"};

    }

    int k = 0;
    for (auto tmfl : template_filled){
      if (!tmfl)
        return {false, "One of the templates " + comparison_templates.at(i).at(k).to_string() + " for the current camera matched with no point in the sample!"};

      k++;
    }

  }

  double error_ratio = ((double)(unidentified_point_count))/((double)(total_point_count));
  ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: " << error_ratio*100.0 << "\% of points are unidentified.");
  if (error_ratio >0.1){
    return {false, "More than 10\% of the points are unidentified!"};
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
