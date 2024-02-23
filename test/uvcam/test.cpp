#include <gtest/gtest.h>

#include <mrs_uav_gazebo_testing/test_generic.h>
#include <sensor_msgs/PointCloud.h>

#define DEF_IMG_WIDTH 752
#define DEF_IMG_HEIGHT 480

struct point {
  double x;
  double y;
  double z;

  std::string to_string(){
    return "[" + std::to_string(x) +":"+ std::to_string(y)+":"+ std::to_string(z)+"]";
  }
};

class Tester : public mrs_uav_gazebo_testing::TestGeneric {

private:
  std::string _uav_name_1_, _uav_name_2_;
  std::string _gazebo_spawner_params_1_, _gazebo_spawner_params_2_;

  std::vector<mrs_lib::SubscribeHandler<sensor_msgs::PointCloud>> sh_uvcam_handlers_;
public:
  Tester();


  std::tuple<bool, std::string>  checkMetaDataPresence(mrs_lib::SubscribeHandler<sensor_msgs::PointCloud> &sh);
  std::vector<std::vector<point>> gatherObservedPoints(double duration_seconds);
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

  {
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Spawning " << _uav_name_1_);
    uh1->spawn(_gazebo_spawner_params_1_);
  }

  sleep(3);

  {
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Testing metadata generation...");
    auto [success_l, message_l] = checkMetaDataPresence(sh_uvcam_handlers_.at(0));
    if (!success_l){
      ROS_ERROR("[%s]: Metadata generation check failed for left camera!: %s", ros::this_node::getName().c_str(), message_l.c_str());
      return false;
    }
    auto [success_r, message_r] = checkMetaDataPresence(sh_uvcam_handlers_.at(1));
    if (!success_r){
      ROS_ERROR("[%s]: Metadata generation check failed for right camera!: %s", ros::this_node::getName().c_str(), message_r.c_str());
      return false;
    }
    auto [success_b, message_b] = checkMetaDataPresence(sh_uvcam_handlers_.at(2));
    if (!success_b){
      ROS_ERROR("[%s]: Metadata generation check failed for back camera!: %s", ros::this_node::getName().c_str(), message_b.c_str());
      return false;
    }

    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Testing metadata presence done.");
    if (!(success_l && success_r && success_b))
      return false;
  }

  auto [uh2o, uh2_message] = getUAVHandler(_uav_name_2_, false);
  if (uh2o == std::nullopt){
   ROS_ERROR("[%s]: Failed to create uav %s: %s", ros::this_node::getName().c_str(), _uav_name_.c_str(), uh2_message.c_str());
    return false;
  }
  auto uh2 = *std::move(uh2o);

  {
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Spawning " << _uav_name_2_);
    uh2->spawn(_gazebo_spawner_params_2_);
  }

  sleep(3);


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
    std::vector<std::vector<point>> test_sample = gatherObservedPoints(1.0); //gather for # seconds
    auto [success, message] = checkObservedPoints(test_sample, {

        {{669.72,233.98, 0.98},{688.90,234.12, 0.98}},
        {{59.13,233.99, 0.98},{39.95,234.13, 0.98}},
        {{1249.72,-2641.80, 0.98}, {1249.52,3100.90, 0.98}}
        });
    if (!success){
      ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: " << message);
      return false;
    }
  }

  {
    auto [success, message] = uh2->moveTo(-5,0,0,1);
    if (!success)
      return false;
  }
  

  sleep(1);
  {
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Testing point retrieval at 2. position...");
    std::vector<std::vector<point>> test_sample = gatherObservedPoints(1.0); //gather for # seconds
    auto [success, message] = checkObservedPoints(test_sample, {
        {{-119.95, 236.41, 0.80}, {-104.16, 236.37, 0.56}, {-104.64, 236.38, 0.80}, {-94.56, 235.70, 0.56}},
        {{823.36, 235.78, 0.80}, {839.33, 236.51, 0.56}, {838.85, 236.50, 0.80}, {848.49, 236.28, 0.56}},
        {{367.64, 244.20, 0.80}, {367.82, 225.95, 0.56}, {367.83, 226.50, 0.80}, {367.60, 215.18, 0.56}}
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

  sh_uvcam_handlers_.push_back(mrs_lib::SubscribeHandler<sensor_msgs::PointCloud>(shopts_, "/gazebo/"+_uav_name_1_+"/uvdar_bluefox_left/image_raw"));
  sh_uvcam_handlers_.push_back(mrs_lib::SubscribeHandler<sensor_msgs::PointCloud>(shopts_, "/gazebo/"+_uav_name_1_+"/uvdar_bluefox_right/image_raw"));
  sh_uvcam_handlers_.push_back(mrs_lib::SubscribeHandler<sensor_msgs::PointCloud>(shopts_, "/gazebo/"+_uav_name_1_+"/uvdar_bluefox_back/image_raw"));
}

std::tuple<bool, std::string>  Tester::checkMetaDataPresence(mrs_lib::SubscribeHandler<sensor_msgs::PointCloud> &sh){
    if (!sh.hasMsg()) {
      return {false, "No messages received on topic " + sh.topicName()};
    }

    return {true, "Topic " + sh.topicName() + " generates data."};
}

std::vector<std::vector<point>> Tester::gatherObservedPoints(double duration_seconds){
  std::vector<std::vector<point>>  output;
  for ([[maybe_unused]] auto& sh : sh_uvcam_handlers_){
    output.push_back({});
  }
  ros::Time init_time = ros::Time::now();
  while ((ros::Time::now() - init_time).toSec() < duration_seconds) {
    int i = 0; 
    for (auto &sh : sh_uvcam_handlers_){
      if (sh.hasMsg()) {
        {
          auto msg = sh.getMsg();
          for (auto &pt : msg->points){
            /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: CAM: " << i << ": X: " << pt.x << "; Y: " << pt.y << "; S: " << pt.z); */
            output.at(i).push_back({pt.x,pt.y,pt.z});

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
        if (pointDistance(pt, tmpl) <= 0.1){ // point is close to the current template
          match_found = true;
          template_filled.at(j) = true;
        }

        j++;
      }

      if (!match_found)
        return {false, "Found a point " + pt.to_string() + " generated by camera " + std::to_string(i) + " not matching any template!"};

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
  diff.z = b.z-a.z;
  return sqrt(diff.x*diff.x + diff.y*diff.y + diff.z*diff.z);
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
