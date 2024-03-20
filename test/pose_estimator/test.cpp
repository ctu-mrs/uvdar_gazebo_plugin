#include <gtest/gtest.h>

#include <mrs_uav_gazebo_testing/test_generic.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>

#include <mrs_lib/geometry/conversions.h>
#include <mrs_lib/geometry/cyclic.h>

#define MAX_GAUSSIAN_VOLUME 1.5

#define AVG_MAHALANOBIS_THRESHOLD 1.5
#define MAX_MAHALANOBIS_THRESHOLD 2.0
#define MIN_MAHALANOBIS_THRESHOLD 0.1


using vec2_t = Eigen::Vector2d;
using vec3_t = Eigen::Vector3d;
using vec3i_t = Eigen::Vector3i;
using vec4_t = Eigen::Vector4d;
using quat_t = Eigen::Quaterniond;
using anax_t = Eigen::AngleAxisd;
using mat3_t = Eigen::Matrix3d;
using mat6_t = Eigen::Matrix<double, 6, 6>;
using rads_t = mrs_lib::geometry::sradians;

struct pose : geometry_msgs::Pose {

  std::string to_string(){
    return "{ " +
      std::to_string(position.x) +":"+ std::to_string(position.y)+":"+ std::to_string(position.z)+"]"+
      "-"+
      "["+std::to_string(orientation.x) +":"+ std::to_string(orientation.y)+":"+ std::to_string(orientation.z)+":"+ std::to_string(orientation.w)+"]"+
      " }";
  }
};

struct poseEstimate : mrs_msgs::PoseWithCovarianceIdentified {

  std::string to_string(){
    return "{ " +
      std::to_string(pose.position.x) +":"+ std::to_string(pose.position.y)+":"+ std::to_string(pose.position.z)+"]"+
      "-"+
      "["+std::to_string(pose.orientation.x) +":"+ std::to_string(pose.orientation.y)+":"+ std::to_string(pose.orientation.z)+":"+ std::to_string(pose.orientation.w)+"]"+
      "-"+
      "-ID:"+std::to_string(id)+" }";
  }
};

class Tester : public mrs_uav_gazebo_testing::TestGeneric {

private:
  std::string _uav_name_1_, _uav_name_2_;
  std::string _gazebo_spawner_params_1_, _gazebo_spawner_params_2_;

  mrs_lib::SubscribeHandler<mrs_msgs::PoseWithCovarianceArrayStamped> sh_pose_handler_;

  std::shared_ptr<mrs_uav_gazebo_testing::UAVHandler> uh1, uh2;
public:
  Tester();


  std::vector<std::vector<poseEstimate>> getObservedPoses(mrs_lib::SubscribeHandler<mrs_msgs::PoseWithCovarianceArrayStamped> &sh, double duration_seconds);
  std::tuple<bool, std::string> checkObservedPoses(std::vector<std::vector<poseEstimate>> As, pose b, int ID);

  std::tuple<bool, std::string> moveAndCheck(Eigen::Vector3d position, double heading, double gatherSeconds);

  std::pair<double, double> mahalanobisDistance(poseEstimate A, pose b);
  std::pair<double, double> determinant(poseEstimate A);
  std::pair<bool, Eigen::Vector3d> eigenvalues(poseEstimate A);
  

  bool test();
};

bool Tester::test() {

  auto [uh1o, uh1_message] = getUAVHandler(_uav_name_1_, false);
  if (uh1o == std::nullopt){
   ROS_ERROR("[%s]: Failed to create uav %s: %s", ros::this_node::getName().c_str(), _uav_name_.c_str(), uh1_message.c_str());
    return false;
  }
  uh1 = *std::move(uh1o);

  auto [uh2o, uh2_message] = getUAVHandler(_uav_name_2_, false);
  if (uh2o == std::nullopt){
    ROS_ERROR("[%s]: Failed to create uav %s: %s", ros::this_node::getName().c_str(), _uav_name_.c_str(), uh2_message.c_str());
    return false;
  }
  uh2 = *std::move(uh2o);

  {
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Spawning " << _uav_name_1_);
    uh1->spawn(_gazebo_spawner_params_1_);
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Spawning " << _uav_name_2_);
    uh2->spawn(_gazebo_spawner_params_2_);
  }

  sleep(2);
  {
    auto [success, message] = setRTFactorPercent(10);
    if (!success){
      ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: Failed to set RT factor: " << message);
      return false;
    }
  }
  {
    auto [success, message] = uh1->moveTo(0,0,0,0);
    if (!success)
      return false;
  }
  sleep(10);

  int pose_id = 1;
  {
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Testing pose retrieval for pose "+std::to_string(pose_id)+"...");
    auto [success, message] = moveAndCheck(Eigen::Vector3d(5,0,0), 0, 3.0);
    if (!success){
      ROS_ERROR("[%s]: Failed to test pose %d: %s", ros::this_node::getName().c_str(), pose_id, message.c_str());
      return false;
    }
    pose_id++;
  }
  {
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Testing pose retrieval for pose "+std::to_string(pose_id)+"...");
    auto [success, message] = moveAndCheck(Eigen::Vector3d(10,0,0), 0, 3.0);
    if (!success){
      ROS_ERROR("[%s]: Failed to test pose %d: %s", ros::this_node::getName().c_str(), pose_id, message.c_str());
      return false;
    }
    pose_id++;
  }
  {
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Testing pose retrieval for pose "+std::to_string(pose_id)+"...");
    auto [success, message] = moveAndCheck(Eigen::Vector3d(15,0,0), 0, 3.0);
    if (!success){
      ROS_ERROR("[%s]: Failed to test pose %d: %s", ros::this_node::getName().c_str(), pose_id, message.c_str());
      return false;
    }
    pose_id++;
  }
  {
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Testing pose retrieval for pose "+std::to_string(pose_id)+"...");
    auto [success, message] = moveAndCheck(Eigen::Vector3d(3,-4,0), 0, 3.0);
    if (!success){
      ROS_ERROR("[%s]: Failed to test pose %d: %s", ros::this_node::getName().c_str(), pose_id, message.c_str());
      return false;
    }
    pose_id++;
  }
  {
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Testing pose retrieval for pose "+std::to_string(pose_id)+"...");
    auto [success, message] = moveAndCheck(Eigen::Vector3d(6,-8,0), 0, 3.0);
    if (!success){
      ROS_ERROR("[%s]: Failed to test pose %d: %s", ros::this_node::getName().c_str(), pose_id, message.c_str());
      return false;
    }
    pose_id++;
  }
  {
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Testing pose retrieval for pose "+std::to_string(pose_id)+"...");
    auto [success, message] = moveAndCheck(Eigen::Vector3d(9,-12,0), 0, 3.0);
    if (!success){
      ROS_ERROR("[%s]: Failed to test pose %d: %s", ros::this_node::getName().c_str(), pose_id, message.c_str());
      return false;
    }
    pose_id++;
  }


  ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Testing pose retrieval done.");




  return true;
}
Tester::Tester() : mrs_uav_gazebo_testing::TestGeneric() {


  pl_ = std::make_shared<mrs_lib::ParamLoader>(nh_, "Test");
  pl_->loadParam("gazebo_spawner_params_1", _gazebo_spawner_params_1_, std::string());
  pl_->loadParam("gazebo_spawner_params_2", _gazebo_spawner_params_2_, std::string());
  pl_->loadParam("uav_name_1", _uav_name_1_, std::string());
  pl_->loadParam("uav_name_2", _uav_name_2_, std::string());

  sh_pose_handler_ = mrs_lib::SubscribeHandler<mrs_msgs::PoseWithCovarianceArrayStamped>(shopts_, "/"+_uav_name_1_+"/uvdar/measuredPoses");
}

std::vector<std::vector<poseEstimate>> Tester::getObservedPoses(mrs_lib::SubscribeHandler<mrs_msgs::PoseWithCovarianceArrayStamped> &sh, double duration_seconds){
  std::vector<std::vector<poseEstimate>>  output;
  ros::Time init_time = ros::Time::now();
  while ((ros::Time::now() - init_time).toSec() < duration_seconds) {
    int i = 0; 
    if (sh.newMsg()) {
      {
        output.push_back({});
        auto msg = sh.getMsg();
        for (auto p : msg->poses){
          /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: X: " << pt.x << "; Y: " << pt.y << "; Val: " << pt.value); */
          poseEstimate tmp;
          tmp.pose = p.pose;
          tmp.id = p.id;
          tmp.covariance = p.covariance;
          output.back().push_back(tmp);

        }
      }
    }

    i++;
  }

  return output;
}

std::tuple<bool, std::string> Tester::checkObservedPoses(std::vector<std::vector<poseEstimate>> As, pose b, int ID){
  if (As.size() == 0 ){
    return {false, "No pose messages were retrieved!"};
  }

  bool found_any_poses = false;

  std::vector<std::pair<double, double>> distances;
  for (auto &At : As){
    if (At.size() > 0){
      found_any_poses = true;
    }

    if (At.size() > 1) {
      std::string poses_string;
      for (auto &A : At){
        poses_string = poses_string +" ; "+ A.to_string();
      }
      return {false, "More than one from the same time were observed! : "+poses_string};
    }

    for (auto &A : At){ //should be only the one
      if ((int)(A.id) != ID){
        return {false, "Pose estimate with ID "+std::to_string(A.id)+" was found, but ID "+std::to_string(ID)+" was expected!"};
      }

      auto det = determinant(A);
      if (det.first > MAX_GAUSSIAN_VOLUME){
        auto [success, eigs] = eigenvalues(A);
        return {false, "The determinant of the position covariance is " + std::to_string(det.first) + " which is considered excessive! The eigenvalues of the covariance are: ["+std::to_string(eigs.x())+","+std::to_string(eigs.y())+","+std::to_string(eigs.z())+"]."};
      }

      distances.push_back(mahalanobisDistance(A, b));
    }
  }

  if (!found_any_poses){
      return {false, "No poses were obtained from the gathered messages!"};
  }

  double maxdist = -INFINITY;
  double mindist = INFINITY;
  double sumdist = 0;

  for (auto d : distances){

        if (d.first > maxdist)
          maxdist = d.first;

        if (d.first < mindist)
          maxdist = d.first;

        sumdist += d.first;
  }

  double avg_dist = (sumdist/((double)(distances.size())));

  if (avg_dist > AVG_MAHALANOBIS_THRESHOLD){
    return {false, "The average Mahalanobis distance of the estimates w.r.t. GT was " + std::to_string(avg_dist) + " which is considered excessive!"};
  }

  if (maxdist > MAX_MAHALANOBIS_THRESHOLD){
    return {false, "The maximum Mahalanobis distance of the estimates w.r.t. GT was " + std::to_string(maxdist) + " which is considered excessive!"};
  }

  if (mindist < MIN_MAHALANOBIS_THRESHOLD){
    return {false, "The smallest Mahalanobis distance of the estimates w.r.t. GT was " + std::to_string(maxdist) + " which is considered suspicious!"};
  }



  if (!found_any_poses) {
    return {false, "No poses were retrieved - all pose messages were empty!"};
  }

  return {true, "Success!"};
}
std::tuple<bool, std::string> Tester::moveAndCheck(Eigen::Vector3d position, double heading, double gatherSeconds){
  {
    auto [success, message] = uh2->moveTo(position.x(),position.y(),position.z(), heading);
    if (!success)
      return {false, "Failed to move UAV 2 to position ["+std::to_string(position.x())+","+std::to_string(position.y())+","+std::to_string(position.z())+","+std::to_string(heading)+"]: "+message};
  }
  sleep(1.0);
  {
    std::vector<std::vector<poseEstimate>> test_sample = getObservedPoses(
        {
        sh_pose_handler_
        },
        gatherSeconds);
    pose expected_pose;
    expected_pose.position.x = position.x();
    expected_pose.position.y = position.y();
    expected_pose.position.z = position.z();

    double qw              = cos(heading / 2.0);
    double qz              = sin(heading / 2.0);
    expected_pose.orientation.x = 0;
    expected_pose.orientation.y = 0;
    expected_pose.orientation.z = qz;
    expected_pose.orientation.w = qw;

    auto [success, message] = checkObservedPoses(test_sample, expected_pose, 0);
    if (!success){
      ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: " << message);
      return {false, message};
    }
  }

  return {true, "Success!"};
}

std::pair<double, double> Tester:: mahalanobisDistance(poseEstimate A, pose b){
  const mat6_t cov_orig = mrs_lib::geometry::toEigenMatrix(A.covariance);

  mat3_t Cp = cov_orig.block<3, 3>(0, 0);
  /* mat3_t Co = cov_orig.block<3, 3>(3, 3); */

  const vec3_t pp = mrs_lib::geometry::toEigen(b.position) - mrs_lib::geometry::toEigen(A.pose.position);;


      /* rads_t psi; // relative heading measurement */
  double md_p = sqrt(pp.transpose() * Cp.inverse() * pp);
  /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: a:"+A.to_string()+"; b:"+b.to_string()+" - md = "+std::to_string(md_p)); */
  return {md_p, 0};
}

std::pair<double, double> Tester::determinant(poseEstimate A){
  const mat6_t cov_orig = mrs_lib::geometry::toEigenMatrix(A.covariance);
  mat3_t Cp = cov_orig.block<3, 3>(0, 0);
  mat3_t Co = cov_orig.block<3, 3>(3, 3);
  return {Cp.determinant(), Co.determinant()};
}

std::pair<bool, Eigen::Vector3d> Tester::eigenvalues(poseEstimate A){
  const mat6_t cov_orig = mrs_lib::geometry::toEigenMatrix(A.covariance);
  mat3_t Cp = cov_orig.block<3, 3>(0, 0);
  Eigen::SelfAdjointEigenSolver<mat3_t> esol(Cp);
  // check success of the solver
  if (esol.info() != Eigen::Success)
  {
    ROS_ERROR("Failed to compute eigen vectors/values for position. Is the covariance matrix correct?");
    return {false, Eigen::Vector3d(INFINITY, INFINITY, INFINITY)};
  }

  return {true, esol.eigenvalues()};
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
