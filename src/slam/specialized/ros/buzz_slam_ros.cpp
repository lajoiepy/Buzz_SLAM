#include "buzz_slam_ros.h"
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <cmath> 
#include "boost/filesystem.hpp"

namespace buzz_slam {
/****************************************/
/****************************************/

BuzzSLAMRos::BuzzSLAMRos() {
}

/****************************************/
/****************************************/

BuzzSLAMRos::~BuzzSLAMRos() {
}

/****************************************/
/****************************************/

void BuzzSLAMRos::Init(buzzvm_t buzz_vm, const gtsam::Point3& t_gt, const gtsam::Rot3& R_gt){
   BuzzSLAM::Init(buzz_vm);

   // Initialize for tracing variables
   number_of_outliers_added_ = 0;
   number_of_inliers_added_ = 0;

   // Initialize random numbers generators
   srand(time(NULL));
   gen_translation_ = std::mt19937{rd_()};
   gen_rotation_ = std::mt19937{rd_()};
   gen_outliers_ = std::mt19937{rd_()};
   normal_distribution_translation_ = std::normal_distribution<>{0, translation_noise_std_};
   normal_distribution_rotation_ = std::normal_distribution<>{0, rotation_noise_std_};
   uniform_distribution_outliers_translation_ = std::uniform_real_distribution<>{0, sensor_range_};
   uniform_distribution_outliers_rotation_ = std::uniform_real_distribution<>{-M_PI, M_PI};
   uniform_distribution_draw_outlier_ = std::uniform_real_distribution<>{0, 1};
   previous_symbol_ = gtsam::Symbol(robot_id_char_, number_of_poses_);

   // Isotropic noise models
   chordal_graph_noise_model_ = gtsam::noiseModel::Isotropic::Variance(12, 1);

   // Initialize log files
   if (is_simulation_ && robot_id_ ==  0 && !boost::filesystem::exists(error_file_name_)) {
      // Write results to csv
      std::ofstream error_file;
      error_file.open(error_file_name_, std::ios::out | std::ios::app);
      error_file << "NumberOfRobots\tNumberOfPoses\tNumberOfSeparators\tOutlierPeriod\tUsesIncrementalSolving"
         "\tRotationNoiseStd\tTranslationNoiseStd\tRotationChangeThreshold\tPoseChangeThreshold"
         "\tOptimizerPeriod\tChiSquaredProbability"
         "\tErrorCentralized\tErrorDecentralized\tErrorInitial\tNumberOfRotationIterations\tNumberOfPoseIterations"
         "\tNumberOfInliers\tNumberOfOutliers\tNumberOfSeparatorsRejected\tNumberOfOutliersNotRejected\n";
      error_file.close();
   }
   std::string log_file_name = log_folder_  + std::to_string(robot_id_) + "_inliers_added_keys.g2o";
   std::remove(log_file_name.c_str());
   log_file_name = log_folder_  + std::to_string(robot_id_) + "_outliers_added_keys.g2o";
   std::remove(log_file_name.c_str());
   log_file_name = log_folder_  + std::to_string(robot_id_) + "_reference_frame.g2o";
   std::remove(log_file_name.c_str());

   start_optimization_triggered_ = false;
}

/****************************************/
/****************************************/

void BuzzSLAMRos::LoadParameters(const double& sensor_range, const int& outlier_period) {
   sensor_range_ = sensor_range;
   outlier_period_ = outlier_period;
}

/****************************************/
/****************************************/

void BuzzSLAMRos::AddOdometryMeasurement(const gtsam::Pose3& measurement, const gtsam::Matrix covariance) {
   
   // Increase the number of poses
   IncrementNumberOfPoses();

   // Next symbol
   gtsam::Symbol current_symbol = gtsam::Symbol(robot_id_char_, number_of_poses_);

   // Add new pose estimate into initial guesses
   auto new_pose = poses_initial_guess_->at<gtsam::Pose3>(previous_symbol_.key()) * measurement;
   poses_initial_guess_->insert(current_symbol.key(), new_pose);
   auto new_pose_no_updates = poses_initial_guess_no_updates_->at<gtsam::Pose3>(previous_symbol_.key()) * measurement;
   poses_initial_guess_no_updates_->insert(current_symbol.key(), new_pose_no_updates);
   auto new_pose_incremental = poses_initial_guess_centralized_incremental_updates_->at<gtsam::Pose3>(previous_symbol_.key()) * measurement;
   poses_initial_guess_centralized_incremental_updates_->insert(current_symbol.key(), new_pose_incremental);

   // Add new factor to local pose graph
   gtsam::SharedNoiseModel noise_model = gtsam::noiseModel::Gaussian::Covariance(covariance);
   gtsam::BetweenFactor<gtsam::Pose3> new_factor = 
            gtsam::BetweenFactor<gtsam::Pose3>(previous_symbol_, current_symbol, measurement, noise_model);
   local_pose_graph_->push_back(new_factor);
   local_pose_graph_no_filtering_->push_back(new_factor);

   // Add transform to local map for pairwise consistency maximization
   robot_local_map_.addTransform(new_factor, covariance);

   // Update attributes
   previous_symbol_ = current_symbol;
}

/****************************************/
/****************************************/

int BuzzSLAMRos::AddSeparatorMeasurement(const gtsam::BetweenFactor<gtsam::Pose3>& separator_factor) {

   AddNewKnownRobot(gtsam::Symbol(separator_factor.key1()).chr());
   AddNewKnownRobot(gtsam::Symbol(separator_factor.key2()).chr());

   // Get factor or make it an outlier
   gtsam::Pose3 measurement = separator_factor.measured();
   number_of_inliers_added_++;

   auto covariance_matrix = boost::dynamic_pointer_cast< gtsam::noiseModel::Gaussian >(separator_factor.noiseModel())->covariance();

   inliers_keys_.insert(std::make_pair(separator_factor.key1(), separator_factor.key2()));

   // Add new factor to local pose graph
   local_pose_graph_->push_back(separator_factor);
   local_pose_graph_no_filtering_->push_back(separator_factor);

   // Add transform to local map for pairwise consistency maximization
   robot_local_map_.addTransform(separator_factor, covariance_matrix);
   covariance_matrix_for_outlier_ = covariance_matrix;

   // Add info for flagged initialization
   IncrementNumberOfSeparatorsWithOtherRobot((int) gtsam::Symbol(separator_factor.key1()).chr() - 97);
   IncrementNumberOfSeparatorsWithOtherRobot((int) gtsam::Symbol(separator_factor.key2()).chr() - 97);
   IncrementNumberOfInliersWithOtherRobot((int) gtsam::Symbol(separator_factor.key1()).chr() - 97);
   IncrementNumberOfInliersWithOtherRobot((int) gtsam::Symbol(separator_factor.key2()).chr() - 97);
   
   return 1;
}

/****************************************/
/****************************************/

bool BuzzSLAMRos::GetStartOptimizationTriggered() {
   return start_optimization_triggered_;
}

/****************************************/
/****************************************/

void BuzzSLAMRos::TriggerOptimization() {
   start_optimization_triggered_ = true;
   start_time_ = std::clock();
}

/****************************************/
/****************************************/

void BuzzSLAMRos::IncrementNumberOfOutliersWithOtherRobot(const int& other_robot_id) {
   if (other_robot_id != robot_id_) {
      if (number_of_outliers_with_each_robot_.count(other_robot_id) == 0) {
         number_of_outliers_with_each_robot_.insert(std::make_pair(other_robot_id, 1));
      } else {
         number_of_outliers_with_each_robot_[other_robot_id] = number_of_outliers_with_each_robot_[other_robot_id] + 1;
      }
   }
}

/****************************************/
/****************************************/

void BuzzSLAMRos::IncrementNumberOfInliersWithOtherRobot(const int& other_robot_id) {
   if (other_robot_id != robot_id_) {
      if (number_of_inliers_with_each_robot_.count(other_robot_id) == 0) {
         number_of_inliers_with_each_robot_.insert(std::make_pair(other_robot_id, 1));
      } else {
         number_of_inliers_with_each_robot_[other_robot_id] = number_of_inliers_with_each_robot_[other_robot_id] + 1;
      }
   }
}

/****************************************/
/****************************************/

gtsam::Pose3 BuzzSLAMRos::OutlierMeasurement(const gtsam::Rot3& R, const gtsam::Point3& t) {
   
   // TODO: Add option to add noise greater than 3 or 5 sigmas, instead of totally random measurment
   // This is why this method takes the measurement in parameter.
   gtsam::Point3 t_outlier = {   uniform_distribution_outliers_translation_(gen_outliers_),  
                                 uniform_distribution_outliers_translation_(gen_outliers_),
                                 uniform_distribution_outliers_translation_(gen_outliers_) };


   gtsam::Rot3 R_outlier = gtsam::Rot3::Ypr( uniform_distribution_outliers_rotation_(gen_outliers_), 
                                             uniform_distribution_outliers_rotation_(gen_outliers_),
                                             uniform_distribution_outliers_rotation_(gen_outliers_));

   number_of_outliers_added_++;

   return gtsam::Pose3(R_outlier, t_outlier);
}

/****************************************/
/****************************************/

int BuzzSLAMRos::AddSeparatorMeasurementOutlier() {
   // Separator symbols
   if (known_other_robots_.empty()){
      return 0;
   }
   auto random_id = std::floor(uniform_distribution_draw_outlier_(gen_outliers_) * number_of_robots_);
   while (random_id == robot_id_ || known_other_robots_.find((char) random_id + 97) == known_other_robots_.end()) {
      random_id = std::floor(uniform_distribution_draw_outlier_(gen_outliers_) * number_of_robots_);
   }
   if (number_of_inliers_with_each_robot_[random_id] <= number_of_outliers_with_each_robot_[random_id] + 1) {
      return 0;
   }
   auto random_index1 = std::floor(uniform_distribution_draw_outlier_(gen_outliers_) * (number_of_poses_-1) + 1);
   auto random_index2 = std::floor(uniform_distribution_draw_outlier_(gen_outliers_) * (number_of_poses_-1) + 1);
   auto loop_closure_keys = std::make_pair(gtsam::Symbol(robot_id_char_, random_index1), gtsam::Symbol((char)(random_id + 97), random_index2));

   // Get an outlier
   gtsam::Pose3 measurement = OutlierMeasurement(gtsam::Rot3(), gtsam::Point3());
   boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3>> new_factor = boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(gtsam::Symbol(loop_closure_keys.first), gtsam::Symbol(loop_closure_keys.second), measurement, gtsam::noiseModel::Gaussian::Covariance(covariance_matrix_for_outlier_));

   UpdateCurrentSeparatorBuzzStructure(   (int)(gtsam::Symbol(loop_closure_keys.first).chr() - 97),
                                          (int)(gtsam::Symbol(loop_closure_keys.second).chr() - 97),
                                          gtsam::Symbol(loop_closure_keys.first).index(),
                                          gtsam::Symbol(loop_closure_keys.second).index(),
                                          measurement.x(),
                                          measurement.y(),
                                          measurement.z(),
                                          measurement.rotation().quaternion()[1],
                                          measurement.rotation().quaternion()[2],
                                          measurement.rotation().quaternion()[3],
                                          measurement.rotation().quaternion()[0],
                                          covariance_matrix_for_outlier_ );

   outliers_keys_.insert(std::make_pair(loop_closure_keys.first, loop_closure_keys.second));

   // Add new factor to local pose graph
   local_pose_graph_->push_back(new_factor);
   local_pose_graph_no_filtering_->push_back(new_factor);

   // Add transform to local map for pairwise consistency maximization
   robot_local_map_.addTransform(*new_factor, covariance_matrix_for_outlier_);

   // Add info for flagged initialization
   IncrementNumberOfOutliersWithOtherRobot((int) gtsam::Symbol(loop_closure_keys.second).chr() - 97);
   
   return 1;
}

/****************************************/
/****************************************/

void BuzzSLAMRos::WriteOptimizedDataset() {
   double duration = ( std::clock() - start_time_ ) / (double) CLOCKS_PER_SEC;
   std::string time_file_name = log_folder_  + std::to_string(robot_id_) + "_time.g2o";
   std::ofstream time_file;
   time_file.open(time_file_name, std::ios::trunc);
   time_file << duration << "\n" ;
   time_file.close();

   BuzzSLAM::WriteOptimizedDataset();

   std::string inliers_added_file_name = log_folder_  + std::to_string(robot_id_) + "_number_of_inliers_added.g2o";
   std::ofstream inliers_added_file;
   inliers_added_file.open(inliers_added_file_name, std::ios::trunc);
   inliers_added_file << number_of_inliers_added_ << "\n" ;
   inliers_added_file.close();

   std::string outliers_keys_file_name = log_folder_  + std::to_string(robot_id_) + "_outliers_added_keys.g2o";
   std::ofstream outliers_keys_file;
   outliers_keys_file.open(outliers_keys_file_name, std::ios::trunc);
   for (const auto& keys : outliers_keys_) {
      outliers_keys_file << keys.first << " " << keys.second << "\n" ;
   }
   outliers_keys_file.close();

   std::string inliers_keys_file_name = log_folder_  + std::to_string(robot_id_) + "_inliers_added_keys.g2o";
   std::ofstream inliers_keys_file;
   inliers_keys_file.open(inliers_keys_file_name, std::ios::trunc);
   for (const auto& keys : inliers_keys_) {
      inliers_keys_file << keys.first << " " << keys.second << "\n" ;
   }
   inliers_keys_file.close();

   std::string reference_frame_file_name = log_folder_  + std::to_string(robot_id_) + "_reference_frame.g2o";
   std::ofstream reference_frame_file;
   reference_frame_file.open(reference_frame_file_name, std::ios::trunc);
   reference_frame_file << lowest_id_included_in_global_map_ << "\n" ;
   reference_frame_file.close();

   start_optimization_triggered_ = false;

   std::string data_transmitted_file_name = log_folder_  + std::to_string(robot_id_) + "_bytes_transmitted.g2o";
   std::ofstream data_transmitted_file;
   data_transmitted_file.open(data_transmitted_file_name, std::ios::trunc);
   data_transmitted_file << number_of_bytes_exchanged_ << "\n" ;
   data_transmitted_file.close();

   std::string number_of_steps_file_name = log_folder_  + std::to_string(robot_id_) + "_number_of_steps.g2o";
   std::ofstream number_of_steps_file;
   number_of_steps_file.open(number_of_steps_file_name, std::ios::trunc);
   number_of_steps_file << number_of_optimization_steps_ << "\n" ;
   number_of_steps_file.close();
}

/****************************************/
/****************************************/

void BuzzSLAMRos::RemoveRejectedKeys() {
   for (const auto& rejected_pair : rejected_keys_) {
      inliers_keys_.erase(rejected_pair);
      inliers_keys_.erase(std::make_pair(rejected_pair.second, rejected_pair.first));
      outliers_keys_.erase(rejected_pair);
      outliers_keys_.erase(std::make_pair(rejected_pair.second, rejected_pair.first));
   }
   rejected_keys_.clear();
}

/****************************************/
/****************************************/

std::set<std::pair<gtsam::Key, gtsam::Key>> BuzzSLAMRos::AggregateOutliersKeys(const std::set<int>& robots) {
   std::set<std::pair<gtsam::Key, gtsam::Key>> outliers_keys;
   for (const auto& i : robots) {
      std::string outliers_keys_file_name = log_folder_  + std::to_string(i) + "_outliers_added_keys.g2o";
      std::ifstream outliers_keys_file(outliers_keys_file_name);
      long unsigned int key1, key2;
      while (outliers_keys_file >> key1)
      {
         outliers_keys_file >> key2;
         outliers_keys.insert(std::make_pair(gtsam::Key(key1), gtsam::Key(key2)));
      }
      outliers_keys_file.close();
   }
   return outliers_keys;
}

/****************************************/
/****************************************/

std::pair<int, int> BuzzSLAMRos::CountInliersAndOutliers(const std::set<int>& robots) {
   std::set<std::pair<gtsam::Key, gtsam::Key>> inliers_keys;
   for (const auto& i : robots) {
      std::string inliers_keys_file_name = log_folder_  + std::to_string(i) + "_inliers_added_keys.g2o";
      std::ifstream inliers_keys_file(inliers_keys_file_name);
      long unsigned int key1, key2;
      while (inliers_keys_file >> key1)
      {
         inliers_keys_file >> key2;
         auto robot_id_1 = (int) (gtsam::Symbol(key1).chr() - 97);
         auto robot_id_2 = (int) (gtsam::Symbol(key2).chr() - 97);
         if (robots.find(robot_id_1) != robots.end() &&
             robots.find(robot_id_2) != robots.end()) {
            inliers_keys.insert(std::make_pair(gtsam::Key(key1), gtsam::Key(key2)));
         }
      }
      inliers_keys_file.close();
   }
   std::set<std::pair<gtsam::Key, gtsam::Key>> outliers_keys;
   for (const auto& i : robots) {
      std::string outliers_keys_file_name = log_folder_  + std::to_string(i) + "_outliers_added_keys.g2o";
      std::ifstream outliers_keys_file(outliers_keys_file_name);
      long unsigned int key1, key2;
      while (outliers_keys_file >> key1)
      {
         outliers_keys_file >> key2;
         auto robot_id_1 = (int) (gtsam::Symbol(key1).chr() - 97);
         auto robot_id_2 = (int) (gtsam::Symbol(key2).chr() - 97);
         if (robots.find(robot_id_1) != robots.end() &&
             robots.find(robot_id_2) != robots.end()) {
            outliers_keys.insert(std::make_pair(gtsam::Key(key1), gtsam::Key(key2)));
         }
      }
      outliers_keys_file.close();
   }
   return std::make_pair(inliers_keys.size(), outliers_keys.size());
}

/****************************************/
/****************************************/

graph_utils::PoseWithCovariance BuzzSLAMRos::GetPoseEstimateAtID(const int& pose_id) {
   auto pose_with_covariance = robot_local_map_.getTrajectory().trajectory_poses.at(gtsam::Symbol(robot_id_char_, pose_id).key()).pose;
   return pose_with_covariance;
}

/****************************************/
/****************************************/

void BuzzSLAMRos::AbortOptimization(const bool& log_info){
   BuzzSLAM::AbortOptimization(log_info);
   if (log_info) {
      // Initialize the set of robots on which to evaluate
      std::set<int> robots = neighbors_within_communication_range_;
      robots.insert(robot_id_);

      auto aggregated_outliers_keys = AggregateOutliersKeys(robots);
      int number_of_separators = 0;
      int number_of_outliers_not_rejected = 0;
      for (const auto& i : robots) {
         std::string dataset_file_name = log_folder_  + std::to_string(i) + "_initial.g2o";
         if (!boost::filesystem::exists(dataset_file_name)) {
            return; // File does not exists yet
         }
         gtsam::GraphAndValues graph_and_values = gtsam::readG2o(dataset_file_name, true);
         int current_index = 0;
         for (const auto &factor: *graph_and_values.first) {
            boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3> > pose3_between = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factor);
            auto robot_1_id = gtsam::Symbol(pose3_between->key1()).chr();
            auto robot_2_id = gtsam::Symbol(pose3_between->key2()).chr();
            if (robot_1_id != robot_2_id) {
               if (robots.find(((int)robot_1_id-97)) != robots.end() && robots.find(((int)robot_2_id-97)) != robots.end()) {
                  number_of_separators++;
                  if (aggregated_outliers_keys.find(std::make_pair(pose3_between->key1(), pose3_between->key2())) != aggregated_outliers_keys.end()) {
                     number_of_outliers_not_rejected++;
                  }
               }
            }
            current_index++;
         }
      }

      // Gather info on outliers rejection
      double total_number_of_separators_rejected_on_all_robots = 0;
      for (const auto& i : robots) {
         std::string separators_rejected_file_name = log_folder_  + std::to_string(i) + "_number_of_separators_rejected.g2o";
         std::ifstream separators_rejected_file(separators_rejected_file_name);
         int number_of_separators_rejected  = 0;
         separators_rejected_file >> number_of_separators_rejected;
         total_number_of_separators_rejected_on_all_robots += number_of_separators_rejected;
         separators_rejected_file.close();
      }
      total_number_of_separators_rejected_on_all_robots /= 2;
      number_of_separators /= 2;
      auto inliers_outliers_added = CountInliersAndOutliers(robots);

      // Write results to csv
      std::ofstream error_file;
      error_file.open(error_file_name_, std::ios::out | std::ios::app);
      auto number_of_poses = optimizer_->numberOfPosesInCurrentEstimate();
      std::string place_holder = "Aborted";
      error_file << robots.size() << "\t" << number_of_poses << "\t" << place_holder << "\t" << outlier_period_ << std::boolalpha 
               << "\t" << incremental_solving_ << "\t" << rotation_noise_std_ << "\t" << translation_noise_std_ 
               << "\t" << rotation_estimate_change_threshold_ << "\t" << pose_estimate_change_threshold_ 
               << "\t" << optimizer_period_ << "\t" << pcm_threshold_
               << "\t" << place_holder << "\t" << place_holder << "\t" << place_holder 
               << "\t" << current_rotation_iteration_ << "\t" << current_pose_iteration_ 
               << "\t" << inliers_outliers_added.first
               << "\t" << inliers_outliers_added.second 
               << "\t" << std::round(total_number_of_separators_rejected_on_all_robots) 
               << "\t" << number_of_outliers_not_rejected
               << "\n";
      error_file.close();
   }
}

}