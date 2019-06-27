#include "buzz_controller_quadmapper_with_dataset.h"
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <cmath> 
#include "boost/filesystem.hpp"

namespace buzz_quadmapper {
/****************************************/
/****************************************/

CBuzzControllerQuadMapperWithDataset::CBuzzControllerQuadMapperWithDataset() :
   CBuzzControllerQuadMapper() {
}

/****************************************/
/****************************************/

CBuzzControllerQuadMapperWithDataset::~CBuzzControllerQuadMapperWithDataset() {
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapperWithDataset::Init(TConfigurationNode& t_node){
   CBuzzControllerQuadMapper::Init(t_node);

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

   // Initialize covariance matrix
   covariance_matrix_ = gtsam::Matrix6::Zero();
   covariance_matrix_(0,0) = std::pow(rotation_noise_std_, 2);
   covariance_matrix_(1,1) = std::pow(rotation_noise_std_, 2);
   covariance_matrix_(2,2) = std::pow(rotation_noise_std_, 2);
   covariance_matrix_(3,3) = std::pow(translation_noise_std_, 2);
   covariance_matrix_(4,4) = std::pow(translation_noise_std_, 2);
   covariance_matrix_(5,5) = std::pow(translation_noise_std_, 2);

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
   std::string log_file_name = "log/datasets/" + std::to_string(robot_id_) + "_inliers_added_keys.g2o";
   std::remove(log_file_name.c_str());
   log_file_name = "log/datasets/" + std::to_string(robot_id_) + "_outliers_added_keys.g2o";
   std::remove(log_file_name.c_str());
   log_file_name = "log/datasets/" + std::to_string(robot_id_) + "_reference_frame.g2o";
   std::remove(log_file_name.c_str());

   // Read .g2o dataset file
   std::string dataset_file_name = "datasets/" + dataset_name_ + "/" + std::to_string(robot_id_) + ".g2o";
   auto dataset_graph_and_values = gtsam::readG2o(dataset_file_name, true);

   // Fill values with odometry starting from ground truth position
   CQuaternion initial_orientation_reading = m_pcPos->GetReading().Orientation;
   auto initial_orientation = gtsam::Rot3(initial_orientation_reading.GetW(),initial_orientation_reading.GetX(),initial_orientation_reading.GetY(),initial_orientation_reading.GetZ());
   CVector3 initial_translation_reading = m_pcPos->GetReading().Position;
   auto initial_translation = gtsam::Point3(initial_translation_reading.GetX(), initial_translation_reading.GetY(), initial_translation_reading.GetZ());
   auto current_pose = gtsam::Pose3(initial_orientation, initial_translation);
   auto current_key = gtsam::Symbol(robot_id_char_, 0).key();
   dataset_graph_and_values.second->insert(current_key, current_pose);

   for (auto factor : *dataset_graph_and_values.first) {
      auto between_factor = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor);
      auto first_key = between_factor->key1();
      auto first_symbol = gtsam::Symbol(first_key);
      auto second_key = between_factor->key2();
      auto second_symbol = gtsam::Symbol(second_key);
      dataset_factors_.insert(std::make_pair(std::make_pair(first_key, second_key), between_factor));
      if (first_symbol.chr() == robot_id_char_ && second_symbol.chr() == robot_id_char_) {
         if (first_key == current_key) {
            // Measurement
            gtsam::Pose3 measurement(between_factor->measured().rotation(), 
                        gtsam::Point3(between_factor->measured().x()/10, between_factor->measured().y()/10, between_factor->measured().z()/10));
            // Compose previous pose and measurement
            current_pose = current_pose * measurement;
            // Add pose
            dataset_graph_and_values.second->insert(second_key, current_pose);
         } else {
            // Find first pose
            current_pose = dataset_graph_and_values.second->at<gtsam::Pose3>(first_key);
            // Measurement
            gtsam::Pose3 measurement(between_factor->measured().rotation(), 
                        gtsam::Point3(between_factor->measured().x()/10, between_factor->measured().y()/10, between_factor->measured().z()/10));
            // Compose previous pose and measurement
            current_pose = current_pose * measurement;
            // Add pose
            dataset_graph_and_values.second->insert(second_key, current_pose);
         }
      } else {
         if (first_symbol.chr() == robot_id_char_ && first_symbol.index() > second_symbol.index()) {
            if (loop_closure_linked_to_key_.count(first_key) == 0) {
               loop_closure_linked_to_key_.insert(std::make_pair(first_key, std::make_pair(first_key, second_key)));
            }
         } else if (second_symbol.chr() == robot_id_char_ && second_symbol.index() > first_symbol.index()) {
            if (loop_closure_linked_to_key_.count(second_key) == 0) {
               loop_closure_linked_to_key_.insert(std::make_pair(second_key, std::make_pair(first_key, second_key)));
            }
         }
      }
   }

   dataset_values_ = dataset_graph_and_values.second;
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapperWithDataset::LoadParameters(const std::string& dataset_name, const double& sensor_range, const int& outlier_period) {
   sensor_range_ = sensor_range;
   dataset_name_ = dataset_name;
   outlier_period_ = outlier_period;
}

/****************************************/
/****************************************/

int CBuzzControllerQuadMapperWithDataset::Move() {
   auto symbol = gtsam::Symbol(robot_id_char_, number_of_poses_);
   if (dataset_values_->exists(symbol.key())) {
      auto pose = dataset_values_->at<gtsam::Pose3>(symbol.key());
      CVector3 position(pose.x(), pose.y(), 2.0f); // Fixed altitude for visualization only
      m_pcPropellers->SetAbsolutePosition(position);
      m_pcPropellers->SetAbsoluteYaw(CRadians(pose.rotation().yaw()));

      // Add measurement
      AddOdometryMeasurement();
   }
   return number_of_poses_;
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapperWithDataset::AddOdometryMeasurement() {
   
   // Increase the number of poses
   IncrementNumberOfPoses();

   // Next symbol
   gtsam::Symbol current_symbol = gtsam::Symbol(robot_id_char_, number_of_poses_);

   // Initialize factor
   if (dataset_factors_.count(std::make_pair(previous_symbol_, current_symbol)) != 0) {
      auto new_factor = dataset_factors_.at(std::make_pair(previous_symbol_, current_symbol));

      // Add gaussian noise
      auto measurement = new_factor->measured();

      // Update attributes
      auto new_pose = poses_initial_guess_->at<gtsam::Pose3>(previous_symbol_.key()) * measurement;
      auto new_pose_no_updates = poses_initial_guess_no_updates_->at<gtsam::Pose3>(previous_symbol_.key()) * measurement;
      poses_initial_guess_no_updates_->insert(current_symbol.key(), new_pose_no_updates);
      auto new_pose_incremental = poses_initial_guess_centralized_incremental_updates_->at<gtsam::Pose3>(previous_symbol_.key()) * measurement;
      poses_initial_guess_centralized_incremental_updates_->insert(current_symbol.key(), new_pose_incremental);
      
      // Update attributes
      previous_symbol_ = current_symbol;

      // Add new factor to local pose graph
      local_pose_graph_->push_back(new_factor);
      local_pose_graph_no_filtering_->push_back(new_factor);

      // Add new pose estimate into initial guess
      poses_initial_guess_->insert(current_symbol.key(), new_pose);

      // Add transform to local map for pairwise consistency maximization
      robot_local_map_.addTransform(*new_factor, covariance_matrix_);
   } else {
      number_of_poses_ --;
   }
}

/****************************************/
/****************************************/

gtsam::Pose3 CBuzzControllerQuadMapperWithDataset::OutlierMeasurement(const gtsam::Rot3& R, const gtsam::Point3& t) {
   
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

int CBuzzControllerQuadMapperWithDataset::AddSeparatorMeasurement() {
   if (loop_closure_linked_to_key_.count(previous_symbol_.key()) == 0) {
      return 0;
   }
   // Separator symbols
   auto loop_closure_keys = loop_closure_linked_to_key_.at(previous_symbol_.key());

   AddNewKnownRobot(gtsam::Symbol(loop_closure_keys.first).chr());
   AddNewKnownRobot(gtsam::Symbol(loop_closure_keys.second).chr());

   // Get factor or make it an outlier
   boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3>> new_factor = dataset_factors_.at(std::make_pair(loop_closure_keys.first, loop_closure_keys.second));
   gtsam::Pose3 measurement = new_factor->measured();
   number_of_inliers_added_++;

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
                                          covariance_matrix_ );

   inliers_keys_.insert(std::make_pair(loop_closure_keys.first, loop_closure_keys.second));

   // Add new factor to local pose graph
   local_pose_graph_->push_back(new_factor);
   local_pose_graph_no_filtering_->push_back(new_factor);

   // Add transform to local map for pairwise consistency maximization
   robot_local_map_.addTransform(*new_factor, covariance_matrix_);

   // Add info for flagged initialization
   IncrementNumberOfSeparatorsWithOtherRobot((int) gtsam::Symbol(loop_closure_keys.first).chr() - 97);
   IncrementNumberOfSeparatorsWithOtherRobot((int) gtsam::Symbol(loop_closure_keys.second).chr() - 97);
   IncrementNumberOfInliersWithOtherRobot((int) gtsam::Symbol(loop_closure_keys.first).chr() - 97);
   IncrementNumberOfInliersWithOtherRobot((int) gtsam::Symbol(loop_closure_keys.second).chr() - 97);

   return 1;
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapperWithDataset::IncrementNumberOfOutliersWithOtherRobot(const int& other_robot_id) {
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

void CBuzzControllerQuadMapperWithDataset::IncrementNumberOfInliersWithOtherRobot(const int& other_robot_id) {
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

int CBuzzControllerQuadMapperWithDataset::AddSeparatorMeasurementOutlier() {
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
   boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3>> new_factor = boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(gtsam::Symbol(loop_closure_keys.first), gtsam::Symbol(loop_closure_keys.second), measurement, noise_model_);

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
                                          covariance_matrix_ );

   outliers_keys_.insert(std::make_pair(loop_closure_keys.first, loop_closure_keys.second));

   // Add new factor to local pose graph
   local_pose_graph_->push_back(new_factor);
   local_pose_graph_no_filtering_->push_back(new_factor);

   // Add transform to local map for pairwise consistency maximization
   robot_local_map_.addTransform(*new_factor, covariance_matrix_);

   // Add info for flagged initialization
   IncrementNumberOfOutliersWithOtherRobot((int) gtsam::Symbol(loop_closure_keys.second).chr() - 97);
   return 1;
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapperWithDataset::WriteOptimizedDataset() {
   CBuzzControllerQuadMapper::WriteOptimizedDataset();

   std::string inliers_added_file_name = "log/datasets/" + std::to_string(robot_id_) + "_number_of_inliers_added.g2o";
   std::ofstream inliers_added_file;
   inliers_added_file.open(inliers_added_file_name, std::ios::trunc);
   inliers_added_file << number_of_inliers_added_ << "\n" ;
   inliers_added_file.close();

   std::string outliers_keys_file_name = "log/datasets/" + std::to_string(robot_id_) + "_outliers_added_keys.g2o";
   std::ofstream outliers_keys_file;
   outliers_keys_file.open(outliers_keys_file_name, std::ios::trunc);
   for (const auto& keys : outliers_keys_) {
      outliers_keys_file << keys.first << " " << keys.second << "\n" ;
   }
   outliers_keys_file.close();

   std::string inliers_keys_file_name = "log/datasets/" + std::to_string(robot_id_) + "_inliers_added_keys.g2o";
   std::ofstream inliers_keys_file;
   inliers_keys_file.open(inliers_keys_file_name, std::ios::trunc);
   for (const auto& keys : inliers_keys_) {
      inliers_keys_file << keys.first << " " << keys.second << "\n" ;
   }
   inliers_keys_file.close();

   std::string reference_frame_file_name = "log/datasets/" + std::to_string(robot_id_) + "_reference_frame.g2o";
   std::ofstream reference_frame_file;
   reference_frame_file.open(reference_frame_file_name, std::ios::trunc);
   reference_frame_file << lowest_id_included_in_global_map_ << "\n" ;
   reference_frame_file.close();
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapperWithDataset::RemoveRejectedKeys() {
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

std::set<std::pair<gtsam::Key, gtsam::Key>> CBuzzControllerQuadMapperWithDataset::AggregateOutliersKeys(const std::set<int>& robots) {
   std::set<std::pair<gtsam::Key, gtsam::Key>> outliers_keys;
   for (const auto& i : robots) {
      std::string outliers_keys_file_name = "log/datasets/" + std::to_string(i) + "_outliers_added_keys.g2o";
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

std::pair<int, int> CBuzzControllerQuadMapperWithDataset::CountInliersAndOutliers(const std::set<int>& robots) {
   std::set<std::pair<gtsam::Key, gtsam::Key>> inliers_keys;
   for (const auto& i : robots) {
      std::string inliers_keys_file_name = "log/datasets/" + std::to_string(i) + "_inliers_added_keys.g2o";
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
      std::string outliers_keys_file_name = "log/datasets/" + std::to_string(i) + "_outliers_added_keys.g2o";
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

bool CBuzzControllerQuadMapperWithDataset::CompareCentralizedAndDecentralizedError() {
   // Initialize the set of robots on which to evaluate
   std::set<int> robots = neighbors_within_communication_range_;
   robots.insert(robot_id_);

   // Compute incremental centralized estimates
   ComputeCentralizedEstimateIncremental(robots, "");
   ComputeCentralizedEstimateIncremental(robots, "_no_filtering");

   // Collect expected estimate size
   std::string local_dataset_file_name = "log/datasets/" + std::to_string(robot_id_) + "_initial.g2o";
   gtsam::GraphAndValues local_graph_and_values = gtsam::readG2o(local_dataset_file_name, true);
   int expected_size = local_graph_and_values.second->size();

   // Aggregate estimates from all the robots
   auto aggregated_outliers_keys = AggregateOutliersKeys(robots);
   gtsam::Values distributed;
   std::vector<gtsam::GraphAndValues> graph_and_values_vec;
   int number_of_separators = 0;
   int number_of_outliers_not_rejected = 0;
   for (const auto& i : robots) {
      std::string dataset_file_name = "log/datasets/" + std::to_string(i) + "_optimized.g2o";
      if (!boost::filesystem::exists(dataset_file_name)) {
         if (debug_level_ >= 3) {
            std::cout << "Robot " << robot_id_ << " Evaluation : Other files do not exist yet" << std::endl;
         }
         RemoveRejectedKeys();
         return false; // File does not exists yet
      }
      gtsam::GraphAndValues graph_and_values = gtsam::readG2o(dataset_file_name, true);
      if (graph_and_values.second->size() < expected_size-2) {
         if (debug_level_ >= 3) {
            std::cout << "Robot " << robot_id_ << " Evaluation : Other file too small expected size=" << expected_size << ", actual size=" << graph_and_values.second->size() << std::endl;
         }
         RemoveRejectedKeys();
         return false; // File not updated yet
      }
      for (const gtsam::Values::ConstKeyValuePair &key_value: *graph_and_values.second) {
         gtsam::Key key = key_value.key;
         if (!distributed.exists(key)) {
            distributed.insert(key, (*graph_and_values.second).at<gtsam::Pose3>(key));
         }
      }
      std::vector<int> factors_to_remove;
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
            } else {
               factors_to_remove.emplace_back(current_index);
            }
         }
         current_index++;
      }
      dataset_file_name = "log/datasets/" + std::to_string(i) + "_initial.g2o";
      graph_and_values = gtsam::readG2o(dataset_file_name, true);
      // Remove factor involving values from robots not in communication range
      int number_of_factors_removed = 0;
      for (const auto & factor : factors_to_remove){
         int factor_index = factor - number_of_factors_removed;
         graph_and_values.first->erase(graph_and_values.first->begin()+factor_index);
         number_of_factors_removed++;
      }
      graph_and_values_vec.push_back(graph_and_values);
   }
   gtsam::GraphAndValues full_graph_and_values = distributed_mapper::evaluation_utils::readFullGraph(robots.size(), graph_and_values_vec);

   try {
      // Compute Error
      gtsam::noiseModel::Diagonal::shared_ptr evaluation_model = gtsam::noiseModel::Isotropic::Variance(6, 1e-12);
      auto errors = distributed_mapper::evaluation_utils::evaluateEstimates(robots.size(),
                                                         full_graph_and_values,
                                                         evaluation_model,
                                                         chordal_graph_noise_model_,
                                                         false,
                                                         distributed,
                                                         (bool) debug_level_);

      // Gather info on outliers rejection
      double total_number_of_separators_rejected_on_all_robots = 0;
      for (const auto& i : robots) {
         std::string separators_rejected_file_name = "log/datasets/" + std::to_string(i) + "_number_of_separators_rejected.g2o";
         std::ifstream separators_rejected_file(separators_rejected_file_name);
         int number_of_separators_rejected  = 0;
         separators_rejected_file >> number_of_separators_rejected;
         total_number_of_separators_rejected_on_all_robots += number_of_separators_rejected;
         separators_rejected_file.close();
      }
      total_number_of_separators_rejected_on_all_robots /= 2;
      number_of_separators /= 2;
      auto inliers_outliers_added = CountInliersAndOutliers(robots);
      number_of_outliers_not_rejected /= 2;

      // Write results to csv
      std::ofstream error_file;
      error_file.open(error_file_name_, std::ios::out | std::ios::app);
      auto number_of_poses = optimizer_->numberOfPosesInCurrentEstimate();
      error_file << robots.size() << "\t" << number_of_poses << "\t" << number_of_separators << "\t" << outlier_period_ << std::boolalpha 
               << "\t" << incremental_solving_ << "\t" << rotation_noise_std_ << "\t" << translation_noise_std_ 
               << "\t" << rotation_estimate_change_threshold_ << "\t" << pose_estimate_change_threshold_ 
               << "\t" << optimizer_period_ << "\t" << confidence_probability_
               << "\t" << std::get<0>(errors) << "\t" << std::get<1>(errors) << "\t" << std::get<2>(errors) 
               << "\t" << current_rotation_iteration_ << "\t" << current_pose_iteration_ 
               << "\t" << inliers_outliers_added.first
               << "\t" << inliers_outliers_added.second 
               << "\t" << total_number_of_separators_rejected_on_all_robots 
               << "\t" << number_of_outliers_not_rejected
               << "\n";
      error_file.close();

      ComputeCentralizedEstimate("");
      ComputeCentralizedEstimate("_no_filtering");

      RemoveRejectedKeys();
      return std::abs(std::get<0>(errors) - std::get<1>(errors)) < 0.1;

   } catch(...) {
      RemoveRejectedKeys();
      return false;
   }
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapperWithDataset::ComputeCentralizedEstimate(const std::string& centralized_extension) {
   // Initialize the set of robots on which to evaluate
   std::set<int> robots;
   for (int i = 0; i < number_of_robots_; i++) {
      robots.insert(i);
   }

   // Aggregate estimates from all the robots
   std::vector<gtsam::GraphAndValues> graph_and_values_vec;
   for (const auto& i : robots) {
      std::string dataset_file_name = "log/datasets/" + std::to_string(i) + "_initial_centralized" + centralized_extension + ".g2o";
      if (boost::filesystem::exists(dataset_file_name)) {
         gtsam::GraphAndValues graph_and_values = gtsam::readG2o(dataset_file_name, true);
         graph_and_values_vec.push_back(graph_and_values);
      }
   }
   gtsam::GraphAndValues full_graph_and_values = distributed_mapper::evaluation_utils::readFullGraph(graph_and_values_vec.size(), graph_and_values_vec);

   gtsam::noiseModel::Diagonal::shared_ptr evaluation_model = gtsam::noiseModel::Isotropic::Variance(6, 1e-12);

   std::pair<gtsam::Values, gtsam::Values> estimates = distributed_mapper::evaluation_utils::centralizedEstimates(full_graph_and_values, 
                              evaluation_model,
                              chordal_graph_noise_model_,
                              false);

   // Split estimates
   std::map<int, gtsam::Values> centralized_values_by_robots;
   for (const auto& i : robots) {
      centralized_values_by_robots.insert(std::make_pair(i, gtsam::Values()));
   }

   for (const gtsam::Values::ConstKeyValuePair &key_value : estimates.first) {
      int value_robot_id = (int)(gtsam::Symbol(key_value.key).chr() - 97);
      centralized_values_by_robots[value_robot_id].insert(key_value.key, key_value.value);
   }

   std::map<int, gtsam::Values> centralized_GN_values_by_robots;
   for (const auto& i : robots) {
      centralized_GN_values_by_robots.insert(std::make_pair(i, gtsam::Values()));
   }

   for (const gtsam::Values::ConstKeyValuePair &key_value : estimates.second) {
      int value_robot_id = (int)(gtsam::Symbol(key_value.key).chr() - 97);
      centralized_GN_values_by_robots[value_robot_id].insert(key_value.key, key_value.value);
   }

   for (const auto& i : robots) {
      std::string centralized_file_name = "log/datasets/" + std::to_string(i) + "_centralized" + centralized_extension + ".g2o";
      gtsam::writeG2o(gtsam::NonlinearFactorGraph(), centralized_values_by_robots[i], centralized_file_name);
      centralized_file_name = "log/datasets/" + std::to_string(i) + "_centralized_GN" + centralized_extension + ".g2o";
      gtsam::writeG2o(gtsam::NonlinearFactorGraph(), centralized_GN_values_by_robots[i], centralized_file_name);
   }

}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapperWithDataset::ComputeCentralizedEstimateIncremental(std::set<int> robots, const std::string& centralized_extension) {

   // Aggregate estimates from all the robots
   std::vector<gtsam::GraphAndValues> graph_and_values_vec;
   std::string dataset_file_name = "log/datasets/" + std::to_string(prior_owner_) + "_initial_centralized" + centralized_extension + "_incremental.g2o";
   if (boost::filesystem::exists(dataset_file_name)) {
      gtsam::GraphAndValues graph_and_values = gtsam::readG2o(dataset_file_name, true);
      graph_and_values_vec.push_back(graph_and_values);
   }
   for (const auto& i : robots) {
      if (i != prior_owner_){
         dataset_file_name = "log/datasets/" + std::to_string(i) + "_initial_centralized" + centralized_extension + "_incremental.g2o";
         if (boost::filesystem::exists(dataset_file_name)) {
            gtsam::GraphAndValues graph_and_values = gtsam::readG2o(dataset_file_name, true);
            graph_and_values_vec.push_back(graph_and_values);
         }
      }
   }
   gtsam::GraphAndValues full_graph_and_values = distributed_mapper::evaluation_utils::readFullGraph(graph_and_values_vec);

   gtsam::noiseModel::Diagonal::shared_ptr evaluation_model = gtsam::noiseModel::Isotropic::Variance(6, 1e-12);

   std::pair<gtsam::Values, gtsam::Values> estimates = distributed_mapper::evaluation_utils::centralizedEstimates(full_graph_and_values, 
                              evaluation_model,
                              chordal_graph_noise_model_,
                              false);

   // Split estimates
   std::map<int, gtsam::Values> centralized_values_by_robots;
   for (const auto& i : robots) {
      centralized_values_by_robots.insert(std::make_pair(i, gtsam::Values()));
   }

   // Get anchor offset
   gtsam::Point3 anchor_offset_translation = gtsam::Point3();
   gtsam::Rot3 anchor_offset_rotation = gtsam::Rot3();
   dataset_file_name = "log/datasets/" + std::to_string(prior_owner_) + "_centralized" + centralized_extension + "_incremental.g2o";
   if (boost::filesystem::exists(dataset_file_name)) {
      gtsam::GraphAndValues graph_and_values = gtsam::readG2o(dataset_file_name, true);
      gtsam::Key first_key = gtsam::KeyVector(graph_and_values.second->keys()).at(0);
      anchor_offset_translation = graph_and_values.second->at<gtsam::Pose3>(first_key).translation();
      anchor_offset_rotation = graph_and_values.second->at<gtsam::Pose3>(first_key).rotation();
   }
   for (const gtsam::Values::ConstKeyValuePair &key_value : estimates.first) {
      int value_robot_id = (int)(gtsam::Symbol(key_value.key).chr() - 97);
      if (value_robot_id == prior_owner_) {
         gtsam::Pose3 value = key_value.value.cast<gtsam::Pose3>();
         anchor_offset_translation = anchor_offset_translation - value.translation();
         anchor_offset_rotation = anchor_offset_rotation * value.rotation().inverse();
         break;
      }
   }
   for (const gtsam::Values::ConstKeyValuePair &key_value : estimates.first) {
      int value_robot_id = (int)(gtsam::Symbol(key_value.key).chr() - 97);
      if (robots.find(value_robot_id) != robots.end()) {
         gtsam::Key new_key = gtsam::Symbol(((char) value_robot_id + 97), gtsam::Symbol(key_value.key).index());
         gtsam::Pose3 value = key_value.value.cast<gtsam::Pose3>();
         gtsam::Pose3 new_value = gtsam::Pose3(value.rotation(), value.translation() + anchor_offset_translation);
         new_value = new_value * gtsam::Pose3(anchor_offset_rotation, gtsam::Point3());
         centralized_values_by_robots[value_robot_id].insert(new_key, new_value);
      }
   }

   std::map<int, gtsam::Values> centralized_GN_values_by_robots;
   for (const auto& i : robots) {
      centralized_GN_values_by_robots.insert(std::make_pair(i, gtsam::Values()));
   }

   anchor_offset_translation = gtsam::Point3();
   dataset_file_name = "log/datasets/" + std::to_string(prior_owner_) + "_centralized_GN" + centralized_extension + "_incremental.g2o";
   if (boost::filesystem::exists(dataset_file_name)) {
      gtsam::GraphAndValues graph_and_values = gtsam::readG2o(dataset_file_name, true);
      gtsam::Key first_key = gtsam::KeyVector(graph_and_values.second->keys()).at(0);
      anchor_offset_translation = graph_and_values.second->at<gtsam::Pose3>(first_key).translation();
   }
   for (const gtsam::Values::ConstKeyValuePair &key_value : estimates.second) {
      int value_robot_id = (int)(gtsam::Symbol(key_value.key).chr() - 97);
      if (value_robot_id == prior_owner_) {
         gtsam::Pose3 value = key_value.value.cast<gtsam::Pose3>();
         anchor_offset_translation = anchor_offset_translation - value.translation();
         break;
      }
   }
   for (const gtsam::Values::ConstKeyValuePair &key_value : estimates.second) {
      int value_robot_id = (int)(gtsam::Symbol(key_value.key).chr() - 97);
      if (robots.find(value_robot_id) != robots.end()) {
         gtsam::Key new_key = gtsam::Symbol(((char) value_robot_id + 97), gtsam::Symbol(key_value.key).index());
         gtsam::Pose3 value = key_value.value.cast<gtsam::Pose3>();
         gtsam::Pose3 new_value = gtsam::Pose3(value.rotation(), value.translation() + anchor_offset_translation);
         centralized_GN_values_by_robots[value_robot_id].insert(new_key, new_value);
      }
   }

   std::string centralized_file_name = "log/datasets/" + std::to_string(robot_id_) + "_centralized" + centralized_extension + "_incremental.g2o";
   gtsam::writeG2o(gtsam::NonlinearFactorGraph(), centralized_values_by_robots[robot_id_], centralized_file_name);
   centralized_file_name = "log/datasets/" + std::to_string(robot_id_) + "_centralized_GN" + centralized_extension + "_incremental.g2o";
   gtsam::writeG2o(gtsam::NonlinearFactorGraph(), centralized_GN_values_by_robots[robot_id_], centralized_file_name);
   IncrementalInitialGuessUpdate(centralized_values_by_robots[robot_id_], poses_initial_guess_centralized_incremental_updates_);

}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapperWithDataset::AbortOptimization(const bool& log_info){
   CBuzzControllerQuadMapper::AbortOptimization(log_info);
   if (log_info) {
      // Initialize the set of robots on which to evaluate
      std::set<int> robots = neighbors_within_communication_range_;
      robots.insert(robot_id_);

      auto aggregated_outliers_keys = AggregateOutliersKeys(robots);
      int number_of_separators = 0;
      int number_of_outliers_not_rejected = 0;
      for (const auto& i : robots) {
         std::string dataset_file_name = "log/datasets/" + std::to_string(i) + "_initial.g2o";
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
         std::string separators_rejected_file_name = "log/datasets/" + std::to_string(i) + "_number_of_separators_rejected.g2o";
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
               << "\t" << optimizer_period_ << "\t" << confidence_probability_
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