#include "buzz_controller_quadmapper.h"
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <fstream>
#include <stdio.h>

namespace buzz_quadmapper {
/****************************************/
/****************************************/

CBuzzControllerQuadMapper::CBuzzControllerQuadMapper() :
   CBuzzControllerSpiri() {
}

/****************************************/
/****************************************/

CBuzzControllerQuadMapper::~CBuzzControllerQuadMapper() {
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::Init(TConfigurationNode& t_node) {
   CBuzzControllerSpiri::Init(t_node);
   // Initialize attributes
   number_of_poses_ = 0;
   robot_id_ = this->GetBuzzVM()->robot;
   robot_id_char_ = (unsigned char)(97 + robot_id_);
   previous_symbol_ = gtsam::Symbol(robot_id_char_, number_of_poses_);
   local_pose_graph_ = boost::make_shared< gtsam::NonlinearFactorGraph >();
   poses_initial_guess_ = boost::make_shared< gtsam::Values >();
   poses_initial_guess_->insert(previous_symbol_.key(), gtsam::Pose3());
   current_rotation_iteration_ = 0;
   current_pose_iteration_ = 0;
   is_estimation_done_ = false;
   end_delay_ = 0;
   total_outliers_rejected_ = 0;
   number_of_poses_at_optimization_end_ = 0;
   neighbor_has_started_optimization_ = false;
   has_sent_start_optimization_flag_ = false;
   previous_neighbor_id_in_optimization_order_ = robot_id_;
   is_prior_added_ = false;

   // Isotropic noise models
   Eigen::VectorXd sigmas(6);
   sigmas << rotation_noise_std_, rotation_noise_std_, rotation_noise_std_, 
            translation_noise_std_, translation_noise_std_, translation_noise_std_;
   noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
   chordal_graph_noise_model_ = gtsam::noiseModel::Isotropic::Variance(12, 1);

   // Delete existent log files
   std::remove("log/datasets/centralized.g2o");
   std::string log_file_name = "log/datasets/" + std::to_string(robot_id_) + ".g2o";
   std::remove(log_file_name.c_str());
   log_file_name = "log/datasets/" + std::to_string(robot_id_) + "_initial.g2o";
   std::remove(log_file_name.c_str());
   log_file_name = "log/datasets/" + std::to_string(robot_id_) + "_optimized.g2o";
   std::remove(log_file_name.c_str());
   log_file_name = "log/datasets/" + std::to_string(robot_id_) + "_number_of_outliers_rejected.g2o";
   std::remove(log_file_name.c_str());


}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::LoadParameters( const int& number_of_steps_before_failsafe, const bool& use_pcm,
                                                const double& confidence_probability, const bool& incremental_solving, const bool& debug,
                                                const float& rotation_noise_std, const float& translation_noise_std,
                                                const float& rotation_estimate_change_threshold, const float& pose_estimate_change_threshold,
                                                const bool& use_flagged_initialization, const bool& is_simulation,
                                                const int& number_of_robots, const std::string& error_file_name) {
   rotation_noise_std_ = rotation_noise_std;
   translation_noise_std_ = translation_noise_std;
   rotation_estimate_change_threshold_ = rotation_estimate_change_threshold;
   pose_estimate_change_threshold_ = pose_estimate_change_threshold;
   use_flagged_initialization_ = use_flagged_initialization;

   number_of_robots_ = number_of_robots;
   is_simulation_ = is_simulation;
   error_file_name_ = error_file_name;
   debug_ = debug;
   incremental_solving_ = incremental_solving;
   confidence_probability_ = confidence_probability;
   use_pcm_ = use_pcm;
   number_of_steps_before_failsafe_ = number_of_steps_before_failsafe;
}

/****************************************/
/****************************************/

OptimizerState CBuzzControllerQuadMapper::GetOptimizerState() {
   return optimizer_state_;
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::UpdateCurrentSeparatorBuzzStructure(  const int& robot_1_id,
                                             const int& robot_2_id,
                                             const int& robot_1_pose_id,
                                             const int& robot_2_pose_id,
                                             const double& x,
                                             const double& y,
                                             const double& z,
                                             const double& q_x,
                                             const double& q_y,
                                             const double& q_z,
                                             const double& q_w  ) {
   // Create empty data table
   buzzobj_t b_separator_measurement = buzzheap_newobj(m_tBuzzVM, BUZZTYPE_TABLE);
   // Store symbol information
   TablePut(b_separator_measurement, "robot_1_id", robot_1_id);
   TablePut(b_separator_measurement, "robot_2_id", robot_2_id);
   TablePut(b_separator_measurement, "robot_1_pose_id", robot_1_pose_id);
   TablePut(b_separator_measurement, "robot_2_pose_id", robot_2_pose_id);
   // Store position data
   TablePut(b_separator_measurement, "x", x);
   TablePut(b_separator_measurement, "y", y);
   TablePut(b_separator_measurement, "z", z);
   // Store orientation data
   TablePut(b_separator_measurement, "q_x", q_x);
   TablePut(b_separator_measurement, "q_y", q_y);
   TablePut(b_separator_measurement, "q_z", q_z);
   TablePut(b_separator_measurement, "q_w", q_w);
   // Register positioning data table as global symbol
   Register("current_separator_measurement", b_separator_measurement);

}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::SetNextPosition(const CVector3& translation) {

   CVector3 new_position = m_pcPos->GetReading().Position + translation;

   new_position.SetZ(2.0f); // To ensure that the quadrotor flies
   
   m_pcPropellers->SetAbsolutePosition(new_position);
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::UpdateNeighborHasStartedOptimizationFlag(const bool& neighbor_has_started_optimization) {
   neighbor_has_started_optimization_ = neighbor_has_started_optimization;
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::UpdateHasSentStartOptimizationFlag(const bool& has_sent_start_optimization_flag) {
   has_sent_start_optimization_flag_ = has_sent_start_optimization_flag;
}

/****************************************/
/****************************************/

bool CBuzzControllerQuadMapper::StartOptimizationCondition() {
   bool periodic_optimization = (number_of_poses_ - number_of_poses_at_optimization_end_) % optimizer_period_ == 0;
   return neighbor_has_started_optimization_ && has_sent_start_optimization_flag_; // TODO: Add parameter to allow periodic optimization
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::FailSafeCheck() {
   if (latest_change_ == optimizer_->latestChange()) {
      number_of_steps_without_changes_++;
   } else {
      number_of_steps_without_changes_ = 0;
      latest_change_ = optimizer_->latestChange();
   }

   if (number_of_steps_without_changes_ > number_of_steps_before_failsafe_){
      std::cout << "Robot " << robot_id_ << " Failsafe!" << std::endl;
      optimizer_state_ = OptimizerState::Idle;
   }
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::IncrementNumberOfPosesAndUpdateState() {
   number_of_poses_++;
   // Update optimizer state
   switch (optimizer_state_) {
      case Idle :
         if (StartOptimizationCondition()) {
            optimizer_state_ = OptimizerState::Start;
            current_rotation_iteration_ = 0;
            current_pose_iteration_ = 0;
            neighbors_within_communication_range_.clear();
            neighbors_rotation_estimation_phase_is_finished_.clear();
            neighbors_pose_estimation_phase_is_finished_.clear();
            neighbors_is_estimation_done_.clear();
            previous_neighbor_id_in_optimization_order_ = robot_id_;
            latest_change_ = -1;
            number_of_steps_without_changes_ = 0;
            if (is_prior_added_) {
               optimizer_->removePrior();
               is_prior_added_ = false;
            }
         }
         break;
      case Start :
         std::cout << "Robot " << robot_id_ << " Start Distributed Pose Graph Optimization" << std::endl;
         optimizer_state_ = OptimizerState::RotationEstimation;
         StartPoseGraphOptimization();
         neighbor_has_started_optimization_ = false;
         has_sent_start_optimization_flag_ = false;
         break;
      case RotationEstimation :
         current_rotation_iteration_++;
         if (RotationEstimationStoppingBarrier()) {
            // Change optimizer state
            optimizer_state_ = OptimizerState::PoseEstimationInitialization;
         } else {
            // Reinitialize neighbors states information
            SetRotationEstimationIsFinishedFlagsToFalse();
         }
         if (current_rotation_iteration_ > number_of_steps_before_failsafe_) {
            RemoveInactiveNeighbors();
         }
         FailSafeCheck();         
         break;
      case PoseEstimationInitialization :
         InitializePoseEstimation();
         // Change optimizer state
         optimizer_state_ = OptimizerState::PoseEstimation;
         break;
      case PoseEstimation :
         current_pose_iteration_++;
         if (PoseEstimationStoppingBarrier()) {
            // Change optimizer state
            optimizer_state_ = OptimizerState::End;
         } else {
            // Reinitialize neighbors states information
            SetPoseEstimationIsFinishedFlagsToFalse();
         }
         FailSafeCheck();
         break;
      case End :
         EndOptimization();
         GetLatestLocalError();
         if (is_simulation_) {
            CompareCentralizedAndDecentralizedError();
         }
         std::cout << "Robot " << robot_id_ << " End Distributed Pose Graph Optimization" << std::endl;
         optimizer_state_ = OptimizerState::PostEndingCommunicationDelay;
         number_of_poses_at_optimization_end_ = number_of_poses_;
         break;
      case PostEndingCommunicationDelay :
         optimizer_state_ = OptimizerState::Idle;
         break;
   }
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::NeighborState(const int& rid, const OptimizerState& state) {
   if (neighbors_state_.find(rid) != neighbors_state_.end()) {
      neighbors_state_[rid] = state;
   } else {
      neighbors_state_.insert(std::make_pair(rid, state));
   }
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::RemoveInactiveNeighbors() {
   std::vector<int> neighbors_to_be_removed;
   for (const auto& neighbor_id : neighbors_within_communication_range_) {
      if (neighbors_state_[neighbor_id] == OptimizerState::Idle) {
         neighbors_to_be_removed.emplace_back(neighbor_id);
      }
   }
   for (const auto& neighbor_id : neighbors_to_be_removed) {
      neighbors_within_communication_range_.erase(neighbor_id);;
      neighbors_rotation_estimation_phase_is_finished_.erase(neighbor_id);
      neighbors_pose_estimation_phase_is_finished_.erase(neighbor_id);
      neighbors_is_estimation_done_.erase(neighbor_id);
   }
   if (neighbors_within_communication_range_.empty()) {
      optimizer_state_ = OptimizerState::Idle;
   }
}

/****************************************/
/****************************************/

OptimizerPhase CBuzzControllerQuadMapper::GetOptimizerPhase() {
   if (is_estimation_done_) {
      if (debug_) {
         std::cout << "Robot " << robot_id_ << " Phase : " << OptimizerPhase::Communication << std::endl;
      }
      return OptimizerPhase::Communication;
   }
   // Smallest ID not done -> estimation
   bool smallest_id_not_done = true;
   if (robot_id_ > 0 && previous_neighbor_id_in_optimization_order_ != robot_id_) {
      auto previous_neighbor_done = neighbors_is_estimation_done_[previous_neighbor_id_in_optimization_order_];
      if (!previous_neighbor_done) {
         smallest_id_not_done = false;
      }
   }      
   auto phase = OptimizerPhase::Communication;
   if (smallest_id_not_done && !neighbors_is_estimation_done_.empty()) {
      phase = OptimizerPhase::Estimation;
   }
   if (debug_) {
      std::cout << "Robot " << robot_id_ << " Phase : " << phase << std::endl;
   }
   return phase;
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::CheckIfAllEstimationDoneAndReset() {
    // Self done + others done, reset
   if (is_estimation_done_) {
      bool all_done = true;
      for (const auto& neighbor_done : neighbors_is_estimation_done_) {
         if (neighbors_within_communication_range_.find(neighbor_done.first) != neighbors_within_communication_range_.end()) {
            all_done &= neighbor_done.second;
         }
      }
      if (all_done) {
         for (auto& neighbor_done : neighbors_is_estimation_done_) {
            neighbor_done.second = false;
         }
         is_estimation_done_ = false;
      }
   }
}

/****************************************/
/****************************************/

bool CBuzzControllerQuadMapper::AllRobotsAreInitialized() {
   bool all_robots_initialized = optimizer_->isRobotInitialized();
   for (const auto& is_robot_initialized : optimizer_->getNeighboringRobotsInit()) {
      if (neighbors_within_communication_range_.find(((int)is_robot_initialized.first-97)) != neighbors_within_communication_range_.end()) {
         all_robots_initialized &= is_robot_initialized.second;
      }
   }
   return all_robots_initialized;
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::AddSeparatorToLocalGraph( const int& robot_1_id,
                                 const int& robot_2_id,
                                 const int& robot_1_pose_id,
                                 const int& robot_2_pose_id,
                                 const double& x,
                                 const double& y,
                                 const double& z,
                                 const double& q_x,
                                 const double& q_y,
                                 const double& q_z,
                                 const double& q_w  ) {

   // Separator symbols
   unsigned char robot_1_id_char = (unsigned char)(97 + robot_1_id);
   AddNewKnownRobot(robot_1_id_char);
   gtsam::Symbol robot_1_symbol = gtsam::Symbol(robot_1_id_char, robot_1_pose_id);
   unsigned char robot_2_id_char = (unsigned char)(97 + robot_2_id);
   AddNewKnownRobot(robot_2_id_char);
   gtsam::Symbol robot_2_symbol = gtsam::Symbol(robot_2_id_char, robot_2_pose_id);

   // Translation
   gtsam::Point3 t = { x, y, z };

   // Rotation
   gtsam::Quaternion quat(q_w, q_x, q_y, q_z);
   gtsam::Rot3 R(quat);

   // Transformation
   gtsam::Pose3 transformation(R, t);

   // Factor
   gtsam::BetweenFactor<gtsam::Pose3> new_factor = gtsam::BetweenFactor<gtsam::Pose3>(robot_1_symbol, robot_2_symbol, transformation, noise_model_);
   local_pose_graph_->push_back(new_factor);
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::WriteCurrentDataset() {
   std::string dataset_file_name = "log/datasets/" + std::to_string(robot_id_) + ".g2o";
   gtsam::writeG2o(*local_pose_graph_, *poses_initial_guess_, dataset_file_name);
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::WriteOptimizedDataset() {
   std::string dataset_file_name = "log/datasets/" + std::to_string(robot_id_) + "_optimized.g2o";
   gtsam::writeG2o(local_pose_graph_before_optimization_, optimizer_->currentEstimate(), dataset_file_name);
   std::string outliers_rejected_file_name = "log/datasets/" + std::to_string(robot_id_) + "_number_of_outliers_rejected.g2o";
   std::ofstream outliers_rejected_file;
   outliers_rejected_file.open(outliers_rejected_file_name, std::ios::trunc);
   outliers_rejected_file << total_outliers_rejected_ << "\n" ;
   outliers_rejected_file.close();
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::WriteInitialDataset() {
   std::string dataset_file_name = "log/datasets/" + std::to_string(robot_id_) + "_initial.g2o";
   gtsam::writeG2o(*local_pose_graph_, *poses_initial_guess_, dataset_file_name);
}


/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::AddNewKnownRobot(const unsigned char& other_robot_char) {
   if (other_robot_char != robot_id_char_) {
      if (std::find(known_other_robots_.begin(), known_other_robots_.end(), other_robot_char) == known_other_robots_.end()) {
         known_other_robots_.insert(other_robot_char);
      }
   }
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::InitOptimizer(const int& period) {

   optimizer_ = boost::shared_ptr<distributed_mapper::DistributedMapper>(new distributed_mapper::DistributedMapper(robot_id_char_));

   graph_and_values_ = std::make_pair(local_pose_graph_, poses_initial_guess_);

   // Use between noise or not in optimizePoses
   optimizer_->setUseBetweenNoiseFlag(false);

   // Use landmarks
   optimizer_->setUseLandmarksFlag(false);

   // Load subgraphs
   optimizer_->loadSubgraphAndCreateSubgraphEdge(graph_and_values_);

   // Verbosity level
   optimizer_->setVerbosity(distributed_mapper::DistributedMapper::ERROR);

   disconnected_graph_ = true;

   optimizer_->setFlaggedInit(use_flagged_initialization_);
   
   optimizer_->setUpdateType(distributed_mapper::DistributedMapper::incUpdate);
   
   optimizer_->setGamma(1.0f);

   optimizer_state_ = OptimizerState::Idle;

   optimizer_period_ = period;

   rotation_estimation_phase_is_finished_ = false;

   pose_estimation_phase_is_finished_ = false;

   is_estimation_done_ = false;
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::StartPoseGraphOptimization() {

   UpdateOptimizer();

   OutliersFiltering();

   SaveInitialGraph();

   optimizer_->updateInitialized(false);
   optimizer_->clearNeighboringRobotInit();

}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::SaveInitialGraph() {

   // Save state before optimization
   local_pose_graph_before_optimization_ = *local_pose_graph_;
   WriteInitialDataset();

}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::UpdateOptimizer() {
   
   // Load subgraphs
   optimizer_->loadSubgraphAndCreateSubgraphEdge(graph_and_values_);

   // Add prior to the first robot
   bool has_smallest_id = true;
   for (const auto& neighbor : neighbors_within_communication_range_) {
      if (neighbor < robot_id_) {
         has_smallest_id = false;
      }
   }
   if (has_smallest_id) {
      gtsam::Key first_key = gtsam::KeyVector(poses_initial_guess_->keys()).at(0);
      optimizer_->addPrior(first_key, poses_initial_guess_->at<gtsam::Pose3>(first_key), noise_model_);
      is_prior_added_ = true;
   }
   
   // Check for graph connectivity
   std::set<char> neighboring_robots = optimizer_->getNeighboringChars();
   if (neighboring_robots.size() > 0) {
      disconnected_graph_ = false;
   }
   bool has_separator_with_neighbor = false;
   for (const auto& neighbor : neighbors_within_communication_range_) {
      if (neighboring_robots.find((char)(neighbor+97)) != neighboring_robots.end()) {
         has_separator_with_neighbor = true;
      }
   }
   if (!has_separator_with_neighbor) {
      optimizer_state_ = OptimizerState::Idle;
   }   
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::OutliersFiltering() {
   
   if (use_pcm_) {
      // Perform pairwise consistency maximization
      int total_max_clique_size = 0;
      for (const auto& robot : neighbors_within_communication_range_) {
         gtsam::Values other_robot_poses;
         for (const auto& estimate_pair : pose_estimates_from_neighbors_.at(robot)) {
            other_robot_poses.insert(estimate_pair.first, estimate_pair.second);
         }      
         bool use_covariance = false;
         auto max_clique_info = distributed_pcm::DistributedPCM::solveDecentralized(robot, optimizer_,
                                 graph_and_values_, other_robot_poses,
                                 confidence_probability_, use_covariance);
         total_max_clique_size += max_clique_info.first;
         total_outliers_rejected_ += max_clique_info.second;
      }

      if (debug_) {
         std::cout << "Robot " << robot_id_ << " Outliers filtering, total max clique size=" << total_max_clique_size 
                   << ", number of outliers rejected=" << total_outliers_rejected_ << std::endl;
      }
   }

}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::UpdateCurrentPoseEstimate(const int& pose_id) {
   buzzobj_t b_pose_estimate = buzzheap_newobj(m_tBuzzVM, BUZZTYPE_TABLE);
   auto pose = poses_initial_guess_->at<gtsam::Pose3>(gtsam::Symbol(robot_id_char_, pose_id).key());
   auto pose_matrix = pose.matrix();
   for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
         double matrix_elem = pose_matrix(i, j);
         TablePut(b_pose_estimate, i*4 + j, matrix_elem);
      }
   }
   // Register pose estimate data table as a global symbol
   Register("current_pose_estimate", b_pose_estimate);
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::UpdatePoseEstimateFromNeighbor(const int& rid, const int& pose_id, const gtsam::Pose3& pose) {
   auto key = gtsam::Symbol(((char)rid+97), pose_id).key();
   if (pose_estimates_from_neighbors_.find(rid) != pose_estimates_from_neighbors_.end()) {
      if (pose_estimates_from_neighbors_.at(rid).find(key) != pose_estimates_from_neighbors_.at(rid).end()) {
         pose_estimates_from_neighbors_.at(rid).at(key) = pose;
      } else {
         pose_estimates_from_neighbors_.at(rid).insert(std::make_pair(key, pose));
      }
   } else {
      std::map<gtsam::Key, gtsam::Pose3> new_map;
      new_map.insert(std::make_pair(key, pose));
      pose_estimates_from_neighbors_.insert(std::make_pair(rid, new_map));
   }
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::AddNeighborWithinCommunicationRange(const int& rid) {
   neighbors_within_communication_range_.insert(rid);
   neighbors_rotation_estimation_phase_is_finished_.insert(std::make_pair(rid, false));
   neighbors_pose_estimation_phase_is_finished_.insert(std::make_pair(rid, false));
   neighbors_is_estimation_done_.insert(std::make_pair(rid, false));
   if (previous_neighbor_id_in_optimization_order_ == robot_id_ && rid < robot_id_) {
      previous_neighbor_id_in_optimization_order_ = rid;
   } else if (previous_neighbor_id_in_optimization_order_ < rid && rid < robot_id_) {
      previous_neighbor_id_in_optimization_order_ = rid;
   }
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::ComputeAndUpdateRotationEstimatesToSend(const int& rid) {

   // Mark the robot with the lowest id as initialized
   if (use_flagged_initialization_ && !optimizer_->isRobotInitialized()) {
      bool is_initialized = true;
      for (const auto& neighbor : neighbors_within_communication_range_) {
         if (neighbor < robot_id_) {
            is_initialized = false;
         }
      }
      optimizer_->updateInitialized(is_initialized);
   }

   // Create empty data table
   buzzobj_t b_rotation_estimates = buzzheap_newobj(m_tBuzzVM, BUZZTYPE_TABLE);

   // for each loop closure
   // share Key, linearized rotation, is init
   int table_size = 0;
   for (const std::pair<gtsam::Symbol, gtsam::Symbol>& separator_symbols: optimizer_->separatorsSymbols()) {

      gtsam::Symbol other_robot_symbol = separator_symbols.first;
      int other_robot_id = (int)(other_robot_symbol.chr() - 97);

      if (rid == other_robot_id) {
         bool optimizer_is_initialized = optimizer_->isRobotInitialized();

         buzzobj_t b_individual_estimate = buzzheap_newobj(m_tBuzzVM, BUZZTYPE_TABLE);

         TablePut(b_individual_estimate, "sender_robot_id", robot_id_);
         int robot_pose_id = separator_symbols.second.index();
         TablePut(b_individual_estimate, "sender_pose_id", robot_pose_id);
         TablePut(b_individual_estimate, "receiver_robot_id", other_robot_id);
         TablePut(b_individual_estimate, "sender_robot_is_initialized", (int) optimizer_is_initialized);
         TablePut(b_individual_estimate, "sender_estimation_is_done", (int) is_estimation_done_);

         gtsam::Vector rotation_estimate = optimizer_->linearizedRotationAt(separator_symbols.second.key());
         buzzobj_t b_individual_estimate_rotation = buzzheap_newobj(m_tBuzzVM, BUZZTYPE_TABLE);

         for (int rotation_elem_index = 0; rotation_elem_index < 9; rotation_elem_index++) {
            TablePut(b_individual_estimate_rotation, rotation_elem_index, rotation_estimate[rotation_elem_index]);
         }         

         TablePut(b_individual_estimate, "rotation_estimate", b_individual_estimate_rotation);

         TablePut(b_rotation_estimates, table_size, b_individual_estimate);
         table_size++;
      }

   }

   // Register positioning data table as a global symbol
   Register("rotation_estimates_to_send", b_rotation_estimates);
   if (debug_) {
      std::cout << "Robot " << robot_id_ << " Rotation Send" << std::endl;
   }
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::UpdateNeighborRotationEstimates(const std::vector<std::vector<rotation_estimate_t>>& rotation_estimates_from_all_robot) {
   if (debug_) {
      std::cout << "Robot " << robot_id_ << " Rotation Receive, State = " << optimizer_state_ << std::endl;
   }
   for (const auto& rotation_estimates_from_one_robot : rotation_estimates_from_all_robot) {
      for (const auto& rotation_estimate : rotation_estimates_from_one_robot) {
         if (rotation_estimate.receiver_robot_id == robot_id_ &&
            neighbors_within_communication_range_.find(rotation_estimate.sender_robot_id) != neighbors_within_communication_range_.end()) {
            gtsam::Symbol symbol((unsigned char)(rotation_estimate.sender_robot_id+97), rotation_estimate.sender_pose_id);
            gtsam::Vector rotation_matrix_vector(9);
            rotation_matrix_vector << rotation_estimate.rotation_matrix[0], rotation_estimate.rotation_matrix[1], rotation_estimate.rotation_matrix[2], 
                                    rotation_estimate.rotation_matrix[3], rotation_estimate.rotation_matrix[4], rotation_estimate.rotation_matrix[5],
                                    rotation_estimate.rotation_matrix[6], rotation_estimate.rotation_matrix[7], rotation_estimate.rotation_matrix[8];
            optimizer_->updateNeighborLinearizedRotations(symbol.key(), rotation_matrix_vector);
            if (optimizer_state_ == OptimizerState::RotationEstimation) {
               optimizer_->updateNeighboringRobotInitialized(symbol.chr(), rotation_estimate.sender_robot_is_initialized); // Used only with flagged initialization
            }
         }
         if (optimizer_state_ == OptimizerState::RotationEstimation) { // && neighbors_within_communication_range_.find(rotation_estimate.sender_robot_id) != neighbors_within_communication_range_.end()
            neighbors_is_estimation_done_[rotation_estimate.sender_robot_id] = rotation_estimate.sender_estimation_is_done;
         }
      }
   }
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::EstimateRotationAndUpdateRotation(){
   if (optimizer_state_ == OptimizerState::RotationEstimation) {
      optimizer_->estimateRotation();
      optimizer_->updateRotation();
      optimizer_->updateInitialized(true);
      is_estimation_done_ = true;
      if (debug_) {
         std::cout << "Robot " << robot_id_ << " Rotation estimation" << std::endl;
      }
   }
}

/****************************************/
/****************************************/

bool CBuzzControllerQuadMapper::RotationEstimationStoppingConditions() {
   // Stopping condition
   rotation_estimation_phase_is_finished_ = false;
   double change = optimizer_->latestChange();
   if (debug_) {
      std::cout << "[optimize rotation] Change (Robot " << robot_id_ << "): " << change << std::endl;
   }
   if((!use_flagged_initialization_ || AllRobotsAreInitialized()) && change < rotation_estimate_change_threshold_ && current_rotation_iteration_ != 0) {
      rotation_estimation_phase_is_finished_ = true;
   }
   return rotation_estimation_phase_is_finished_;
}

/****************************************/
/****************************************/

bool CBuzzControllerQuadMapper::RotationEstimationStoppingBarrier() {
   bool all_other_finished_rotation_estimation = true;
   for (const auto& neighbor : neighbors_within_communication_range_) {
      all_other_finished_rotation_estimation &= (neighbors_state_[neighbor] == OptimizerState::PoseEstimation);
   } 
   if (all_other_finished_rotation_estimation) { 
      // If others have finished the rotation estimation, this robot should too. 
      return true;
   }
   bool stop_rotation_estimation = rotation_estimation_phase_is_finished_;
   for (const auto& is_finished : neighbors_rotation_estimation_phase_is_finished_) {
      stop_rotation_estimation &= is_finished.second;
   }   
   return stop_rotation_estimation;
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::InitializePoseEstimation() {
   optimizer_->convertLinearizedRotationToPoses();
   gtsam::Values neighbors = optimizer_->neighbors();
   for(const gtsam::Values::ConstKeyValuePair& key_value: neighbors){
      gtsam::Key key = key_value.key;
      // Pick linear rotation estimate from *robot*
      gtsam::VectorValues lin_rot_estimate_neighbor;
      lin_rot_estimate_neighbor.insert( key,  optimizer_->neighborsLinearizedRotationsAt(key) );
      // Make a pose out of it
      gtsam::Values rot_estimate_neighbor = gtsam::InitializePose3::normalizeRelaxedRotations(lin_rot_estimate_neighbor);
      gtsam::Values pose_estimate_neighbor = distributed_mapper::evaluation_utils::pose3WithZeroTranslation(rot_estimate_neighbor);
      // Store it
      optimizer_->updateNeighbor(key, pose_estimate_neighbor.at<gtsam::Pose3>(key));
   }

   // Reset flags for flagged initialization.
   optimizer_->updateInitialized(false);
   optimizer_->clearNeighboringRobotInit();
   is_estimation_done_ = false;
   for (auto& neighbor_done : neighbors_is_estimation_done_) {
      neighbor_done.second = false;
   }
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::ComputeAndUpdatePoseEstimatesToSend(const int& rid) {

   // Mark the robot with the lowest id as initialized
   if (use_flagged_initialization_ && !optimizer_->isRobotInitialized()) {
      bool is_initialized = true;
      for (const auto& neighbor : neighbors_within_communication_range_) {
         if (neighbor < robot_id_) {
            is_initialized = false;
         }
      }
      optimizer_->updateInitialized(is_initialized);
   }

   // Create empty data table
   buzzobj_t b_pose_estimates = buzzheap_newobj(m_tBuzzVM, BUZZTYPE_TABLE);

   // for each loop closure
   //  Key, linearized pose, is init
   int table_size = 0;
   for (const std::pair<gtsam::Symbol, gtsam::Symbol>& separator_symbols: optimizer_->separatorsSymbols()) {

      gtsam::Symbol other_robot_symbol = separator_symbols.first;
      int other_robot_id = (int)(other_robot_symbol.chr() - 97);

      if (rid == other_robot_id) {
         bool optimizer_is_initialized = optimizer_->isRobotInitialized();

         buzzobj_t b_individual_estimate = buzzheap_newobj(m_tBuzzVM, BUZZTYPE_TABLE);

         TablePut(b_individual_estimate, "sender_robot_id", robot_id_);
         int robot_pose_id = separator_symbols.second.index();
         TablePut(b_individual_estimate, "sender_pose_id", robot_pose_id);
         TablePut(b_individual_estimate, "receiver_robot_id", other_robot_id);
         TablePut(b_individual_estimate, "sender_robot_is_initialized", (int) optimizer_is_initialized);
         TablePut(b_individual_estimate, "sender_estimation_is_done", (int) is_estimation_done_);

         gtsam::Vector pose_estimate = optimizer_->linearizedPosesAt(separator_symbols.second.key());
         buzzobj_t b_individual_estimate_pose = buzzheap_newobj(m_tBuzzVM, BUZZTYPE_TABLE);

         for (int pose_elem_index = 0; pose_elem_index < 6; pose_elem_index++) {
            TablePut(b_individual_estimate_pose, pose_elem_index, pose_estimate[pose_elem_index]);
         }         

         TablePut(b_individual_estimate, "pose_estimate", b_individual_estimate_pose);

         TablePut(b_pose_estimates, table_size, b_individual_estimate);
         table_size++;

         gtsam::Symbol debug_symbol(robot_id_char_, robot_pose_id);
      }

   }

   // Register positioning data table as global symbol
   Register("pose_estimates_to_send", b_pose_estimates);
   if (debug_) {
      std::cout << "Robot " << robot_id_ << " Pose Send" << std::endl;
   }
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::UpdateNeighborPoseEstimates(const std::vector<std::vector<pose_estimate_t>>& pose_estimates_from_all_robot) {
   if (debug_) {
      std::cout << "Robot " << robot_id_ << " Pose Receive, State = " << optimizer_state_ << std::endl;
   }
   for (const auto& pose_estimates_from_one_robot : pose_estimates_from_all_robot) {
      for (const auto& pose_estimate : pose_estimates_from_one_robot) {
         if (pose_estimate.receiver_robot_id == robot_id_ &&
            neighbors_within_communication_range_.find(pose_estimate.sender_robot_id) != neighbors_within_communication_range_.end()) {
            gtsam::Symbol symbol((unsigned char)(pose_estimate.sender_robot_id+97), pose_estimate.sender_pose_id);
            gtsam::Vector pose_data_vector(6);
            pose_data_vector << pose_estimate.pose_data[0], pose_estimate.pose_data[1], pose_estimate.pose_data[2], 
                                    pose_estimate.pose_data[3], pose_estimate.pose_data[4], pose_estimate.pose_data[5];
            optimizer_->updateNeighborLinearizedPoses(symbol.key(), pose_data_vector);
            if (optimizer_state_ == OptimizerState::PoseEstimation) {
               optimizer_->updateNeighboringRobotInitialized(symbol.chr(), pose_estimate.sender_robot_is_initialized); // Used only with flagged initialization
            }
         }
         if (optimizer_state_ == OptimizerState::PoseEstimation) { //  && neighbors_within_communication_range_.find(pose_estimate.sender_robot_id) != neighbors_within_communication_range_.end()
            neighbors_is_estimation_done_[pose_estimate.sender_robot_id] = pose_estimate.sender_estimation_is_done;
         }
      }
   }
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::EstimatePoseAndUpdatePose(){
   optimizer_->estimatePoses();
   optimizer_->updatePoses();
   optimizer_->updateInitialized(true);
   is_estimation_done_ = true;
   if (debug_) {
      std::cout << "Robot " << robot_id_ << " Pose estimation" << std::endl;
   }
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::NeighborRotationEstimationIsFinished(const int& rid) {
   if ( neighbors_within_communication_range_.find(rid) != neighbors_within_communication_range_.end() ) {
      neighbors_rotation_estimation_phase_is_finished_.at(rid) = true;
   }
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::NeighborPoseEstimationIsFinished(const int& rid) {
   if ( neighbors_within_communication_range_.find(rid) != neighbors_within_communication_range_.end() ) {
      neighbors_pose_estimation_phase_is_finished_.at(rid) = true;
   }
}

/****************************************/
/****************************************/

bool CBuzzControllerQuadMapper::PoseEstimationStoppingConditions() {
   // Stopping condition
   pose_estimation_phase_is_finished_ = false;
   double change = optimizer_->latestChange();
   if (debug_) {
      std::cout << "[optimize pose] Change (Robot " << robot_id_ << "): " << change << std::endl;
   }
   if((!use_flagged_initialization_ || AllRobotsAreInitialized()) && change < pose_estimate_change_threshold_ && current_pose_iteration_ != 0) {
      pose_estimation_phase_is_finished_ = true;
   }
   return pose_estimation_phase_is_finished_;
}

/****************************************/
/****************************************/

bool CBuzzControllerQuadMapper::PoseEstimationStoppingBarrier() {
   bool all_other_finished_pose_estimation = true;
   for (const auto& neighbor : neighbors_within_communication_range_) {
      bool other_robot_finished = (neighbors_state_[neighbor] != OptimizerState::PoseEstimation) &&
                                  (neighbors_state_[neighbor] != OptimizerState::PoseEstimationInitialization) &&
                                  (neighbors_state_[neighbor] != OptimizerState::RotationEstimation);
      all_other_finished_pose_estimation &= other_robot_finished;
   } 

   if (all_other_finished_pose_estimation) { 
      // If others have finished the pose estimation, this robot should too. 
      return true;
   }
   bool stop_pose_estimation = pose_estimation_phase_is_finished_;
   for (const auto& is_finished : neighbors_pose_estimation_phase_is_finished_) {
      stop_pose_estimation &= is_finished.second;
   }   
   return stop_pose_estimation;
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::SetRotationEstimationIsFinishedFlagsToFalse() {
   for (auto& is_finished : neighbors_rotation_estimation_phase_is_finished_) {
      is_finished.second = false;
   }
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::SetPoseEstimationIsFinishedFlagsToFalse() {
   for (auto& is_finished : neighbors_pose_estimation_phase_is_finished_) {
      is_finished.second = false;
   }
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::EndOptimization() {
   optimizer_->retractPose3Global();
   if (incremental_solving_) {
      poses_initial_guess_->update(optimizer_->currentEstimate());

      for (auto factor : *local_pose_graph_) {
         auto between_factor = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factor);
         auto first_key = between_factor->key1();
         auto second_key = between_factor->key2();
         if (!optimizer_->currentEstimate().exists(second_key) && gtsam::Symbol(second_key).chr() == robot_id_char_) {
            // Get previous pose
            auto previous_pose = poses_initial_guess_->at<gtsam::Pose3>(first_key);

            // Compose previous pose and measurement
            auto current_pose = previous_pose * between_factor->measured();

            // Update pose in initial guess
            poses_initial_guess_->update(second_key ,current_pose);
         }
      }
   }
   WriteOptimizedDataset();
}

/****************************************/
/****************************************/

double CBuzzControllerQuadMapper::GetLatestLocalError() {
   // We need the aggregate values for the total error, but we can get the local one.
   std::pair<double, double> errors = optimizer_->latestError();
   // Returns pose error
   return errors.second;
}

/****************************************/
/****************************************/

bool CBuzzControllerQuadMapper::CompareCentralizedAndDecentralizedError() {

}

}