#include "buzz_controller_quadmapper.h"
#include <iostream>
#include <stdlib.h>
#include <time.h>

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
   // Initialize constant attributes // TODO: add paramaters for them or get them by buzz
   rotation_noise_std_ = 0.01;
   translation_noise_std_ = 0.1;
   rotation_estimate_change_threshold_ = 1e-1;
   pose_estimate_change_threshold_ = 1e-1;
   use_flagged_initialization_ = false;

   // Initialize attributes
   number_of_poses_ = 0;
   robot_id_ = this->GetBuzzVM()->robot;
   robot_id_char_ = (unsigned char)(97 + robot_id_);
   previous_symbol_ = gtsam::Symbol(robot_id_char_, number_of_poses_);
   previous_pose_ = gtsam::Pose3();
   local_pose_graph_ = boost::make_shared< gtsam::NonlinearFactorGraph >();
   poses_initial_guess_ = boost::make_shared< gtsam::Values >();
   poses_initial_guess_->insert(previous_symbol_.key(), previous_pose_);
   current_rotation_iteration_ = 0;
   current_pose_iteration_ = 0;

   // Isotropic noise models
   Eigen::VectorXd sigmas(6);
   sigmas << rotation_noise_std_, rotation_noise_std_, rotation_noise_std_, 
            translation_noise_std_, translation_noise_std_, translation_noise_std_;
   noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
   chordal_graph_noise_model_ = gtsam::noiseModel::Isotropic::Variance(12, 1);
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

void CBuzzControllerQuadMapper::IncrementNumberOfPosesAndUpdateState() {
   number_of_poses_++;
   // Update optimizer state
   switch (optimizer_state_) {
      case Idle :
         if (number_of_poses_ % optimizer_period_ == 0) {
            optimizer_state_ = OptimizerState::Start;
            current_rotation_iteration_ = 0;
            current_pose_iteration_ = 0;
            neighbors_within_communication_range_.clear();
            neighbors_rotation_estimation_phase_is_finished_.clear();
            neighbors_pose_estimation_phase_is_finished_.clear();
         }
         break;
      case Start :
         StartPoseGraphOptimization();
         optimizer_state_ = OptimizerState::RotationEstimation;
         break;
      case RotationEstimation :
         current_rotation_iteration_++;
         if (RotationEstimationStoppingBarrier()) {
            // Change optimizer state
            optimizer_state_ = OptimizerState::PoseEstimation;
            InitializePoseEstimation();
         } else {
            // Reinitialize neighbors states information
            SetRotationEstimationIsFinishedFlagsToFalse();
         }
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
         break;
      case End :
         EndOptimization();
         GetLatestLocalError();
         optimizer_state_ = OptimizerState::Idle;
         break;
   }
}

/****************************************/
/****************************************/

bool CBuzzControllerQuadMapper::IsAllowedToEstimate() {
   if (!use_flagged_initialization_) {
      return true;
   }
   bool is_allowed_to_estimate = true;
   for (const auto& neighbor_init : optimizer_->getNeighboringRobotsInit()) {
      if (!neighbor_init.second && ((((int) neighbor_init.first) - 97) < robot_id_)) {
         is_allowed_to_estimate = false;
      }
   }
   return is_allowed_to_estimate;
}

/****************************************/
/****************************************/

bool CBuzzControllerQuadMapper::AllRobotsAreInitialized() {
   bool all_robots_initialized = optimizer_->isRobotInitialized();
   for (const auto& is_robot_initialized : optimizer_->getNeighboringRobotsInit()) {
      all_robots_initialized &= is_robot_initialized.second;
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

void CBuzzControllerQuadMapper::WriteDataset(const uint16_t& robot_id) {
   std::string dataset_file_name = "log/datasets/" + std::to_string(robot_id) + ".g2o";
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

   // Add prior to the first robot
   if (robot_id_ == 0) {
      gtsam::Key first_key = gtsam::KeyVector(poses_initial_guess_->keys()).at(0);
      optimizer_->addPrior(first_key, poses_initial_guess_->at<gtsam::Pose3>(first_key), noise_model_);
   }

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
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::StartPoseGraphOptimization() {

   UpdateOptimizer();

   OutliersFiltering();

   optimizer_->updateInitialized(false);
   optimizer_->clearNeighboringRobotInit();

}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::UpdateOptimizer() {
   
   // Load subgraphs
   optimizer_->loadSubgraphAndCreateSubgraphEdge(graph_and_values_);

   // Add prior to the first robot
   if (robot_id_ == 0) {
      gtsam::Key first_key = gtsam::KeyVector(poses_initial_guess_->keys()).at(0);
      optimizer_->addPrior(first_key, poses_initial_guess_->at<gtsam::Pose3>(first_key), noise_model_);
   }
   
   // Check for graph connectivity
   std::set<char> neighboring_robots = optimizer_->getNeighboringChars();
   if (neighboring_robots.size() > 0) {
      disconnected_graph_ = false;
   }
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::OutliersFiltering() {
   
   // Perform pairwise consistency maximization

}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::AddNeighborWithinCommunicationRange(const int& rid) {
   neighbors_within_communication_range_.emplace_back(rid);
   neighbors_rotation_estimation_phase_is_finished_.insert(std::make_pair(rid, false));
   neighbors_pose_estimation_phase_is_finished_.insert(std::make_pair(rid, false));
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::ComputeAndUpdateRotationEstimatesToSend(const int& rid) {

   // Create empty data table
   buzzobj_t b_rotation_estimates = buzzheap_newobj(m_tBuzzVM, BUZZTYPE_TABLE);

   // for each loop closure
   //  Key, linearized rotation, is init
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
         TablePut(b_individual_estimate, "sender_robot_is_initialized", (int) optimizer_is_initialized);

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

   // Register positioning data table as global symbol
   Register("rotation_estimates_to_send", b_rotation_estimates);
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::UpdateNeighborRotationEstimates(const std::vector<std::vector<rotation_estimate_t>>& rotation_estimates_from_all_robot) {
   for (const auto& rotation_estimates_from_one_robot : rotation_estimates_from_all_robot) {
      for (const auto& rotation_estimate : rotation_estimates_from_one_robot) {
         gtsam::Symbol symbol((unsigned char)(rotation_estimate.sender_robot_id+97), rotation_estimate.sender_pose_id);
         gtsam::Vector rotation_matrix_vector(9);
         rotation_matrix_vector << rotation_estimate.rotation_matrix[0], rotation_estimate.rotation_matrix[1], rotation_estimate.rotation_matrix[2], 
                                 rotation_estimate.rotation_matrix[3], rotation_estimate.rotation_matrix[4], rotation_estimate.rotation_matrix[5],
                                 rotation_estimate.rotation_matrix[6], rotation_estimate.rotation_matrix[7], rotation_estimate.rotation_matrix[8];
         optimizer_->updateNeighborLinearizedRotations(symbol.key(), rotation_matrix_vector);
         optimizer_->updateNeighboringRobotInitialized(symbol.chr(), rotation_estimate.sender_robot_is_initialized); // Used only with flagged initialization
      }
   }
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::EstimateRotationAndUpdateRotation(){
   optimizer_->estimateRotation();
   optimizer_->updateRotation();
   optimizer_->updateInitialized(true);
}

/****************************************/
/****************************************/

bool CBuzzControllerQuadMapper::RotationEstimationStoppingConditions() {
   // Stopping condition
   rotation_estimation_phase_is_finished_ = false;
   double change = optimizer_->latestChange();
   std::cout << "[optimize rotation] Change (Robot " << robot_id_ << "): " << change << std::endl;
   if((!use_flagged_initialization_ || AllRobotsAreInitialized()) && change < rotation_estimate_change_threshold_ && current_rotation_iteration_ != 0) {
      rotation_estimation_phase_is_finished_ = true;
   }
   return rotation_estimation_phase_is_finished_;
}

/****************************************/
/****************************************/

bool CBuzzControllerQuadMapper::RotationEstimationStoppingBarrier() {
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
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::ComputeAndUpdatePoseEstimatesToSend(const int& rid) {

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
         TablePut(b_individual_estimate, "sender_robot_is_initialized", (int) optimizer_is_initialized);

         gtsam::Vector pose_estimate = optimizer_->linearizedPosesAt(separator_symbols.second.key());
         buzzobj_t b_individual_estimate_pose = buzzheap_newobj(m_tBuzzVM, BUZZTYPE_TABLE);

         for (int pose_elem_index = 0; pose_elem_index < 6; pose_elem_index++) {
            TablePut(b_individual_estimate_pose, pose_elem_index, pose_estimate[pose_elem_index]);
         }         

         TablePut(b_individual_estimate, "pose_estimate", b_individual_estimate_pose);

         TablePut(b_pose_estimates, table_size, b_individual_estimate);
         table_size++;
      }

   }

   // Register positioning data table as global symbol
   Register("pose_estimates_to_send", b_pose_estimates);
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::UpdateNeighborPoseEstimates(const std::vector<std::vector<pose_estimate_t>>& pose_estimates_from_all_robot) {
   for (const auto& pose_estimates_from_one_robot : pose_estimates_from_all_robot) {
      for (const auto& pose_estimate : pose_estimates_from_one_robot) {
         gtsam::Symbol symbol((unsigned char)(pose_estimate.sender_robot_id+97), pose_estimate.sender_pose_id);
         gtsam::Vector pose_data_vector(6);
         pose_data_vector << pose_estimate.pose_data[0], pose_estimate.pose_data[1], pose_estimate.pose_data[2], 
                                 pose_estimate.pose_data[3], pose_estimate.pose_data[4], pose_estimate.pose_data[5];
         optimizer_->updateNeighborLinearizedPoses(symbol.key(), pose_data_vector);
         optimizer_->updateNeighboringRobotInitialized(symbol.chr(), pose_estimate.sender_robot_is_initialized); // Used only with flagged initialization
      }
   }
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::EstimatePoseAndUpdatePose(){
   optimizer_->estimatePoses();
   optimizer_->updatePoses();
   optimizer_->updateInitialized(true);
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::NeighborRotationEstimationIsFinished(const int& rid) {
   neighbors_rotation_estimation_phase_is_finished_.at(rid) = true;
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::NeighborPoseEstimationIsFinished(const int& rid) {
   neighbors_pose_estimation_phase_is_finished_.at(rid) = true;
}

/****************************************/
/****************************************/

bool CBuzzControllerQuadMapper::PoseEstimationStoppingConditions() {
   // Stopping condition
   pose_estimation_phase_is_finished_ = false;
   double change = optimizer_->latestChange();
   std::cout << "[optimize pose] Change (Robot " << robot_id_ << "): " << change << std::endl;
   if((!use_flagged_initialization_ || AllRobotsAreInitialized()) && change < pose_estimate_change_threshold_ && current_pose_iteration_ != 0) {
      pose_estimation_phase_is_finished_ = true;
   }
   return pose_estimation_phase_is_finished_;
}

/****************************************/
/****************************************/

bool CBuzzControllerQuadMapper::PoseEstimationStoppingBarrier() {
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
   poses_initial_guess_->update(optimizer_->currentEstimate());
}

/****************************************/
/****************************************/

double CBuzzControllerQuadMapper::GetLatestLocalError() {
   // We need the aggregate values for the total error, but we can get the local one.
   std::pair<double, double> errors = optimizer_->latestError();
   // Returns pose error
   std::cout << "[optimize pose] Final Error (Robot " << robot_id_ << "): " << errors.second << std::endl;
   return errors.second;
}

}