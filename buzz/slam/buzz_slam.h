#ifndef BUZZ_SLAM_H
#define BUZZ_SLAM_H

#include <buzz/buzzvm.h>
#include <buzz/buzzdebug.h>
#include <string>
#include <list>
#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/graph.h>
#include <boost/make_shared.hpp>
#include <random>
#include <cmath>
#include <distributed_mapper/distributed_mapper.h>
#include <distributed_pcm/distributed_pcm.h>

namespace buzz_slam {
/*
*  Enum of all the possible state of the optimizer
*/
enum OptimizerState { Idle, Start, Initialization, RotationEstimation, PoseEstimationInitialization, PoseEstimation, End, PostEndingCommunicationDelay };

/*
*  Enum of the phases during the distributed optimization
*/
enum OptimizerPhase { Communication, Estimation };

/*
*  Rotation estimate message
*/
typedef struct {
   int sender_robot_id;
   int receiver_robot_id;
   int sender_pose_id;
   bool sender_robot_is_initialized;
   bool sender_estimation_is_done;
   double rotation_matrix[9];
} rotation_estimate_t;

/*
*  Pose estimate message
*/
typedef struct {
   int sender_robot_id;
   int receiver_robot_id;
   int sender_pose_id;
   bool sender_robot_is_initialized;
   bool sender_estimation_is_done;
   double pose_data[6];
} pose_estimate_t;

/*
* Buzz SLAM class, should be used as a base class for the controller.
*/
class BuzzSLAM {

public:

   BuzzSLAM();
   
   virtual ~BuzzSLAM();

   void Init(buzzvm_t buzz_vm);

   void LoadParameters( const int& period, const int& number_of_steps_before_failsafe, const bool& use_pcm,
                        const double& confidence_probability, const bool& incremental_solving, const int& debug,
                        const float& rotation_noise_std, const float& translation_noise_std,
                        const float& rotation_estimate_change_threshold, const float& pose_estimate_change_threshold,
                        const bool& use_flagged_initialization, const bool& is_simulation,
                        const int& number_of_robots, const std::string& error_file_name,
                        const int& max_number_of_rotation_estimation_steps, const int& max_number_of_pose_estimation_steps);

   // Functions related to the measurements
   void AddSeparatorToLocalGraph( const int& robot_1_id,
                                    const int& robot_2_id,
                                    const int& robot_1_pose_id,
                                    const int& robot_2_pose_id,
                                    const double& x,
                                    const double& y,
                                    const double& z,
                                    const double& q_x,
                                    const double& q_y,
                                    const double& q_z,
                                    const double& q_w,
                                    const gtsam::Matrix6& covariance_matrix );

   // Functions related to distributed_mapper
   void InitOptimizer();

   void OptimizerTick();

   void AddNeighborWithinCommunicationRange(const int& rid);

   OptimizerState GetOptimizerState();

   void ComputeAndUpdateRotationEstimatesToSend(const int& rid);

   void ComputeAndUpdatePoseEstimatesToSend(const int& rid);

   void UpdateNeighborRotationEstimates(const std::vector<std::vector<rotation_estimate_t>>& rotation_estimates);

   void UpdateNeighborPoseEstimates(const std::vector<std::vector<pose_estimate_t>>& pose_estimates);
   
   void EstimateRotationAndUpdateRotation();

   void EstimatePoseAndUpdatePose();

   bool RotationEstimationStoppingConditions();

   bool PoseEstimationStoppingConditions();

   void NeighborRotationEstimationIsFinished(const int& rid);

   void NeighborPoseEstimationIsFinished(const int& rid, const gtsam::Point3& anchor_offset);

   OptimizerPhase GetOptimizerPhase();

   void CheckIfAllEstimationDoneAndReset();

   void NeighborState(const int& rid, const OptimizerState& state, const int& lowest_id_included_in_global_map);

   void UpdateCurrentPoseEstimate(const int& pose_id);

   void UpdatePoseEstimateFromNeighbor(const int& rid, const int& pose_id, const graph_utils::PoseWithCovariance& pose);

   void UpdateNeighborHasStartedOptimizationFlag(const bool& neighbor_has_started_optimization,
                                                const int& other_robot_id);

   void UpdateHasSentStartOptimizationFlag(const bool& has_sent_start_optimization_flag);

   void UpdateAdjacencyVector();

   void ReceiveAdjacencyVectorFromNeighbor(const int& other_robot_id, const std::vector<int>& adjacency_vector);

   int GetNumberOfPoses();

   virtual buzzvm_state RegisterSLAMFunctions(buzzvm_t buzz_vm);

protected:
   // Functions for link with buzz VM
   void UpdateCurrentSeparatorBuzzStructure( const int& robot_1_id,
                                             const int& robot_2_id,
                                             const int& robot_1_pose_id,
                                             const int& robot_2_pose_id,
                                             const double& x,
                                             const double& y,
                                             const double& z,
                                             const double& q_x,
                                             const double& q_y,
                                             const double& q_z,
                                             const double& q_w,
                                             const gtsam::Matrix6& covariance_matrix );

   // Utility functions
   void WriteCurrentDataset();

   virtual void WriteInitialDataset();

   virtual void WriteOptimizedDataset();

   // Wrapper functions related to distributed_mapper
   void IncrementNumberOfPoses();

   bool StartOptimizationCondition();

   virtual void InitializePoseGraphOptimization();

   void UpdateOptimizer();

   void SaveInitialGraph();

   void OutliersFiltering();

   void AddNewKnownRobot(const unsigned char& other_robot_char);

   void InitializePoseEstimation();

   void EndOptimization();

   double GetLatestLocalError();

   bool RotationEstimationStoppingBarrier();

   bool PoseEstimationStoppingBarrier();

   void SetRotationEstimationIsFinishedFlagsToFalse();

   void SetPoseEstimationIsFinishedFlagsToFalse();

   bool AllRobotsAreInitialized();

   virtual bool CompareCentralizedAndDecentralizedError();

   void UpdateOptimizerPhase();

   void RemoveInactiveNeighbors();

   void RemoveDisconnectedNeighbors();

   void FailSafeCheck();

   void SaveAcceptedAndRejectedKeys(const std::set<std::pair<gtsam::Key, gtsam::Key>>& accepted_keys, const std::set<std::pair<gtsam::Key, gtsam::Key>>& rejected_keys);

   void FillPoseGraphForCentralizedEvaluation();

   void SaveBackup();

   virtual void AbortOptimization(const bool& log_info);

   void IncrementalInitialGuessUpdate(const gtsam::Values& new_poses, boost::shared_ptr<gtsam::Values>& poses_to_be_updated);

   void IncrementNumberOfSeparatorsWithOtherRobot(const int& other_robot_id);

   void ComputeOptimizationOrder();

protected:
   // Buzz attributes
   buzzvm_t buzz_vm_; 

   // General attributes of the robot
   uint16_t robot_id_;

   unsigned char robot_id_char_;

   // Measurements
   boost::shared_ptr<gtsam::NonlinearFactorGraph> local_pose_graph_;

   boost::shared_ptr<gtsam::NonlinearFactorGraph> local_pose_graph_no_filtering_;

   boost::shared_ptr<gtsam::NonlinearFactorGraph> local_pose_graph_for_centralized_evaluation_;

   std::set<std::pair<gtsam::Key, gtsam::Key>> factors_in_pose_graph_for_centralized_evaluation_;

   boost::shared_ptr<gtsam::Values> poses_initial_guess_;

   boost::shared_ptr<gtsam::Values> poses_initial_guess_no_updates_, poses_initial_guess_centralized_incremental_updates_;

   gtsam::GraphAndValues graph_and_values_;

   robot_measurements::RobotLocalMap robot_local_map_;

   // Current state of the controller
   gtsam::Symbol previous_symbol_;

   int number_of_poses_;

   int number_of_poses_at_optimization_end_;

   std::set<unsigned char> known_other_robots_;

   std::set<int> neighbors_within_communication_range_;

   // Distributed mapping attributes
   boost::shared_ptr<distributed_mapper::DistributedMapper> optimizer_;

   bool disconnected_graph_;

   int current_rotation_iteration_, current_pose_iteration_;

   OptimizerState optimizer_state_;

   int optimizer_period_;

   bool rotation_estimation_phase_is_finished_, pose_estimation_phase_is_finished_;

   std::map<int, bool> neighbors_rotation_estimation_phase_is_finished_, neighbors_pose_estimation_phase_is_finished_, neighbors_is_estimation_done_;

   gtsam::Point3 anchor_offset_;

   std::map<int, gtsam::Point3> neighbors_anchor_offset_;

   int prior_owner_;

   std::map<int, OptimizerState> neighbors_state_;

   std::map<int, int> neighbors_lowest_id_included_in_global_map_;

   bool is_estimation_done_;

   gtsam::NonlinearFactorGraph local_pose_graph_before_optimization_;

   std::map<int, graph_utils::Trajectory> pose_estimates_from_neighbors_;

   std::map<int, int> number_of_separators_with_each_robot_;

   bool is_prior_added_;

   int total_outliers_rejected_;

   bool has_sent_start_optimization_flag_;

   std::map<int, bool> neighbors_has_started_optimization_;

   double latest_change_;

   int number_of_steps_without_changes_;

   int number_of_steps_before_failsafe_;

   int number_of_optimization_run_;

   int lowest_id_to_include_in_global_map_, lowest_id_included_in_global_map_;

   std::set<std::pair<gtsam::Key, gtsam::Key>> accepted_keys_, rejected_keys_;

   std::set<gtsam::Key> other_robot_keys_for_optimization_;

   gtsam::Matrix adjacency_matrix_;

   std::vector<int> optimization_order_;

   // Backups in case of abort
   robot_measurements::RobotLocalMap robot_local_map_backup_;

   // Parameters
   double rotation_noise_std_, translation_noise_std_;

   double rotation_estimate_change_threshold_, pose_estimate_change_threshold_;

   bool use_flagged_initialization_;

   int debug_level_;

   int end_delay_;

   bool incremental_solving_;

   int max_number_of_rotation_estimation_steps_, max_number_of_pose_estimation_steps_;

   int number_of_robots_;

   // Pairwise consistency maximization parameters
   double confidence_probability_;

   bool use_pcm_;

   // Parameter for evaluation
   bool is_simulation_;

   std::string error_file_name_;

};
}

#endif