#ifndef BUZZ_CONTROLLER_QUADMAPPER_H
#define BUZZ_CONTROLLER_QUADMAPPER_H

#include <buzz/argos/buzz_controller_spiri.h>
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_position_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_perspective_camera_sensor.h>
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

using namespace argos;

namespace buzz_quadmapper {
/*
*  Enum of all the possible state of the optimizer
*/
enum OptimizerState { Idle, Start, RotationEstimation, PoseEstimationInitialization, PoseEstimation, End, PostEndingCommunicationDelay };

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
* Buzz controller to support 3D robust distributed pose graph optimization.
* This class should not be instanciated, you should use its derived classes (with or without sensing)
*/
class CBuzzControllerQuadMapper : public CBuzzControllerSpiri {

public:

   CBuzzControllerQuadMapper();
   
   virtual ~CBuzzControllerQuadMapper();

   virtual void Init(TConfigurationNode& t_node);

   void LoadParameters( const bool& incremental_solving, const bool& debug,
                        const float& rotation_noise_std, const float& translation_noise_std,
                        const float& rotation_estimate_change_threshold, const float& translation_estimate_change_threshold,
                        const bool& use_flagged_initialization, const bool& is_simulation,
                        const int& number_of_robots, const std::string& error_file_name);

   // Control functions
   void SetNextPosition(const CVector3& c_heading);

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
                                    const double& q_w  );

   // Wrapper function related to distributed_mapper
   void InitOptimizer(const int& period);

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

   void NeighborPoseEstimationIsFinished(const int& rid);

   OptimizerPhase GetOptimizerPhase();

   void CheckIfAllEstimationDoneAndReset();

   void NeighborState(const int& rid, const OptimizerState& state);

protected:

   // Functions for link with buzz VM
   void UpdateCurrentSeparatorBuzzStructure(  const int& robot_1_id,
                                                const int& robot_2_id,
                                                const int& robot_1_pose_id,
                                                const int& robot_2_pose_id,
                                                const double& x,
                                                const double& y,
                                                const double& z,
                                                const double& q_x,
                                                const double& q_y,
                                                const double& q_z,
                                                const double& q_w  );

   virtual buzzvm_state RegisterFunctions();

   // Utility functions
   void WriteCurrentDataset();

   void WriteInitialDataset();

   void WriteOptimizedDataset();

   // Wrapper function related to distributed_mapper
   void IncrementNumberOfPosesAndUpdateState();

   void StartPoseGraphOptimization();

   void UpdateOptimizer();

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

   bool CompareCentralizedAndDecentralizedError();

   void UpdateOptimizerPhase();

protected:
   // General attributes of the controller
   uint16_t robot_id_;

   unsigned char robot_id_char_;

   // Measurements
   boost::shared_ptr<gtsam::NonlinearFactorGraph> local_pose_graph_;

   boost::shared_ptr<gtsam::Values> poses_initial_guess_;

   gtsam::GraphAndValues graph_and_values_;

   // Current state of the controller
   gtsam::Symbol previous_symbol_;

   int number_of_poses_;

   std::set<unsigned char> known_other_robots_;

   std::vector<int> neighbors_within_communication_range_;

   // Distributed mapping attributes
   boost::shared_ptr<distributed_mapper::DistributedMapper> optimizer_;

   bool disconnected_graph_;

   int current_rotation_iteration_, current_pose_iteration_;

   OptimizerState optimizer_state_;

   int optimizer_period_;

   bool rotation_estimation_phase_is_finished_, pose_estimation_phase_is_finished_;

   std::map<int, bool> neighbors_rotation_estimation_phase_is_finished_, neighbors_pose_estimation_phase_is_finished_, neighbors_is_estimation_done_;

   std::map<int, OptimizerState> neighbors_state_;

   bool is_estimation_done_;

   gtsam::NonlinearFactorGraph local_pose_graph_before_optimization_;

   // Parameters
   double rotation_noise_std_, translation_noise_std_;

   gtsam::SharedNoiseModel noise_model_;

   gtsam::noiseModel::Isotropic::shared_ptr chordal_graph_noise_model_;

   double rotation_estimate_change_threshold_, pose_estimate_change_threshold_;

   bool use_flagged_initialization_;

   bool debug_;

   int end_delay_;

   bool incremental_solving_;

   // Parameter for evaluation
   int number_of_robots_;

   bool is_simulation_;

   std::string error_file_name_;

};
}
#endif
