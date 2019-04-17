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
enum OptimizerState { Idle, Start, RotationEstimation, PoseEstimation, End };

/*
*  Rotation estimate message
*/
typedef struct {
   int sender_robot_id;
   int sender_pose_id;
   bool sender_robot_is_initialized;
   double rotation_matrix[9];
} rotation_estimate_t;

/*
*  Pose estimate message
*/
typedef struct {
   int receiver_robot_id;
   int receiver_pose_id;
   bool sender_robot_is_initialized;
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
   void WriteDataset(const uint16_t& robot_id);

   // Wrapper function related to distributed_mapper
   void IncrementNumberOfPosesAndUpdateState();

   void StartPoseGraphOptimization();

   void UpdateOptimizer();

   void OutliersFiltering();

   std::vector<size_t> TrivialOrdering();

   std::vector<size_t> FlaggedInitializationOrdering();

   bool RotationEstimationStoppingConditions();

   void UpdateLocalEstimates();

   void AddNewKnownRobot(const unsigned char& other_robot_char);

   void InitializePoseEstimation();

   bool PoseEstimationStoppingConditions();

   void EndOptimization();

   double EvaluateCurrentEstimate();

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

   gtsam::Pose3 previous_pose_;

   int number_of_poses_;

   std::set<unsigned char> known_other_robots_;

   std::vector<int> neighbors_within_communication_range_;

   // Distributed mapping attributes
   boost::shared_ptr<distributed_mapper::DistributedMapper> optimizer_;

   bool disconnected_graph_;

   int current_rotation_iteration_, current_pose_iteration_;

   OptimizerState optimizer_state_;

   int optimizer_period_;

   // Constants that should be parameters in the future
   double rotation_noise_std_, translation_noise_std_;

   gtsam::SharedNoiseModel noise_model_;

   gtsam::noiseModel::Isotropic::shared_ptr chordal_graph_noise_model_;

   int maximum_number_of_optimization_iterations_;

   int optimization_phase_length_;

   double rotation_estimate_change_threshold_, pose_estimate_change_threshold_;

};
}
#endif
