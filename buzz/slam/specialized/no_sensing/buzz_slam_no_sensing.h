#ifndef BUZZ_SLAM_NO_SENSING_H
#define BUZZ_SLAM_NO_SENSING_H

#include "../../buzz_slam.h"
#include <buzz/argos/buzz_controller_spiri.h>
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_position_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_perspective_camera_sensor.h>
#include <map>

namespace buzz_slam {

/*
*  This version is for simulations without sensing.
*/
class BuzzSLAMNoSensing : public BuzzSLAM {

public:

   BuzzSLAMNoSensing() {};
   
   virtual ~BuzzSLAMNoSensing() {};

   virtual void Init();

   // Fake measurements generation
   int ComputeNoisyFakeSeparatorMeasurement(const CQuaternion& gt_orientation, const CVector3& gt_translation, 
                                          const int& pose_id, const int& robot_id, const int& this_robot_pose_id);

   void LoadParameters(const double& sensor_range, const double& outlier_probability);

   virtual buzzvm_state RegisterSLAMFunctions(buzzvm_t buzz_vm);

public:

   // Fake measurements generation
   void ComputeNoisyFakeOdometryMeasurement();

   gtsam::Pose3 AddGaussianNoiseToMeasurement(const gtsam::Rot3& R, const gtsam::Point3& t);

   gtsam::Pose3 OutlierMeasurement(const gtsam::Rot3& R, const gtsam::Point3& t);

   void SavePoseGroundTruth();

protected:

   virtual void InitializePoseGraphOptimization();
   
   virtual bool CompareCentralizedAndDecentralizedError();

   void ComputeCentralizedEstimate(const std::string& centralized_extension);

   void ComputeCentralizedEstimateIncremental(std::set<int> robots, const std::string& centralized_extension);

   virtual void WriteInitialDataset();

   virtual void WriteOptimizedDataset();

   std::set<std::pair<gtsam::Key, gtsam::Key>> AggregateOutliersKeys(const std::set<int>& robots);

   std::pair<int, int> CountInliersAndOutliers(const std::set<int>& robots);

   void RemoveRejectedKeys();

   virtual void AbortOptimization(const bool& log_info);

private:

   // Ground truth information to compute fake measurements
   std::map<int, gtsam::Pose3> ground_truth_poses_;
   boost::shared_ptr<gtsam::Values> ground_truth_data_;
   argos::CCI_PositioningSensor::SReading previous_simulation_gt_pose_;
   gtsam::SharedNoiseModel noise_model_;
   gtsam::noiseModel::Isotropic::shared_ptr chordal_graph_noise_model_;
   gtsam::Matrix6 covariance_matrix_;

   // Random numbers generation
   std::random_device rd_{};
   std::mt19937 gen_translation_, gen_rotation_, gen_outliers_;
   std::normal_distribution<> normal_distribution_translation_, normal_distribution_rotation_;
   std::uniform_real_distribution<> uniform_distribution_outliers_translation_, 
                                    uniform_distribution_outliers_rotation_,
                                    uniform_distribution_draw_outlier_;

   // Current state of the simulation
   int number_of_outliers_added_;
   int number_of_inliers_added_;
   double outlier_probability_;
   double sensor_range_;
   std::set<std::pair<gtsam::Key, gtsam::Key>> outliers_keys_;
   std::set<std::pair<gtsam::Key, gtsam::Key>> inliers_keys_;

};
}
#endif
