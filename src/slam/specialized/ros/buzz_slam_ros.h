#ifndef BUZZ_SLAM_ROS_H
#define BUZZ_SLAM_ROS_H

#include "../../buzz_slam.h"
#include <map>

namespace buzz_slam {

/*
*  Buzz SLAM when measurements are given by the front end
*/
class BuzzSLAMRos : public BuzzSLAM {

public:

   BuzzSLAMRos();
   
   virtual ~BuzzSLAMRos();

   virtual void Init(buzzvm_t buzz_vm, const gtsam::Point3& t_gt, const gtsam::Rot3& R_gt);

   // Fake measurements generation
   int AddSeparatorMeasurement();

   int AddSeparatorMeasurementOutlier();

   void LoadParameters(const double& sensor_range, const int& outlier_period);

   virtual buzzvm_state RegisterSLAMFunctions(buzzvm_t buzz_vm);

   // Fake measurements generation
   void AddOdometryMeasurement();
   
private:

   gtsam::Pose3 OutlierMeasurement(const gtsam::Rot3& R, const gtsam::Point3& t);

   void IncrementNumberOfInliersWithOtherRobot(const int& other_robot_id);

   void IncrementNumberOfOutliersWithOtherRobot(const int& other_robot_id);

protected:

   virtual void WriteOptimizedDataset();

   std::set<std::pair<gtsam::Key, gtsam::Key>> AggregateOutliersKeys(const std::set<int>& robots);

   std::pair<int, int> CountInliersAndOutliers(const std::set<int>& robots);

   void RemoveRejectedKeys();

   virtual void AbortOptimization(const bool& log_info);

private:

   // Information for generate outliers
   gtsam::noiseModel::Isotropic::shared_ptr chordal_graph_noise_model_;
   gtsam::Matrix covariance_matrix_for_outlier_;

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
   double sensor_range_;
   int outlier_period_;
   std::map<int, int> number_of_inliers_with_each_robot_, number_of_outliers_with_each_robot_;
   std::set<std::pair<gtsam::Key, gtsam::Key>> outliers_keys_;
   std::set<std::pair<gtsam::Key, gtsam::Key>> inliers_keys_;

};
}
#endif
