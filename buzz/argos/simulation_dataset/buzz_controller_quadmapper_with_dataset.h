#ifndef BUZZ_CONTROLLER_QUADMAPPER_WITH_DATATSET_H
#define BUZZ_CONTROLLER_QUADMAPPER_WITH_DATATSET_H

#include <argos/buzz_controller_quadmapper.h>
#include <map>

using namespace argos;
namespace buzz_quadmapper {

/*
*  Buzz controller to support 3D robust distributed pose graph optimization.
*  This version is tighly coupled with the argos3 simulation and reads the measurements from datasets
*/
class CBuzzControllerQuadMapperWithDataset : public CBuzzControllerQuadMapper {

public:

   CBuzzControllerQuadMapperWithDataset();
   
   virtual ~CBuzzControllerQuadMapperWithDataset();

   virtual void Init(TConfigurationNode& t_node);

   // Control functions
   int Move();

   // Fake measurements generation
   int AddSeparatorMeasurement();

   int AddSeparatorMeasurementOutlier();

   void LoadParameters(const std::string& dataset_name, const double& sensor_range, const int& outlier_period);

private:

   // Fake measurements generation
   void AddOdometryMeasurement();

   gtsam::Pose3 OutlierMeasurement(const gtsam::Rot3& R, const gtsam::Point3& t);

   void IncrementNumberOfInliersWithOtherRobot(const int& other_robot_id);

   void IncrementNumberOfOutliersWithOtherRobot(const int& other_robot_id);

protected:

   // Functions for link with buzz VM
   virtual buzzvm_state RegisterFunctions();

   virtual bool CompareCentralizedAndDecentralizedError();

   void ComputeCentralizedEstimate(const std::string& centralized_extension);

   void ComputeCentralizedEstimateIncremental(std::set<int> robots, const std::string& centralized_extension);

   virtual void WriteOptimizedDataset();

   std::set<std::pair<gtsam::Key, gtsam::Key>> AggregateOutliersKeys(const std::set<int>& robots);

   std::pair<int, int> CountInliersAndOutliers(const std::set<int>& robots);

   void RemoveRejectedKeys();

   virtual void AbortOptimization(const bool& log_info);

private:

   // Information from the dataset
   std::string dataset_name_;
   boost::shared_ptr<gtsam::Values> dataset_values_;
   std::map<std::pair<gtsam::Key, gtsam::Key>, boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3>>> dataset_factors_;
   std::map<gtsam::Key, std::pair<gtsam::Key, gtsam::Key>> loop_closure_linked_to_key_;

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
   bool dataset_reading_ended_;

};
}
#endif
