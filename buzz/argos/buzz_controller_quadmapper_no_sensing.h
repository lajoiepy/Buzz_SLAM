#ifndef BUZZ_CONTROLLER_QUADMAPPER_NO_SENSING_H
#define BUZZ_CONTROLLER_QUADMAPPER_NO_SENSING_H

#include <argos/buzz_controller_quadmapper.h>
#include <map>

using namespace argos;

class CBuzzControllerQuadMapperNoSensing : public CBuzzControllerQuadMapper {

public:

   CBuzzControllerQuadMapperNoSensing();
   
   virtual ~CBuzzControllerQuadMapperNoSensing();

   virtual void Init(TConfigurationNode& t_node);

   int MoveForwardFakeOdometry(const CVector3& distance, const uint16_t& robot_id);

   int ComputeNoisyFakeLoopClosureMeasurement(const CQuaternion& gt_orientation, const CVector3& gt_translation, 
                                          const int& pose_id, const int& robot_id, const int& this_robot_pose_id);

private:

   void ComputeNoisyFakeOdometryMeasurement(const CQuaternion& current_orientation, const CVector3& translation);

   gtsam::Pose3 AddGaussianNoiseToMeasurement(const gtsam::Rot3& R, const gtsam::Point3& t);

   gtsam::Pose3 OutlierMeasurement(const gtsam::Rot3& R, const gtsam::Point3& t);

   void SavePoseGroundTruth();

protected:

   virtual buzzvm_state RegisterFunctions();

private:

   std::map<int, gtsam::Pose3> ground_truth_poses_;

   std::random_device rd_{};
   std::mt19937 gen_translation_, gen_rotation_, gen_outliers_;
   std::normal_distribution<> normal_distribution_translation_, normal_distribution_rotation_;
   std::uniform_real_distribution<> uniform_distribution_outliers_translation_, 
                                    uniform_distribution_outliers_rotation_,
                                    uniform_distribution_draw_outlier_;

   int number_of_outliers_added_;
   double outlier_probability_;
   double sensor_range_;

};

#endif
