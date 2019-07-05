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
   gtsam::Pose3 pose_gt = GetGroundTruthPose();
   reinterpret_cast<buzz_slam::BuzzSLAMNoSensing*>(buzz_slam::BuzzSLAMSingleton::GetInstance().GetBuzzSLAM(m_tBuzzVM->robot).get())->Init(m_tBuzzVM, pose_gt.translation(), pose_gt.rotation());
}

/****************************************/
/****************************************/

int CBuzzControllerQuadMapperWithDataset::Move() {
   auto number_of_poses = reinterpret_cast<buzz_slam::BuzzSLAMDataset*>(buzz_slam::BuzzSLAMSingleton::GetInstance().GetBuzzSLAM(m_tBuzzVM->robot).get())->GetNumberOfPoses();
   auto symbol = gtsam::Symbol((char)(m_tBuzzVM->robot +97), number_of_poses);
   if (reinterpret_cast<buzz_slam::BuzzSLAMDataset*>(buzz_slam::BuzzSLAMSingleton::GetInstance().GetBuzzSLAM(m_tBuzzVM->robot).get())->KeyExists(symbol.key())) {
      auto pose = reinterpret_cast<buzz_slam::BuzzSLAMDataset*>(buzz_slam::BuzzSLAMSingleton::GetInstance().GetBuzzSLAM(m_tBuzzVM->robot).get())->GetPoseAtKey(symbol.key());
      CVector3 position(pose.x(), pose.y(), 2.0f); // Fixed altitude for visualization only
      m_pcPropellers->SetAbsolutePosition(position);
      m_pcPropellers->SetAbsoluteYaw(CRadians(pose.rotation().yaw()));

      // Add measurement
      reinterpret_cast<buzz_slam::BuzzSLAMDataset*>(buzz_slam::BuzzSLAMSingleton::GetInstance().GetBuzzSLAM(m_tBuzzVM->robot).get())->AddOdometryMeasurement();
   }
   return number_of_poses;
}

/****************************************/
/****************************************/

gtsam::Pose3 CBuzzControllerQuadMapperWithDataset::GetGroundTruthPose() {
   gtsam::Point3 t_gt = {  m_pcPos->GetReading().Position.GetX(), 
                           m_pcPos->GetReading().Position.GetY(),
                           m_pcPos->GetReading().Position.GetZ() };
   
   CQuaternion current_orientation = m_pcPos->GetReading().Orientation;
   gtsam::Quaternion current_quat_gtsam(current_orientation.GetW(), current_orientation.GetX(), current_orientation.GetY(), current_orientation.GetZ());
   gtsam::Rot3 R_gt(current_quat_gtsam);

   return gtsam::Pose3(R_gt, t_gt);
}

}