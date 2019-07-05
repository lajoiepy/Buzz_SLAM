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
}

/****************************************/
/****************************************/

int CBuzzControllerQuadMapperWithDataset::Move() {
   auto number_of_poses = reinterpret_cast<buzz_slam::BuzzSLAMDataset*>(buzz_slam::BuzzSLAMSingleton::GetInstance().GetBuzzSLAM(m_tBuzzVM->robot))->GetNumberOfPoses();
   auto symbol = gtsam::Symbol((char)(m_tBuzzVM->robot +97), number_of_poses);
   if (reinterpret_cast<buzz_slam::BuzzSLAMDataset*>(buzz_slam::BuzzSLAMSingleton::GetInstance().GetBuzzSLAM(m_tBuzzVM->robot))->KeyExists(symbol.key())) {
      auto pose = reinterpret_cast<buzz_slam::BuzzSLAMDataset*>(buzz_slam::BuzzSLAMSingleton::GetInstance().GetBuzzSLAM(m_tBuzzVM->robot))->GetPoseAtKey(symbol.key());
      CVector3 position(pose.x(), pose.y(), 2.0f); // Fixed altitude for visualization only
      m_pcPropellers->SetAbsolutePosition(position);
      m_pcPropellers->SetAbsoluteYaw(CRadians(pose.rotation().yaw()));

      // Add measurement
      reinterpret_cast<buzz_slam::BuzzSLAMDataset*>(buzz_slam::BuzzSLAMSingleton::GetInstance().GetBuzzSLAM(m_tBuzzVM->robot))->AddOdometryMeasurement();
   }
   return number_of_poses;
}

}