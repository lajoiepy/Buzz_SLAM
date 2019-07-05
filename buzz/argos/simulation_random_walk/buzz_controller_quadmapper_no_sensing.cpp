#include "buzz_controller_quadmapper_no_sensing.h"
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <cmath> 
#include "boost/filesystem.hpp"
#include "../../slam/specialized/no_sensing/buzz_slam_no_sensing.h"

namespace buzz_quadmapper {
/****************************************/
/****************************************/

CBuzzControllerQuadMapperNoSensing::CBuzzControllerQuadMapperNoSensing() :
   CBuzzControllerQuadMapper() {
}

/****************************************/
/****************************************/

CBuzzControllerQuadMapperNoSensing::~CBuzzControllerQuadMapperNoSensing() {
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapperNoSensing::Init(TConfigurationNode& t_node){
   CBuzzControllerQuadMapper::Init(t_node);
   simulation_step_ = 0;
}


/****************************************/
/****************************************/

int CBuzzControllerQuadMapperNoSensing::MoveForwardFakeOdometry(const CVector3& distance, const int& simulation_time_divider) {
   simulation_step_ ++;

   // Move
   CQuaternion current_orientation = m_pcPos->GetReading().Orientation;
   CRadians c_z_angle, c_y_angle, c_x_angle;
   current_orientation.ToEulerAngles(c_z_angle, c_y_angle, c_x_angle);

   CVector3 translation;
   translation.SetX(distance.GetX()*std::cos(c_z_angle.GetValue()));
   translation.SetY(distance.GetX()*std::sin(c_z_angle.GetValue()));
   translation.SetZ(0);

   CVector3 new_position;
   new_position.SetX(m_pcPos->GetReading().Position.GetX() + translation.GetX());
   new_position.SetY(m_pcPos->GetReading().Position.GetY() + translation.GetY());

   new_position.SetZ(2.0f); // To ensure that the quadrotor flies

   m_pcPropellers->SetAbsolutePosition(new_position);
   
   if (simulation_step_ % simulation_time_divider == 0) {
      // Add noisy measurement
      reinterpret_cast<buzz_slam::BuzzSLAMNoSensing*>(buzz_slam::BuzzSLAMSingleton::GetInstance().GetBuzzSLAM(m_tBuzzVM->robot))->ComputeNoisyFakeOdometryMeasurement();

      // Log data
      //WriteCurrentDataset();   
   }

   return buzz_slam::BuzzSLAMSingleton::GetInstance().GetBuzzSLAM(m_tBuzzVM->robot)->GetNumberOfPoses();
}

}