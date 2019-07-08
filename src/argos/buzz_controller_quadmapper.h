#ifndef BUZZ_CONTROLLER_QUADMAPPER_H
#define BUZZ_CONTROLLER_QUADMAPPER_H

#include <buzz/argos/buzz_controller_spiri.h>
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_position_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_perspective_camera_sensor.h>
#include "../slam/buzz_slam_singleton.h"

using namespace argos;

namespace buzz_quadmapper {

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

protected:

   virtual buzzvm_state RegisterFunctions();

};
}
#endif
