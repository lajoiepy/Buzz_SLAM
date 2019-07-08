#ifndef BUZZ_CONTROLLER_QUADMAPPER_WITH_DATATSET_H
#define BUZZ_CONTROLLER_QUADMAPPER_WITH_DATATSET_H

#include <argos/buzz_controller_quadmapper.h>
#include <map>
#include "../../slam/specialized/dataset/buzz_slam_dataset.h"

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

   buzzvm_state RegisterFunctions();

   gtsam::Pose3 GetGroundTruthPose();
};
}
#endif
