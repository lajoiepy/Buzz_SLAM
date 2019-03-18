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

using namespace argos;

class CBuzzControllerQuadMapper : public CBuzzControllerSpiri {

public:

   CBuzzControllerQuadMapper();
   
   virtual ~CBuzzControllerQuadMapper();

   virtual void Init(TConfigurationNode& t_node);

   void SetNextPosition(const CVector3& c_heading);

   void AddLoopClosureToLocalGraph( const int& robot_1_id,
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

protected:

   void UpdateCurrentLoopClosureBuzzStructure(  const int& robot_1_id,
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

   void WriteDataset(const uint16_t& robot_id);

protected:

   gtsam::NonlinearFactorGraph local_pose_graph_;
   gtsam::Values poses_initial_guess_;
   CQuaternion previous_orientation_;
   gtsam::Symbol previous_symbol_;
   gtsam::Pose3 previous_pose_;
   int number_of_poses_;
   unsigned char robot_id_char_;

   double rotation_noise_std_, translation_noise_std_;

};

#endif
