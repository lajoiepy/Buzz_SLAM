#include "buzz_controller_quadmapper_no_sensing.h"
#include <iostream>
#include <stdlib.h>
#include <time.h>

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

   // Save ground truth for fake loop closure creation
   SavePoseGroundTruth();
}

/****************************************/
/****************************************/

static int BuzzComputeFakeRendezVousLoopClosures(buzzvm_t vm) {
   /* Push the vector components */
   buzzvm_lload(vm, 1);
   buzzvm_lload(vm, 2);
   buzzvm_lload(vm, 3);
   buzzvm_lload(vm, 4);
   buzzvm_lload(vm, 5);
   buzzvm_lload(vm, 6);
   buzzvm_lload(vm, 7);
   /* Retrieve parameters and check their types */
   buzzobj_t b_other_robot_pose_id = buzzvm_stack_at(vm, 7);
   buzzobj_t b_gt_x = buzzvm_stack_at(vm, 6);
   buzzobj_t b_gt_y = buzzvm_stack_at(vm, 5);
   buzzobj_t b_gt_z = buzzvm_stack_at(vm, 4);
   buzzobj_t b_gt_yaw = buzzvm_stack_at(vm, 3);
   buzzobj_t b_other_robot_id = buzzvm_stack_at(vm, 2);
   buzzobj_t b_this_robot_pose_id = buzzvm_stack_at(vm, 1);
   CVector3 estimate_translation, gt_translation;
   CQuaternion estimate_orientation, gt_orientation;
   int other_robot_id, other_robot_pose_id, this_robot_pose_id;
   if(b_this_robot_pose_id->o.type == BUZZTYPE_INT &&
      b_other_robot_id->o.type == BUZZTYPE_INT &&
      b_other_robot_pose_id->o.type == BUZZTYPE_INT &&
      b_gt_x->o.type == BUZZTYPE_FLOAT &&
      b_gt_y->o.type == BUZZTYPE_FLOAT &&
      b_gt_z->o.type == BUZZTYPE_FLOAT &&
      b_gt_yaw->o.type == BUZZTYPE_FLOAT) {

      // Fill in ground truth
      gt_translation.SetX(b_gt_x->f.value);
      gt_translation.SetY(b_gt_y->f.value);
      gt_translation.SetZ(b_gt_z->f.value);
      CRadians gt_angle(b_gt_yaw->f.value);
      CVector3 gt_axis(0, 0, 1);
      gt_orientation = gt_orientation.FromAngleAxis(gt_angle, gt_axis);

      // Get information for symbols
      other_robot_id = b_other_robot_id->i.value;
      other_robot_pose_id = b_other_robot_pose_id->i.value;
      this_robot_pose_id = b_this_robot_pose_id->i.value;

   } else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "wrong parameter type for compute_fake_rendezvous_loop_closures."
         );
      return vm->state;
   }
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   reinterpret_cast<CBuzzControllerQuadMapperNoSensing*>(buzzvm_stack_at(vm, 1)->u.value)->ComputeNoisyFakeLoopClosureMeasurement(gt_orientation, gt_translation, other_robot_pose_id, other_robot_id, this_robot_pose_id);
   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzMoveForwardFakeOdometry(buzzvm_t vm) {
   /* Push the vector components */
   buzzvm_lload(vm, 1);
   /* Create a new vector with that */
   CVector3 translation;
   buzzobj_t step = buzzvm_stack_at(vm, 1);
   if(step->o.type == BUZZTYPE_INT) translation.SetX(step->i.value);
   else if(step->o.type == BUZZTYPE_FLOAT) translation.SetX(step->f.value);
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "move_forward(x,y): expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_FLOAT],
                      buzztype_desc[step->o.type]
         );
      return vm->state;
   }      
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   int current_pose_id = reinterpret_cast<CBuzzControllerQuadMapperNoSensing*>(buzzvm_stack_at(vm, 1)->u.value)->MoveForwardFakeOdometry(translation, vm->robot);
   
   buzzvm_pushi(vm, current_pose_id);

   return buzzvm_ret1(vm);
}

/****************************************/
/****************************************/

int CBuzzControllerQuadMapperNoSensing::MoveForwardFakeOdometry(const CVector3& distance, const uint16_t& robot_id) {

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
   
   // Add noisy measurement
   ComputeNoisyFakeOdometryMeasurement(current_orientation, translation);

   // Log data
   WriteDataset(robot_id);   

   return number_of_poses_;
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapperNoSensing::ComputeNoisyFakeOdometryMeasurement(const CQuaternion& current_orientation, const CVector3& translation) {
   
   // Increase the number of poses
   number_of_poses_++;

   // Next symbol
   gtsam::Symbol current_symbol_ = gtsam::Symbol(robot_id_char_, number_of_poses_);

   // Conversion of the previous orientation (quaternion to rotation matrix)
   gtsam::Quaternion previous_quat_gtsam(previous_orientation_.GetW(), previous_orientation_.GetX(), previous_orientation_.GetY(), previous_orientation_.GetZ());
   gtsam::Rot3 previous_R(previous_quat_gtsam);
   
   // Conversion of the current orientation (quaternion to rotation matrix)
   gtsam::Quaternion current_quat_gtsam(current_orientation.GetW(), current_orientation.GetX(), current_orientation.GetY(), current_orientation.GetZ());
   gtsam::Rot3 current_R(current_quat_gtsam);

   // Compute transformation between rotations
   gtsam::Rot3 R = previous_R.inverse() * current_R;

   // Convert translation information to gtsam format and perform the appropriate rotation
   gtsam::Point3 t = { translation.GetX(), translation.GetY(), translation.GetZ()};
   t = previous_R.inverse() * t;

   // Add gaussian noise
   auto measurement = AddGaussianNoiseToMeasurement(R, t);

   // Isotropic noise model
   Eigen::VectorXd sigmas(6);
   sigmas << rotation_noise_std_, rotation_noise_std_, rotation_noise_std_, 
            translation_noise_std_, translation_noise_std_, translation_noise_std_;
   gtsam::SharedNoiseModel noise_model = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

   // Initialize factor
   gtsam::BetweenFactor<gtsam::Pose3> new_factor(previous_symbol_, current_symbol_, measurement, noise_model);

   // Update attributes value
   previous_orientation_ = m_pcPos->GetReading().Orientation;
   previous_pose_ = previous_pose_ * measurement;
   previous_symbol_ = current_symbol_;

   // Add new factor to local pose graph
   local_pose_graph_.push_back(new_factor);

   // Add new pose estimate into initial guess
   poses_initial_guess_.insert(previous_symbol_.key(), previous_pose_);

   // Save ground truth for fake loop closure creation
   SavePoseGroundTruth();
}

/****************************************/
/****************************************/

gtsam::Pose3 CBuzzControllerQuadMapperNoSensing::AddGaussianNoiseToMeasurement(const gtsam::Rot3& R, const gtsam::Point3& t) {
   
   gtsam::Point3 t_noisy = {  t.x() + normal_distribution_translation_(gen_translation_),  
                              t.y() + normal_distribution_translation_(gen_translation_),
                              t.z() + normal_distribution_translation_(gen_translation_) };
   
   gtsam::Point3 R_noise_vector = { normal_distribution_rotation_(gen_rotation_),  
                                    normal_distribution_rotation_(gen_rotation_),
                                    normal_distribution_rotation_(gen_rotation_) };

   gtsam::Rot3 R_noise_matrix = gtsam::Rot3::AxisAngle(R_noise_vector.normalized(), R_noise_vector.norm());

   gtsam::Rot3 R_noisy = R * R_noise_matrix;

   return gtsam::Pose3(R_noisy, t_noisy);
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapperNoSensing::ComputeNoisyFakeLoopClosureMeasurement(const CQuaternion& gt_orientation, const CVector3& gt_translation, 
                                                               const int& other_robot_pose_id, const int& other_robot_id, const int& this_robot_pose_id) {
   // Loop closure symbols
   gtsam::Symbol this_robot_symbol = gtsam::Symbol(robot_id_char_, this_robot_pose_id);
   unsigned char other_robot_id_char = (unsigned char)(97 + other_robot_id);
   gtsam::Symbol other_robot_symbol = gtsam::Symbol(other_robot_id_char, other_robot_pose_id);

   // Get this robot pose
   gtsam::Pose3 this_robot_pose = ground_truth_poses_.find(this_robot_pose_id)->second;

   // Conversion of the other robot orientation (quaternion to rotation matrix)
   gtsam::Quaternion other_robot_quat_gtsam(gt_orientation.GetW(), gt_orientation.GetX(), gt_orientation.GetY(), gt_orientation.GetZ());
   gtsam::Rot3 other_robot_R(other_robot_quat_gtsam);

   // Compute transformation between rotations
   gtsam::Rot3 R = this_robot_pose.rotation().inverse() * other_robot_R;

   // Convert translation information to gtsam format and perform the appropriate rotation
   gtsam::Point3 t = { gt_translation.GetX() - this_robot_pose.translation().x(), 
                       gt_translation.GetY() - this_robot_pose.translation().y(),
                       gt_translation.GetZ() - this_robot_pose.translation().z() };
   t = this_robot_pose.rotation().inverse() * t;

   // Add gaussian noise
   auto measurement = AddGaussianNoiseToMeasurement(R, t);

   // Isotropic noise model
   Eigen::VectorXd sigmas(6);
   sigmas << rotation_noise_std_, rotation_noise_std_, rotation_noise_std_, 
            translation_noise_std_, translation_noise_std_, translation_noise_std_;
   gtsam::SharedNoiseModel noise_model = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

   // Initialize factor
   // Enforce an order for loop closure measurement. (lower_id, higher_id).
   gtsam::BetweenFactor<gtsam::Pose3> new_factor;
   if (other_robot_symbol.chr() > this_robot_symbol.chr()) {
      new_factor = gtsam::BetweenFactor<gtsam::Pose3>(this_robot_symbol, other_robot_symbol, measurement, noise_model);

      UpdateCurrentLoopClosureBuzzStructure( this->GetBuzzVM()->robot,
                                             other_robot_id,
                                             this_robot_pose_id,
                                             other_robot_pose_id,
                                             measurement.x(),
                                             measurement.y(),
                                             measurement.z(),
                                             measurement.rotation().quaternion()[1],
                                             measurement.rotation().quaternion()[2],
                                             measurement.rotation().quaternion()[3],
                                             measurement.rotation().quaternion()[0] );
   } else {      
      measurement = measurement.inverse();
      new_factor = gtsam::BetweenFactor<gtsam::Pose3>(other_robot_symbol, this_robot_symbol, measurement, noise_model);

      UpdateCurrentLoopClosureBuzzStructure( other_robot_id,
                                             this->GetBuzzVM()->robot,
                                             other_robot_pose_id,
                                             this_robot_pose_id,
                                             -measurement.x(),
                                             -measurement.y(),
                                             measurement.z(),
                                             measurement.rotation().quaternion()[1],
                                             measurement.rotation().quaternion()[2],
                                             measurement.rotation().quaternion()[3],
                                             measurement.rotation().quaternion()[0] );
   }   

   // Add new factor to local pose graph
   local_pose_graph_.push_back(new_factor);
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapperNoSensing::SavePoseGroundTruth(){
   gtsam::Point3 t_gt = {  m_pcPos->GetReading().Position.GetX(), 
                           m_pcPos->GetReading().Position.GetY(),
                           m_pcPos->GetReading().Position.GetZ() };
   
   CQuaternion current_orientation = m_pcPos->GetReading().Orientation;
   gtsam::Quaternion current_quat_gtsam(current_orientation.GetW(), current_orientation.GetX(), current_orientation.GetY(), current_orientation.GetZ());
   gtsam::Rot3 R_gt(current_quat_gtsam);

   gtsam::Pose3 pose_gt(R_gt, t_gt);
   ground_truth_poses_.insert(std::make_pair(number_of_poses_, pose_gt));   
}

/****************************************/
/****************************************/

buzzvm_state CBuzzControllerQuadMapperNoSensing::RegisterFunctions() {
   CBuzzControllerQuadMapper::RegisterFunctions();

   /* Register mapping without sensing specific functions */
   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "compute_fake_rendezvous_loop_closures", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzComputeFakeRendezVousLoopClosures));
   buzzvm_gstore(m_tBuzzVM);
   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "move_forward_fake_odometry", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzMoveForwardFakeOdometry));
   buzzvm_gstore(m_tBuzzVM);

   return m_tBuzzVM->state;
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CBuzzControllerQuadMapperNoSensing, "buzz_controller_quadmapper_no_sensing");
