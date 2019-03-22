#include "buzz_controller_quadmapper_no_sensing.h"
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <cmath> 

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

   // Initialize constant attributes // TODO: add paramaters for them or get them by buzz
   sensor_range_ = 15;
   outlier_probability_ = 0.0; // TODO: Currently no outlier until I integrate the distributed pairwise consistency maximization.
   number_of_outliers_added_ = 0;

   // Initialize random numbers generators
   srand(time(NULL));
   gen_translation_ = std::mt19937{rd_()};
   gen_rotation_ = std::mt19937{rd_()};
   gen_outliers_ = std::mt19937{rd_()};
   normal_distribution_translation_ = std::normal_distribution<>{0, translation_noise_std_};
   normal_distribution_rotation_ = std::normal_distribution<>{0, rotation_noise_std_};
   uniform_distribution_outliers_translation_ = std::uniform_real_distribution<>{0, sensor_range_};
   uniform_distribution_outliers_rotation_ = std::uniform_real_distribution<>{-M_PI, M_PI};
   uniform_distribution_draw_outlier_ = std::uniform_real_distribution<>{0, 1};
   simulation_step_ = 0;
   previous_simulation_gt_pose_ = m_pcPos->GetReading();

   // Save ground truth for fake separator creation
   SavePoseGroundTruth();
}

/****************************************/
/****************************************/

static int BuzzComputeFakeRendezVousSeparator(buzzvm_t vm) {
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
                      "wrong parameter type for compute_fake_rendezvous_separator."
         );
      return vm->state;
   }
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   int is_outlier = reinterpret_cast<CBuzzControllerQuadMapperNoSensing*>(buzzvm_stack_at(vm, 1)->u.value)->ComputeNoisyFakeSeparatorMeasurement(gt_orientation, gt_translation, other_robot_pose_id, other_robot_id, this_robot_pose_id);
   buzzvm_pushi(vm, is_outlier);

   return buzzvm_ret1(vm);
}

/****************************************/
/****************************************/

static int BuzzMoveForwardFakeOdometry(buzzvm_t vm) {
   /* Push the vector components */
   buzzvm_lload(vm, 1);
   buzzvm_lload(vm, 2);
   /* Create a new vector with that */
   CVector3 translation;
   int simulation_time_divider;
   buzzobj_t step = buzzvm_stack_at(vm, 2);
   buzzobj_t b_simulation_time_divider = buzzvm_stack_at(vm, 1);
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
   if(b_simulation_time_divider->o.type == BUZZTYPE_INT) simulation_time_divider = b_simulation_time_divider->i.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "move_forward(x,y): expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[b_simulation_time_divider->o.type]
         );
      return vm->state;
   }    
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   int current_pose_id = reinterpret_cast<CBuzzControllerQuadMapperNoSensing*>(buzzvm_stack_at(vm, 1)->u.value)->MoveForwardFakeOdometry(translation, simulation_time_divider);
   
   buzzvm_pushi(vm, current_pose_id);

   return buzzvm_ret1(vm);
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
      ComputeNoisyFakeOdometryMeasurement();

      // Log data
      WriteDataset(this->GetBuzzVM()->robot);   
   }

   return number_of_poses_;
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapperNoSensing::ComputeNoisyFakeOdometryMeasurement() {
   
   // Extract info
   CQuaternion previous_orientation = previous_simulation_gt_pose_.Orientation;
   CQuaternion current_orientation = m_pcPos->GetReading().Orientation;
   CVector3 previous_position = previous_simulation_gt_pose_.Position;
   CVector3 current_position = m_pcPos->GetReading().Position;

   // Increase the number of poses
   IncrementNumberOfPosesAndUpdateState();

   // Next symbol
   gtsam::Symbol current_symbol_ = gtsam::Symbol(robot_id_char_, number_of_poses_);

   // Conversion of the previous orientation (quaternion to rotation matrix)
   gtsam::Quaternion previous_quat_gtsam(previous_orientation.GetW(), previous_orientation.GetX(), previous_orientation.GetY(), previous_orientation.GetZ());
   gtsam::Rot3 previous_R(previous_quat_gtsam);
   
   // Conversion of the current orientation (quaternion to rotation matrix)
   gtsam::Quaternion current_quat_gtsam(current_orientation.GetW(), current_orientation.GetX(), current_orientation.GetY(), current_orientation.GetZ());
   gtsam::Rot3 current_R(current_quat_gtsam);

   // Compute transformation between rotations
   gtsam::Rot3 R = previous_R.inverse() * current_R;

   // Convert translation information to gtsam format and perform the appropriate rotation
   gtsam::Point3 t = {  current_position.GetX() - previous_position.GetX(), 
                        current_position.GetY() - previous_position.GetY(),
                        current_position.GetZ() - previous_position.GetZ()};
   t = previous_R.inverse() * t;

   // Add gaussian noise
   auto measurement = AddGaussianNoiseToMeasurement(R, t);

   // Initialize factor
   gtsam::BetweenFactor<gtsam::Pose3> new_factor(previous_symbol_, current_symbol_, measurement, noise_model_);

   // Update attributes value
   previous_simulation_gt_pose_ = m_pcPos->GetReading();
   previous_pose_ = previous_pose_ * measurement;
   previous_symbol_ = current_symbol_;

   // Add new factor to local pose graph
   local_pose_graph_->push_back(new_factor);

   // Add new pose estimate into initial guess
   poses_initial_guess_->insert(previous_symbol_.key(), previous_pose_);

   // Save ground truth for fake separator creation
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

gtsam::Pose3 CBuzzControllerQuadMapperNoSensing::OutlierMeasurement(const gtsam::Rot3& R, const gtsam::Point3& t) {
   
   // TODO: Add option to add noise greater than 3 or 5 sigmas, instead of totally random measurment
   // This is why this method takes the measurment in parameter.
   gtsam::Point3 t_outlier = {   uniform_distribution_outliers_translation_(gen_outliers_),  
                                 uniform_distribution_outliers_translation_(gen_outliers_),
                                 uniform_distribution_outliers_translation_(gen_outliers_) };


   gtsam::Rot3 R_outlier = gtsam::Rot3::Ypr( uniform_distribution_outliers_rotation_(gen_outliers_), 
                                             uniform_distribution_outliers_rotation_(gen_outliers_),
                                             uniform_distribution_outliers_rotation_(gen_outliers_));

   number_of_outliers_added_++;

   return gtsam::Pose3(R_outlier, t_outlier);
}

/****************************************/
/****************************************/

int CBuzzControllerQuadMapperNoSensing::ComputeNoisyFakeSeparatorMeasurement(const CQuaternion& gt_orientation, const CVector3& gt_translation, 
                                                               const int& other_robot_pose_id, const int& other_robot_id, const int& this_robot_pose_id) {
   // Separator symbols
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

   // Add gaussian noise or make it an outlier
   gtsam::Pose3 measurement;
   int is_outlier = 0;
   if ( uniform_distribution_draw_outlier_(gen_outliers_) < outlier_probability_) {
      measurement = OutlierMeasurement(R, t);
      is_outlier = 1;
   } else {
      measurement = AddGaussianNoiseToMeasurement(R, t);
   }

   // Initialize factor
   // Enforce an order for separator measurement. (lower_id, higher_id).
   gtsam::BetweenFactor<gtsam::Pose3> new_factor;
   if (other_robot_symbol.chr() > this_robot_symbol.chr()) {
      new_factor = gtsam::BetweenFactor<gtsam::Pose3>(this_robot_symbol, other_robot_symbol, measurement, noise_model_);

      UpdateCurrentSeparatorBuzzStructure( this->GetBuzzVM()->robot,
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
      new_factor = gtsam::BetweenFactor<gtsam::Pose3>(other_robot_symbol, this_robot_symbol, measurement, noise_model_);

      UpdateCurrentSeparatorBuzzStructure( other_robot_id,
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
   local_pose_graph_->push_back(new_factor);

   return is_outlier;
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
   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "compute_fake_rendezvous_separator", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzComputeFakeRendezVousSeparator));
   buzzvm_gstore(m_tBuzzVM);
   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "move_forward_fake_odometry", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzMoveForwardFakeOdometry));
   buzzvm_gstore(m_tBuzzVM);

   return m_tBuzzVM->state;
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CBuzzControllerQuadMapperNoSensing, "buzz_controller_quadmapper_no_sensing");
