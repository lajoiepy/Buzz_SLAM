#include "buzz_slam_singleton.h"

namespace buzz_slam {

/****************************************/
/************ Buzz Closures *************/
/****************************************/

static int BuzzInitOptimizer(buzzvm_t vm){

   // Initialize optimizer
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->InitOptimizer();

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzOptimizerState(buzzvm_t vm){

   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   int state = BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->GetOptimizerState();
   buzzvm_pushi(vm, state);

   return buzzvm_ret1(vm);
}

/****************************************/
/****************************************/

static int BuzzOptimizerTick(buzzvm_t vm) {
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->OptimizerTick();
   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzOptimizerPhase(buzzvm_t vm){

   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   int phase = BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->GetOptimizerPhase();
   buzzvm_pushi(vm, phase);

   return buzzvm_ret1(vm);
}

/****************************************/
/****************************************/

static int BuzzCheckIfAllEstimationDoneAndReset(buzzvm_t vm) {
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->CheckIfAllEstimationDoneAndReset();
   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzAddNeighborWithinCommunicationRange(buzzvm_t vm){

   buzzvm_lload(vm, 1);
   
   buzzobj_t buzz_rid = buzzvm_stack_at(vm, 1);
   int rid;

   if(buzz_rid->o.type == BUZZTYPE_INT) rid = buzz_rid->i.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "AddNeighborWithinCommunicationRange: expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[buzz_rid->o.type]
         );
      return vm->state;
   } 

   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->AddNeighborWithinCommunicationRange(rid);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzComputeAndUpdateRotationEstimatesToSend(buzzvm_t vm){

   buzzvm_lload(vm, 1);
   
   buzzobj_t buzz_rid = buzzvm_stack_at(vm, 1);
   int rid;

   if(buzz_rid->o.type == BUZZTYPE_INT) rid = buzz_rid->i.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "BuzzComputeAndUpdateRotationEstimatesToSend: expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[buzz_rid->o.type]
         );
      return vm->state;
   } 

   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->ComputeAndUpdateRotationEstimatesToSend(rid);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzUpdateNeighborRotationEstimates(buzzvm_t vm){

   buzzvm_lnum_assert(vm, 1);

   buzzvm_lload(vm, 1);

   buzzvm_type_assert(vm, 1, BUZZTYPE_TABLE);

   std::vector<std::vector<rotation_estimate_t>> received_rotation_estimates;

   buzzobj_t b_rotation_estimates_table = buzzvm_stack_at(vm, 1);

   for (int32_t i = 0; i < buzzdict_size(b_rotation_estimates_table->t.value); ++i) {
      buzzvm_dup(vm);
      buzzvm_pushi(vm, i);
      buzzvm_tget(vm);
      buzzobj_t b_rotation_estimates_from_robot_i = buzzvm_stack_at(vm, 1);

      received_rotation_estimates.emplace_back(std::vector<rotation_estimate_t>());
      for (int32_t j = 0; j < buzzdict_size(b_rotation_estimates_from_robot_i->t.value); ++j) {
         buzzvm_dup(vm);
         buzzvm_pushi(vm, j);
         buzzvm_tget(vm);
         buzzobj_t b_individual_rotation_estimate_j = buzzvm_stack_at(vm, 1);

         rotation_estimate_t rotation_estimate;

         buzzvm_dup(vm);
         buzzvm_pushs(vm, buzzvm_string_register(vm, "sender_robot_id", 1));
         buzzvm_tget(vm);
         buzzobj_t b_sender_robot_id = buzzvm_stack_at(vm, 1);
         rotation_estimate.sender_robot_id = b_sender_robot_id->i.value;
         buzzvm_pop(vm);

         buzzvm_dup(vm);
         buzzvm_pushs(vm, buzzvm_string_register(vm, "receiver_robot_id", 1));
         buzzvm_tget(vm);
         buzzobj_t b_receiver_robot_id = buzzvm_stack_at(vm, 1);
         rotation_estimate.receiver_robot_id = b_receiver_robot_id->i.value;
         buzzvm_pop(vm);

         buzzvm_dup(vm);
         buzzvm_pushs(vm, buzzvm_string_register(vm, "sender_pose_id", 1));
         buzzvm_tget(vm);
         buzzobj_t b_sender_pose_id = buzzvm_stack_at(vm, 1);
         buzzvm_pop(vm);
         rotation_estimate.sender_pose_id = b_sender_pose_id->i.value;

         buzzvm_dup(vm);
         buzzvm_pushs(vm, buzzvm_string_register(vm, "sender_robot_is_initialized", 1));
         buzzvm_tget(vm);
         buzzobj_t b_sender_robot_is_initialized = buzzvm_stack_at(vm, 1);
         buzzvm_pop(vm);
         rotation_estimate.sender_robot_is_initialized = (bool)b_sender_robot_is_initialized->i.value;

         buzzvm_dup(vm);
         buzzvm_pushs(vm, buzzvm_string_register(vm, "sender_estimation_is_done", 1));
         buzzvm_tget(vm);
         buzzobj_t b_sender_estimation_is_done = buzzvm_stack_at(vm, 1);
         buzzvm_pop(vm);
         rotation_estimate.sender_estimation_is_done = (bool)b_sender_estimation_is_done->i.value;

         buzzvm_dup(vm);
         buzzvm_pushs(vm, buzzvm_string_register(vm, "rotation_estimate", 1));
         buzzvm_tget(vm);
         buzzobj_t b_rotation_estimate = buzzvm_stack_at(vm, 1);
         for (int32_t k = 0; k < buzzdict_size(b_rotation_estimate->t.value); ++k) {
            buzzvm_dup(vm);
            buzzvm_pushi(vm, k);
            buzzvm_tget(vm);
            buzzobj_t b_rotation_matrix_elem = buzzvm_stack_at(vm, 1);
            buzzvm_pop(vm);
            rotation_estimate.rotation_matrix[k] = b_rotation_matrix_elem->f.value;
         }
         buzzvm_pop(vm);
         buzzvm_pop(vm);
         received_rotation_estimates.at(i).emplace_back(rotation_estimate);
      }
      buzzvm_pop(vm);
   }

   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->UpdateNeighborRotationEstimates(received_rotation_estimates);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzEstimateRotationAndUpdateRotation(buzzvm_t vm) {
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->EstimateRotationAndUpdateRotation();
   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzComputeAndUpdatePoseEstimatesToSend(buzzvm_t vm){

   buzzvm_lload(vm, 1);
   
   buzzobj_t buzz_rid = buzzvm_stack_at(vm, 1);
   int rid;

   if(buzz_rid->o.type == BUZZTYPE_INT) rid = buzz_rid->i.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "BuzzComputeAndUpdatePoseEstimatesToSend: expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[buzz_rid->o.type]
         );
      return vm->state;
   } 

   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->ComputeAndUpdatePoseEstimatesToSend(rid);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzUpdateNeighborPoseEstimates(buzzvm_t vm){

   buzzvm_lnum_assert(vm, 1);

   buzzvm_lload(vm, 1);

   buzzvm_type_assert(vm, 1, BUZZTYPE_TABLE);

   std::vector<std::vector<pose_estimate_t>> received_pose_estimates;

   buzzobj_t b_pose_estimates_table = buzzvm_stack_at(vm, 1);

   for (int32_t i = 0; i < buzzdict_size(b_pose_estimates_table->t.value); ++i) {
      buzzvm_dup(vm);
      buzzvm_pushi(vm, i);
      buzzvm_tget(vm);
      buzzobj_t b_pose_estimates_from_robot_i = buzzvm_stack_at(vm, 1);

      received_pose_estimates.emplace_back(std::vector<pose_estimate_t>());
      for (int32_t j = 0; j < buzzdict_size(b_pose_estimates_from_robot_i->t.value); ++j) {
         buzzvm_dup(vm);
         buzzvm_pushi(vm, j);
         buzzvm_tget(vm);
         buzzobj_t b_individual_pose_estimate_j = buzzvm_stack_at(vm, 1);

         pose_estimate_t pose_estimate;

         buzzvm_dup(vm);
         buzzvm_pushs(vm, buzzvm_string_register(vm, "sender_robot_id", 1));
         buzzvm_tget(vm);
         buzzobj_t b_sender_robot_id = buzzvm_stack_at(vm, 1);
         pose_estimate.sender_robot_id = b_sender_robot_id->i.value;
         buzzvm_pop(vm);

         buzzvm_dup(vm);
         buzzvm_pushs(vm, buzzvm_string_register(vm, "receiver_robot_id", 1));
         buzzvm_tget(vm);
         buzzobj_t b_receiver_robot_id = buzzvm_stack_at(vm, 1);
         pose_estimate.receiver_robot_id = b_receiver_robot_id->i.value;
         buzzvm_pop(vm);

         buzzvm_dup(vm);
         buzzvm_pushs(vm, buzzvm_string_register(vm, "sender_pose_id", 1));
         buzzvm_tget(vm);
         buzzobj_t b_sender_pose_id = buzzvm_stack_at(vm, 1);
         buzzvm_pop(vm);
         pose_estimate.sender_pose_id = b_sender_pose_id->i.value;

         buzzvm_dup(vm);
         buzzvm_pushs(vm, buzzvm_string_register(vm, "sender_robot_is_initialized", 1));
         buzzvm_tget(vm);
         buzzobj_t b_sender_robot_is_initialized = buzzvm_stack_at(vm, 1);
         buzzvm_pop(vm);
         pose_estimate.sender_robot_is_initialized = (bool)b_sender_robot_is_initialized->i.value;

         buzzvm_dup(vm);
         buzzvm_pushs(vm, buzzvm_string_register(vm, "sender_estimation_is_done", 1));
         buzzvm_tget(vm);
         buzzobj_t b_sender_estimation_is_done = buzzvm_stack_at(vm, 1);
         buzzvm_pop(vm);
         pose_estimate.sender_estimation_is_done = (bool)b_sender_estimation_is_done->i.value;

         buzzvm_dup(vm);
         buzzvm_pushs(vm, buzzvm_string_register(vm, "pose_estimate", 1));
         buzzvm_tget(vm);
         buzzobj_t b_pose_estimate = buzzvm_stack_at(vm, 1);
         for (int32_t k = 0; k < buzzdict_size(b_pose_estimate->t.value); ++k) {
            buzzvm_dup(vm);
            buzzvm_pushi(vm, k);
            buzzvm_tget(vm);
            buzzobj_t b_pose_matrix_elem = buzzvm_stack_at(vm, 1);
            buzzvm_pop(vm);
            pose_estimate.pose_data[k] = b_pose_matrix_elem->f.value;
         }
         buzzvm_pop(vm);
         buzzvm_pop(vm);
         received_pose_estimates.at(i).emplace_back(pose_estimate);
      }
      buzzvm_pop(vm);
   }

   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->UpdateNeighborPoseEstimates(received_pose_estimates);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzEstimatePoseAndUpdatePose(buzzvm_t vm) {
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->EstimatePoseAndUpdatePose();
   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzUpdateCurrentPoseEstimate(buzzvm_t vm) {buzzvm_lload(vm, 1);
   
   buzzobj_t buzz_pose_id = buzzvm_stack_at(vm, 1);
   int pose_id;

   if(buzz_pose_id->o.type == BUZZTYPE_INT) pose_id = buzz_pose_id->i.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "BuzzUpdateCurrentPoseEstimate: expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[buzz_pose_id->o.type]
         );
      return vm->state;
   } 

   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->UpdateCurrentPoseEstimate(pose_id);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzUpdatePoseEstimateFromNeighbor(buzzvm_t vm){

   buzzvm_lnum_assert(vm, 3);

   buzzvm_lload(vm, 1);
   buzzvm_lload(vm, 2);
   buzzvm_lload(vm, 3);
   buzzobj_t b_rid = buzzvm_stack_at(vm, 3);
   buzzobj_t b_pose_id = buzzvm_stack_at(vm, 2);

   buzzvm_type_assert(vm, 3, BUZZTYPE_INT);
   buzzvm_type_assert(vm, 2, BUZZTYPE_INT);
   buzzvm_type_assert(vm, 1, BUZZTYPE_TABLE);

   buzzobj_t b_estimate = buzzvm_stack_at(vm, 1);

   int table_size = buzzdict_size(b_estimate->t.value);
   
   double estimate_elem;
   std::vector<double> pose_estimate;
   for (int32_t i = 0; i < table_size; ++i) {
      buzzvm_dup(vm);
      buzzvm_pushi(vm, i);
      buzzvm_tget(vm);
      buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
      buzzobj_t b_estimate_elem = buzzvm_stack_at(vm, 1);
      buzzvm_pop(vm);
      estimate_elem = b_estimate_elem->f.value;
      pose_estimate.emplace_back(estimate_elem);
   }

   gtsam::Matrix4 estimate_matrix; 
   for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
         estimate_matrix(i,j) = pose_estimate.at(i*4+j);
      }
   }
   gtsam::Pose3 pose(estimate_matrix);

   gtsam::Matrix6 covariance_matrix;
   for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
         covariance_matrix(i,j) = pose_estimate.at(16+i*6+j);
      }
   }

   graph_utils::PoseWithCovariance pose_with_covariance;
   pose_with_covariance.pose = pose;
   pose_with_covariance.covariance_matrix = covariance_matrix;

   int rid = b_rid->i.value;
   int pose_id = b_pose_id->i.value;
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->UpdatePoseEstimateFromNeighbor(rid, pose_id, pose_with_covariance);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzSRand(buzzvm_t vm){

   buzzvm_lload(vm, 1);
   
   buzzobj_t buzz_seed = buzzvm_stack_at(vm, 1);
   int seed;

   if(buzz_seed->o.type == BUZZTYPE_INT) seed = buzz_seed->i.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "srand(x,y): expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[buzz_seed->o.type]
         );
      return vm->state;
   } 

   srand(time(NULL)+seed);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzRandUniform(buzzvm_t vm){

   buzzvm_lload(vm, 1);
   buzzvm_lload(vm, 2);
   buzzobj_t buzz_range_lowest = buzzvm_stack_at(vm, 2);
   buzzobj_t buzz_range_highest = buzzvm_stack_at(vm, 1);
   int range_lowest, range_highest;

   if(buzz_range_lowest->o.type == BUZZTYPE_INT) range_lowest = buzz_range_lowest->i.value;
   else if(buzz_range_lowest->o.type == BUZZTYPE_FLOAT) range_lowest = (int)buzz_range_lowest->f.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "rand_uniform(x,y): expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[buzz_range_lowest->o.type]
         );
      return vm->state;
   } 
   if(buzz_range_highest->o.type == BUZZTYPE_INT) range_highest = buzz_range_highest->i.value;
   else if(buzz_range_highest->o.type == BUZZTYPE_FLOAT) range_highest = (int)buzz_range_highest->f.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "rand_uniform(x,y): expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[buzz_range_highest->o.type]
         );
      return vm->state;
   } 

   float random_value = (rand()%(range_highest-range_lowest)+range_lowest);

   buzzvm_pushf(vm, random_value);

   return buzzvm_ret1(vm);
}

/****************************************/
/****************************************/

static int BuzzAddSeparatorToLocalGraph(buzzvm_t vm) {
   /* Push the vector components */
   buzzvm_lload(vm, 1);
   buzzvm_lload(vm, 2);
   buzzvm_lload(vm, 3);
   buzzvm_lload(vm, 4);
   buzzvm_lload(vm, 5);
   buzzvm_lload(vm, 6);
   buzzvm_lload(vm, 7);
   buzzvm_lload(vm, 8);
   buzzvm_lload(vm, 9);
   buzzvm_lload(vm, 10);
   buzzvm_lload(vm, 11);
   buzzvm_lload(vm, 12);
   /* Retrieve parameters and check their types */
   buzzobj_t b_robot_1_id = buzzvm_stack_at(vm, 12);
   buzzobj_t b_robot_2_id = buzzvm_stack_at(vm, 11);
   buzzobj_t b_robot_1_pose_id = buzzvm_stack_at(vm, 10);
   buzzobj_t b_robot_2_pose_id = buzzvm_stack_at(vm, 9);
   buzzobj_t b_x = buzzvm_stack_at(vm, 8);
   buzzobj_t b_y = buzzvm_stack_at(vm, 7);
   buzzobj_t b_z = buzzvm_stack_at(vm, 6);
   buzzobj_t b_q_x = buzzvm_stack_at(vm, 5);
   buzzobj_t b_q_y = buzzvm_stack_at(vm, 4);
   buzzobj_t b_q_z = buzzvm_stack_at(vm, 3);
   buzzobj_t b_q_w = buzzvm_stack_at(vm, 2);
   buzzobj_t b_covariance_matrix = buzzvm_stack_at(vm, 1);
   int robot_1_id, robot_2_id, robot_1_pose_id, robot_2_pose_id;
   float x, y, z, q_x, q_y, q_z, q_w;
   std::vector<double> covariance_values;
   if(b_robot_1_id->o.type == BUZZTYPE_INT &&
      b_robot_2_id->o.type == BUZZTYPE_INT &&
      b_robot_1_pose_id->o.type == BUZZTYPE_INT &&
      b_robot_2_pose_id->o.type == BUZZTYPE_INT &&
      b_x->o.type == BUZZTYPE_FLOAT &&
      b_y->o.type == BUZZTYPE_FLOAT &&
      b_z->o.type == BUZZTYPE_FLOAT &&
      b_q_x->o.type == BUZZTYPE_FLOAT &&
      b_q_y->o.type == BUZZTYPE_FLOAT &&
      b_q_z->o.type == BUZZTYPE_FLOAT &&
      b_q_w->o.type == BUZZTYPE_FLOAT &&
      b_covariance_matrix->o.type == BUZZTYPE_TABLE) {

      // Fill in variables
      robot_1_id = b_robot_1_id->i.value;
      robot_2_id = b_robot_2_id->i.value;
      robot_1_pose_id = b_robot_1_pose_id->i.value;
      robot_2_pose_id = b_robot_2_pose_id->i.value;
      x = b_x->f.value;
      y = b_y->f.value;
      z = b_z->f.value;
      q_x = b_q_x->f.value;
      q_y = b_q_y->f.value;
      q_z = b_q_z->f.value;
      q_w = b_q_w->f.value;

      int table_size = buzzdict_size(b_covariance_matrix->t.value);
      double covariance_elem;
      for (int32_t i = 0; i < table_size; ++i) {
         buzzvm_dup(vm);
         buzzvm_pushi(vm, i);
         buzzvm_tget(vm);
         buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
         buzzobj_t b_covariance_value = buzzvm_stack_at(vm, 1);
         buzzvm_pop(vm);
         covariance_elem = b_covariance_value->f.value;
         covariance_values.emplace_back(covariance_elem);
      }

   } else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "wrong parameter type for add_separator_to_local_graph."
         );
      return vm->state;
   }

   gtsam::Matrix6 covariance_matrix;
   for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
         covariance_matrix(i,j) = covariance_values.at(i*6+j);
      }
   }

   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->AddSeparatorToLocalGraph( robot_1_id, robot_2_id,
                                                                                                            robot_1_pose_id, robot_2_pose_id,
                                                                                                            x, y, z,
                                                                                                            q_x, q_y, q_z, q_w,
                                                                                                            covariance_matrix );
   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzLoadParameters(buzzvm_t vm) {
   /* Push the vector components */
   buzzvm_lload(vm, 1);
   buzzvm_lload(vm, 2);
   buzzvm_lload(vm, 3);
   buzzvm_lload(vm, 4);
   buzzvm_lload(vm, 5);
   buzzvm_lload(vm, 6);
   buzzvm_lload(vm, 7);
   buzzvm_lload(vm, 8);
   buzzvm_lload(vm, 9);
   buzzvm_lload(vm, 10);
   buzzvm_lload(vm, 11);
   buzzvm_lload(vm, 12);
   buzzvm_lload(vm, 13);
   buzzvm_lload(vm, 14);
   buzzvm_lload(vm, 15);
   buzzvm_lload(vm, 16);
   /* Retrieve parameters and check their types */
   buzzobj_t b_optimizer_period = buzzvm_stack_at(vm, 15);
   buzzobj_t b_max_steps_rotation = buzzvm_stack_at(vm, 15);
   buzzobj_t b_max_steps_pose = buzzvm_stack_at(vm, 14);
   buzzobj_t b_number_of_steps_before_failsafe = buzzvm_stack_at(vm, 13);
   buzzobj_t b_use_pcm = buzzvm_stack_at(vm, 12);
   buzzobj_t b_confidence_probability = buzzvm_stack_at(vm, 11);
   buzzobj_t b_incremental_solving = buzzvm_stack_at(vm, 10);
   buzzobj_t b_debug = buzzvm_stack_at(vm, 9);
   buzzobj_t b_rotation_noise_std = buzzvm_stack_at(vm, 8);
   buzzobj_t b_translation_noise_std = buzzvm_stack_at(vm, 7);
   buzzobj_t b_rotation_estimate_change_threshold = buzzvm_stack_at(vm, 6);
   buzzobj_t b_pose_estimate_change_threshold = buzzvm_stack_at(vm, 5);
   buzzobj_t b_use_flagged_initialization = buzzvm_stack_at(vm, 4);
   buzzobj_t b_is_simulation = buzzvm_stack_at(vm, 3);
   buzzobj_t b_number_of_robots = buzzvm_stack_at(vm, 2);
   buzzobj_t b_error_file_name = buzzvm_stack_at(vm, 1);
   float rotation_noise_std, translation_noise_std, 
      rotation_estimate_change_threshold, pose_estimate_change_threshold;
   bool use_flagged_initialization, is_simulation, incremental_solving;
   int number_of_robots, debug_level;
   std::string error_file_name;
   double confidence_probability;
   bool use_pcm;
   int number_of_steps_before_failsafe;
   int max_steps_rotation, max_steps_pose;
   int optimizer_period;

   if(b_rotation_noise_std->o.type == BUZZTYPE_FLOAT &&
      b_translation_noise_std->o.type == BUZZTYPE_FLOAT &&
      b_rotation_estimate_change_threshold->o.type == BUZZTYPE_FLOAT &&
      b_pose_estimate_change_threshold->o.type == BUZZTYPE_FLOAT &&
      b_use_flagged_initialization->o.type == BUZZTYPE_INT &&
      b_is_simulation->o.type == BUZZTYPE_INT &&
      b_number_of_robots->o.type == BUZZTYPE_INT &&
      b_error_file_name->o.type == BUZZTYPE_STRING &&
      b_debug->o.type == BUZZTYPE_INT &&
      b_incremental_solving->o.type == BUZZTYPE_INT &&
      b_confidence_probability->o.type == BUZZTYPE_FLOAT &&
      b_use_pcm->o.type == BUZZTYPE_INT &&
      b_number_of_steps_before_failsafe->o.type == BUZZTYPE_INT &&
      b_max_steps_rotation->o.type == BUZZTYPE_INT &&
      b_max_steps_pose->o.type == BUZZTYPE_INT &&
      b_optimizer_period->o.type == BUZZTYPE_INT) {

      // Fill in variables
      rotation_noise_std = b_rotation_noise_std->f.value;
      translation_noise_std = b_translation_noise_std->f.value;
      rotation_estimate_change_threshold = b_rotation_estimate_change_threshold->f.value;
      pose_estimate_change_threshold = b_pose_estimate_change_threshold->f.value;
      use_flagged_initialization = (bool) b_use_flagged_initialization->i.value;
      is_simulation = (bool) b_is_simulation->i.value;
      number_of_robots = b_number_of_robots->i.value;
      error_file_name = b_error_file_name->s.value.str;
      debug_level = b_debug->i.value;
      incremental_solving = b_incremental_solving->i.value;
      confidence_probability = b_confidence_probability->f.value;
      use_pcm = (bool) b_use_pcm->i.value;
      number_of_steps_before_failsafe = b_number_of_steps_before_failsafe->i.value;
      max_steps_rotation = b_max_steps_rotation->i.value;
      max_steps_pose = b_max_steps_pose->i.value;
      optimizer_period = b_optimizer_period->i.value;

   } else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "wrong parameter type for load_cpp_controller_parameters."
         );
      return vm->state;
   }
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->LoadParameters(optimizer_period, number_of_steps_before_failsafe,
                     use_pcm, confidence_probability, incremental_solving, debug_level,
                     rotation_noise_std, translation_noise_std,
                     rotation_estimate_change_threshold, pose_estimate_change_threshold,
                     use_flagged_initialization, is_simulation,
                     number_of_robots, error_file_name,
                     max_steps_rotation, max_steps_pose);
   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzRotationEstimationStoppingConditions(buzzvm_t vm){

   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   bool is_finished = BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->RotationEstimationStoppingConditions();
   buzzvm_pushi(vm, is_finished);

   return buzzvm_ret1(vm);
}

/****************************************/
/****************************************/

static int BuzzPoseEstimationStoppingConditions(buzzvm_t vm){

   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   bool is_finished = BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->PoseEstimationStoppingConditions();
   buzzvm_pushi(vm, is_finished);

   return buzzvm_ret1(vm);
}

/****************************************/
/****************************************/

static int BuzzNeighborRotationEstimationIsFinished(buzzvm_t vm){

   buzzvm_lload(vm, 1);
   
   buzzobj_t buzz_rid = buzzvm_stack_at(vm, 1);
   int rid;

   if(buzz_rid->o.type == BUZZTYPE_INT) rid = buzz_rid->i.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "BuzzNeighborRotationEstimationIsFinished: expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[buzz_rid->o.type]
         );
      return vm->state;
   } 

   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->NeighborRotationEstimationIsFinished(rid);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzNeighborPoseEstimationIsFinished(buzzvm_t vm){

   buzzvm_lload(vm, 1);
   buzzvm_lload(vm, 2);
   
   buzzobj_t buzz_rid = buzzvm_stack_at(vm, 2);
   int rid;
   buzzobj_t buzz_anchor_offset = buzzvm_stack_at(vm, 1);
   gtsam::Point3 anchor_offset;

   if(buzz_rid->o.type == BUZZTYPE_INT &&
      buzz_anchor_offset->o.type == BUZZTYPE_TABLE) {
      rid = buzz_rid->i.value;

      int table_size = buzzdict_size(buzz_anchor_offset->t.value);
      double anchor_elem;
      std::vector<double> anchor_elems;
      for (int32_t i = 0; i < table_size; ++i) {
         buzzvm_dup(vm);
         buzzvm_pushi(vm, i);
         buzzvm_tget(vm);
         buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
         buzzobj_t b_anchor_value = buzzvm_stack_at(vm, 1);
         buzzvm_pop(vm);
         anchor_elem = b_anchor_value->f.value;
         anchor_elems.emplace_back(anchor_elem);
      }
      anchor_offset = gtsam::Point3(anchor_elems[0], anchor_elems[1], anchor_elems[2]);

   } 
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "BuzzNeighborPoseEstimationIsFinished: expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[buzz_rid->o.type]
         );
      return vm->state;
   } 

   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->NeighborPoseEstimationIsFinished(rid, anchor_offset);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzNeighborState(buzzvm_t vm){

   buzzvm_lload(vm, 1);
   buzzvm_lload(vm, 2);
   buzzvm_lload(vm, 3);
   
   buzzobj_t buzz_rid = buzzvm_stack_at(vm, 3);
   int rid;
   buzzobj_t buzz_state = buzzvm_stack_at(vm, 2);
   int state;
   buzzobj_t buzz_lowest_id = buzzvm_stack_at(vm, 1);
   int lowest_id;

   if(buzz_rid->o.type == BUZZTYPE_INT &&
      buzz_state->o.type == BUZZTYPE_INT &&
      buzz_lowest_id->o.type == BUZZTYPE_INT) {
      rid = buzz_rid->i.value;
      state = buzz_state->i.value;
      lowest_id = buzz_lowest_id->i.value;
   }
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "BuzzNeighborState: expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[buzz_rid->o.type]
         );
      return vm->state;
   } 

   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->NeighborState(rid, (buzz_slam::OptimizerState) state, lowest_id);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzUpdateHasSentStartOptimizationFlag(buzzvm_t vm) {
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->UpdateHasSentStartOptimizationFlag(true);
}

/****************************************/
/****************************************/

static int BuzzUpdateAdjacencyVector(buzzvm_t vm) {
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->UpdateAdjacencyVector();
}

/****************************************/
/****************************************/

static int BuzzReceiveAdjacencyVectorFromNeighbor(buzzvm_t vm) {
   buzzvm_lload(vm, 1);
   buzzvm_lload(vm, 2);
   
   buzzobj_t buzz_rid = buzzvm_stack_at(vm, 2);
   int rid;
   buzzobj_t buzz_adjacency_vector = buzzvm_stack_at(vm, 1);
   std::vector<int> adjacency_vector;

   if(buzz_rid->o.type == BUZZTYPE_INT &&
      buzz_adjacency_vector->o.type == BUZZTYPE_TABLE) {
      rid = buzz_rid->i.value;

      int table_size = buzzdict_size(buzz_adjacency_vector->t.value);
      int elem;
      for (int32_t i = 0; i < table_size; ++i) {
         buzzvm_dup(vm);
         buzzvm_pushi(vm, i);
         buzzvm_tget(vm);
         buzzvm_type_assert(vm, 1, BUZZTYPE_INT);
         buzzobj_t b_adjacency_value = buzzvm_stack_at(vm, 1);
         buzzvm_pop(vm);
         elem = b_adjacency_value->i.value;
         adjacency_vector.emplace_back(elem);
      }

   } 
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "BuzzNeighborPoseEstimationIsFinished: expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[buzz_rid->o.type]
         );
      return vm->state;
   } 

   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->ReceiveAdjacencyVectorFromNeighbor(rid, adjacency_vector);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzUpdateNeighborHasStartedOptimizationFlag(buzzvm_t vm) {


   buzzvm_lload(vm, 1);
   
   buzzobj_t buzz_rid = buzzvm_stack_at(vm, 1);
   int rid;

   if(buzz_rid->o.type == BUZZTYPE_INT) {
      rid = buzz_rid->i.value;
   }
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "BuzzUpdateNeighborHasStartedOptimizationFlag: expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[buzz_rid->o.type]
         );
      return vm->state;
   } 

   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAM>(vm->robot)->UpdateNeighborHasStartedOptimizationFlag(true, rid);
   return buzzvm_ret0(vm);
}

/****************************************/
/************ Registration **************/
/****************************************/

buzzvm_state BuzzSLAM::RegisterSLAMFunctions(buzzvm_t buzz_vm) {
   /* Register mapping specific functions */
   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "load_parameters", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzLoadParameters));
   buzzvm_gstore(buzz_vm);

   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "srand", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzSRand));
   buzzvm_gstore(buzz_vm);

   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "rand_uniform", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzRandUniform));
   buzzvm_gstore(buzz_vm);

   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "add_separator_to_local_graph", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzAddSeparatorToLocalGraph));
   buzzvm_gstore(buzz_vm);
   
   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "init_optimizer", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzInitOptimizer));
   buzzvm_gstore(buzz_vm);

   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "optimizer_state", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzOptimizerState));
   buzzvm_gstore(buzz_vm);

   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "optimizer_tick", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzOptimizerTick));
   buzzvm_gstore(buzz_vm);

   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "add_neighbor_within_communication_range", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzAddNeighborWithinCommunicationRange));
   buzzvm_gstore(buzz_vm);
   
   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "compute_and_update_rotation_estimates_to_send", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzComputeAndUpdateRotationEstimatesToSend));
   buzzvm_gstore(buzz_vm);
   
   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "compute_and_update_pose_estimates_to_send", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzComputeAndUpdatePoseEstimatesToSend));
   buzzvm_gstore(buzz_vm);

   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "update_neighbor_rotation_estimates", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzUpdateNeighborRotationEstimates));
   buzzvm_gstore(buzz_vm);

   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "update_neighbor_pose_estimates", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzUpdateNeighborPoseEstimates));
   buzzvm_gstore(buzz_vm);

   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "estimate_rotation_and_update_rotation", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzEstimateRotationAndUpdateRotation));
   buzzvm_gstore(buzz_vm);

   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "estimate_pose_and_update_pose", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzEstimatePoseAndUpdatePose));
   buzzvm_gstore(buzz_vm);

   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "rotation_estimation_stopping_conditions", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzRotationEstimationStoppingConditions));
   buzzvm_gstore(buzz_vm);

   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "pose_estimation_stopping_conditions", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzPoseEstimationStoppingConditions));
   buzzvm_gstore(buzz_vm);

   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "neighbor_rotation_estimation_is_finished", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzNeighborRotationEstimationIsFinished));
   buzzvm_gstore(buzz_vm);

   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "neighbor_pose_estimation_is_finished", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzNeighborPoseEstimationIsFinished));
   buzzvm_gstore(buzz_vm);

   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "optimizer_phase", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzOptimizerPhase));
   buzzvm_gstore(buzz_vm);

   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "check_if_all_estimation_done_and_reset", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzCheckIfAllEstimationDoneAndReset));
   buzzvm_gstore(buzz_vm);

   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "neighbor_state", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzNeighborState));
   buzzvm_gstore(buzz_vm);

   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "update_current_pose_estimate", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzUpdateCurrentPoseEstimate));
   buzzvm_gstore(buzz_vm);

   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "update_pose_estimate_from_neighbor", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzUpdatePoseEstimateFromNeighbor));
   buzzvm_gstore(buzz_vm);
   
   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "update_has_sent_start_optimization_flag", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzUpdateHasSentStartOptimizationFlag));
   buzzvm_gstore(buzz_vm);

   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "update_adjacency_vector", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzUpdateAdjacencyVector));
   buzzvm_gstore(buzz_vm);

   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "receive_adjacency_vector_from_neighbor", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzReceiveAdjacencyVectorFromNeighbor));
   buzzvm_gstore(buzz_vm);

   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "update_neighbor_has_started_optimization_flag", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzUpdateNeighborHasStartedOptimizationFlag));
   buzzvm_gstore(buzz_vm);

   return buzz_vm->state;
}

}