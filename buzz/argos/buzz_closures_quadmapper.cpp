#include "buzz_controller_quadmapper.h"

namespace buzz_quadmapper {

/****************************************/
/************ Buzz Closures *************/
/****************************************/

static int BuzzInitOptimizer(buzzvm_t vm){

   buzzvm_lload(vm, 1);
   
   buzzobj_t buzz_period = buzzvm_stack_at(vm, 1);
   int period;

   if(buzz_period->o.type == BUZZTYPE_INT) period = buzz_period->i.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "srand(x,y): expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[buzz_period->o.type]
         );
      return vm->state;
   } 

   // Initialize optimizer
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->InitOptimizer(period);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzOptimizerState(buzzvm_t vm){

   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   int state = reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->GetOptimizerState();
   buzzvm_pushi(vm, state);

   return buzzvm_ret1(vm);
}

/****************************************/
/****************************************/

static int BuzzOptimizerPhase(buzzvm_t vm){

   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   int phase = reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->GetOptimizerPhase();
   buzzvm_pushi(vm, phase);

   return buzzvm_ret1(vm);
}

/****************************************/
/****************************************/

static int BuzzCheckIfAllEstimationDoneAndReset(buzzvm_t vm) {
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->CheckIfAllEstimationDoneAndReset();
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
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->AddNeighborWithinCommunicationRange(rid);

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
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->ComputeAndUpdateRotationEstimatesToSend(rid);

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
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->UpdateNeighborRotationEstimates(received_rotation_estimates);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzEstimateRotationAndUpdateRotation(buzzvm_t vm) {
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->EstimateRotationAndUpdateRotation();
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
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->ComputeAndUpdatePoseEstimatesToSend(rid);

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
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->UpdateNeighborPoseEstimates(received_pose_estimates);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzEstimatePoseAndUpdatePose(buzzvm_t vm) {
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->EstimatePoseAndUpdatePose();
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
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->UpdateCurrentPoseEstimate(pose_id);

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
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->UpdatePoseEstimateFromNeighbor(rid, pose_id, pose_with_covariance);

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

static int BuzzGotoAbs(buzzvm_t vm) {
   /* Push the vector components */
   buzzvm_lload(vm, 1);
   buzzvm_lload(vm, 2);
   buzzvm_lload(vm, 3);
   /* Create a new vector with that */
   CVector3 translation;
   buzzobj_t tX = buzzvm_stack_at(vm, 3);
   buzzobj_t tY = buzzvm_stack_at(vm, 2);
   buzzobj_t tZ = buzzvm_stack_at(vm, 1);
   if(tX->o.type == BUZZTYPE_INT) translation.SetX(tX->i.value);
   else if(tX->o.type == BUZZTYPE_FLOAT) translation.SetX(tX->f.value);
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "goto_abs(x,y): expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_FLOAT],
                      buzztype_desc[tX->o.type]
         );
      return vm->state;
   }      
   if(tY->o.type == BUZZTYPE_INT) translation.SetY(tY->i.value);
   else if(tY->o.type == BUZZTYPE_FLOAT) translation.SetY(tY->f.value);
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "goto_abs(x,y): expected %s, got %s in second argument",
                      buzztype_desc[BUZZTYPE_FLOAT],
                      buzztype_desc[tY->o.type]
         );
      return vm->state;
   }
   if(tZ->o.type == BUZZTYPE_INT) translation.SetZ(tZ->i.value);
   else if(tZ->o.type == BUZZTYPE_FLOAT) translation.SetZ(tZ->f.value);
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "goto_abs(x,y): expected %s, got %s in third argument",
                      buzztype_desc[BUZZTYPE_FLOAT],
                      buzztype_desc[tZ->o.type]
         );
      return vm->state;
   }
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->SetNextPosition(translation);
   return buzzvm_ret0(vm);
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
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->AddSeparatorToLocalGraph( robot_1_id, robot_2_id,
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
   /* Retrieve parameters and check their types */
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
      b_number_of_steps_before_failsafe->o.type == BUZZTYPE_INT) {

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
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->LoadParameters(number_of_steps_before_failsafe,
                     use_pcm, confidence_probability, incremental_solving, debug_level,
                     rotation_noise_std, translation_noise_std,
                     rotation_estimate_change_threshold, pose_estimate_change_threshold,
                     use_flagged_initialization, is_simulation,
                     number_of_robots, error_file_name);
   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzRotationEstimationStoppingConditions(buzzvm_t vm){

   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   bool is_finished = reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->RotationEstimationStoppingConditions();
   buzzvm_pushi(vm, is_finished);

   return buzzvm_ret1(vm);
}

/****************************************/
/****************************************/

static int BuzzPoseEstimationStoppingConditions(buzzvm_t vm){

   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   bool is_finished = reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->PoseEstimationStoppingConditions();
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
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->NeighborRotationEstimationIsFinished(rid);

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
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->NeighborPoseEstimationIsFinished(rid, anchor_offset);

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
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->NeighborState(rid, (buzz_quadmapper::OptimizerState) state, lowest_id);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzUpdateHasSentStartOptimizationFlag(buzzvm_t vm) {
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->UpdateHasSentStartOptimizationFlag(true);
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
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->UpdateNeighborHasStartedOptimizationFlag(true, rid);
}

/****************************************/
/************ Registration **************/
/****************************************/

buzzvm_state CBuzzControllerQuadMapper::RegisterFunctions() {
   CBuzzControllerSpiri::RegisterFunctions();

   /* Register mapping specific functions */
   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "load_parameters", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzLoadParameters));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "srand", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzSRand));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "rand_uniform", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzRandUniform));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "add_separator_to_local_graph", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzAddSeparatorToLocalGraph));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "goto_abs", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzGotoAbs));
   buzzvm_gstore(m_tBuzzVM);
   
   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "init_optimizer", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzInitOptimizer));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "optimizer_state", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzOptimizerState));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "add_neighbor_within_communication_range", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzAddNeighborWithinCommunicationRange));
   buzzvm_gstore(m_tBuzzVM);
   
   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "compute_and_update_rotation_estimates_to_send", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzComputeAndUpdateRotationEstimatesToSend));
   buzzvm_gstore(m_tBuzzVM);
   
   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "compute_and_update_pose_estimates_to_send", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzComputeAndUpdatePoseEstimatesToSend));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "update_neighbor_rotation_estimates", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzUpdateNeighborRotationEstimates));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "update_neighbor_pose_estimates", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzUpdateNeighborPoseEstimates));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "estimate_rotation_and_update_rotation", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzEstimateRotationAndUpdateRotation));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "estimate_pose_and_update_pose", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzEstimatePoseAndUpdatePose));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "rotation_estimation_stopping_conditions", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzRotationEstimationStoppingConditions));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "pose_estimation_stopping_conditions", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzPoseEstimationStoppingConditions));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "neighbor_rotation_estimation_is_finished", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzNeighborRotationEstimationIsFinished));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "neighbor_pose_estimation_is_finished", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzNeighborPoseEstimationIsFinished));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "optimizer_phase", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzOptimizerPhase));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "check_if_all_estimation_done_and_reset", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzCheckIfAllEstimationDoneAndReset));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "neighbor_state", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzNeighborState));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "update_current_pose_estimate", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzUpdateCurrentPoseEstimate));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "update_pose_estimate_from_neighbor", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzUpdatePoseEstimateFromNeighbor));
   buzzvm_gstore(m_tBuzzVM);
   
   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "update_has_sent_start_optimization_flag", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzUpdateHasSentStartOptimizationFlag));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "update_neighbor_has_started_optimization_flag", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzUpdateNeighborHasStartedOptimizationFlag));
   buzzvm_gstore(m_tBuzzVM);

   return m_tBuzzVM->state;
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CBuzzControllerQuadMapper, "buzz_controller_quadmapper");
}