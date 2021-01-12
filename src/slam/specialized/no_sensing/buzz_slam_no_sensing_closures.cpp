#include "buzz_slam_no_sensing.h"
#include "../../buzz_slam_singleton.h"

namespace buzz_slam {

/****************************************/
/************ Buzz Closures *************/
/****************************************/

static int BuzzComputeFakeRendezVousloopclosure(buzzvm_t vm) {
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
   gtsam::Point3 gt_translation;
   gtsam::Rot3 gt_rotation;
   int other_robot_id, other_robot_pose_id, this_robot_pose_id;
   if(b_this_robot_pose_id->o.type == BUZZTYPE_INT &&
      b_other_robot_id->o.type == BUZZTYPE_INT &&
      b_other_robot_pose_id->o.type == BUZZTYPE_INT &&
      b_gt_x->o.type == BUZZTYPE_FLOAT &&
      b_gt_y->o.type == BUZZTYPE_FLOAT &&
      b_gt_z->o.type == BUZZTYPE_FLOAT &&
      b_gt_yaw->o.type == BUZZTYPE_FLOAT) {

      // Fill in ground truth
      gt_translation = gtsam::Point3(b_gt_x->f.value, b_gt_y->f.value, b_gt_z->f.value);
      gt_rotation = gtsam::Rot3::Rz(b_gt_yaw->f.value);

      // Get information for symbols
      other_robot_id = b_other_robot_id->i.value;
      other_robot_pose_id = b_other_robot_pose_id->i.value;
      this_robot_pose_id = b_this_robot_pose_id->i.value;

   } else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "wrong parameter type for compute_fake_rendezvous_loopclosure."
         );
      return vm->state;
   }
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   int is_outlier = BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAMNoSensing>(vm->robot)->ComputeNoisyFakeloopclosureMeasurement(gt_translation, gt_rotation, other_robot_pose_id, other_robot_id, this_robot_pose_id);
   buzzvm_pushi(vm, is_outlier);

   return buzzvm_ret1(vm);
}

/****************************************/
/****************************************/

static int BuzzLoadNoSensingParameters(buzzvm_t vm) {
   /* Push the vector components */
   buzzvm_lload(vm, 1);
   buzzvm_lload(vm, 2);
   /* Create a new vector with that */
   float sensor_range;
   float outlier_probability;
   buzzobj_t b_sensor_range = buzzvm_stack_at(vm, 2);
   buzzobj_t b_outlier_probability = buzzvm_stack_at(vm, 1);
   if(b_sensor_range->o.type == BUZZTYPE_FLOAT) sensor_range = b_sensor_range->f.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "load_no_sensing_parameters: expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_FLOAT],
                      buzztype_desc[b_sensor_range->o.type]
         );
      return vm->state;
   }   
   if(b_outlier_probability->o.type == BUZZTYPE_FLOAT) outlier_probability = b_outlier_probability->f.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "load_no_sensing_parameters: expected %s, got %s in second argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[b_outlier_probability->o.type]
         );
      return vm->state;
   }    
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   buzz_slam::BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<buzz_slam::BuzzSLAMNoSensing>(vm->robot)->LoadParameters(sensor_range, outlier_probability);
   
   return buzzvm_ret0(vm);
}

/****************************************/
/************ Registration **************/
/****************************************/

buzzvm_state BuzzSLAMNoSensing::RegisterSLAMFunctions(buzzvm_t buzz_vm) {
   BuzzSLAM::RegisterSLAMFunctions(buzz_vm);
   /* Register mapping without sensing specific functions */
   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "compute_fake_rendezvous_loopclosure", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzComputeFakeRendezVousloopclosure));
   buzzvm_gstore(buzz_vm);

   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "load_no_sensing_parameters", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzLoadNoSensingParameters));
   buzzvm_gstore(buzz_vm);

   return buzz_vm->state;
}

}