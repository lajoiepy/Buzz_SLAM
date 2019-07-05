#include "buzz_slam_no_sensing.h"
#include "../../buzz_slam_singleton.h"

namespace buzz_slam {

/****************************************/
/************ Buzz Closures *************/
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
   int is_outlier = reinterpret_cast<BuzzSLAMNoSensing*>(BuzzSLAMSingleton::GetInstance().GetBuzzSLAM(vm->robot).get())->ComputeNoisyFakeSeparatorMeasurement(gt_orientation, gt_translation, other_robot_pose_id, other_robot_id, this_robot_pose_id);
   buzzvm_pushi(vm, is_outlier);

   return buzzvm_ret1(vm);
}

/****************************************/
/************ Registration **************/
/****************************************/

buzzvm_state BuzzSLAMNoSensing::RegisterSLAMFunctions(buzzvm_t buzz_vm) {
   BuzzSLAM::RegisterSLAMFunctions(buzz_vm);
   /* Register mapping without sensing specific functions */
   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "compute_fake_rendezvous_separator", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzComputeFakeRendezVousSeparator));
   buzzvm_gstore(buzz_vm);

   return buzz_vm->state;
}

}