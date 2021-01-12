#include "buzz_slam_ros.h"
#include "../../buzz_slam_singleton.h"

namespace buzz_slam {
/****************************************/
/************ Buzz Closures *************/
/****************************************/

static int BuzzAddloopclosureOutlier(buzzvm_t vm) {
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   int is_added = BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAMRos>(vm->robot)->AddloopclosureMeasurementOutlier();
   buzzvm_pushi(vm, is_added);

   return buzzvm_ret1(vm);
}

/****************************************/
/****************************************/

static int BuzzLoadRosParameters(buzzvm_t vm) {
   /* Push the vector components */
   buzzvm_lload(vm, 1);
   buzzvm_lload(vm, 2);
   /* Create a new vector with that */
   float sensor_range;
   int outlier_period;
   buzzobj_t b_sensor_range = buzzvm_stack_at(vm, 2);
   buzzobj_t b_outlier_period = buzzvm_stack_at(vm, 1);
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
   if(b_outlier_period->o.type == BUZZTYPE_INT) outlier_period = b_outlier_period->i.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "load_no_sensing_parameters: expected %s, got %s in second argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[b_outlier_period->o.type]
         );
      return vm->state;
   }    
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   buzz_slam::BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<buzz_slam::BuzzSLAMRos>(vm->robot)->LoadParameters(sensor_range, outlier_period);
   
   return buzzvm_ret0(vm);
} 

/****************************************/
/****************************************/

static int BuzzStartOptimizationTriggered(buzzvm_t vm) {
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   bool start_triggered = BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAMRos>(vm->robot)->GetStartOptimizationTriggered();
   buzzvm_pushi(vm, start_triggered);

   return buzzvm_ret1(vm);
}

/****************************************/
/************ Registration **************/
/****************************************/

buzzvm_state BuzzSLAMRos::RegisterSLAMFunctions(buzzvm_t buzz_vm) {

   BuzzSLAM::RegisterSLAMFunctions(buzz_vm);
   /* Register mapping without sensing specific functions */
   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "add_loopclosure_outlier", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzAddloopclosureOutlier));
   buzzvm_gstore(buzz_vm);
   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "load_ros_parameters", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzLoadRosParameters));
   buzzvm_gstore(buzz_vm);
   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "start_optimization_triggered", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzStartOptimizationTriggered));
   buzzvm_gstore(buzz_vm);

   return buzz_vm->state;
}

}