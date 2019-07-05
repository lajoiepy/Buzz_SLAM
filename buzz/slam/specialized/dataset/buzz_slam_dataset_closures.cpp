#include "buzz_slam_dataset.h"
#include "../../buzz_slam_singleton.h"

namespace buzz_slam {
/****************************************/
/************ Buzz Closures *************/
/****************************************/

static int BuzzAddSeparator(buzzvm_t vm) {
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   int is_added = BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAMDataset>(vm->robot)->AddSeparatorMeasurement();
   buzzvm_pushi(vm, is_added);

   return buzzvm_ret1(vm);
}

/****************************************/
/****************************************/

static int BuzzAddSeparatorOutlier(buzzvm_t vm) {
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   int is_added = BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<BuzzSLAMDataset>(vm->robot)->AddSeparatorMeasurementOutlier();
   buzzvm_pushi(vm, is_added);

   return buzzvm_ret1(vm);
}

/****************************************/
/************ Registration **************/
/****************************************/

buzzvm_state BuzzSLAMDataset::RegisterSLAMFunctions(buzzvm_t buzz_vm) {

   BuzzSLAM::RegisterSLAMFunctions(buzz_vm);
   /* Register mapping without sensing specific functions */
   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "add_separator", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzAddSeparator));
   buzzvm_gstore(buzz_vm);
   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "add_separator_outlier", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzAddSeparatorOutlier));
   buzzvm_gstore(buzz_vm);
   return buzz_vm->state;
}

}