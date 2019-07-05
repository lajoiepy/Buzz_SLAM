#include "buzz_controller_quadmapper_with_dataset.h"
#include "../../buzz_slam_singleton.h"

namespace buzz_quadmapper {

/****************************************/
/************ Buzz Closures *************/
/****************************************/

static int BuzzAddSeparator(buzzvm_t vm) {
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   int is_added = reinterpret_cast<BuzzSLAMDataset*>(BuzzSLAMSingleton::GetInstance().GetBuzzSLAM(vm->robot))->AddSeparatorMeasurement();
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
   int is_added = reinterpret_cast<BuzzSLAMDataset*>(BuzzSLAMSingleton::GetInstance().GetBuzzSLAM(vm->robot))->AddSeparatorMeasurementOutlier();
   buzzvm_pushi(vm, is_added);

   return buzzvm_ret1(vm);
}

/****************************************/
/****************************************/

static int BuzzLoadDatasetParameters(buzzvm_t vm) {
   /* Push the vector components */
   buzzvm_lload(vm, 1);
   buzzvm_lload(vm, 2);
   buzzvm_lload(vm, 3);
   /* Create a new vector with that */
   float sensor_range;
   int outlier_period;
   std::string dataset_name;
   buzzobj_t b_dataset_name = buzzvm_stack_at(vm, 3);
   buzzobj_t b_sensor_range = buzzvm_stack_at(vm, 2);
   buzzobj_t b_outlier_period = buzzvm_stack_at(vm, 1);
   if(b_dataset_name->o.type == BUZZTYPE_STRING) dataset_name = b_dataset_name->s.value.str;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "load_no_sensing_parameters: expected %s, got %s in second argument",
                      buzztype_desc[BUZZTYPE_STRING],
                      buzztype_desc[b_dataset_name->o.type]
         );
      return vm->state;
   } 
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
   reinterpret_cast<BuzzSLAMDataset*>(BuzzSLAMSingleton::GetInstance().GetBuzzSLAM(vm->robot))->LoadParameters(dataset_name, sensor_range, outlier_period);
   
   return buzzvm_ret0(vm);
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
   buzzvm_pushs(buzz_vm, buzzvm_string_register(buzz_vm, "load_dataset_parameters", 1));
   buzzvm_pushcc(buzz_vm, buzzvm_function_register(buzz_vm, BuzzLoadDatasetParameters));
   buzzvm_gstore(buzz_vm);

   return buzz_vm->state;
}

}