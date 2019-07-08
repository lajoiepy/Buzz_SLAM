#include "buzz_controller_quadmapper_with_dataset.h"

namespace buzz_quadmapper {

/****************************************/
/************ Buzz Closures *************/
/****************************************/

static int BuzzMove(buzzvm_t vm) {
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   int current_pose_id = reinterpret_cast<CBuzzControllerQuadMapperWithDataset*>(buzzvm_stack_at(vm, 1)->u.value)->Move();
   
   buzzvm_pushi(vm, current_pose_id);

   return buzzvm_ret1(vm);
}

/****************************************/
/************ Registration **************/
/****************************************/

buzzvm_state CBuzzControllerQuadMapperWithDataset::RegisterFunctions() {
   CBuzzControllerQuadMapper::RegisterFunctions();
   buzz_slam::BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<buzz_slam::BuzzSLAMDataset>(m_tBuzzVM->robot)->RegisterSLAMFunctions(this->GetBuzzVM());

   /* Register mapping without sensing specific functions */
   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "move", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzMove));
   buzzvm_gstore(m_tBuzzVM);
   
   return m_tBuzzVM->state;
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CBuzzControllerQuadMapperWithDataset, "buzz_controller_quadmapper_with_dataset");

}