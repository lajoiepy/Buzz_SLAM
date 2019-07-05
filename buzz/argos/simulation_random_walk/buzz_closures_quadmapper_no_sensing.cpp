#include "buzz_controller_quadmapper_no_sensing.h"

namespace buzz_quadmapper {

/****************************************/
/************ Buzz Closures *************/
/****************************************/

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
                      "move_forward(x,y): expected %s, got %s in second argument",
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
/************ Registration **************/
/****************************************/

buzzvm_state CBuzzControllerQuadMapperNoSensing::RegisterFunctions() {
   fprintf(stdout, "ROBOT %d , No sensing Register functions \n", m_tBuzzVM->robot);
   CBuzzControllerQuadMapper::RegisterFunctions();
   buzz_slam::BuzzSLAMSingleton::GetInstance().InsertBuzzSLAM(m_tBuzzVM->robot, new buzz_slam::BuzzSLAMNoSensing());
   buzz_slam::BuzzSLAMSingleton::GetInstance().GetBuzzSLAM(m_tBuzzVM->robot)->RegisterSLAMFunctions(this->GetBuzzVM());

   /* Register mapping without sensing specific functions */
   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "move_forward_fake_odometry", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzMoveForwardFakeOdometry));
   buzzvm_gstore(m_tBuzzVM);

   return m_tBuzzVM->state;
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CBuzzControllerQuadMapperNoSensing, "buzz_controller_quadmapper_no_sensing");

}