#include "buzz_controller_quadmapper.h"
#include <iostream>
#include <stdlib.h>
#include <time.h>

/****************************************/
/****************************************/

CBuzzControllerQuadMapper::CBuzzControllerQuadMapper() :
   CBuzzControllerSpiri() {
}

/****************************************/
/****************************************/

CBuzzControllerQuadMapper::~CBuzzControllerQuadMapper() {
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::Init(TConfigurationNode& t_node) {
   CBuzzControllerSpiri::Init(t_node);
   // Initialize constant attributes // TODO: add paramaters for them or get them by buzz
   rotation_noise_std_ = 0.01;
   translation_noise_std_ = 0.1;
   maximum_number_of_optimization_iterations_ = 1000;
   optimization_phase_length_ = 20;

   // Initialize attributes
   number_of_poses_ = 0;
   robot_id_ = this->GetBuzzVM()->robot;
   robot_id_char_ = (unsigned char)(97 + robot_id_);
   previous_symbol_ = gtsam::Symbol(robot_id_char_, number_of_poses_);
   previous_pose_ = gtsam::Pose3();
   local_pose_graph_ = boost::make_shared< gtsam::NonlinearFactorGraph >();
   poses_initial_guess_ = boost::make_shared< gtsam::Values >();
   poses_initial_guess_->insert(previous_symbol_.key(), previous_pose_);
   current_optimization_iteration_ = 0;

   // Isotropic noise model
   Eigen::VectorXd sigmas(6);
   sigmas << rotation_noise_std_, rotation_noise_std_, rotation_noise_std_, 
            translation_noise_std_, translation_noise_std_, translation_noise_std_;
   noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

/****************************************/
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
   /* Retrieve parameters and check their types */
   buzzobj_t b_robot_1_id = buzzvm_stack_at(vm, 11);
   buzzobj_t b_robot_2_id = buzzvm_stack_at(vm, 10);
   buzzobj_t b_robot_1_pose_id = buzzvm_stack_at(vm, 9);
   buzzobj_t b_robot_2_pose_id = buzzvm_stack_at(vm, 8);
   buzzobj_t b_x = buzzvm_stack_at(vm, 7);
   buzzobj_t b_y = buzzvm_stack_at(vm, 6);
   buzzobj_t b_z = buzzvm_stack_at(vm, 5);
   buzzobj_t b_q_x = buzzvm_stack_at(vm, 4);
   buzzobj_t b_q_y = buzzvm_stack_at(vm, 3);
   buzzobj_t b_q_z = buzzvm_stack_at(vm, 2);
   buzzobj_t b_q_w = buzzvm_stack_at(vm, 1);
   int robot_1_id, robot_2_id, robot_1_pose_id, robot_2_pose_id;
   float x, y, z, q_x, q_y, q_z, q_w;
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
      b_q_w->o.type == BUZZTYPE_FLOAT) {

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

   } else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "wrong parameter type for add_separator_to_local_graph."
         );
      return vm->state;
   }
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->AddSeparatorToLocalGraph(  robot_1_id, robot_2_id,
                                                                                                               robot_1_pose_id, robot_2_pose_id,
                                                                                                               x, y, z,
                                                                                                               q_x, q_y, q_z, q_w  );
   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

OptimizerState CBuzzControllerQuadMapper::GetOptimizerState() {
   return optimizer_state_;
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::UpdateCurrentSeparatorBuzzStructure(  const int& robot_1_id,
                                             const int& robot_2_id,
                                             const int& robot_1_pose_id,
                                             const int& robot_2_pose_id,
                                             const double& x,
                                             const double& y,
                                             const double& z,
                                             const double& q_x,
                                             const double& q_y,
                                             const double& q_z,
                                             const double& q_w  ) {
   // Create empty data table
   buzzobj_t b_separator_measurement = buzzheap_newobj(m_tBuzzVM, BUZZTYPE_TABLE);
   // Store symbol information
   TablePut(b_separator_measurement, "robot_1_id", robot_1_id);
   TablePut(b_separator_measurement, "robot_2_id", robot_2_id);
   TablePut(b_separator_measurement, "robot_1_pose_id", robot_1_pose_id);
   TablePut(b_separator_measurement, "robot_2_pose_id", robot_2_pose_id);
   // Store position data
   TablePut(b_separator_measurement, "x", x);
   TablePut(b_separator_measurement, "y", y);
   TablePut(b_separator_measurement, "z", z);
   // Store orientation data
   TablePut(b_separator_measurement, "q_x", q_x);
   TablePut(b_separator_measurement, "q_y", q_y);
   TablePut(b_separator_measurement, "q_z", q_z);
   TablePut(b_separator_measurement, "q_w", q_w);
   // Register positioning data table as global symbol
   Register("current_separator_measurement", b_separator_measurement);

}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::SetNextPosition(const CVector3& translation) {

   CVector3 new_position = m_pcPos->GetReading().Position + translation;

   new_position.SetZ(2.0f); // To ensure that the quadrotor flies
   
   m_pcPropellers->SetAbsolutePosition(new_position);
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::AddNeighborWithinCommunicationRange(const int& rid) {
   neighbors_within_communication_range_.emplace_back(rid);
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::IncrementNumberOfPosesAndUpdateState() {
   number_of_poses_++;
   // Update optimizer state
   switch (optimizer_state_) {
      case Idle :
         if (number_of_poses_ % optimizer_period_ == 0) {
            optimizer_state_ = OptimizerState::Start;
            current_optimization_iteration_ = 0;
            neighbors_within_communication_range_.clear();
         }
         break;
      case Start :
         optimizer_state_ = OptimizerState::RotationEstimation;
         break;
      case RotationEstimation :
         std::cout << "ROBOT" << robot_id_ << " : Rotation estimation nposes=" << number_of_poses_ << "N: ";
         for (auto neighbor : neighbors_within_communication_range_) {
            std::cout << neighbor << ", ";
         }
         std::cout << std::endl;
         if (number_of_poses_ % (optimizer_period_ + optimization_phase_length_) == 0) {
            optimizer_state_ = OptimizerState::Idle;
         }
         break;
      case PoseEstimation :

         break;
      case End :

         break;
   }
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::AddSeparatorToLocalGraph( const int& robot_1_id,
                                 const int& robot_2_id,
                                 const int& robot_1_pose_id,
                                 const int& robot_2_pose_id,
                                 const double& x,
                                 const double& y,
                                 const double& z,
                                 const double& q_x,
                                 const double& q_y,
                                 const double& q_z,
                                 const double& q_w  ) {

   // Separator symbols
   unsigned char robot_1_id_char = (unsigned char)(97 + robot_1_id);
   AddNewKnownRobot(robot_1_id_char);
   gtsam::Symbol robot_1_symbol = gtsam::Symbol(robot_1_id_char, robot_1_pose_id);
   unsigned char robot_2_id_char = (unsigned char)(97 + robot_2_id);
   AddNewKnownRobot(robot_2_id_char);
   gtsam::Symbol robot_2_symbol = gtsam::Symbol(robot_2_id_char, robot_2_pose_id);

   // Translation
   gtsam::Point3 t = { x, y, z };

   // Rotation
   gtsam::Quaternion quat(q_w, q_x, q_y, q_z);
   gtsam::Rot3 R(quat);

   // Transformation
   gtsam::Pose3 transformation(R, t);

   // Factor
   gtsam::BetweenFactor<gtsam::Pose3> new_factor = gtsam::BetweenFactor<gtsam::Pose3>(robot_1_symbol, robot_2_symbol, transformation, noise_model_);
   local_pose_graph_->push_back(new_factor);
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::WriteDataset(const uint16_t& robot_id) {
   std::string dataset_file_name = "log/datasets/" + std::to_string(robot_id) + ".g2o";
   gtsam::writeG2o(*local_pose_graph_, *poses_initial_guess_, dataset_file_name);
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::AddNewKnownRobot(const unsigned char& other_robot_char) {
   if (other_robot_char != robot_id_char_) {
      if (std::find(known_other_robots_.begin(), known_other_robots_.end(), other_robot_char) == known_other_robots_.end()) {
         known_other_robots_.insert(other_robot_char);
      }
   }
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::InitOptimizer(const int& period) {

   optimizer_ = boost::shared_ptr<distributed_mapper::DistributedMapper>(new distributed_mapper::DistributedMapper(robot_id_char_));

   graph_and_values_ = std::make_pair(local_pose_graph_, poses_initial_guess_);

   // Use between noise or not in optimizePoses
   optimizer_->setUseBetweenNoiseFlag(false);

   // Use landmarks
   optimizer_->setUseLandmarksFlag(false);

   // Load subgraphs
   optimizer_->loadSubgraphAndCreateSubgraphEdge(graph_and_values_);

   // Add prior to the first robot
   if (robot_id_ == 0) {
      gtsam::Key first_key = gtsam::KeyVector(poses_initial_guess_->keys()).at(0);
      optimizer_->addPrior(first_key, poses_initial_guess_->at<gtsam::Pose3>(first_key), noise_model_);
   }

   // Verbosity level
   optimizer_->setVerbosity(distributed_mapper::DistributedMapper::ERROR);

   disconnected_graph_ = true;

   optimizer_->setFlaggedInit(true);
   
   optimizer_->setUpdateType(distributed_mapper::DistributedMapper::incUpdate);
   
   optimizer_->setGamma(1.0f);

   optimizer_state_ = OptimizerState::Idle;

   optimizer_period_ = period;
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::StartPoseGraphOptimization() {

   UpdateOptimizer();

   OutliersFiltering();

   // TODO: Add flagged initialization
   std::vector<size_t> ordering = TrivialOrdering();

   optimizer_->updateInitialized(false);
   optimizer_->clearNeighboringRobotInit();

   optimizer_state_ = OptimizerState::RotationEstimation;

}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::UpdateOptimizer() {
   
   // Load subgraphs
   optimizer_->loadSubgraphAndCreateSubgraphEdge(graph_and_values_);

   // Check for graph connectivity
   std::set<char> neighboring_robots = optimizer_->getNeighboringChars();
   if (neighboring_robots.size() > 0) {
      disconnected_graph_ = false;
   }
}

/****************************************/
/****************************************/

std::vector<size_t> CBuzzControllerQuadMapper::TrivialOrdering() {
   std::vector<size_t> ordering;
   for(const gtsam::Values::ConstKeyValuePair& key_value: optimizer_->neighbors()){
      char symbol = gtsam::symbolChr(key_value.key);
      ordering.emplace_back(((int) symbol) - 97);
   }
}

/****************************************/
/****************************************/

std::vector<size_t> CBuzzControllerQuadMapper::FlaggedInitializationOrdering() {
   // TODO
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::OutliersFiltering() {
   
   // Perform pairwise consistency maximization

}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::OptimizeRotationsIteration() {

   // Stopping condition
   // Change optimizer state
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapper::UpdateLocalEstimates() {

}

/****************************************/
/****************************************/

buzzvm_state CBuzzControllerQuadMapper::RegisterFunctions() {
   CBuzzControllerSpiri::RegisterFunctions();

   /* Register mapping specific functions */
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


   return m_tBuzzVM->state;
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CBuzzControllerQuadMapper, "buzz_controller_quadmapper");
