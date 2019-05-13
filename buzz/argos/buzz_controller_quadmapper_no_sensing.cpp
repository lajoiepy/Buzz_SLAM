#include "buzz_controller_quadmapper_no_sensing.h"
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <cmath> 
#include "boost/filesystem.hpp"

namespace buzz_quadmapper {
/****************************************/
/****************************************/

CBuzzControllerQuadMapperNoSensing::CBuzzControllerQuadMapperNoSensing() :
   CBuzzControllerQuadMapper() {
}

/****************************************/
/****************************************/

CBuzzControllerQuadMapperNoSensing::~CBuzzControllerQuadMapperNoSensing() {
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapperNoSensing::Init(TConfigurationNode& t_node){
   CBuzzControllerQuadMapper::Init(t_node);

   // Initialize for tracing variables
   number_of_outliers_added_ = 0;

   // Initialize random numbers generators
   srand(time(NULL));
   gen_translation_ = std::mt19937{rd_()};
   gen_rotation_ = std::mt19937{rd_()};
   gen_outliers_ = std::mt19937{rd_()};
   normal_distribution_translation_ = std::normal_distribution<>{0, translation_noise_std_};
   normal_distribution_rotation_ = std::normal_distribution<>{0, rotation_noise_std_};
   uniform_distribution_outliers_translation_ = std::uniform_real_distribution<>{0, sensor_range_};
   uniform_distribution_outliers_rotation_ = std::uniform_real_distribution<>{-M_PI, M_PI};
   uniform_distribution_draw_outlier_ = std::uniform_real_distribution<>{0, 1};
   simulation_step_ = 0;
   previous_simulation_gt_pose_ = m_pcPos->GetReading();

   // Save ground truth for fake separator creation
   SavePoseGroundTruth();

   // Initialize log files
   if (is_simulation_ && robot_id_ ==  0 && !boost::filesystem::exists(error_file_name_)) {
      // Write results to csv
      std::ofstream error_file;
      error_file.open(error_file_name_, std::ios::out | std::ios::app);
      error_file << "NumberOfRobots\tNumberOfPoses\tNumberOfSeparators\tProbabilityOfOutliers\tUsesIncrementalSolving\tRotationNoiseStd\tTranslationNoiseStd\tRotationChangeThreshold\tPoseChangeThreshold\tErrorCentralized\tErrorDecentralized\tErrorInitial\tNumberOfRotationIterations\tNumberOfPoseIterations\n";
      error_file.close();
   }
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapperNoSensing::LoadParameters(const double& sensor_range, const double& outlier_probability) {
   sensor_range_ = sensor_range;
   outlier_probability_ = outlier_probability;
}

/****************************************/
/****************************************/

int CBuzzControllerQuadMapperNoSensing::MoveForwardFakeOdometry(const CVector3& distance, const int& simulation_time_divider) {
   simulation_step_ ++;

   // Move
   CQuaternion current_orientation = m_pcPos->GetReading().Orientation;
   CRadians c_z_angle, c_y_angle, c_x_angle;
   current_orientation.ToEulerAngles(c_z_angle, c_y_angle, c_x_angle);

   CVector3 translation;
   translation.SetX(distance.GetX()*std::cos(c_z_angle.GetValue()));
   translation.SetY(distance.GetX()*std::sin(c_z_angle.GetValue()));
   translation.SetZ(0);

   CVector3 new_position;
   new_position.SetX(m_pcPos->GetReading().Position.GetX() + translation.GetX());
   new_position.SetY(m_pcPos->GetReading().Position.GetY() + translation.GetY());

   new_position.SetZ(2.0f); // To ensure that the quadrotor flies

   m_pcPropellers->SetAbsolutePosition(new_position);
   
   if (simulation_step_ % simulation_time_divider == 0) {
      // Add noisy measurement
      ComputeNoisyFakeOdometryMeasurement();

      // Log data
      //WriteCurrentDataset();   
   }

   return number_of_poses_;
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapperNoSensing::ComputeNoisyFakeOdometryMeasurement() {
   
   // Extract info
   CQuaternion previous_orientation = previous_simulation_gt_pose_.Orientation;
   CQuaternion current_orientation = m_pcPos->GetReading().Orientation;
   CVector3 previous_position = previous_simulation_gt_pose_.Position;
   CVector3 current_position = m_pcPos->GetReading().Position;

   // Increase the number of poses
   IncrementNumberOfPosesAndUpdateState();

   // Next symbol
   gtsam::Symbol current_symbol_ = gtsam::Symbol(robot_id_char_, number_of_poses_);

   // Conversion of the previous orientation (quaternion to rotation matrix)
   gtsam::Quaternion previous_quat_gtsam(previous_orientation.GetW(), previous_orientation.GetX(), previous_orientation.GetY(), previous_orientation.GetZ());
   gtsam::Rot3 previous_R(previous_quat_gtsam);
   
   // Conversion of the current orientation (quaternion to rotation matrix)
   gtsam::Quaternion current_quat_gtsam(current_orientation.GetW(), current_orientation.GetX(), current_orientation.GetY(), current_orientation.GetZ());
   gtsam::Rot3 current_R(current_quat_gtsam);

   // Compute transformation between rotations
   gtsam::Rot3 R = previous_R.inverse() * current_R;

   // Convert translation information to gtsam format and perform the appropriate rotation
   gtsam::Point3 t = {  current_position.GetX() - previous_position.GetX(), 
                        current_position.GetY() - previous_position.GetY(),
                        current_position.GetZ() - previous_position.GetZ()};
   t = previous_R.inverse() * t;

   // Add gaussian noise
   auto measurement = AddGaussianNoiseToMeasurement(R, t);

   // Initialize factor
   gtsam::BetweenFactor<gtsam::Pose3> new_factor(previous_symbol_, current_symbol_, measurement, noise_model_);

   // Update attributes value
   previous_simulation_gt_pose_ = m_pcPos->GetReading();
   auto new_pose_ = poses_initial_guess_->at<gtsam::Pose3>(previous_symbol_.key()) * measurement;
   previous_symbol_ = current_symbol_;

   // Add new factor to local pose graph
   local_pose_graph_->push_back(new_factor);

   // Add new pose estimate into initial guess
   poses_initial_guess_->insert(current_symbol_.key(), new_pose_);

   // Save ground truth for fake separator creation
   SavePoseGroundTruth();
}

/****************************************/
/****************************************/

gtsam::Pose3 CBuzzControllerQuadMapperNoSensing::AddGaussianNoiseToMeasurement(const gtsam::Rot3& R, const gtsam::Point3& t) {
   
   gtsam::Point3 t_noisy = {  t.x() + normal_distribution_translation_(gen_translation_),  
                              t.y() + normal_distribution_translation_(gen_translation_),
                              t.z() + normal_distribution_translation_(gen_translation_) };
   
   gtsam::Point3 R_noise_vector = { normal_distribution_rotation_(gen_rotation_),  
                                    normal_distribution_rotation_(gen_rotation_),
                                    normal_distribution_rotation_(gen_rotation_) };

   gtsam::Rot3 R_noise_matrix = gtsam::Rot3::AxisAngle(R_noise_vector.normalized(), R_noise_vector.norm());

   gtsam::Rot3 R_noisy = R * R_noise_matrix;

   return gtsam::Pose3(R_noisy, t_noisy);
}

/****************************************/
/****************************************/

gtsam::Pose3 CBuzzControllerQuadMapperNoSensing::OutlierMeasurement(const gtsam::Rot3& R, const gtsam::Point3& t) {
   
   // TODO: Add option to add noise greater than 3 or 5 sigmas, instead of totally random measurment
   // This is why this method takes the measurment in parameter.
   gtsam::Point3 t_outlier = {   uniform_distribution_outliers_translation_(gen_outliers_),  
                                 uniform_distribution_outliers_translation_(gen_outliers_),
                                 uniform_distribution_outliers_translation_(gen_outliers_) };


   gtsam::Rot3 R_outlier = gtsam::Rot3::Ypr( uniform_distribution_outliers_rotation_(gen_outliers_), 
                                             uniform_distribution_outliers_rotation_(gen_outliers_),
                                             uniform_distribution_outliers_rotation_(gen_outliers_));

   number_of_outliers_added_++;

   return gtsam::Pose3(R_outlier, t_outlier);
}

/****************************************/
/****************************************/

int CBuzzControllerQuadMapperNoSensing::ComputeNoisyFakeSeparatorMeasurement(const CQuaternion& gt_orientation, const CVector3& gt_translation, 
                                                               const int& other_robot_pose_id, const int& other_robot_id, const int& this_robot_pose_id) {
   // Separator symbols
   gtsam::Symbol this_robot_symbol = gtsam::Symbol(robot_id_char_, this_robot_pose_id);
   unsigned char other_robot_id_char = (unsigned char)(97 + other_robot_id);
   gtsam::Symbol other_robot_symbol = gtsam::Symbol(other_robot_id_char, other_robot_pose_id);

   // Get this robot pose
   gtsam::Pose3 this_robot_pose = ground_truth_poses_.find(this_robot_pose_id)->second;

   // Conversion of the other robot orientation (quaternion to rotation matrix)
   gtsam::Quaternion other_robot_quat_gtsam(gt_orientation.GetW(), gt_orientation.GetX(), gt_orientation.GetY(), gt_orientation.GetZ());
   gtsam::Rot3 other_robot_R(other_robot_quat_gtsam);

   // Compute transformation between rotations
   gtsam::Rot3 R = this_robot_pose.rotation().inverse() * other_robot_R;

   // Convert translation information to gtsam format and perform the appropriate rotation
   gtsam::Point3 t = { gt_translation.GetX() - this_robot_pose.translation().x(), 
                       gt_translation.GetY() - this_robot_pose.translation().y(),
                       gt_translation.GetZ() - this_robot_pose.translation().z() };
   t = this_robot_pose.rotation().inverse() * t;

   // Add gaussian noise or make it an outlier
   gtsam::Pose3 measurement;
   int is_outlier = 0;
   if ( uniform_distribution_draw_outlier_(gen_outliers_) < outlier_probability_) {
      measurement = OutlierMeasurement(R, t);
      is_outlier = 1;
   } else {
      measurement = AddGaussianNoiseToMeasurement(R, t);
   }

   // Initialize factor
   // Enforce an order for separator measurement. (lower_id, higher_id).
   gtsam::BetweenFactor<gtsam::Pose3> new_factor;
   if (other_robot_symbol.chr() > this_robot_symbol.chr()) {
      new_factor = gtsam::BetweenFactor<gtsam::Pose3>(this_robot_symbol, other_robot_symbol, measurement, noise_model_);

      UpdateCurrentSeparatorBuzzStructure( this->GetBuzzVM()->robot,
                                             other_robot_id,
                                             this_robot_pose_id,
                                             other_robot_pose_id,
                                             measurement.x(),
                                             measurement.y(),
                                             measurement.z(),
                                             measurement.rotation().quaternion()[1],
                                             measurement.rotation().quaternion()[2],
                                             measurement.rotation().quaternion()[3],
                                             measurement.rotation().quaternion()[0] );
   } else {      
      measurement = measurement.inverse();
      new_factor = gtsam::BetweenFactor<gtsam::Pose3>(other_robot_symbol, this_robot_symbol, measurement, noise_model_);

      UpdateCurrentSeparatorBuzzStructure( other_robot_id,
                                             this->GetBuzzVM()->robot,
                                             other_robot_pose_id,
                                             this_robot_pose_id,
                                             -measurement.x(),
                                             -measurement.y(),
                                             measurement.z(),
                                             measurement.rotation().quaternion()[1],
                                             measurement.rotation().quaternion()[2],
                                             measurement.rotation().quaternion()[3],
                                             measurement.rotation().quaternion()[0] );
   }   

   // Add new factor to local pose graph
   local_pose_graph_->push_back(new_factor);

   return is_outlier;
}

/****************************************/
/****************************************/

void CBuzzControllerQuadMapperNoSensing::SavePoseGroundTruth(){
   gtsam::Point3 t_gt = {  m_pcPos->GetReading().Position.GetX(), 
                           m_pcPos->GetReading().Position.GetY(),
                           m_pcPos->GetReading().Position.GetZ() };
   
   CQuaternion current_orientation = m_pcPos->GetReading().Orientation;
   gtsam::Quaternion current_quat_gtsam(current_orientation.GetW(), current_orientation.GetX(), current_orientation.GetY(), current_orientation.GetZ());
   gtsam::Rot3 R_gt(current_quat_gtsam);

   gtsam::Pose3 pose_gt(R_gt, t_gt);
   ground_truth_poses_.insert(std::make_pair(number_of_poses_, pose_gt));   
}


/****************************************/
/****************************************/

bool CBuzzControllerQuadMapperNoSensing::CompareCentralizedAndDecentralizedError() {
   // Collect expected estimate size
   std::string local_dataset_file_name = "log/datasets/" + std::to_string(robot_id_) + "_initial.g2o";
   gtsam::GraphAndValues local_graph_and_values = gtsam::readG2o(local_dataset_file_name, true);
   int expected_size = local_graph_and_values.second->size();

   // Aggregate estimates from all the robots
   gtsam::Values distributed;
   std::vector<gtsam::GraphAndValues> graph_and_values_vec;
   int number_of_separators = 0;
   for (size_t i = 0; i < number_of_robots_; i++) {
      std::string dataset_file_name = "log/datasets/" + std::to_string(i) + "_optimized.g2o";
      if (!boost::filesystem::exists(dataset_file_name)) {
         return false; // File does not exists yet
      }
      gtsam::GraphAndValues graph_and_values = gtsam::readG2o(dataset_file_name, true);
      if (graph_and_values.second->size() != expected_size) {
         return false; // File not update yet
      }
      for (const gtsam::Values::ConstKeyValuePair &key_value: *graph_and_values.second) {
         gtsam::Key key = key_value.key;
         if (!distributed.exists(key)) {
            distributed.insert(key, (*graph_and_values.second).at<gtsam::Pose3>(key));
         }
      }
      for (const auto &factor: *graph_and_values.first) {
         boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3> > pose3_between = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factor);
         auto robot_1_id = gtsam::Symbol(pose3_between->key1()).chr();
         auto robot_2_id = gtsam::Symbol(pose3_between->key2()).chr();
         if (robot_1_id != robot_2_id) {
            number_of_separators ++;
         }
      }
      dataset_file_name = "log/datasets/" + std::to_string(i) + "_initial.g2o";
      graph_and_values = gtsam::readG2o(dataset_file_name, true);
      graph_and_values_vec.push_back(graph_and_values);
   }
   gtsam::GraphAndValues full_graph_and_values = distributed_mapper::evaluation_utils::readFullGraph(number_of_robots_, graph_and_values_vec);

   // Compute Error
   gtsam::noiseModel::Diagonal::shared_ptr evaluation_model = gtsam::noiseModel::Isotropic::Variance(6, 1e-12);
   auto errors = distributed_mapper::evaluation_utils::evaluateEstimates(number_of_robots_,
                                                      full_graph_and_values,
                                                      evaluation_model,
                                                      chordal_graph_noise_model_,
                                                      false,
                                                      distributed );

   // Write results to csv
   std::ofstream error_file;
   error_file.open(error_file_name_, std::ios::out | std::ios::app);
   auto number_of_poses = optimizer_->numberOfPosesInCurrentEstimate();
   error_file << number_of_robots_ << "\t" << number_of_poses << "\t" << number_of_separators << "\t" << outlier_probability_ << std::boolalpha << "\t" << incremental_solving_
            << "\t" << rotation_noise_std_ << "\t" << translation_noise_std_ << "\t" << rotation_estimate_change_threshold_ << "\t" << pose_estimate_change_threshold_
            << "\t" << std::get<0>(errors) << "\t" << std::get<1>(errors) << "\t" << std::get<2>(errors) << "\t" << current_rotation_iteration_ << "\t" << current_pose_iteration_ << "\n";
   error_file.close();

   return std::abs(std::get<0>(errors) - std::get<1>(errors)) < 0.1;
}

}