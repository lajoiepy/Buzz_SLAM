#include <buzz/buzzasm.h>
#include "utils/buzz_utils.h"
#include "message_handler_utils/message_handler_utils.h"
#include <std_srvs/Empty.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <chrono>
#include <thread>

static int done = 0;
static graph_utils::PoseWithCovariance accumulated_measurement_;
static double rotation_std_, translation_std_;
static int next_robot_id_; // TODO: This only works in the 3 robots case where the sender/receivers are preassigned

static void ctrlc_handler(int sig) {
   done = 1;
}

static void add_odometry(const nav_msgs::Odometry::ConstPtr &msg)
{
   // Fill the received transform with the converted pose and covariance
   graph_utils::PoseWithCovariance measurement;
   pose_ros_to_gtsam(msg->pose.pose, measurement.pose);
   covariance_to_matrix(msg->pose.covariance, measurement.covariance_matrix);

   // Accumulate the transforms
   poseCompose(accumulated_measurement_,
               measurement,
               accumulated_measurement_);

   // Add a node if the current frame is a keyframe
   //if (msg->keyFrameAdded) // TODO: double check
   //{
      set_covariance_matrix(accumulated_measurement_.covariance_matrix, rotation_std_, translation_std_);

      buzz_slam::BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<buzz_slam::BuzzSLAMRos>(VM->robot)->AddOdometryMeasurement(accumulated_measurement_.pose, accumulated_measurement_.covariance_matrix);
      ROS_INFO("Add odometry measurement");
      accumulated_measurement_.pose = gtsam::Pose3();
   //}
}

static bool add_separators(loop_closure_transform::ReceiveSeparators::Request &req,
                          loop_closure_transform::ReceiveSeparators::Response &res)
{
   for (int idx = 0; idx < req.kf_ids_from.size(); idx++)
   {
      gtsam::Symbol robot_symbol_from;
      gtsam::Symbol robot_symbol_to;
      
      // local robot goes with kf local since it computed the matches
      robot_symbol_from = gtsam::Symbol('a' + req.robot_from_id, req.kf_ids_from[idx]);
      robot_symbol_to = gtsam::Symbol('a' + req.robot_to_id, req.kf_ids_to[idx]);

      gtsam::Pose3 measurement;
      pose_ros_to_gtsam(req.separators[idx].pose, measurement);

      gtsam::Matrix covariance_matrix;
      set_covariance_matrix(covariance_matrix, rotation_std_, translation_std_);

      gtsam::SharedNoiseModel noise_model = gtsam::noiseModel::Gaussian::Covariance(covariance_matrix);

      ROS_INFO("Add separator, %llu, %llu", robot_symbol_from.key(), robot_symbol_to.key());
      gtsam::BetweenFactor<gtsam::Pose3> new_factor = gtsam::BetweenFactor<gtsam::Pose3>(robot_symbol_from, robot_symbol_to, measurement, noise_model);
      buzz_slam::BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<buzz_slam::BuzzSLAMRos>(VM->robot)->AddSeparatorMeasurement(new_factor);

      if (VM->robot == req.robot_from_id) {
         graph_utils::PoseWithCovariance pose;
         pose_with_covariance_from_msg(req.pose_estimates_to[idx], pose);
         buzz_slam::BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<buzz_slam::BuzzSLAMRos>(VM->robot)->UpdatePoseEstimateFromNeighbor(req.robot_to_id, req.kf_ids_to[idx], pose);
      } else if (VM->robot == req.robot_to_id) {
         graph_utils::PoseWithCovariance pose;
         pose_with_covariance_from_msg(req.pose_estimates_from[idx], pose);
         buzz_slam::BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<buzz_slam::BuzzSLAMRos>(VM->robot)->UpdatePoseEstimateFromNeighbor(req.robot_from_id, req.kf_ids_from[idx], pose);
      }
   }
   res.success = true;
   return true;
}

static bool get_pose_estimates(loop_closure_transform::PoseEstimates::Request &req,
                          loop_closure_transform::PoseEstimates::Response &res)
{
   ROS_INFO("Get pose estimate");
   std::vector<geometry_msgs::PoseWithCovariance> pose_estimates;
   for (const auto pose_id : req.pose_ids)
   {
      auto pose_estimate = buzz_slam::BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<buzz_slam::BuzzSLAMRos>(VM->robot)->GetPoseEstimateAtID(pose_id);
      geometry_msgs::PoseWithCovariance pose_estimate_msg;
      pose_with_covariance_to_msg(pose_estimate, pose_estimate_msg);
      pose_estimates.emplace_back(pose_estimate_msg);
   }
   res.pose_estimates = pose_estimates;
   return true;
}

static bool trigger_optimization(std_srvs::Empty::Request  &req,
                                 std_srvs::Empty::Response &res) {
   buzz_slam::BuzzSLAMSingleton::GetInstance().GetBuzzSLAM<buzz_slam::BuzzSLAMRos>(VM->robot)->TriggerOptimization();
   return true;
}

int main(int argc, char** argv) {
   // Intialization
   ros::init( argc, argv, "buzz_generic_robot_slam_ros" );
   ros::NodeHandle nh_;

   // Parse parameters
   int msg_sz;
   if (!nh_.getParam("/"+ros::this_node::getName()+"/message_size", msg_sz))
  	{
  		msg_sz = 100000;
  	}

   std::string buzz_script_name, bcfname, dbgfname;
   if (!nh_.getParam("/"+ros::this_node::getName()+"/buzz_script_name", buzz_script_name))
  	{
  		buzz_script_name = "";
      ROS_ERROR("You must specify a Buzz script to execute.");
  	}
   bcfname = buzz_script_name + ".bo";
   dbgfname = buzz_script_name + ".bdb";

   int robot_id;
   if (!nh_.getParam("/"+ros::this_node::getName()+"/robot_id", robot_id))
  	{
  		robot_id = 0;
  	}

   std::string port;
   if (!nh_.getParam("/"+ros::this_node::getName()+"/port", port))
  	{
  		port = "24580";
  	}
   
   if (!nh_.getParam("/"+ros::this_node::getName()+"/next_robot_id", next_robot_id_))
  	{
  		next_robot_id_ = (robot_id + 1) % 3;
  	}

   if (!nh_.getParam("/"+ros::this_node::getName()+"/rotation_std", rotation_std_))
  	{
  		rotation_std_ = 0.01;
  	}
   
   if (!nh_.getParam("/"+ros::this_node::getName()+"/translation_std", translation_std_))
  	{
  		translation_std_ = 0.1;
  	}

   /* Wait for connection */
   if(!buzz_listen("tcp", msg_sz, port.c_str())) return 1;
   /* Set CTRL-C handler */
   signal(SIGTERM, ctrlc_handler);
   signal(SIGINT, ctrlc_handler);

   /* Set the Buzz bytecode */
   if(buzz_script_set<buzz_slam::BuzzSLAMRos>(robot_id, bcfname.c_str(), dbgfname.c_str())) {

      /* Initialize the ROS subcribers */
      accumulated_measurement_.pose = gtsam::Pose3();
      accumulated_measurement_.covariance_matrix = gtsam::eye(6,6);
      ros::Subscriber sub_odom = nh_.subscribe("odom_info", 1000, &add_odometry);
      ros::ServiceServer s_add_separators = nh_.advertiseService("add_separators_pose_graph", &add_separators);
      ros::ServiceServer s_get_pose_estimates = nh_.advertiseService("get_pose_estimates", &get_pose_estimates);
      ros::ServiceServer s_trigger_optimization = nh_.advertiseService("start_optimization", &trigger_optimization);

      /* Main loop */
      while(!done && !buzz_script_done() && ros::ok()){
         buzz_script_step();
         ros::spinOnce();
      }
      /* Cleanup */
      buzz_script_destroy();
   }
   
   /* Stop the robot */
   buzz_slam::BuzzSLAMSingleton::GetInstance().Destroy();
   /* All done */
   return 0;
}
