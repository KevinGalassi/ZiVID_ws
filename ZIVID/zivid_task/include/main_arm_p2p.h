
#ifndef MAIN_ARM_PIN2PIN_H
#define MAIN_ARM_PIN2PIN_H

#include <ros/ros.h>
#include <time.h>
#include <map>
#include "boost/variant.hpp"


#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <move_rt/ExecutingTrajectoryAction.h>
#include <move_rt/ExecutingTrajectoryGoal.h>
#include <move_rt/ExecutingTrajectoryResult.h>

#include <move_rt/UpdateFrame.h>
#include <move_rt/TaskParamUpdate.h>

#include <uc1_p2p/WaitAction.h>
#include <uc1_p2p/WaitGoal.h>
#include <uc1_p2p/PinToPinAction.h>
#include <uc1_p2p/PinToPinResult.h>
#include <uc1_p2p/PinToPinFeedback.h>
#include <uc1_p2p/findComponentAction.h>
#include <uc1_p2p/findComponentGoal.h>
#include <uc1_p2p/findComponentResult.h>
#include <uc1_p2p/ScrewdriverAction.h>
#include <uc1_p2p/ScrewdriverActionResult.h>

#include <database_wires/DataSrv.h>
#include <database_wires/connection_data.h>
#include <database_wires/pin_data.h>
#include <database_wires/cad_data.h>
#include <database_wires/cad_pose.h>


#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include <string.h>

typedef database_wires::connection_data::Response connection_data_res;
typedef database_wires::connection_data::Request connection_data_req;

class MainArmPin2Pin
{

public:

   //MainArmPin2Pin(std::string name, std::string robot = "");
   MainArmPin2Pin(std::string robot = "");
   ~MainArmPin2Pin();

   void connection_cb(const std_msgs::Bool &msg);
   void error_cb(const std_msgs::Float64MultiArray& msg);

   bool resultGood(int result, std::string component, std::string traj_type);
   void toolRotation();
   geometry_msgs::Pose findComponent(geometry_msgs::Pose connection_pose);
   geometry_msgs::Pose findComponentCamera(database_wires::connection_data::Response connection);

   bool activate_deactivate_arm(move_rt::TaskParamUpdate::Request &req, move_rt::TaskParamUpdate::Response &res);
   int compute_p2p(connection_data_res main_connection, 
                   connection_data_res aux_connection, 
                   geometry_msgs::Pose main_pose, 
                   geometry_msgs::Pose aux_pose);

   void destination();
   void runDestination();
   void CheckNearestArm(geometry_msgs::Pose& main_pose,
                        geometry_msgs::Pose& dest_pose, 
                        connection_data_res& orig_centered, 
                        connection_data_res& dest_centered);
protected:


   ros::NodeHandle nh_;
   
//Action
 //  actionlib::SimpleActionServer<uc1_p2p::PinToPinAction> main_as;
   std::string action_name;
  // void execute_cb(const uc1_p2p::PinToPinGoalConstPtr& goal);


   actionlib::SimpleActionClient<move_rt::ExecutingTrajectoryAction> move_client;
   actionlib::SimpleActionClient<uc1_p2p::WaitAction> op_client;
   actionlib::SimpleActionClient<uc1_p2p::PinToPinAction> p2p_client;
   actionlib::SimpleActionClient<uc1_p2p::findComponentAction> find_client;

   move_rt::ExecutingTrajectoryGoal move_client_goal;
   move_rt::ExecutingTrajectoryResultConstPtr move_client_result; 
   move_rt::ExecutingTrajectoryGoal move_home_goal;

   uc1_p2p::PinToPinFeedback main_arm_as_feedback;
   uc1_p2p::PinToPinResult main_arm_as_result;    

   uc1_p2p::WaitGoal op_client_goal;
   uc1_p2p::PinToPinGoal auxiliary_arm_goal;

   uc1_p2p::findComponentGoal findClientGoal;
   uc1_p2p::findComponentResultConstPtr findClientResult;

   
// Sub/Pub
   ros::Subscriber error_sub;
   ros::Subscriber connection_sub;
   
// Service
   ros::ServiceServer enableArm; 

   ros::ServiceClient resetList;
   ros::ServiceClient resetToList;
   ros::ServiceClient nextConnection;
   ros::ServiceClient pinPose;
   ros::ServiceClient componentPose;

   ros::ServiceClient eeEnable;
   ros::ServiceClient emergencyEnable;
   ros::ServiceClient setToolFrame;
   ros::ServiceClient clearTraj;

   database_wires::DataSrv data_srv;
   connection_data_res connection_srv;
   connection_data_req connection_req;
   database_wires::pin_data pin_data_srv;
   database_wires::cad_pose cad_pose_srv;
   
   std_srvs::Trigger Trigger_srv;

   move_rt::TaskParamUpdate TaskParamUpdate_srv;
   move_rt::UpdateFrame UpdateFrame_srv;

// Other variables

   std::vector<std::string> error_list;
   std::string new_error;
   std::string arm_name;

   float component_descent;
   float terminal_descent;
   float additional_descent;
   float squareNormError;
   float error_value;

   bool arm_active;
   bool arm_motion;
   bool rotation_direction;

   int error_code;

   std::vector<std_msgs::Float64MultiArray> traj_list;
   std_msgs::Float64MultiArray home_position;
   std_msgs::Float64MultiArray camera_orientation;
   std_msgs::Float64MultiArray tool_orientation;
   
};


#endif

