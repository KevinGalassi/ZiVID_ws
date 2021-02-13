#ifndef AUXILIARY_ARM_PIN2PIN_H
#define AUXILIARY_ARM_PIN2PIN_H

#include <ros/ros.h>
#include <time.h>
#include <list>
#include <string.h>

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
#include <uc1_p2p/ScrewdriverAction.h>
#include <uc1_p2p/ScrewdriverActionResult.h>

//#include <XmlRpc/XmlRpcValue.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>


class TesterPin2Pin
{
public:

   TesterPin2Pin(std::string name, std::string robot = "");
   ~TesterPin2Pin();

   void error_cb(const std_msgs::Float64MultiArray& msg);
   bool resultGood(int result, std::string component, std::string traj_type);
   void toolRotation();
   bool activate_deactivate_arm(move_rt::TaskParamUpdate::Request &req, move_rt::TaskParamUpdate::Response &res);

protected:

   ros::NodeHandle nh_;
   
//Action
   actionlib::SimpleActionServer<uc1_p2p::PinToPinAction> auxiliary_as;
   std::string action_name;
   void execute_cb(const uc1_p2p::PinToPinGoalConstPtr& goal);


   actionlib::SimpleActionClient<move_rt::ExecutingTrajectoryAction> move_client;
   actionlib::SimpleActionClient<uc1_p2p::WaitAction> op_client;

   move_rt::ExecutingTrajectoryGoal move_client_goal;
   move_rt::ExecutingTrajectoryResultConstPtr move_client_result; 
   move_rt::ExecutingTrajectoryGoal move_home_goal;

   uc1_p2p::PinToPinFeedback auxiliary_as_feedback;
   uc1_p2p::PinToPinResult auxiliary_as_result;    
   uc1_p2p::WaitGoal op_client_goal;

// Sub/Pub
   ros::Subscriber error_sub;
   ros::Publisher connection_pub;

   std_msgs::Bool connection_msg;


// Service
   ros::ServiceServer enableArm; 

   ros::ServiceClient eeEnable;
   ros::ServiceClient emergencyEnable;
   ros::ServiceClient setToolFrame;
   ros::ServiceClient clearTraj;

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

//   std::map<std::string, float> traj_list;

   //std::list<std::map<std::string,float>> 

   //static XmlRpc::XmlRpcValue prova = XmlRpc::XmlRpcValue::makeArray();

   XmlRpc::XmlRpcValue prova;

   std::vector<std_msgs::Float64MultiArray> traj_list;
   std_msgs::Float64MultiArray home_position;
   
};


#endif

