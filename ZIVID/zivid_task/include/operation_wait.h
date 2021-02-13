
#ifndef OPERATION_ACTION_H
#define OPERATION_ACTION_H


#include <ros/ros.h>
#include <time.h>
#include <string.h>

#include <actionlib/server/simple_action_server.h>

#include <uc1_p2p/WaitAction.h>
#include <uc1_p2p/WaitFeedback.h>
#include <uc1_p2p/WaitResult.h>



class OperationAction
{
public:

   OperationAction(std::string name);
   ~OperationAction();

protected:

   ros::NodeHandle nh_;

   actionlib::SimpleActionServer<uc1_p2p::WaitAction> as_;
   std::string action_name;
   void execute_cb(const uc1_p2p::WaitGoalConstPtr& goal);

   uc1_p2p::WaitFeedback _as_feedback;
   uc1_p2p::WaitResult _as_result;

};


#endif // OPERATION_ACTION_H