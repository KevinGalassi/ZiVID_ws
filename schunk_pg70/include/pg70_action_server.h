#ifndef PG70_ACTION_SERVER_H
#define PG70_ACTION_SERVER_H

#include <ros/ros.h>
#include <serial/serial.h>

#include <time.h>
#include <list>
#include <string.h>

#include <actionlib/server/simple_action_server.h>



#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include <schunk_pg70/Grasp.h>


class pg70ActionServer
{
public:

   pg70ActionServer(std::string name, std::string robot="");
   ~pg70ActionServer();
   



protected:

   ros::NodeHandle nh_;

   actionlib::SimpleActionServer<schunk_pg70::Grasp> pg70_as;
   std::string action_name;

   void execute_cb(const schunk_pg70::GraspGoalConstPtr& goal);

   ros::Subscriber  position_sub;

}