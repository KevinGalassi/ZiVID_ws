#ifndef PG70_ACTION_SERVER_H
#define PG70_ACTION_SERVER_H

#include <ros/ros.h>
#include <serial/serial.h>

#include <time.h>
#include <list>
#include <string.h>

#include <actionlib/server/simple_action_server.h>


#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include <schunk_pg70/GraspAction.h>
#include <schunk_pg70/GraspGoal.h>
#include <schunk_pg70/GraspResult.h>


#include <schunk_pg70/set_pvac.h>


class pg70ActionServer
{
public:

   pg70ActionServer(std::string name);
   ~pg70ActionServer();
   
   


protected:

   ros::NodeHandle nh_;
//   ros::Rate loop_rate_(60);

   int gripper_id_;
   std::string port_name_;
   int baudrate_;
   serial::Serial *com_port_;
   
   float act_position_;
   float act_velocity_;
   float act_current_;
   uint8_t pg70_error_;


   actionlib::SimpleActionServer<schunk_pg70::GraspAction> pg70_as_;
   std::string action_name;

   schunk_pg70::GraspAction grasp_action;
   schunk_pg70::GraspGoal grasp_goal;
   schunk_pg70::GraspResultConstPtr grasp_result;
   schunk_pg70::GraspResult pg70_as_result_;
   sensor_msgs::JointState joint_state_;

   serial::Serial *port;

   static constexpr double MIN_GRIPPER_POS_LIMIT = 0;
   static constexpr double MAX_GRIPPER_POS_LIMIT = 69;
   static constexpr double MIN_GRIPPER_VEL_LIMIT = 0;
   static constexpr double MAX_GRIPPER_VEL_LIMIT = 83;
   static constexpr double MIN_GRIPPER_ACC_LIMIT = 0;
   static constexpr double MAX_GRIPPER_ACC_LIMIT = 320;
   static constexpr double MIN_GRIPPER_CUR_LIMIT = 0;
   static constexpr double MAX_GRIPPER_CUR_LIMIT = 2999;
   static constexpr double WAIT_FOR_RESPONSE_INTERVAL = 0.5;
   static constexpr double INPUT_BUFFER_SIZE = 64;
   static const int    URDF_SCALE_FACTOR = 2000;
   static constexpr float TF_UPDATE_PERIOD = 0.1;   



   float IEEE_754_to_float(uint8_t *raw);
   void float_to_IEEE_754(float position, unsigned int *output_array);
   uint16_t CRC16(uint16_t crc, uint16_t data);   
   uint8_t getError(serial::Serial *port);

   void execute_cb(const schunk_pg70::GraspGoalConstPtr& goal);
   void getPeriodicPositionUpdate(serial::Serial *port, float update_period);
   void joint_callback(const sensor_msgs::JointState &msg);
   ros::Subscriber  joint_sub_;
   ros::ServiceClient set_pvac;
   schunk_pg70::set_pvac new_command_;


};

#endif