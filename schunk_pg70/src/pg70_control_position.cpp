#include "ros/ros.h"
#include "schunk_pg70/message_control_position.h"
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64.h"
#include "schunk_pg70/set_pvac.h"
#include "schunk_pg70/reference.h"
#include <pg70_rs232_control.h>

float current = 2999;

void setPositionCallBack(const schunk_pg70::message_control_position &msg);
void setPointCallBack(const sensor_msgs::JointState &msg);
void jointCommandCallBack(const sensor_msgs::JointState &msg);
void setMaxCurrentCallBack(const std_msgs::Float64 &msg);

int main (int argc, char **argv)
{

	ros::init (argc, argv, "schunk_pg70_controlPosition");

	ros::NodeHandle control;
	ros::ServiceClient client;

	control.getParam("/pg70_control_position/current", current);
	std::cout<< "Current := " << current << "\n";


	client = control.serviceClient<schunk_pg70::reference>("schunk_pg70/reference", true);
	schunk_pg70::reference srvRef;
	client.call(srvRef);

	ros::Subscriber setPosition = control.subscribe("schunk_pg70/control_position", 1000, setPositionCallBack);
	ros::Subscriber setPoint = control.subscribe("schunk_pg70/joint_setpoint", 1000, setPointCallBack);
	ros::Subscriber jointCommand = control.subscribe("schunk_pg70/joint_command", 1000, jointCommandCallBack);
	ros::Subscriber setMaxCurrent = control.subscribe("schunk_pg70/set_max_current", 1000, setMaxCurrentCallBack);

	ros::spin();

	return 0;
}


void setPositionCallBack(const schunk_pg70::message_control_position &msg)
{
	ros::NodeHandle control;
	ros::ServiceClient client;

	client = control.serviceClient<schunk_pg70::set_pvac>("schunk_pg70/set_pvac", true);

	if(!client)
	{
		ROS_ERROR("Failed to call service");
		return;
	}

	schunk_pg70::set_pvac srv;
	srv.request.goal_position = msg.position;
	srv.request.goal_velocity = msg.velocity;
	srv.request.goal_acceleration = msg.acceleration;
	srv.request.goal_current = msg.current;

	client.call(srv);

	return;
}

void setPointCallBack(const sensor_msgs::JointState &msg)
{
	ros::NodeHandle control;
	ros::ServiceClient client;

	client = control.serviceClient<schunk_pg70::set_pvac>("schunk_pg70/set_pvac", true);

	if(!client)
	{
		ROS_ERROR("Failed to call service");
		return;
	}

	schunk_pg70::set_pvac srv;
	srv.request.goal_position = msg.position[0];
	srv.request.goal_velocity = msg.velocity[0];
	srv.request.goal_acceleration = msg.effort[0];
	srv.request.goal_current = current;

	client.call(srv);

	return;
}

void jointCommandCallBack(const sensor_msgs::JointState &msg)
{
	ros::NodeHandle control;
	ros::ServiceClient client;

	client = control.serviceClient<schunk_pg70::set_pvac>("schunk_pg70/set_pvac", true);

	if(!client)
	{
		ROS_ERROR("Failed to call service");
		return;
	}

	int position = (msg.position[0]*1000) + (msg.position[1]*1000);

	schunk_pg70::set_pvac srv;
	srv.request.goal_position = position;
	srv.request.goal_velocity = 50;
	srv.request.goal_acceleration = 50;
	srv.request.goal_current = current;

	client.call(srv);

	return;
}

void setMaxCurrentCallBack(const std_msgs::Float64 &msg)
{
	current = msg.data;
	ROS_INFO ("IL VALORE DI CORRENTE MASSIMA È STATO CAMBIATO");
}