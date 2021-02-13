#include <ros/ros.h>

#include <trajectory_msgs/JointTrajectory.h> //visualization_msgs::Marker marker
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#include <sensor_msgs/JointState.h>

class PositionInterface {
private: 
    ros::NodeHandle &nh;    
    
    std::vector<std::string> ordered_joints;
    
    ros::Time prev_update_time_;
    
    int n;
    
    trajectory_msgs::JointTrajectory position_msg;
    trajectory_msgs::JointTrajectoryPoint point_msg;
    
    bool initialized;
    
    std::vector<double> q;
    
    ros::Publisher position_pub; 
    
    void callbackJState(const sensor_msgs::JointState::ConstPtr &msg);
    
    
    void callbackVelocity(const std_msgs::Float64MultiArray::ConstPtr &msg);
    
    
public:
    PositionInterface(ros::NodeHandle &nh, std::string position_topic);
    
    ~PositionInterface(){};
};

PositionInterface::PositionInterface(ros::NodeHandle &nh, std::string position_topic): nh(nh) 
{ 
    std::string joint_state_topic, controller_topic;
                    
    nh.getParam("n", n);
    
    nh.getParam("controller_topic", controller_topic);
    nh.getParam("joint_state_topic", joint_state_topic);
    
    nh.getParam("ordered_joints", ordered_joints);
    
    position_msg.joint_names = ordered_joints;  
    
    position_msg.points.resize(1);
    
    q.resize(n);
    
    point_msg.positions.resize(n);
    point_msg.velocities.resize(n);    
    point_msg.accelerations.resize(n);      
    point_msg.effort.resize(n);
    
    initialized = false;

    ros::Subscriber joint_state_sub = nh.subscribe<sensor_msgs::JointState>(joint_state_topic, 1, &PositionInterface::callbackJState, this);
    
    ros::Subscriber velocity_sub = nh.subscribe<std_msgs::Float64MultiArray>(controller_topic + "/command", 1, &PositionInterface::callbackVelocity, this);
        
    position_pub = nh.advertise<trajectory_msgs::JointTrajectory>(position_topic, 1);
                        
    prev_update_time_ = ros::Time::now();
    
    ROS_INFO("PositionInterface: started!");
    
    ros::spin();
}


void PositionInterface::callbackJState(const sensor_msgs::JointState::ConstPtr &msg) 
{

    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < msg->position.size(); i++) {
            if (msg->name[i] == ordered_joints[j]) {
                ROS_DEBUG_STREAM("PositionInterface::callbackJState: joint_states.name["
                                << i << "] = " << msg->name[i]);
                ROS_DEBUG_STREAM(
                    "PositionInterface::callbackJState: joint_states.position["
                    << i << "] = " << msg->position[i]);

                q[i] = msg->position[i];                
                i = msg->position.size();
            }
        }
    }

    for (int j = 0; j < n; ++j)
        ROS_DEBUG_STREAM("PositionInterface::callbackJState: q(" << j
                                                            << ") = " << q[j]);

    if (!initialized) 
    {
        for (int j = 0; j < n; ++j) point_msg.positions[j] = q[j];
        initialized = true;
        prev_update_time_ = ros::Time::now();
    }
}

void PositionInterface::callbackVelocity(const std_msgs::Float64MultiArray::ConstPtr &msg) 
{
    ros::Time time_now = ros::Time::now();
    ros::Duration step_time = time_now - prev_update_time_;
    prev_update_time_ = time_now;    
    
    for(int i = 0; i < msg->data.size(); i++)           
            if(initialized) 
            {
                point_msg.positions[i] = point_msg.positions[i] + msg->data[i] * step_time.toSec();
                point_msg.velocities[i] = 0.0;//msg->data[i];
                point_msg.accelerations[i] = 0.0;
                point_msg.effort[i] = 0.0;
            }                
            else 
            {
                point_msg.positions[i] = q[i];
                point_msg.velocities[i] = 0.0;
                point_msg.accelerations[i] = 0.0;
                point_msg.effort[i] = 0.0;
            }
        
    point_msg.time_from_start = step_time;
    
    position_msg.points[0] = point_msg;
    
    position_pub.publish(position_msg);
};

int main(int argc, char **argv) {

  if (argc < 2) {
    std::cout << "position_interface: please provide position topic" << std::endl;
    return -1;
  }
  
  ros::init(argc, argv, "position_interface"); // node name

  ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    
  PositionInterface pos_interface(nh, argv[1]);
    
}
