#ifndef ROBOTSAFETY_H
#define ROBOTSAFETY_H

#include <ros/ros.h> //ALWAYS need to include this

#include <move_rt/controlmanager.hpp>
#include <move_rt/prioritylevel.hpp>
#include <std_msgs/Bool.h>

class RobotSafety: public PriorityLevel
{
public:
    
    RobotSafety(ros::NodeHandle &nodehandle, Eigen::Matrix<long double,Eigen::Dynamic,1> &_q);
    ~RobotSafety();

    //void updatexm(Eigen::Matrix<long double,Eigen::Dynamic,1> xm);
    void updateA();
    void updateConstraints();

    void show();

    string get_task();

    Eigen::Matrix<long double,Eigen::Dynamic,Eigen::Dynamic> get_JK(){return JK;};


    Eigen::Matrix<long double,Eigen::Dynamic,1> &joint_vel;

private:

    ros::Subscriber sub_safetyactivation;
    bool safety_val;
    void callbackactivation(const std_msgs::Bool::ConstPtr& msg);
};

#endif // ROBOTSAFETY_H
