#include <move_rt/controlmanager.hpp>
#include <xmlrpcpp/XmlRpcValue.h>
#include <move_rt/TaskParamUpdate.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "trajectory_node"); // node name

  ros::NodeHandle
      nh; // create a node handle; need to pass this to the class constructor

  double cycleHz = 1;

  ros::Rate rate(cycleHz);

  if (argc < 2) {
    ROS_INFO("trajectory_node: please provide trajectory name");
    return -1;
  }

  if (!nh.hasParam(argv[1])) {
    ROS_INFO(
        "trajectory_node: trajectory %s does not exist in parameter server",
        argv[1]);
    return -1;
  }

  XmlRpc::XmlRpcValue waypoints_vector;

  nh.getParam(argv[1], waypoints_vector);

  if (waypoints_vector.getType() == XmlRpc::XmlRpcValue::Type::TypeArray &&
      waypoints_vector.size() > 0) {
    ROS_INFO("trajectory_node: trajectory %s loaded with %d points", argv[1],
             waypoints_vector.size());
    // waypoints_vector[0] is a 'TypeArray' aka vector
  } else {
    ROS_INFO("trajectory_node: trajectory is empty");
    return -1;
  }

  ros::Publisher ee_traj_point_pub =
      nh.advertise<std_msgs::Float64MultiArray>("ee_setpoint", 1);

  ros::Duration(1).sleep();

  ros::ServiceClient Emergency_client = nh.serviceClient<move_rt::TaskParamUpdate>("EmergencyStop/setEnable");
  
  ros::ServiceClient EePosition_client = nh.serviceClient<move_rt::TaskParamUpdate>("EePosition/setEnable");
  
  move_rt::TaskParamUpdate srv_msg;
  srv_msg.request.data.clear();
  srv_msg.request.data.push_back(0);
  
  if (Emergency_client.call(srv_msg))
  {
    ROS_INFO("EmergencyStop_setEnable: Success");
  }
  else
  {
    ROS_ERROR("EmergencyStop_setEnable: Failed");
    return 1;
  }
  
  srv_msg.request.data.clear();
  srv_msg.request.data.push_back(1);
  
  if (EePosition_client.call(srv_msg))
  {
    ROS_INFO("EePosition_setEnable: Success");
  }
  else
  {
    ROS_ERROR("EePosition_setEnable: Failed");
    return 1;
  }
  
  std_msgs::Float64MultiArray trajectory_point;

  trajectory_point.layout.dim.push_back(
      std_msgs::MultiArrayDimension()); // one structure MultiArrayDimension per
                                        // dimension
  trajectory_point.layout.data_offset = 0;
  trajectory_point.layout.dim[0].label = "";
  trajectory_point.layout.dim[0].size = 8;
  trajectory_point.layout.dim[0].stride = 0;

  std::vector<std::string> fields{"pos_x",    "pos_y",    "pos_z",
                                  "orient_w", "orient_x", "orient_y",
                                  "orient_z", "time"};

  for (int i = 0; i < waypoints_vector.size(); i++) {
    trajectory_point.data.clear();

    if (waypoints_vector[i].getType() ==
        XmlRpc::XmlRpcValue::Type::TypeStruct) {
      for (int j = 0; j < fields.size(); j++) {
        if (waypoints_vector[i].hasMember(fields[j])) {
          trajectory_point.data.push_back(
              double(waypoints_vector[i][fields[j]]));
        } else {
          ROS_INFO("trajectory_node: trajectory waypoint[%d] has no member %s",
                   i, fields[j].c_str());
          return -1;
        }
      }
    } else {
      ROS_INFO("trajectory_node: trajectory waypoint[%d] type is not a Struct",
               i);
      return -1;
    }

    ee_traj_point_pub.publish(trajectory_point);

    ros::spinOnce();

    ros::Duration(0.1).sleep();
  }

  ROS_INFO("trajectory_node: trajectory is sent");

  return 0;
}
