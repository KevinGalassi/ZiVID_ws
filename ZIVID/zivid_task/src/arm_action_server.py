#! /usr/bin/env python
import sys
import rospy
import time
import actionlib
import copy
from zivid_msgs.msg import WaitAction, WaitGoal, moveWireAction, moveWireResult, moveWireFeedback
from move_rt.msg import ExecutingTrajectoryAction, ExecutingTrajectoryGoal, ExecutingTrajectoryResult
from move_rt.srv import *
from std_msgs.msg import Float64MultiArray, Bool
from schunk_pg70.msg import *
from schunk_pg70.srv import set_position

class ArmActionServer(object):
    # create messages that are used to publish feedback/result
    ArmActionServer_as_feedback = moveWireFeedback()
    ArmActionServer_as_result = moveWireResult()
    moveClientGoal = ExecutingTrajectoryGoal()

    def __init__(self, name, robot = ''):
        #definition of action server, services and action clients
        self.ArmActionServer_as_name = robot[:-1] + name
        self.ArmActionServer_as = actionlib.SimpleActionServer(self.ArmActionServer_as_name, moveWireAction, execute_cb=self.execute_cb, auto_start = False)
        
        self.arm = robot
        self.active = True

        print('trajectory execution Action Client: Waiting')
        self.move_client = actionlib.SimpleActionClient('{}ee_execute_trajectory'.format(self.arm), ExecutingTrajectoryAction)
        self.move_client.wait_for_server()
        print('trajectory execution Action Client: OK\n')

        print('end-effector enable service: Waiting')
        rospy.wait_for_service('{}EePosition/setEnable'.format(self.arm))
        self.eeEnable = rospy.ServiceProxy('{}EePosition/setEnable'.format(self.arm), TaskParamUpdate)
        print('end-effector enable service: OK\n')

        print('emergency enable service: Waiting')        
        rospy.wait_for_service('{}EmergencyStop/setEnable'.format(self.arm))
        self.emergencyEnable = rospy.ServiceProxy('{}EmergencyStop/setEnable'.format(self.arm), TaskParamUpdate)
        print('emergency enable service: OK\n')

        print('set tool frame service: Waiting')
        rospy.wait_for_service('{}EePosition/setToolFrame'.format(self.arm))
        self.setToolFrame = rospy.ServiceProxy('{}EePosition/setToolFrame'.format(self.arm), UpdateFrame)
        print('set tool frame service: OK\n')

        print('clear trajecotory service: Waiting')
        rospy.wait_for_service('{}EePosition/clearTrajectory'.format(self.arm))
        self.clearTraj = rospy.ServiceProxy('{}EePosition/clearTrajectory'.format(self.arm), TaskParamUpdate)
        print('clear trajecotory service: OK\n')
        
        print('end-effector enable service: Waiting')
        rospy.wait_for_service('{}JointPosition/setEnable'.format(self.arm))
        self.JointEnable = rospy.ServiceProxy('{}JointPosition/setEnable'.format(self.arm), TaskParamUpdate)
        print('end-effector enable service: OK\n')


        # Gripper Client

        print('Gripper client connection Waiting')
        rospy.wait_for_service('schunk_pg70/set_position')
        self.gripper_client = rospy.ServiceProxy('schunk_pg70/set_position', set_position)
        print('Gripper Service: OK\n')


        self.joint_pub = rospy.Publisher('{}joint_setpoint'.format(self.arm),Float64MultiArray, queue_size=10)     


        self.clearTraj([0])
        self.moveClientGoal = ExecutingTrajectoryGoal()
        self.moveHome = ExecutingTrajectoryGoal()
        self.moveClientResult = ExecutingTrajectoryResult()
        self.joint_goal = Float64MultiArray()
        self.home_position = [{'orient_w': -0.123, 'orient_x': 0.7, 'orient_y': 0.122, 'orient_z': 0.69, 'pos_x':  0.23347, 'pos_y': -0.35757, 'pos_z': 0.41, 'time': 5.0}]

        self.pause_timeout = 0.5

        self.moveHome.trajectory_name = '/{}traj_home'.format(self.arm)
        rospy.set_param(self.moveHome.trajectory_name, self.home_position)
       

        self.starting_height = 0.10
        
        self.gripper_velocity = 50
        self.gripper_acceleration = 100


        self.ArmActionServer_as.start()  
        print('ArmActionServer arm Server Ready')





    def execute_cb(self, goal):

        if goal.requested_action == 'Moving':
            self.eeEnable([1])
            self.setToolFrame('fingers') 
            

            print('Reach Starting position')
            self.traj_list = copy.deepcopy(self.home_position)
            self.traj_list[0]['orient_w'] = goal.target_pose.orientation.w
            self.traj_list[0]['orient_x'] = goal.target_pose.orientation.x
            self.traj_list[0]['orient_y'] = goal.target_pose.orientation.y
            self.traj_list[0]['orient_z'] = goal.target_pose.orientation.z
            self.traj_list[0]['pos_x'] = goal.target_pose.position.x
            self.traj_list[0]['pos_y'] = goal.target_pose.position.y
            self.traj_list[0]['pos_z'] = goal.target_pose.position.z + self.starting_height

            self.moveClientGoal.trajectory_name = '/{}traj_approach'.format(self.arm)
            rospy.set_param(self.moveClientGoal.trajectory_name, self.traj_list)

            print('sending Approaching goal to action server: \ntrajectory name'.format(self.moveClientGoal.trajectory_name))

            self.emergencyEnable([0])
            self.eeEnable([1])
            self.move_client.send_goal(self.moveClientGoal)
            result_ok = self.move_client.wait_for_result()
            print(result_ok)

            time.sleep(self.pause_timeout)
                        
            self.clearTraj([0])

            # Descend to the component location
            self.traj_list[0]['pos_z'] = self.traj_list[0]['pos_z'] - self.starting_height
            self.moveClientGoal.trajectory_name = '/{}Descending'.format(self.arm)
            rospy.set_param(self.moveClientGoal.trajectory_name, self.traj_list)

            time.sleep(self.pause_timeout)

            print('sending Descending goal to action server: \ntrajectory name = {} \nerror threshold = {}'.format(self.moveClientGoal.trajectory_name, self.moveClientGoal.ee_error_th))
            self.move_client.send_goal(self.moveClientGoal)
            self.move_client.wait_for_result()
            self.moveClientResult = self.move_client.get_result()
            print(self.move_client.get_result())

            err_count = 0

            # Send command to gripper
            self.emergencyEnable([1])
            self.eeEnable([0])
            self.clearTraj([0])   

            print('Gripper Closing')
            result = gripper_client(10, self.gripper_velocity, self.gripper_acceleration)


            # Ascending
            self.traj_list[0]['pos_z'] = self.traj_list[0]['pos_z'] + self.starting_height
            self.moveClientGoal.trajectory_name = '/{}Ascending'.format(self.arm)
            rospy.set_param(self.moveClientGoal.trajectory_name, self.traj_list)

            self.emergencyEnable([0])
            self.eeEnable([1])
            print('sending Ascending goal to action server: \ntrajectory name = {} \nerror threshold = {}'.format(self.moveClientGoal.trajectory_name, self.moveClientGoal.ee_error_th))
            self.move_client.send_goal(self.moveClientGoal)
            result_ok = self.move_client.wait_for_result()
            print(result_ok)
            self.clearTraj([0])   

            # Moving
            self.traj_list[0]['pos_x'] = self.traj_list[0]['pos_x'] + goal.displacement.x
            self.traj_list[0]['pos_y'] = self.traj_list[0]['pos_y'] + goal.displacement.y
            #Sistemare orientamento con z
            self.moveClientGoal.trajectory_name = '/{}Moving'.format(self.arm)
            rospy.set_param(self.moveClientGoal.trajectory_name, self.traj_list)

            print('Sending moving goal')
            self.move_client.send_goal(self.moveClientGoal)
            result_ok = self.move_client.wait_for_result()
            print(result_ok)
            self.clearTraj([0])   


            # Release

            self.clearTraj([0])
            self.emergencyEnable([1])
            self.eeEnable([0])
            print('Gripper Closing')
            result = gripper_client(10, self.gripper_velocity, self.gripper_acceleration)


        if goal.requested_action == 'Homing':
            self.eeEnable([0])
            self.JointEnable([1])
            self.emergencyEnable([0])
            self.joint_goal.data = [-1.57, -0.785, -1.60, -2.16, 1.57, 0.0, 5.0]
            self.joint_pub.publish(self.joint_goal)
            time.sleep(10)
            self.JointEnable([0])
            self.emergencyEnable([1])
        
            self.testerPin2Pin_as_result.success = True
            self.testerPin2Pin_as.set_succeeded(self.testerPin2Pin_as_result)
            print('Arm Home\n')


        self.ArmActionServer_as_result.success = True
        self.ArmActionServer_as.set_succeeded(self.ArmActionServer_as_result)
        print('ArmActionServer arm Home\n')


if __name__ == '__main__':
    rospy.init_node('ArmActionServer')
    if len(sys.argv) > 1:
        print(sys.argv[1])
        server = ArmActionServer(rospy.get_name(), sys.argv[1]) 
    else:
        server = ArmActionServer(rospy.get_name())
    rospy.spin()