#! /usr/bin/env python
import sys
import rospy
import time
import actionlib
import copy
import tf
import os
import numpy as np
from zivid_msgs.msg import moveWireAction, moveWireResult, moveWireFeedback
from move_rt.msg import ExecutingTrajectoryAction, ExecutingTrajectoryGoal, ExecutingTrajectoryResult
from move_rt.srv import *
from std_msgs.msg import Float64MultiArray, Bool
from schunk_pg70.srv import set_pvac
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import moveit_commander
import moveit_msgs.msg


class ArmActionServer(object):
    # create messages that are used to publish feedback/result
    ArmActionServer_as_feedback = moveWireFeedback()
    ArmActionServer_as_result = moveWireResult()
    moveClientGoal = ExecutingTrajectoryGoal()

    def __init__(self, name, robot = ''):
        self.transiction_counter = 1
        self.grasp_fails_txt = "grasp_fails.txt"
        if os.path.exists(self.grasp_fails_txt):
            os.remove(self.grasp_fails_txt)

        #definition of action server, services and action clients
        self.ArmActionServer_as_name = robot[:-1] + name
        self.ArmActionServer_as = actionlib.SimpleActionServer(self.ArmActionServer_as_name, moveWireAction, execute_cb=self.execute_cb, auto_start = False)
        
        self.arm = robot
        self.active = True

        print('EE trajectory execution Action Client: Waiting')
        self.move_client = actionlib.SimpleActionClient('{}ee_execute_trajectory'.format(self.arm), ExecutingTrajectoryAction)
        self.move_client.wait_for_server()
        print('EE trajectory execution Action Client: OK\n')

        print('Joint trajectory execution Action Client: Waiting')
        self.joint_client = actionlib.SimpleActionClient('{}simple_joint_trajectory'.format(self.arm), ExecutingTrajectoryAction)
        self.joint_client.wait_for_server()
        print('Joint rajectory execution Action Client: OK\n')


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
        rospy.wait_for_service('/schunk_pg70/set_pvac')
        self.gripper_client = rospy.ServiceProxy('/schunk_pg70/set_pvac', set_pvac)
        print('Gripper Service: OK\n')


        self.joint_pub = rospy.Publisher('{}joint_setpoint'.format(self.arm),Float64MultiArray, queue_size=10)     
        self.effort_current_value = 0
        self.effort_threshold = 0.15


        self.clearTraj([0])
        self.emergencyEnable([1])
        self.JointEnable([0])
        self.eeEnable([0])
        
        self.moveClientGoal = ExecutingTrajectoryGoal()
        self.moveClientGoal.ee_error_th = 0.01
        self.moveClientResult = ExecutingTrajectoryResult()
        self.joint_goal = Float64MultiArray()
        self.traj_list = [{'orient_w': -0.123, 'orient_x': 0.7, 'orient_y': 0.122, 'orient_z': 0.69, 'pos_x':  0.23347, 'pos_y': -0.35757, 'pos_z': 0.41, 'time': 3.0}]

        self.joint_list = [{'joint_1': -1.34, 'joint_2': -1.15, 'joint_3': -1.75, 'joint_4':-1.69, 'joint_5':  1.57, 'joint_6': -0.6, 'time': 2.0}, {'joint_1': -1.34, 'joint_2': -1.15, 'joint_3': -1.75, 'joint_4':-1.69, 'joint_5':  1.57, 'joint_6': -0.6, 'time': 2.0}]
        
        self.joint_list[0]['joint_1'] = -1.34
        self.joint_list[0]['joint_2'] = -1.15
        self.joint_list[0]['joint_3'] = -1.75
        self.joint_list[0]['joint_4'] = -1.69
        self.joint_list[0]['joint_5'] = 1.57
        self.joint_list[0]['joint_6'] = -0.6
        self.joint_list[0]['time'] = 1.5

        self.joint_list[1]['joint_1'] = -1.34
        self.joint_list[1]['joint_2'] = -1.10
        self.joint_list[1]['joint_3'] = -1.70
        self.joint_list[1]['joint_4'] = -1.69
        self.joint_list[1]['joint_5'] = 1.57
        self.joint_list[1]['joint_6'] = -0.6
        self.joint_list[1]['time'] = 0.5
        
        
        self.pause_timeout = 0.5


        self.starting_height = 0.05
        
        self.gripper_velocity = 50
        self.gripper_acceleration = 100
        self.setToolFrame('schunk_pg70_object_link')
        self.ArmActionServer_as.start()
        
        self.max_x = 0.2
        self.max_y = 0.3
        self.max_z = 0.18

        self.target_pose = Pose()

        '''
        self.move_group = moveit_commander.MoveGroupCommander("ur5")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
    '''
        # self.rate = rospy.Rate(rospy.get_param("hz", 30))

        # self.br = tf.TransformBroadcaster()

        print('ArmActionServer arm Server Ready')


    def check_limit(self, check_pose):

        if (check_pose.position.x > self.max_x) :
            check_pose.position.x = self.max_x
            print(' sat x')

        if (check_pose.position.x < -self.max_x) :
            check_pose.position.x = -self.max_x
            print(' sat x')

        if (check_pose.position.y > self.max_y) :
            check_pose.position.y = self.max_y
            print(' sat y')

        if (check_pose.position.y < -self.max_y) :
            check_pose.position.y = -self.max_y
            print(' sat y')

        if (check_pose.position.z > self.max_z) :
            check_pose.position.z = self.max_z
            print(' sat z')

        if (check_pose.position.z < 0.025) :
            check_pose.position.z = 0.025
            print(' sat z')

        return check_pose
        
        
    def execute_cb(self, goal):

        if goal.requested_action == 'Moving':
            self.eeEnable([1])

            self.clearTraj([0])
            self.emergencyEnable([1])
            self.eeEnable([0])
            print('Gripper Opening')
            result = self.gripper_client(60, self.gripper_velocity, self.gripper_acceleration, 2000)
            

            '''
            mat2 = np.dot(mat1, t)

            trans2 = tf.transformations.translation_from_matrix(mat2)
            rot2 = tf.transformations.quaternion_from_matrix(mat2)
            
            br.sendTransform(trans2,  rot2, rospy.get_rostime(), "world", "target_pose")
            '''

            print('Reach Starting position')

             
            self.target_pose = copy.deepcopy(goal.pick_pose)
            self.target_pose.position.z += self.starting_height
            self.target_pose = self.check_limit(self.target_pose)

            self.traj_list[0]['orient_w'] = self.target_pose.orientation.w
            self.traj_list[0]['orient_x'] = self.target_pose.orientation.x
            self.traj_list[0]['orient_y'] = self.target_pose.orientation.y
            self.traj_list[0]['orient_z'] = self.target_pose.orientation.z
            self.traj_list[0]['pos_x'] = self.target_pose.position.x
            self.traj_list[0]['pos_y'] = self.target_pose.position.y
            self.traj_list[0]['pos_z'] = self.target_pose.position.z
            self.traj_list[0]['time'] = 2.0
            print('Target pose z: ', self.target_pose.position.z)


            print('sending Approaching goal to action server \n')
            self.emergencyEnable([0])
            self.eeEnable([1])

            self.moveClientGoal.trajectory_name = '/{}traj_approach'.format(self.arm)
            rospy.set_param(self.moveClientGoal.trajectory_name, self.traj_list)
            
            self.move_client.send_goal(self.moveClientGoal)
            result_ok = self.move_client.wait_for_result()
            print(result_ok)

            #time.sleep(self.pause_timeout)
                        
            self.clearTraj([0])

            # Descend to the component location

            print('sending Descending goal to action server \n')

            self.target_pose  = copy.deepcopy(goal.pick_pose)
            self.target_pose = self.check_limit(self.target_pose)
            self.traj_list[0]['orient_w'] = self.target_pose.orientation.w
            self.traj_list[0]['orient_x'] = self.target_pose.orientation.x
            self.traj_list[0]['orient_y'] = self.target_pose.orientation.y
            self.traj_list[0]['orient_z'] = self.target_pose.orientation.z
            self.traj_list[0]['pos_x'] = self.target_pose.position.x
            self.traj_list[0]['pos_y'] = self.target_pose.position.y
            self.traj_list[0]['pos_z'] = self.target_pose.position.z
            self.traj_list[0]['time'] = 3.0

            self.moveClientGoal.trajectory_name = '/{}Descending'.format(self.arm)
            rospy.set_param(self.moveClientGoal.trajectory_name, self.traj_list)
            self.move_client.send_goal(self.moveClientGoal)
            self.move_client.wait_for_result()
            self.moveClientResult = self.move_client.get_result()

            # Send command to gripper
            print('Gripper Closing')
            self.emergencyEnable([1])
            self.eeEnable([0])
            self.clearTraj([0])   
            result = self.gripper_client(30, self.gripper_velocity, self.gripper_acceleration, 300)
            time.sleep(3)


            # Ascendinga
            print('sending Ascending goal to action server')

            self.emergencyEnable([0])
            self.eeEnable([1])
            self.target_pose  = copy.deepcopy(goal.pick_pose)
            self.target_pose = self.check_limit(self.target_pose)
            self.traj_list[0]['orient_w'] = self.target_pose.orientation.w
            self.traj_list[0]['orient_x'] = self.target_pose.orientation.x
            self.traj_list[0]['orient_y'] = self.target_pose.orientation.y
            self.traj_list[0]['orient_z'] = self.target_pose.orientation.z
            self.traj_list[0]['pos_x'] = self.target_pose.position.x
            self.traj_list[0]['pos_y'] = self.target_pose.position.y
            self.traj_list[0]['pos_z'] = goal.place_pose.position.z + goal.release_height
            self.traj_list[0]['time'] = 2.0

            self.moveClientGoal.trajectory_name = '/{}Descending'.format(self.arm)
            rospy.set_param(self.moveClientGoal.trajectory_name, self.traj_list)
            self.move_client.send_goal(self.moveClientGoal)
            self.move_client.wait_for_result()
            self.moveClientResult = self.move_client.get_result()  

            # Moving


            self.target_pose = copy.deepcopy(goal.place_pose)
            self.target_pose = self.check_limit(goal.place_pose)
            self.traj_list[0]['orient_w'] = self.target_pose.orientation.w
            self.traj_list[0]['orient_x'] = self.target_pose.orientation.x
            self.traj_list[0]['orient_y'] = self.target_pose.orientation.y
            self.traj_list[0]['orient_z'] = self.target_pose.orientation.z
            self.traj_list[0]['pos_x'] = self.target_pose.position.x
            self.traj_list[0]['pos_y'] = self.target_pose.position.y
            self.traj_list[0]['pos_z'] = self.target_pose.position.z + goal.release_height
            self.traj_list[0]['time'] = 2.0

            #Sistemare orientamento con z
            self.moveClientGoal.trajectory_name = '/{}Moving'.format(self.arm)
            rospy.set_param(self.moveClientGoal.trajectory_name, self.traj_list)

            print('Sending moving goal')
            self.move_client.send_goal(self.moveClientGoal)
            result_ok = self.move_client.wait_for_result()
            self.clearTraj([0])   



            
            self.target_pose = copy.deepcopy(goal.place_pose)
            self.target_pose = self.check_limit(goal.place_pose)
            self.traj_list[0]['orient_w'] = self.target_pose.orientation.w
            self.traj_list[0]['orient_x'] = self.target_pose.orientation.x
            self.traj_list[0]['orient_y'] = self.target_pose.orientation.y
            self.traj_list[0]['orient_z'] = self.target_pose.orientation.z
            self.traj_list[0]['pos_x'] = self.target_pose.position.x
            self.traj_list[0]['pos_y'] = self.target_pose.position.y
            self.traj_list[0]['pos_z'] = self.target_pose.position.z
            self.traj_list[0]['time'] = 2.0

            #Sistemare orientamento con z
            self.moveClientGoal.trajectory_name = '/{}Moving'.format(self.arm)
            rospy.set_param(self.moveClientGoal.trajectory_name, self.traj_list)

            print('Sending moving goal')
            self.move_client.send_goal(self.moveClientGoal)
            result_ok = self.move_client.wait_for_result()
            self.clearTraj([0])  

            '''
            # CHECK GRASP SUCCESS
            gripper_status = rospy.wait_for_message("schunk_pg70/joint_states",JointState)
            if gripper_status.effort[0] < self.effort_threshold:
                print("-----------> GRASP FAIL!") 
                f = open(self.grasp_fails_txt, 'a')
                f.write(self.transiction_counter)
                f.close()
            '''

            # Release

            print('Gripper Opening')
            result = self.gripper_client(60, self.gripper_velocity, self.gripper_acceleration, 1500)
            time.sleep(1.5)

            self.traj_list[0]['pos_z'] += 0.015
            self.traj_list[0]['time'] = 2.0
            self.moveClientGoal.trajectory_name = '/{}Ascending'.format(self.arm)
            rospy.set_param(self.moveClientGoal.trajectory_name, self.traj_list)

            print('Sending Pre-Homing')
            self.move_client.send_goal(self.moveClientGoal)
            result_ok = self.move_client.wait_for_result()
            self.clearTraj([0])   
            self.emergencyEnable([1])
            self.eeEnable([0])

            self.transiction_counter += 1
            

        if goal.requested_action == 'Homing':
            
            '''
            
            self.eeEnable([0])
            self.JointEnable([1])
            self.emergencyEnable([0])


            joint_goal = move_group.get_current_joint_value()
            joint_goal[0] = -pi/2
            joint_goal[1] = -pi/4
            joint_goal[2] = -pi/2
            joint_goal[3] = -pi/2
            joint_goal[4] = -2*pi/3
            joint_goal[5] = pi
            joint_goal[6] = 0.0


            self.move_group.go(joint_goal, wait=True)
            self.move_group.stop()

            self.JointEnable([0])
            self.emergencyEnable([1])
            
            
            '''
            
            
            self.eeEnable([0])
            self.JointEnable([1])
            self.emergencyEnable([0])

            self.moveClientGoal.trajectory_name = '/{}Ascending'.format(self.arm)
            rospy.set_param(self.moveClientGoal.trajectory_name, self.joint_list)

            print('Sending Pre-Homing')
            self.joint_client.send_goal(self.moveClientGoal)
            result_ok = self.joint_client.wait_for_result()


            '''
            self.joint_list
            self.joint_goal.data = [-1.57, -0.785, -1.60, -2.16, 1.57, 0.0, 2.0]
            self.joint_pub.publish(self.joint_goal)
            time.sleep(4)
            '''
            self.JointEnable([0])
            self.emergencyEnable([1])

            print('Arm Home\n')
            

        self.ArmActionServer_as_result.success = True
        self.ArmActionServer_as.set_succeeded(self.ArmActionServer_as_result)
        print('ArmActionServer arm concluded\n')


if __name__ == '__main__':
    rospy.init_node('ArmActionServer')
    if len(sys.argv) > 1:
        print(sys.argv[1])
        server = ArmActionServer(rospy.get_name(), sys.argv[1]) 
    else:
        server = ArmActionServer(rospy.get_name())
    rospy.spin()