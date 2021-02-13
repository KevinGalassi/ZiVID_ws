#!/usr/bin/env python
import sys
import rospy
import time
import actionlib
import copy
from geometry_msgs.msg import Quaternion
from zivid_msgs.msg import  moveWireAction, moveWireGoal
from move_rt.srv import *

class MainArmWaypoint(object):


    #initialization method
    def __init__(self, robot = ''):

        self.traj_list = [{'orient_w': None, 'orient_x': None, 'orient_y': None, 'orient_z':  None, 'pos_x': None, 'pos_y': None, 'pos_z': None, 'time': None}]
        #definition of services and actions
        self.arm = robot
        self.active = True

      

        # MOVE-RT ACTION CLIENT

        print('Right Arm pin to pin Action Client: Waiting')
        self.right_arm_client = actionlib.SimpleActionClient('right_arm/ArmActionServer', moveWireAction)
        self.right_arm_client.wait_for_server()
        print('Right Arm pin to pin Action Client: OK\n')


        self.rightArmMoveWireGoal = moveWireGoal()

        self.tool_orientation = [{'orient_w': 0.5, 'orient_x': 0.5, 'orient_y': -0.5, 'orient_z': 0.5, 'time': 5.0}]
        self.camera_orientation = [{'orient_w': 0.0, 'orient_x': 0.707, 'orient_y': 0.0, 'orient_z': 0.707, 'time': 5.0}]


        self.rightToolOrientation = Quaternion()
        self.rightToolOrientation.w = 0.5
        self.rightToolOrientation.x = 0.5
        self.rightToolOrientation.y = -0.5
        self.rightToolOrientation.z = 0.5



        traj_home = [{'orient_w': 0.0, 'orient_x': 0.707, 'orient_y': 0.0, 'orient_z':  0.707, 'pos_x': -0.10931, 'pos_y': +0.38207, 'pos_z': 0.41, 'time': 5.0}]
       
        rospy.set_param('/{}traj_home'.format(self.arm), traj_home)

        print('Supervisor Ready')



    def move_wire(self, target_pose, displacement, release_height):                        

        # Send instruction to right arm
        self.rightArmMoveWireGoal.target_pose.position = target_pose.position
        self.rightArmMoveWireGoal.target_pose.orientation = rightToolOrientation
        self.rightArmMoveWireGoal.dispalcement = displacement
        self.rightArmMoveWireGoal.release_height = release_height
        self.rightArmMoveWireGoal.requested_action = "Moving"

        self.right_arm_client.send_goal(self.rightArmMoveWireGoal)
        print('Send request to right arm')
        self.right_arm_client.wait_for_result()
        result = self.right_arm_client.wait_for_result()

        return (result)

    def move_arm(self):

        self.rightArmMoveWireGoal.requested_action = "Homing"

        self.right_arm_client.send_goal(self.rightArmMoveWireGoal)
        print('Send request to right arm')
        self.right_arm_client.wait_for_result()
        result = self.right_arm_client.wait_for_result()

        return (result)



    def run_destination(self):

        while(True):
            continue
            
    



if __name__ == '__main__':
    rospy.init_node('main_arm')
    if len(sys.argv) > 1:
        print(sys.argv[1])
        server = MainArmWaypoint(sys.argv[1])
    else:
        server = MainArmWaypoint()
    server.run_destination()
