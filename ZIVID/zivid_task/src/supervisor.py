#!/usr/bin/env python
import sys
import rospy
import time
import actionlib
import copy
from geometry_msgs.msg import Quaternion, Pose, Point
from zivid_msgs.msg import  moveWireAction, moveWireGoal
from zivid_msgs.srv import takeFrame, takeFrameResponse
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


        print('Take Camera service')
        rospy.wait_for_service('take_frame_service')
        self.takeFrame_srv = rospy.ServiceProxy('take_frame_service', takeFrame)
        print('Camera Service Service OK')


        self.rightArmMoveWireGoal = moveWireGoal()
        self.newFrame_resp = takeFrameResponse()

        self.release_height = 0.05


        self.rightToolOrientation = Quaternion()
        self.rightToolOrientation.w = 0.0
        self.rightToolOrientation.x = 0.707
        self.rightToolOrientation.y = -0.707
        self.rightToolOrientation.z = 0.0

        
        # self.rightToolOrientation.w = 0.5
        # self.rightToolOrientation.x = 0.5
        # self.rightToolOrientation.y = -0.5
        # self.rightToolOrientation.z = 0.5
        


        traj_home = [{'orient_w': 0.0, 'orient_x': 0.707, 'orient_y': 0.0, 'orient_z':  0.707, 'pos_x': -0.10931, 'pos_y': +0.38207, 'pos_z': 0.41, 'time': 5.0}]
       
        rospy.set_param('/{}traj_home'.format(self.arm), traj_home)

        print('Supervisor Ready')



    def move_wire(self, new_pick_pose, new_place_pose, release_height):                        

        # Send instruction to right arm
        self.rightArmMoveWireGoal.pick_pose = new_pick_pose
        self.rightArmMoveWireGoal.place_pose = new_place_pose
        self.rightArmMoveWireGoal.release_height = release_height
        self.rightArmMoveWireGoal.requested_action = "Moving"

        self.right_arm_client.send_goal(self.rightArmMoveWireGoal)
        print('Send request to right arm')
        self.right_arm_client.wait_for_result()
        result = self.right_arm_client.wait_for_result()

        return (result)

    def move_homing(self):

        self.rightArmMoveWireGoal.requested_action = "Homing"

        self.right_arm_client.send_goal(self.rightArmMoveWireGoal)
        print('Send request to right arm')
        self.right_arm_client.wait_for_result()
        result = self.right_arm_client.wait_for_result()

        return (result)



    def run_destination(self):

        while(True):

            print('Call ZiVID Service')
            self.newFrame_resp = self.takeFrame_srv()

            pick_pose = Pose()
            place_pose = Pose()


            pick_pose = self.newFrame_resp.pick_pose
            pick_pose.position.z -= 0.02
            place_pose = self.newFrame_resp.place_pose

            print('target position \n',pick_pose.position)

            #pick_pose.position.z -= 0.02

            #print('Final target position', pick_pose.position)


            self.move_wire(pick_pose, place_pose, self.release_height)

            print('Homing')
            self.move_homing()

            print('Addio')

            continue
            
    



if __name__ == '__main__':
    rospy.init_node('main_arm')
    if len(sys.argv) > 1:
        print(sys.argv[1])
        server = MainArmWaypoint(sys.argv[1])
    else:
        server = MainArmWaypoint()
    server.run_destination()
