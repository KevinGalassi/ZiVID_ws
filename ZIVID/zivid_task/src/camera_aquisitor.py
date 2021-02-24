#!/usr/bin/env python
import sys
import rospy
import time
import actionlib
import copy
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, Pose, Point
from zivid_msgs.msg import *
from zivid_msgs.srv import *
from move_rt.srv import *



class MainArmWaypoint(object):


   #initialization method
   def __init__(self, robot = ''):

      self.traj_list = [{'orient_w': None, 'orient_x': None, 'orient_y': None, 'orient_z':  None, 'pos_x': None, 'pos_y': None, 'pos_z': None, 'time': None}]
      #definition of services and actions
      self.arm = robot
      self.active = True

   

      # MOVE-RT ACTION CLIENT

   #   print('Right Arm pin to pin Action Client: Waiting')
   #   self.right_arm_client = actionlib.SimpleActionClient('right_arm/ArmActionServer', moveWireAction)
   #   self.right_arm_client.wait_for_server()
   #   print('Right Arm pin to pin Action Client: OK\n')

   
      print('Left Arm Camera Server Action Client: Waiting')
      self.left_arm_client = actionlib.SimpleActionClient('/left_arm/CameraActionServer', cameraAquisitionAction)
      self.left_arm_client.wait_for_server()
      print('Left Arm Camera Server Action Client: OK\n')


      # Servizio per nuovi punti
      print('New Pose Service: Waiting')
      rospy.wait_for_service('get_new_pose_array')
      self.getNewPoseArray_client = rospy.ServiceProxy('get_new_pose_array', newPoseArray)
      print('New Pose Service: OK')

      self.cameraGoal = cameraAquisitionActionGoal()
      self.newPoseArray_resp = newPoseArrayResponse()      
      
      self.file_name = String()
      self.file_name.data = "robot_poses.txt"


      print('Supervisor Ready')





   def run_destination(self):

      while(True):

         #prendere nuova posa
         print('Call ZiVID Service')
         self.NewPoseArray_resp = self.getNewPoseArray_client(self.file_name)


         print('Sending Pose to Action Server')

         self.cameraGoal.goal.pose_sequence = self.NewPoseArray_resp.array
         
         self.left_arm_client.send_goal(self.cameraGoal.goal)


         result = self.left_arm_client.wait_for_result()
         print('Camera concluded, result : ', result)

         print('Ciao')

         break



if __name__ == '__main__':
   rospy.init_node('main_arm')
   if len(sys.argv) > 1:
      print(sys.argv[1])
      server = MainArmWaypoint(sys.argv[1])
   else:
      server = MainArmWaypoint()
   server.run_destination()
