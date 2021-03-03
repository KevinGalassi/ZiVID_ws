#! /usr/bin/env python
import sys
import rospy
import time
import actionlib
import copy
import tf
import numpy as np
from zivid_msgs.msg import cameraAquisitionAction, cameraAquisitionGoal, cameraAquisitionResult, cameraAquisitionFeedback
from zivid_msgs.srv import CameraSampleDataset, CameraSampleDatasetResponse
from move_rt.msg import ExecutingTrajectoryAction, ExecutingTrajectoryGoal, ExecutingTrajectoryResult
from move_rt.srv import *

from std_msgs.msg import Float64MultiArray, Bool


from geometry_msgs.msg import Pose, PoseArray

import moveit_commander
import moveit_msgs.msg


class CameraActionServer(object):
   # create messages that are used to publish feedback/result
   CameraActionServer_as_feedback = cameraAquisitionFeedback()
   CameraActionServer_as_result = cameraAquisitionResult()
   moveClientGoal = ExecutingTrajectoryGoal()

   def __init__(self, name, robot = ''):
      #definition of action server, services and action clients
      self.CameraActionServer_as_name = robot[:-1] + name
      print( self.CameraActionServer_as_name)
      self.CameraActionServer_as = actionlib.SimpleActionServer(self.CameraActionServer_as_name, cameraAquisitionAction, execute_cb=self.execute_cb, auto_start = False)
      
      self.arm = robot
      self.active = True

      print(self.arm)

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


      # Camera Client # DA COMPLETARE

      print('Camera Aquisition service: ')
      rospy.wait_for_service('/take_sample')
      self.camera_client = rospy.ServiceProxy('/take_sample', CameraSampleDataset)
      print('Camera Service: OK\n')

   

      self.joint_pub = rospy.Publisher('{}joint_setpoint'.format(self.arm),Float64MultiArray, queue_size=10)     


      self.clearTraj([0])
      self.emergencyEnable([1])
      self.JointEnable([0])
      self.eeEnable([0])
      
      self.moveClientGoal = ExecutingTrajectoryGoal()
      self.moveClientGoal.ee_error_th = 0.005
      self.moveClientResult = ExecutingTrajectoryResult()
      self.joint_goal = Float64MultiArray()
      self.traj_list = [{'orient_w': -0.123, 'orient_x': 0.7, 'orient_y': 0.122, 'orient_z': 0.69, 'pos_x':  0.23347, 'pos_y': -0.35757, 'pos_z': 0.41, 'time': 3.0}]
      self.pause_timeout = 0.5
      


      self.target_pose = Pose()
      self.camera_res = CameraSampleDatasetResponse()

      '''
      self.move_group = moveit_commander.MoveGroupCommander("ur5")
      self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                             moveit_msgs.msg.DisplayTrajectory,
                                             queue_size=20)
   '''
      # self.rate = rospy.Rate(rospy.get_param("hz", 30))

      # self.br = tf.TransformBroadcaster()

      self.CameraActionServer_as.start()
      print('CameraActionServer arm Server Ready')

       
        
   def execute_cb(self, goal):

      self.eeEnable([0])
      self.clearTraj([0])
      self.emergencyEnable([1])

      self.setToolFrame('ee_link')

      print('{}ee_link'.format(self.arm))

      self.target_pose = copy.deepcopy(goal.pose_sequence.poses[0])

      self.traj_list[0]['orient_w'] = self.target_pose.orientation.w
      self.traj_list[0]['orient_x'] = self.target_pose.orientation.x
      self.traj_list[0]['orient_y'] = self.target_pose.orientation.y
      self.traj_list[0]['orient_z'] = self.target_pose.orientation.z
      self.traj_list[0]['pos_x'] = self.target_pose.position.x
      self.traj_list[0]['pos_y'] = self.target_pose.position.y
      self.traj_list[0]['pos_z'] = self.target_pose.position.z
      self.traj_list[0]['time'] = 10.0

      self.eeEnable([1])
      self.emergencyEnable([0])

      self.moveClientGoal.trajectory_name = '/{}move_to_target'.format(self.arm)
      rospy.set_param(self.moveClientGoal.trajectory_name, self.traj_list)
      
      self.move_client.send_goal(self.moveClientGoal)
      result_ok = self.move_client.wait_for_result()
      print(result_ok)



      for i in range( len(goal.pose_sequence.poses) ):
         
         print('Move to pose: ', i)
         print(goal.pose_sequence.poses[i])

         self.target_pose = copy.deepcopy(goal.pose_sequence.poses[i])

         self.traj_list[0]['orient_w'] = self.target_pose.orientation.w
         self.traj_list[0]['orient_x'] = self.target_pose.orientation.x
         self.traj_list[0]['orient_y'] = self.target_pose.orientation.y
         self.traj_list[0]['orient_z'] = self.target_pose.orientation.z
         self.traj_list[0]['pos_x'] = self.target_pose.position.x
         self.traj_list[0]['pos_y'] = self.target_pose.position.y
         self.traj_list[0]['pos_z'] = self.target_pose.position.z
         self.traj_list[0]['time'] = 1.0

         self.eeEnable([1])
         self.emergencyEnable([0])

         self.moveClientGoal.trajectory_name = '/{}move_to_target'.format(self.arm)
         rospy.set_param(self.moveClientGoal.trajectory_name, self.traj_list)
         
         self.move_client.send_goal(self.moveClientGoal)
         result_ok = self.move_client.wait_for_result()
         print(result_ok)

         self.emergencyEnable([1])
         self.eeEnable([0])
         self.clearTraj([1])
         
         counter = 0
         time.sleep(0.5)
         while(counter < 3) : 
            self.camera_res = self.camera_client()
            
            if (self.camera_res.result.data):
               break

            counter = counter + 1

         if counter >= 3 :
            print('Camera Not WORKING')
         
               

      self.CameraActionServer_as_result.success = True
      self.CameraActionServer_as.set_succeeded(self.CameraActionServer_as_result)
      print('CameraActionServer arm concluded\n')


if __name__ == '__main__':
    rospy.init_node('CameraActionServer')
    if len(sys.argv) > 1:
        print(sys.argv[1])
        server = CameraActionServer(rospy.get_name(), sys.argv[1]) 
    else:
        server = CameraActionServer(rospy.get_name())
    rospy.spin()