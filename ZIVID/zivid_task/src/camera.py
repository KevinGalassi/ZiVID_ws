#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import os
import time
import sys
import cv2
import numpy as np
import copy
import PyKDL

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import rospy
import open3d as o3d

from superros.logger import Logger
from superros.comm import RosNode
import superros.transformations as transformations
from visionpylib.cameras import CameraRGB, CameraRGBD


def call_camera(req):
 """ Camera callback. produce FrameRGBD object """
    output = frame.rgb_image.copy()
    output_depth = frame.depth_image.copy() 

    tf_target = None
    if tf_enabled:
        tf_target = node.retrieveTransform(node.getParameter("TF_TARGET"), node.getParameter("TF_BASE"), -1)
        if tf_target == None:
            print("no tf!")
            return

    cv2.imshow("output", output)
    c = cv2.waitKey(1)

    if c > 0:
        if c == KEY_SPACE:
            print("saving...")
            if not os.path.exists(node.getParameter("OUTPUT_PATH")):
                os.makedirs(node.getParameter("OUTPUT_PATH"))

            coutner_str = str(frame_save_counter).zfill(5)
            frame_name = "frame_{}.png".format(coutner_str)
            frame_name_depth = "depth_{}.exr".format(coutner_str)
            #tffile_name = "pose_{}.txt".format(coutner_str)
            frame_save_counter += 1
            filename = os.path.join(node.getParameter("OUTPUT_PATH"), frame_name)
            filename_depth = os.path.join(node.getParameter("OUTPUT_PATH"), frame_name_depth)
            #filenametf = os.path.join(node.getParameter("OUTPUT_PATH"), tffile_name)

            cv2.imwrite(filename, output)
            cv2.imwrite(filename_depth, output_depth)

            cloud_msg = rospy.wait_for_message(node.getParameter("CAMERA_TOPIC_CLOUD"), PointCloud2, 10)
            cloud_points = list(pc2.read_points(cloud_msg, skip_nans=False, field_names = ("x", "y", "z")))  

            cloud_name = "cloud_{}.pcd".format(coutner_str)
            filename_cloud = os.path.join(node.getParameter("OUTPUT_PATH"), cloud_name)
            out_pcd = o3d.geometry.PointCloud()    
            out_pcd.points = o3d.utility.Vector3dVector(cloud_points)
            o3d.io.write_point_cloud(filename_cloud,out_pcd)
            
            if tf_enabled:
                f = open(filenametf, "a")
                line_to_save = np.insert(transformations.KDLtoNumpyVector(tf_target), 0, [frame_save_counter], axis=1)
                np.savetxt(f, line_to_save)
                f.close()
                print("pose added!")
                #np.savetxt(filenametf, transformations.KDLtoNumpyVector(tf_target))

            Logger.log("Saved frames:{}".format(coutner_str))
        if c == KEY_Q:
            print("EXIT")
            sys.exit(0)



if __name__ == "__main__":
    rospy.init_node('camera_server')
    s = rospy.Service('take_frame_service', takeFrame, call_camera)
    rospy.spin()







def cameraCallback(frame):

    global frame_save_counter
    filenametf = os.path.join(node.getParameter("OUTPUT_PATH"), "robot_poses.txt")

   



#⬢⬢⬢⬢⬢➤ Camera Msgs Callback
camera.registerUserCallabck(cameraCallback)

while node.isActive():
    node.tick()
