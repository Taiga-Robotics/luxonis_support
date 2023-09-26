#!/usr/bin/env python3

import time
import cv2
import depthai as dai

import numpy as np
from numpy import array, mean, median
from math import pi, tan
from copy import deepcopy

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from typing import List, Tuple
from iris_ml_ros.utilities import CameraParameters
from iris_ml_ros.interfaces import CameraInterfaceRGBD
from iris_ml_ros.interfaces import tf_interfaces
from iris_ml_ros.utilities import pointcloud

# ros startup
rospy.init_node("ruler", disable_signals=False)

camera = CameraInterfaceRGBD(img_topic="/camera/color/image_raw", dep_topic="/camera/aligned_depth_to_color/image_raw", format_use='rgb')

bridge = CvBridge()

rate = rospy.Rate(10)

while not (rospy.is_shutdown() or rospy.core.is_shutdown_requested()): #rospy.core._shutdown_flag:
    s, rgb, dep, _ = camera.poll_for_image()
    if s:
        pnt, nrm = pointcloud.get_pick_point(depth=dep,row=240,col=320,win_size=20,
                                             median_filt=False,optical_z_out=True,z_cutoff=1.9,min_pcd_count=10,publish_debug_pcd=True)
        if (pnt is not None) and (nrm is not None):
            rospy.loginfo(f"\tPNT: {pnt[0]:.3f}, {pnt[1]:.3f}, {pnt[2]:.3f}\n\t\t\t\tNRM: {nrm[0]:.3f}, {nrm[1]:.3f}, {nrm[2]:.3f}")


    rate.sleep()
