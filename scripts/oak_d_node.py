#!/usr/bin/env python3

import time
import cv2
import depthai as dai

import numpy as np
from numpy import array
from math import pi, tan
from copy import deepcopy

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from luxonis_support.srv import SetUint8, SetUint8Request, SetUint8Response

from typing import List, Tuple

#globals, because im lazy today
inspection_trigger=False
insp_lensPos = 150 # good for card inspection approx 20cm from card
inspection_focus_time = -1
focus_state = "nav"
do_set_focus = False
mono_iso = 100
new_mono_iso = None
mono_exposure = 2000
new_mono_exposure = None
mono_laser_power = 100
new_mono_laser_power = None

def inspection_capture_callback(req: TriggerRequest):
    global inspection_trigger
    res = TriggerResponse()
    inspection_trigger = True
    res.success = True
    return res

def inspection_focus_callback(req: TriggerRequest):
    global focus_state, do_set_focus, insp_lensPos
    res = TriggerResponse()
    if focus_state != "inspection":
        focus_state = "inspection"
        do_set_focus = True
    res.message = f"setting focus state to inspection. corresponding lens position [0-255] is: {insp_lensPos}"
    rospy.loginfo(f"{time.time()} {res.message}")
    res.success = True
    return res

def nav_focus_callback(req:TriggerRequest):
    global focus_state, do_set_focus
    res = TriggerResponse()
    if focus_state != "nav":
        focus_state = "nav"
        do_set_focus = True
    res.message = f"setting focus state to nav" # TODO. corresponding lens position [0-255] is: {nav_lensPos}"
    rospy.loginfo(f"{time.time()} {res.message}")
    res.success = True
    return res

def set_insp_lens_pos_callback(req:SetUint8Request):
    global insp_lensPos, do_set_focus
    res = SetUint8Response()
    new_lens_pos = req.val_int
    if new_lens_pos < 0 or new_lens_pos > 255:
        res.message = "invalid setting for lens position. acceptable values are [0-255]"
        rospy.logwarn(f"{time.time()} {res.message}")
        res.success = False
        return res
    
    insp_lensPos = new_lens_pos
    # set focus again if we're already in inspection mode
    if focus_state == "inspection":
        do_set_focus = True
    res.message = f"inspection lens position updated to {new_lens_pos} and will be applied when a request to set the focus state to inspection"
    rospy.loginfo(f"{time.time()} {res.message}")
    res.success = True
    return res

def set_mono_iso(req:SetUint8Request):
    global new_mono_iso    
    res = SetUint8Response()

    new_mono_iso = req.val_int*10
    res.success = True
    return res

def set_mono_exposure(req:SetUint8Request):
    global new_mono_exposure    
    res = SetUint8Response()

    new_mono_exposure = req.val_int*10
    res.success = True
    return res

def set_mono_laser(req:SetUint8Request):
    global new_mono_laser_power    
    res = SetUint8Response()

    new_mono_laser_power = req.val_int*10
    res.success = True
    return res


def populate_caminfo_from_dai_intrinsics(width:int, height:int, intrinsics: List[List[float]]) -> CameraInfo:
    # https://docs.luxonis.com/projects/api/en/latest/references/python/#depthai.CalibrationHandler.getCameraIntrinsics
    # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
    info_data = CameraInfo()
    info_data.height = int(height)
    info_data.width = int(width)
    info_data.K = array(intrinsics).flatten()
    info_data.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    info_data.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    k=info_data.K
    info_data.P = [k[0],k[1],k[2],0, k[3],k[4],k[5], 0, k[6],k[7],k[8],0] 

    return(info_data)

def adjust_intrinsics(sensor_res:Tuple[int, int], initial_crop:Tuple[int, int], output_resolution:Tuple[int, int], 
                      intrinsics: List[List[float]], keepAspectRatio:bool = True) -> List[List[float]]:
    """
    sensor_res: resolution of sensor for which the intrinsics will be supplied
    initial_crop: cropping of sensor res before scaling
    output resolution: scaling of sensor and initial crop
    intrinsics: 3x3 [[f_x, 0, c_x],[0, f_y, c_y], [0,0,1]]
    keepaspectratio: if true then the smallest scaling factor will be applied from (width, height) and the other axis is assumed to be scaled and cropped to output resolution
    x is width, y is height. 
    """
    intrinsics = deepcopy(intrinsics)
    # adjust optical centre due to initial crop

    # print(intrinsics)   # debugging

    if initial_crop:
        # TODO review this for off by one errors.
        xoff = (sensor_res[0] - initial_crop[0])/2
        yoff = (sensor_res[1] - initial_crop[1])/2
        intrinsics[0][2]-= xoff
        intrinsics[1][2]-= yoff
        #update sensor res for subsequent math
        sensor_res=initial_crop

    # print(intrinsics)   # debugging

    # scale centre and f
    if output_resolution:
        scalex = output_resolution[0]/sensor_res[0]
        scaley = output_resolution[1]/sensor_res[1]
        cropaxis=0
        cropoffset=0.0
        if keepAspectRatio:
            if scalex >= scaley:
                scaley=scalex
                cropaxis = 1
            if scaley > scalex:
                scalex=scaley
                cropaxis = 0
            
            # calculate adjustment to c_(x||y) to be applied after scaling
            cropoffset = (sensor_res[cropaxis]*scalex - output_resolution[cropaxis])/2.0

        intrinsics[0][0]*=scalex
        intrinsics[0][2]*=scalex
        intrinsics[1][1]*=scaley
        intrinsics[1][2]*=scaley

        # TODO review this for off by one errors.
        intrinsics[cropaxis][2]-=cropoffset
    
    # print(intrinsics)   # debugging
    # print("===================")

    return intrinsics


# depthai link: link the output of the parent to the input of the argument : link_output(thing.input)
# https://docs.luxonis.com/projects/api/en/latest/components/nodes/color_camera/
# essentially the video engine (which feeds preview) is limited to 3840*2160, so to get aligned depth and preview you need to be this or less, 
# so we use the_4_k which is exactly 3840*2160 (1.777:1)

# Create pipeline
pipeline = dai.Pipeline()

#### inspection cam config
inspection_cam = pipeline.create(dai.node.ColorCamera)
inspection_cam.setBoardSocket(dai.CameraBoardSocket.RGB)
inspection_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_4_K) # full res THE_12_MP 4056*3040, THE_4K is 3840*2160 

# isp processing is camera level, ruins the full res.
# inspection still encoder
# https://docs.luxonis.com/projects/api/en/latest/components/nodes/video_encoder/
videoEnc = pipeline.create(dai.node.VideoEncoder)
videoEnc.setDefaultProfilePreset(1, dai.VideoEncoderProperties.Profile.MJPEG)
inspection_cam.still.link(videoEnc.input)
inspection_cam_xlink = pipeline.create(dai.node.XLinkOut)
inspection_cam_xlink.setStreamName("inspection_stream")
videoEnc.bitstream.link(inspection_cam_xlink.input)


#### control config
inspection_cam_ctrl_xlink = pipeline.create(dai.node.XLinkIn)
inspection_cam_ctrl_xlink.setStreamName("inspection_control")
inspection_cam_ctrl_xlink.out.link(inspection_cam.inputControl)

#### realsense emulation config, uses inspection cam preview
## rgb
inspection_cam.setPreviewSize(640,480)
inspection_cam.setPreviewKeepAspectRatio(True)
nav_cam_xlink = pipeline.create(dai.node.XLinkOut)
nav_cam_xlink.setStreamName("nav_stream")
inspection_cam.preview.link(nav_cam_xlink.input)

## stereodepth
# lasers set below after device has been linked.

# Define sources and outputs
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
depth_cam = pipeline.create(dai.node.StereoDepth)
depth_cam_xlink = pipeline.create(dai.node.XLinkOut)
depth_cam_xlink.setStreamName("depth_stream")

"""
monoLeft_cam_xlink = pipeline.create(dai.node.XLinkOut)
monoLeft_cam_xlink.setStreamName("monoLeft_stream")
monoLeft.out.link(monoLeft_cam_xlink.input)
monoRight_cam_xlink = pipeline.create(dai.node.XLinkOut)
monoRight_cam_xlink.setStreamName("monoRight_stream")
monoRight.out.link(monoRight_cam_xlink.input)
"""

# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

############ SETTING ONE OF THE MONO CAMS CONTROL SETTINGS APPLIES IT TO BOTH ############
# monoLeft_cam_ctrl_xlink = pipeline.create(dai.node.XLinkIn)
# monoLeft_cam_ctrl_xlink.setStreamName("monoLeft_control")
# monoLeft_cam_ctrl_xlink.out.link(monoLeft.inputControl)

monoRight_cam_ctrl_xlink = pipeline.create(dai.node.XLinkIn)
monoRight_cam_ctrl_xlink.setStreamName("monoRight_control")
monoRight_cam_ctrl_xlink.out.link(monoRight.inputControl)

# Create a node that will produce the depth map
# https://docs.luxonis.com/projects/api/en/latest/components/nodes/stereo_depth/
# https://docs.luxonis.com/projects/api/en/latest/components/messages/stereo_depth_config/
depth_cam.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
depth_cam.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth_cam.setLeftRightCheck(True)       # Better handling for occlusions:
depth_cam.setExtendedDisparity(True)    # Closer-in minimum depth, disparity range is doubled (from 95 to 190):
depth_cam.setSubpixel(False)            # Better accuracy for longer distance, fractional disparity 32-levels:
depth_cam.setOutputSize(640, 480)
depth_cam.setDepthAlign(dai.CameraBoardSocket.RGB)


# Linking
monoLeft.out.link(depth_cam.left)
monoRight.out.link(depth_cam.right)
depth_cam.depth.link(depth_cam_xlink.input)


# ros startup
rospy.init_node("camera", disable_signals=False)
rgb_pub = rospy.Publisher("~color/image_raw", Image, queue_size=1)
rgb_data = Image()
rgb_info_pub = rospy.Publisher("~color/camera_info", CameraInfo, queue_size=1)
rgb_info_data = CameraInfo()
dep_pub = rospy.Publisher("~aligned_depth_to_color/image_raw", Image, queue_size=1)
dep_data = Image()
dep_info_pub = rospy.Publisher("~aligned_depth_to_color/camera_info", CameraInfo, queue_size=1)
dep_info_data = CameraInfo()
ins_pub = rospy.Publisher("~inspection/image_raw", Image, queue_size=1)
ins_data = Image()
ins_info_pub = rospy.Publisher("~inspection/camera_info", CameraInfo, queue_size=1)
ins_info_data = CameraInfo()
ins_trigger_srv = rospy.Service("~inspection/capture", Trigger, inspection_capture_callback)
ins_focus_srv = rospy.Service("~inspection/focus", Trigger, inspection_focus_callback)
ins_lens_pos_set_srv = rospy.Service("~inspection/set_inspection_lens_position", SetUint8, set_insp_lens_pos_callback)
nav_focus_srv = rospy.Service("~color/focus", Trigger, nav_focus_callback)

mono_exposure_srv = rospy.Service("~mono/set_exposure", SetUint8, set_mono_exposure)
mono_iso_srv = rospy.Service("~mono/set_iso", SetUint8, set_mono_iso)
mono_laser_srv = rospy.Service("~mono/set_laser", SetUint8, set_mono_laser)

bridge = CvBridge()

rate = rospy.Rate(30)
rospy.core._in_shutdown
rospy.core._shutdown_flag

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    device.setIrLaserDotProjectorBrightness(mono_laser_power)
    device.setLogOutputLevel(dai.LogLevel.TRACE)
    # get calibration info
    calibData = device.readCalibration()
    intrinsics = calibData.getCameraIntrinsics(dai.CameraBoardSocket.RGB)
    # rgbd_intrinsics = adjust_intrinsics((4056,3040), (3840,2160), (640,480), intrinsics)
    rgbd_intrinsics = adjust_intrinsics((3840,2160),None, (640,480), intrinsics)
    rgb_info_data = populate_caminfo_from_dai_intrinsics(640,480, rgbd_intrinsics)
    inspection_intrinsics = adjust_intrinsics((3840,2160),None, None, intrinsics)
    ins_info_data = populate_caminfo_from_dai_intrinsics(3840,2160, inspection_intrinsics)


    # Output queue will be used to get the rgb frames from the output defined above
    q_nav_stream = device.getOutputQueue(name="nav_stream", maxSize=1, blocking=False)
    q_depth_stream = device.getOutputQueue(name="depth_stream", maxSize=1, blocking=False)
    # monoLeft_stream = device.getOutputQueue(name="monoLeft_stream", maxSize=1, blocking=False)
    # monoRight_stream = device.getOutputQueue(name="monoRight_stream", maxSize=1, blocking=False)
    
    ################### YOU ONLY NEED ONE MONO CONTROL INTERFACE ###################
    # monoLeft_control = device.getInputQueue(name="monoLeft_control")
    monoRight_control = device.getInputQueue(name="monoRight_control")
    
    q_inspection_stream = device.getOutputQueue(name="inspection_stream", maxSize=1, blocking=True)
    q_inspection_control = device.getInputQueue(name="inspection_control")

    nav_lensPos = calibData.getLensPosition(dai.CameraBoardSocket.RGB)

    ctrl = dai.CameraControl()
    ctrl.setManualFocus(nav_lensPos)
    q_inspection_control.send(ctrl)
    
    ctrl2 = dai.CameraControl()
    ctrl2.setManualExposure(mono_exposure, mono_iso)
    monoRight_control.send(ctrl2)

    rgbframe = None
    depframe = None
    insframe = None
    while not (rospy.is_shutdown() or rospy.core.is_shutdown_requested()): #rospy.core._shutdown_flag:
        # set everything back to None to avoid lingering images
        rgbframe = None
        depframe = None
        insframe = None
        
        inRgb = q_nav_stream.tryGet()  # Non-blocking call, will return a new data that has arrived or None otherwise
        if inRgb is not None:
            rgbframe = inRgb.getCvFrame()

        inD = q_depth_stream.tryGet()  # Non-blocking call, will return a new data that has arrived or None otherwise
        if inD is not None:
            depframe = array(inD.getCvFrame())
            #TODO: if depth values for "invalid" pixels are not large numbers then replace them with large numbers

        if q_inspection_stream.has():
            insframe = cv2.imdecode(q_inspection_stream.get().getData(), cv2.IMREAD_UNCHANGED)

            rospy.loginfo(f"{time.time()} got an inspection frame ")

        # MONO params
        if (new_mono_exposure is not None) and (new_mono_exposure!=mono_exposure):
            mono_exposure = new_mono_exposure
            new_mono_exposure = None
            ctrl2.setManualExposure(mono_exposure, mono_iso)
            monoRight_control.send(ctrl2)

        if (new_mono_iso is not None) and (new_mono_iso!=mono_iso):
            mono_iso = new_mono_iso
            new_mono_iso = None
            ctrl2.setManualExposure(mono_exposure, mono_iso)
            monoRight_control.send(ctrl2)

        # LASER
        if (new_mono_laser_power is not None) and (new_mono_laser_power!=mono_laser_power):
            mono_laser_power = new_mono_laser_power
            new_mono_laser_power = None
            device.setIrLaserDotProjectorBrightness(mono_laser_power)

        # Focus
        if do_set_focus:
            ctrl = dai.CameraControl()
            if focus_state == "nav":
                ctrl.setManualFocus(nav_lensPos)
            if focus_state == "inspection":
                ctrl.setManualFocus(insp_lensPos)
                inspection_focus_time = time.time()
            q_inspection_control.send(ctrl)
            
            do_set_focus = False

        if (inspection_trigger) and (focus_state == "inspection") and (time.time() - inspection_focus_time > 0.4) :
            inspection_trigger = False
            ctrl = dai.CameraControl()
            ctrl.setCaptureStill(True)
            q_inspection_control.send(ctrl)
            rospy.loginfo(f"{time.time()} Sent 'still' event to the camera!")

        if rgbframe is not None:
            rgb_data = bridge.cv2_to_imgmsg(rgbframe, "bgr8")
            #TODO frame and proper time in header.
            rgb_data.header.frame_id = "camera_color_optical_frame"
            rgb_data.header.stamp = rospy.Time.now()
            rgb_info_data.header.stamp = rgb_data.header.stamp
            rgb_pub.publish(rgb_data)
            rgb_info_pub.publish(rgb_info_data)


        if depframe is not None:
            dep_data = bridge.cv2_to_imgmsg(depframe, "passthrough")
            #TODO frame and proper time in header.
            dep_data.header.frame_id = "camera_color_optical_frame"
            dep_data.header.stamp = rospy.Time.now()
            dep_info_data.header.stamp = dep_data.header.stamp
            dep_pub.publish(dep_data)
            dep_info_pub.publish(rgb_info_data)



        if insframe is not None:
            ins_data = bridge.cv2_to_imgmsg(insframe, "bgr8")
            #TODO frame and proper time in header.
            ins_data.header.frame_id = "camera_color_optical_frame"
            ins_data.header.stamp = rospy.Time.now()
            ins_info_data.header.stamp = ins_data.header.stamp
            ins_pub.publish(ins_data)
            ins_info_pub.publish(ins_info_data)


        rate.sleep()

"""
https://docs.luxonis.com/projects/api/en/latest/components/messages/camera_control/
can set focus
trigger still capture

"""