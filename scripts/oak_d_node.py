#!/usr/bin/env python3

import time
import cv2
import depthai as dai

import numpy as np
from numpy import array
from math import pi, tan

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

#globals, because im lazy today
inspection_trigger=False

def inspection_capture_callback(req: TriggerRequest):
    global inspection_trigger
    res = TriggerResponse()
    inspection_trigger = True
    res.success = True
    return res

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
#isp processing is camera level, ruins the full res.
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
# inspection_cam.setVideoSize does a centre crop on the stream, doesnt resize.
# cannot mjpeg the preview stream booo
# inspection_cam.video.link(videoEnc2.input)
# videoEnc2 = pipeline.create(dai.node.VideoEncoder)
# videoEnc2.setDefaultProfilePreset(1, dai.VideoEncoderProperties.Profile.MJPEG)
# videoEnc2.bitstream.link(nav_cam_xlink.input) # linked off the inspection cam, without processing
inspection_cam.preview.link(nav_cam_xlink.input)

## stereodepth
# lasers set beloow after device has been linked.
extended_disparity = True # Closer-in minimum depth, disparity range is doubled (from 95 to 190):
subpixel = False # Better accuracy for longer distance, fractional disparity 32-levels:
lr_check = True # Better handling for occlusions:

# Define sources and outputs
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
depth_cam = pipeline.create(dai.node.StereoDepth)
depth_cam_xlink = pipeline.create(dai.node.XLinkOut)

depth_cam_xlink.setStreamName("disp_stream")

# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

# Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
depth_cam.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
depth_cam.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth_cam.setLeftRightCheck(lr_check)
depth_cam.setExtendedDisparity(extended_disparity)
depth_cam.setSubpixel(subpixel)
depth_cam.setOutputSize(640, 480)
depth_cam.setDepthAlign(dai.CameraBoardSocket.RGB)

# Linking
monoLeft.out.link(depth_cam.left)
monoRight.out.link(depth_cam.right)
# TODO make below disparity into depth and remove depth calc from loop further down.
# TURD MONKEY
depth_cam.disparity.link(depth_cam_xlink.input)


# ros startup
rospy.init_node("camera")
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

bridge = CvBridge()

rate = rospy.Rate(30)


# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    device.setIrLaserDotProjectorBrightness(1000)
    device.setLogOutputLevel(dai.LogLevel.TRACE)
    # get calibration info
    calibData = device.readCalibration()
    rgbd_intrinsics = calibData.getCameraIntrinsics(dai.CameraBoardSocket.RGB, resizeWidth=640, resizeHeight=480)
    rgb_info_data.height = 480
    rgb_info_data.width = 640
    rgb_info_data.K = array(rgbd_intrinsics).flatten()
    rgb_info_data.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    rgb_info_data.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    k=rgb_info_data.K
    rgb_info_data.P = [k[0],k[1],k[2],0, k[3],k[4],k[5], 0, k[6],k[7],k[8],0] 
    inspection_intrinsics = calibData.getCameraIntrinsics(dai.CameraBoardSocket.RGB)
    ins_info_data.K = array(inspection_intrinsics).flatten()
    # distortion = calibData.getDistortionCoefficients(dai.CameraBoardSocket.RGB)
    # focal_length_in_pixels = image_width_in_pixels * 0.5 / tan(HFOV * 0.5 * PI/180)
    HFOV = calibData.getFov(dai.CameraBoardSocket.LEFT)
    bld = calibData.getBaselineDistance(dai.CameraBoardSocket.LEFT, dai.CameraBoardSocket.RIGHT) * 10 #mm, ros expects depth images in mm
    flpx = 640*0.5/tan(HFOV * 0.5 * pi/180)


    # Output queue will be used to get the rgb frames from the output defined above
    q_nav_stream = device.getOutputQueue(name="nav_stream", maxSize=1, blocking=False)
    q_depth_stream = device.getOutputQueue(name="disp_stream", maxSize=1, blocking=False)
    q_inspection_stream = device.getOutputQueue(name="inspection_stream", maxSize=1, blocking=True)
    q_inspection_control = device.getInputQueue(name="inspection_control")

    rgbframe = None
    depframe = None
    insframe = None

    while True:
        inRgb = q_nav_stream.tryGet()  # Non-blocking call, will return a new data that has arrived or None otherwise
        if inRgb is not None:
            rgbframe = inRgb.getCvFrame()
            # frame = cv2.imdecode(inRgb.getData(), cv2.IMREAD_UNCHANGED)
            # cv2.imshow("nav_stream", rgbframe)

        inD = q_depth_stream.tryGet()  # Non-blocking call, will return a new data that has arrived or None otherwise
        if inD is not None:
            depframe = array(inD.getCvFrame())
            # frame = cv2.imdecode(inRgb.getData(), cv2.IMREAD_UNCHANGED)
            #depth = 441.25 * 7.5 / 50 = 66.19
            depframe[depframe==0] = 1   # far far away, like 30 meters
            depframe = array(flpx * bld / depframe, dtype=np.uint16)
            # cv2.imshow("depth_stream", depframe)

        if q_inspection_stream.has():
            insframe = cv2.imdecode(q_inspection_stream.get().getData(), cv2.IMREAD_UNCHANGED)
            # _ = q_inspection_stream.get().getData()
            # insframe = cv2.pyrDown(insframe)
            # insframe = cv2.pyrDown(insframe)
            # cv2.imshow("highres_snap", insframe)

            rospy.loginfo(f"{time.time()} got an inspection frame ")

        # key = cv2.waitKey(1)
        # if key == ord('q'):
        #     break
        # elif key == ord('c'):
        if inspection_trigger:
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
            rgb_pub.publish(rgb_data)

        if depframe is not None:
            dep_data = bridge.cv2_to_imgmsg(depframe, "passthrough")
            #TODO frame and proper time in header.
            dep_data.header.frame_id = "camera_color_optical_frame"
            dep_data.header.stamp = rospy.Time.now()
            dep_pub.publish(dep_data)


        if insframe is not None:
            ins_data = bridge.cv2_to_imgmsg(insframe, "bgr8")
            #TODO frame and proper time in header.
            ins_data.header.frame_id = "camera_color_optical_frame"
            ins_data.header.stamp = rospy.Time.now()
            ins_pub.publish(ins_data)


        rgb_info_data.header.stamp = rgb_data.header.stamp
        rgb_info_pub.publish(rgb_info_data)
        dep_info_data.header.stamp = dep_data.header.stamp
        dep_info_pub.publish(rgb_info_data)



        rate.sleep()

"""
https://docs.luxonis.com/projects/api/en/latest/components/messages/camera_control/
can set focus
trigger still capture

"""