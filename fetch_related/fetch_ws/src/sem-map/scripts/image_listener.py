#!/usr/bin/env python
"""ROS image listener"""

import sys
import threading
import numpy as np
import cv2

import rospy
import tf
import tf2_ros
import message_filters
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo, PointCloud
from cv_bridge import CvBridge, CvBridgeError
from ros_utils import ros_qt_to_rt, ros_pose_to_rt
import ros_numpy


from grasp_utils import compute_xyz

lock = threading.Lock()


class ImageListener:

    def __init__(self, camera='Fetch'):

        self.cv_bridge = CvBridge()

        self.im = None
        self.depth = None
        self.rgb_frame_id = None
        self.rgb_frame_stamp = None

        # initialize a node before instantiating the class
        self.tf_listener = tf.TransformListener()   

        if camera == 'Fetch':
            self.base_frame = 'base_link'
            self.world_frame = 'odom'
            rgb_sub = message_filters.Subscriber('/head_camera/rgb/image_raw', Image, queue_size=10)
            depth_sub = message_filters.Subscriber('/head_camera/depth_registered/image_raw', Image, queue_size=10)
            msg = rospy.wait_for_message('/head_camera/rgb/camera_info', CameraInfo)
            self.camera_frame = 'head_camera_rgb_optical_frame'
            self.target_frame_for_cam = self.base_frame
            self.target_frame_for_base = self.world_frame        
        elif camera == 'Realsense':
            # use RealSense camera
            self.base_frame = 'measured/base_link'
            rgb_sub = message_filters.Subscriber('/camera/color/image_raw', Image, queue_size=10)
            depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, queue_size=10)
            msg = rospy.wait_for_message('/camera/color/camera_info', CameraInfo)
            self.camera_frame = 'measured/camera_color_optical_frame'
            self.target_frame = self.base_frame
        elif camera == 'Azure':
            self.base_frame = 'measured/base_link'
            rgb_sub = message_filters.Subscriber('/k4a/rgb/image_raw', Image, queue_size=10)
            depth_sub = message_filters.Subscriber('/k4a/depth_to_rgb/image_raw', Image, queue_size=10)
            msg = rospy.wait_for_message('/k4a/rgb/camera_info', CameraInfo)
            self.camera_frame = 'rgb_camera_link'
            self.target_frame = self.base_frame
        else:
            print('camera %s is not supported in image listener' % camera)
            sys.exit(1)

        # update camera intrinsics
        intrinsics = np.array(msg.K).reshape(3, 3)
        self.intrinsics = intrinsics
        self.fx = intrinsics[0, 0]
        self.fy = intrinsics[1, 1]
        self.px = intrinsics[0, 2]
        self.py = intrinsics[1, 2]
        print(intrinsics)

        queue_size = 1
        slop_seconds = 0.1
        ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size, slop_seconds)
        ts.registerCallback(self.callback_rgbd)


    def callback_rgbd(self, rgb, depth):
    
        # get camera pose in base
        try:
             trans, rot = self.tf_listener.lookupTransform(self.base_frame, self.camera_frame, rospy.Time(0))
             RT_camera = ros_qt_to_rt(rot, trans)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Update failed... " + str(e))
            RT_camera = None 

        try:
             trans, rot = self.tf_listener.lookupTransform(self.world_frame, self.base_frame, rospy.Time(0))
             RT_base = ros_qt_to_rt(rot, trans)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Update failed... " + str(e))
            RT_base = None             

        if depth.encoding == '32FC1':
            # if your cv2_bridge is not broken, uncomment the below line and comment its next one
            # depth_cv = self.cv_bridge.imgmsg_to_cv2(depth)
            depth_cv = ros_numpy.numpify(depth)
        elif depth.encoding == '16UC1':
            # if your cv2_bridge is not broken, uncomment the below two lines and comment its next two lines
            # depth_cv = self.cv_bridge.imgmsg_to_cv2(depth).copy().astype(np.float32)
            # depth_cv /= 1000.0
            depth_cv = ros_numpy.numpify(depth).copy().astype(np.float32)
            depth_cv = depth_cv/1000
        else:
            rospy.logerr_throttle(
                1, 'Unsupported depth type. Expected 16UC1 or 32FC1, got {}'.format(
                    depth.encoding))
            return

        # if your cv2_bridge is not broken, uncomment the below line and comment its next one
        # im = self.cv_bridge.imgmsg_to_cv2(rgb, 'bgr8')
        im = ros_numpy.numpify(rgb)
        with lock:
            self.im = im.copy()
            self.depth = depth_cv.copy()
            self.rgb_frame_id = rgb.header.frame_id
            self.rgb_frame_stamp = rgb.header.stamp
            self.height = depth_cv.shape[0]
            self.width = depth_cv.shape[1]
            self.RT_camera = RT_camera
            self.RT_base = RT_base


    def get_data(self):

        with lock:
            if self.im is None:
                return None, None, None, None, None, self.intrinsics
            im_color = self.im.copy()
            depth_image = self.depth.copy()
            rgb_frame_id = self.rgb_frame_id
            rgb_frame_stamp = self.rgb_frame_stamp
            RT_camera = self.RT_camera.copy()
            RT_base = self.RT_base.copy()

        xyz_image = compute_xyz(depth_image, self.fx, self.fy, self.px, self.py, self.height, self.width)
        xyz_array = xyz_image.reshape((-1, 3))
        xyz_base = np.matmul(RT_camera[:3, :3], xyz_array.T) + RT_camera[:3, 3].reshape(3, 1)
        xyz_base = xyz_base.T.reshape((self.height, self.width, 3))
        #TODO: convert from base frame to world frame. check dimensions
        xyz_world =  np.matmul(RT_base[:3, :3], xyz_base.T) + RT_base[:3, 3].reshape(3, 1)
        xyz_base = xyz_world.T.reshape((self.height, self.width, 3))
        return im_color, depth_image, xyz_image, xyz_base, xyz_world, RT_camera, RT_base, self.intrinsics


if __name__ == '__main__':
    # test_basic_img()
    # test_point_publisher()
    rospy.init_node("test_subscription")
    il = ImageListener()
    import time
    time.sleep(2)
    print(il.get_data())
