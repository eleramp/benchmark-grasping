#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This file contains the code of the benchmark service client.
Its features are:

1. connect to the ros camera topics to read rgb, depth, point cloud, camera parameters
2. send a request to a grasp planning algorithm of type graspPlanner<algorithm>.srv
3. connect with robot to send cartesian grasp pose commands
4. assess if the grasp was successful or not

"""

import rospy
import warnings
import message_filters
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import Transform
from std_msgs.msg import Bool
import tf2_ros

from benchmark_grasping_ros.srv import *

from dexnet_ros.msg import GQCNNGrasp

import numpy as np


NEW_MSG = {
"new_data": False,
"data": {},
}

# TODO: find a way to define the grasp service depending on the type of grasp planner
GRASP_PLANNER_SRV = {
    'GraspPlanner': GraspPlanner,
    'GraspPlannerCloud': GraspPlannerCloud,
    }


class BenchmarkGraspingManager(object):
    def __init__(self, grasp_planner_service_name, grasp_planner_service, user_cmd_service_name, verbose=False):

        self._verbose = verbose

        # --- new grasp command service --- #
        self._new_grasp_srv = rospy.Service(user_cmd_service_name, UserCmd, self.user_cmd)

        # --- grasp planner service --- #
        self._grasp_planner_srv = GRASP_PLANNER_SRV[grasp_planner_service]

        rospy.wait_for_service(grasp_planner_service_name, timeout=60.0)
        self._grasp_planner = rospy.ServiceProxy(grasp_planner_service_name, self._grasp_planner_srv)
        rospy.loginfo("BenchmarkGraspingManager: Connected with service {}".format(grasp_planner_service_name))

        # --- subscribers to camera topics --- #
        self._cam_info_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo)
        self._rgb_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        self._depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        self._pc_sub = message_filters.Subscriber('/camera/depth/color/points', PointCloud2)

        # synchronizer
        self._tss = message_filters.ApproximateTimeSynchronizer([self._cam_info_sub, self._rgb_sub, self._depth_sub, self._pc_sub],
                                                                queue_size=1, slop=0.5)

        self._tss.registerCallback(self._camera_data_callback)

        self._tfBuffer = tf2_ros.buffer.Buffer()
        listener = tf2_ros.transform_listener.TransformListener(self._tfBuffer)

        self._seg_sub = rospy.Subscriber('rgb/image_seg', Image, self.seg_img_callback, queue_size=10)
        self._cam_pose_sub = rospy.Subscriber('/camera/camera_pose', Transform, self.camera_pose_callback, queue_size=10)

        # --- camera messages --- #
        self._cam_info_msg = None
        self._rgb_msg = None
        self._depth_msg = None
        self._pc_msg = None

        self._seg_msg = NEW_MSG
        self._cam_pose_msg = NEW_MSG

        self._new_camera_data = False

    # ---------------------- #
    # Grasp planning handler #
    # ---------------------- #
    def user_cmd(self, req):
        """New grasp request handler

        Parameters
        ---------
        req: :obj:`ROS ServiceRequest`
            ROS `ServiceRequest` for grasp planner service.
        """
        if self._verbose:
            print("Received new command from user...")

        if req.cmd.data != "grasp":
            warnings.warn("Wrong input {}{}. "
            "The only supported command is 'grasp', to compute and execute a new grasp".format(type(req.cmd), req.cmd))
            return Bool(False)

        # --- get images --- #
        if self._verbose:
            print("... waiting for images ...")

        count = 0
        while not self._new_camera_data and count < 100:
            count += 1

        if count >= 100:
            print("...no images received")
            return Bool(False)

        self._new_camera_data = False

        # --- create srv request --- #
        # GraspPlanner
        if self._grasp_planner_srv is GraspPlanner:

            planner_req = GraspPlannerRequest()

            planner_req.color_image = self._rgb_msg
            planner_req.depth_image = self._depth_msg
            planner_req.camera_info = self._cam_info_msg

        # or GraspPlannerCloud
        elif self._grasp_planner_srv is GraspPlannerCloud:

            planner_req = GraspPlannerCloudRequest()

            # define cloud
            planner_req.cloud = self._pc_msg

            # define camera view point
            try:
                camera_pose_tf = self._tfBuffer.lookup_transform(self._pc_msg.header.frame_id, 'world', rospy.Time())
                camera_pose = camera_pose_tf.transform

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                warnings.warn("tf listener could not get camera pose. Are you publishing camera poses on tf?")

                camera_pose = geometry_msgs.msg.Transform()
                camera_pose.rotation.w = 1.0


            camera_pose_msg = geometry_msgs.msg.Pose()

            camera_pose_msg.position.x = camera_pose.translation.x
            camera_pose_msg.position.y = camera_pose.translation.y
            camera_pose_msg.position.z = camera_pose.translation.z
            camera_pose_msg.orientation = camera_pose.rotation

            planner_req.view_point = camera_pose_msg

        if self._verbose:
            print("... send request to server ...")

        try:

            reply = self._grasp_planner(planner_req)

            print("Service {} reply is: \n{}" .format(self._grasp_planner.resolved_name, reply))

            return Bool(True)

        except rospy.ServiceException as e:
            print("Service {} call failed: {}" .format(self._grasp_planner.resolved_name, e))

            return Bool(False)

    # ------------------- #
    # Camera data handler #
    # ------------------- #
    def _camera_data_callback(self, cam_info, rgb, depth, pc):

        # rospy.loginfo("New data from camera!")
        self._cam_info_msg = cam_info
        self._rgb_msg = rgb
        self._depth_msg = depth
        self._pc_msg = pc

        self._new_camera_data = True


    def camera_pose_callback(self, data):
        if self._verbose:
            print("Got camera pose...")

        self._cam_pose_msg['data'] = data
        self._cam_pose_msg['new_data'] = True

    def seg_img_callback(self, data):
        if self._verbose:
            print("Got segmentation image...")

        self._seg_msg['data'] = data
        self._seg_msg['new_data'] = True



if __name__ == "__main__":
    # Init node
    rospy.init_node('benchmark_grasping_manager')

    # Get rosparam config
    grasp_planner_service_name = rospy.get_param("~grasp_planner_service_name")
    grasp_planner_service = rospy.get_param("~grasp_planner_service")
    new_grasp_service_name = rospy.get_param("~user_cmd_service_name")

    # Instantiate benchmark client class
    bench_manager = BenchmarkGraspingManager(grasp_planner_service_name, grasp_planner_service, new_grasp_service_name, verbose=True)

    # Spin forever.
    rospy.spin()