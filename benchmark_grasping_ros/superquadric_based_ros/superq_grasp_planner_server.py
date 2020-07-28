#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Copyright ©2017. The Regents of the University of California (Regents).
All Rights Reserved. Permission to use, copy, modify, and distribute this
software and its documentation for educational, research, and not-for-profit
purposes, without fee and without a signed licensing agreement, is hereby
granted, provided that the above copyright notice, this paragraph and the
following two paragraphs appear in all copies, modifications, and
distributions. Contact The Office of Technology Licensing, UC Berkeley, 2150
Shattuck Avenue, Suite 510, Berkeley, CA 94720-1620, (510) 643-7201,
otl@berkeley.edu,
http://ipira.berkeley.edu/industry-info for commercial licensing opportunities.

IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL,
INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF
THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF REGENTS HAS BEEN
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY, PROVIDED
HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

ROS Server for planning GQ-CNN grasps.

Author
-----
Vishal Satish & Jeff Mahler
"""
import json
import math
import os
import time

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy
import ros_numpy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from benchmark_grasping_ros.srv import GraspPlannerCloud
from benchmark_grasping_ros.msg import BenchmarkGrasp

from benchmark_grasping.base.base_grasp_planner import CameraData

from benchmark_grasping.superquadric_based.superquadrics_grasp_planner import SuperquadricsGraspPlanner


class SuperquadricGraspPlannerService(SuperquadricsGraspPlanner):
    def __init__(self, cfg_file, grasp_service_name, grasp_publisher_name):
        """
        Parameters
        ----------
        model_config_file (str): path to model configuration file of type config.json
        fully_conv (bool): flag to use fully-convolutional network
        cv_bridge: (obj:`CvBridge`): ROS `CvBridge`

        grasp_pose_publisher: (obj:`Publisher`): ROS publisher to publish pose of planned grasp for visualization.
        """

        super(SuperquadricGraspPlannerService, self).__init__(cfg_file)
        
        # Create publisher to publish pose of final grasp.
        self.grasp_pose_publisher = None
        if grasp_publisher_name is not None:
            self.grasp_pose_publisher = rospy.Publisher(grasp_publisher_name, PoseStamped, queue_size=10)

        # Initialize the ROS service.
        self._grasp_planning_service = rospy.Service(grasp_service_name, GraspPlannerCloud,
                                            self.plan_grasp_handler)


    def read_images(self, req):
        """Reads images from a ROS service request.

        Parameters
        ---------
        req: :obj:`ROS ServiceRequest`
            ROS ServiceRequest for grasp planner service.
        """
        # Get the ROS data
        ros_cloud = req.cloud
        cam_position = np.array([req.view_point.position.x, req.view_point.position.y, req.view_point.position.z])
        cam_quat = np.array([req.view_point.orientation.x, req.view_point.orientation.y, 
                             req.view_point.orientation.z, req.view_point.orientation.w])

        # pointcloud2 to numpy array
        pc_data = ros_numpy.numpify(ros_cloud)
        points = np.c_[pc_data['x'], pc_data['y'], pc_data['z']]
        colors = pc_data['rgb']

        return self.create_camera_data(points, colors, cam_position, cam_quat)

    def plan_grasp_handler(self, req):
        """Grasp planner request handler.

        Parameters
        ---------
        req: :obj:`ROS ServiceRequest`
            ROS `ServiceRequest` for grasp planner service.
        """
        camera_data = self.read_images(req)

        self.grasp_poses = []
        ok = self.plan_grasp(camera_data, n_candidates=1)

        self.visualize()

        if ok:
            return self._create_grasp_planner_srv_msg()
        else:
            return None
            

    def _create_grasp_planner_srv_msg(self):
        if len(self.grasp_poses) == 0:
            return False

        # --- Create `BenchmarkGrasp` return message --- #
        grasp_msg = BenchmarkGrasp()

        # set pose...
        p = PoseStamped()
        p.header.frame_id = self.best_grasp.ref_frame
        p.header.stamp = rospy.Time.now()

        p.pose.position.x = self.best_grasp.position[0]
        p.pose.position.y = self.best_grasp.position[1]
        p.pose.position.z = self.best_grasp.position[2]

        p.pose.orientation.w = self.best_grasp.quaternion[3]
        p.pose.orientation.x = self.best_grasp.quaternion[0]
        p.pose.orientation.y = self.best_grasp.quaternion[1]
        p.pose.orientation.z = self.best_grasp.quaternion[2]

        grasp_msg.pose = p

        # ...score
        grasp_msg.score.data = self.best_grasp.score

        # ... and width
        grasp_msg.width.data = self.best_grasp.width

        if self.grasp_pose_publisher is not None:
            # Publish the pose alone for easy visualization of grasp
            # pose in Rviz.
            print("publish grasp!! .")
            self.grasp_pose_publisher.publish(p)

        return grasp_msg


if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node("Superquadric_based_Grasp_Planner")


    # Get configs.
    config_file = rospy.get_param("~config_file")
    grasp_service_name = rospy.get_param("~grasp_planner_service_name")
    grasp_publisher_name = rospy.get_param("~grasp_publisher_name")

    # Instantiate the grasp planner.
    grasp_planner = SuperquadricGraspPlannerService(config_file,
                                              grasp_service_name, grasp_publisher_name)

    rospy.loginfo("Superquadric-based grasp detection server initialized, waiting for a point cloud ...")

    # Spin forever.
    rospy.spin()
