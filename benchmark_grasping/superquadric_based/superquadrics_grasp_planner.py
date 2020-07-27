#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import os
import warnings
import yaml

import numpy as np
from benchmark_grasping.base.base_grasp_planner import BaseGraspPlanner, CameraData
from benchmark_grasping.base.grasp import Grasp6D
import superquadric_bindings  as sb
from benchmark_grasping.base import transformations as tr


class SuperquadricsGraspPlanner(BaseGraspPlanner):
    """Superquadric-based grasp planner

    """
    def __init__(self, cfg_file="cfg/config_panda.yaml"):
        """
        Parameters
        ----------
        cfg : dict
            Dictionary of configuration parameters.
        """

        self._camera_data = CameraData()

        # parse cfg yaml file
        with open(cfg_file) as f:
    
            self.cfg = yaml.load(f, Loader=yaml.FullLoader)

        super(SuperquadricsGraspPlanner).__init__(self.cfg)

        self._robot = self.cfg['robot']['name']
        if self._robot[-3:] == 'Sim' or self._robot[-3:] == 'sim':
            pass  # TODO something

        self._icub_base_T_robot_base = np.array(self.cfg['robot']['icub_base_T_robot_base'])
        self._icub_hand_T_robot_hand = np.array(self.cfg['robot']['icub_hand_T_robot_hand'])

        self._superqs = sb.vector_superquadric()
        self._grasp_res = sb.GraspResults()
        self._sq_estimator = sb.SuperqEstimatorApp()
        self._grasp_estimator = sb.GraspEstimatorApp()
        self._visualizer = sb.Visualizer()

        self.configure(self.cfg)

    def configure(self, cfg):
        
        # ------ Set Visualizer parameters ------ #
        self._visualizer.setPosition(cfg['visualizer']['x'], cfg['visualizer']['y'])
        self._visualizer.setSize(cfg['visualizer']['width'], cfg['visualizer']['height'])

        self._visualizer.resetPoints()
        self._visualizer.resetSuperq()
        self._visualizer.resetPoses()

        # ------ Set Superquadric Model parameters ------ #
        self._sq_estimator.SetNumericValue("tol", cfg['sq_model']['tol'])
        self._sq_estimator.SetIntegerValue("print_level", 0)
        self._sq_estimator.SetIntegerValue("optimizer_points", cfg['sq_model']['optimizer_points'])
        self._sq_estimator.SetBoolValue("random_sampling", cfg['sq_model']['random_sampling'])

        # ------ Set Superquadric Grasp parameters ------ #
        self._grasp_estimator.SetIntegerValue("print_level", 0)
        self._grasp_estimator.SetNumericValue("tol", cfg['sq_grasp']['tol'])
        self._grasp_estimator.SetIntegerValue("max_superq", cfg['sq_grasp']['max_superq'])
        self._grasp_estimator.SetNumericValue("constr_tol", cfg['sq_grasp']['constr_tol'])
        self._grasp_estimator.SetStringValue("left_or_right", cfg['sq_grasp']['hand'])
        self._grasp_estimator.setVector("plane", np.array(cfg['sq_grasp']['plane_table']))
        self._grasp_estimator.setVector("displacement", np.array(cfg['sq_grasp']['displacement']))
        self._grasp_estimator.setVector("hand", np.array(cfg['sq_grasp']['hand_sq']))
        self._grasp_estimator.setMatrix("bounds_right", np.array(cfg['sq_grasp']['bounds_right']))
        self._grasp_estimator.setMatrix("bounds_left", np.array(cfg['sq_grasp']['bounds_left']))

    def reset(self):
        self.grasp_poses = []
        self._best_grasp = None
        self._camera_data = CameraData()

        self._superqs = sb.vector_superquadric()
        self._grasp_res = sb.GraspResults()

        self._visualizer.resetPoints()
        self._visualizer.resetSuperq()
        self._visualizer.resetPoses()

    def create_camera_data(self, pointcloud: np.ndarray, colors: np.ndarray, cam_pos: np.ndarray, cam_quat: np.ndarray):
        """ Create the CameraData object in the right format expected by the superquadric-based grasp planner

        Parameters
        ---------
            pointcloud (np.ndarray): array of 3D points
            colors (np.ndarray):array of RGB colors
            cam_pos (np.ndarray): 3D position of the camera expressed wrt the robot base
            cam_quat (np.ndarray): quaternion (x, y, z, w) of the camera orientation expressed wrt the robot base

        Returns:
            CameraData: object that stores the input data required by plan_grasp()

        """

        self._camera_data = CameraData()

        # ----------------------------- #
        # --- Create the pointcloud --- #
        # ----------------------------- #
        self._camera_data.pc_img = sb.PointCloud()

        # We need to express the pointcloud wrt the icub base frame:
        #     icub_base_T_cloud = icub_base_T_robot_base * robot_base_T_cam * cam_points

        # create robot_base_T_cloud
        robot_T_cam = np.eye(4)
        robot_R_cam = tr.quaternion_to_matrix(cam_quat)
        robot_T_cam[:3] = np.append(robot_R_cam, np.array([cam_pos]).T, axis=1)

        # create icub_base_T_cam
        icub_T_cam = np.matmul(self._icub_base_T_robot_base, robot_T_cam)

        # fill pointcloud
        sb_points = sb.deque_Vector3d()
        sb_colors = sb.vector_vector_uchar()

        for i, pt in enumerate(pointcloud):
            icub_pt = icub_T_cam.dot(np.append(pt, 1))

            col = colors[i][:3]
            if type(col) is np.ndarray:
                col = col.astype(int)
                col = col.tolist()
            else:
                col = [int(c) for c in col]

            sb_points.push_back(icub_pt[:3])
            sb_colors.push_back(col)

        if sb_points.size() >= self.cfg['sq_model']['minimum_points']:
            self._camera_data.pc_img.setPoints(sb_points)
            self._camera_data.pc_img.setColors(sb_colors)

        self._camera_data.extrinsic_params['position'] = cam_pos
        self._camera_data.extrinsic_params['rotation'] = robot_R_cam

        return self._camera_data

    def plan_grasp(self, camera_data, n_candidates=1):
        """Grasp Planner
            Compute candidate 6D grasp poses

        Args:
            camera_data (obj): `CameraData`. Contains the data (img, camera params) necessary to compute the grasp poses
            n_candidates (int): number of candidate grasp poses to return
        """
        # ---------------------------------- #
        # --- Compute superquadric model --- #
        # ---------------------------------- #
        ok = self._compute_superquadric(camera_data)

        if not ok:
            warnings.warn("Impossible to compute a superquadric model of the given pointcloud")
            return False

        # -------------------------- #
        # --- Compute grasp pose --- #
        # -------------------------- #
        ok = self._compute_grasp_poses(self._superqs)
        if not ok:
            warnings.warn("Cannot compute a valid grasp pose from the given data")
            return False

        return True

    def visualize(self):
        """Plot the pointcloud, superquadric and grasp poses
        """
        self._visualizer.resetPoints()
        self._visualizer.resetSuperq()
        self._visualizer.resetPoses()

        self._visualizer.addPoints(self._camera_data.pc_img, False)
        self._visualizer.addSuperq(self._superqs)
        self._visualizer.addPoses(self._grasp_res.grasp_poses)

        self._visualizer.render()

    @property
    def superqs(self):
        sq_out = sb.vector_superquadric(np.size(self._superqs, 0))
        for i, sq in enumerate(self._superqs):
            sq_out[i] = sq

            # Express the sq pose wrt the robot base, instead of the icub base
            quat_sq = tr.axis_angle_to_quaternion(
                (sq.axisangle[0][0], sq.axisangle[0][1], sq.axisangle[0][2], sq.axisangle[0][3]))
            pos_sq = [sq.center[0][0], sq.center[0][1], sq.center[0][2]]

            robot_base_T_icub_base = np.linalg.inv(self._icub_base_T_robot_base)

            icub_base_T_icub_sq = np.eye(4)
            icub_base_R_icub_sq = tr.quaternion_to_matrix(quat_sq)
            icub_base_T_icub_sq[:3] = np.append(icub_base_R_icub_sq, np.array([pos_sq]).T, axis=1)

            robot_base_T_icub_sq = np.matmul(robot_base_T_icub_base, icub_base_T_icub_sq)

            vec_aa_sq = tr.quaternion_to_axis_angle(tr.matrix_to_quaternion(robot_base_T_icub_sq[:3, :3]))

            sq_out[i].setSuperqOrientation(vec_aa_sq)
            sq_out[i].setSuperqCenter(robot_base_T_icub_sq[:3, 3])

        return sq_out

    def _compute_icub_base_T_robot_base(self):
        """ Compute the transformation from the robot base to the icub base

         We suppose the robot base frame has x pointing forward, y to the left, z upward.
         The superquadric planner considers as reference the icub base frame, which has x pointing backward and y to the right.
         Thus we need to transform pointcloud and grasp pose to/from the icub base frame.

        Returns
        -------
        icub_T_robot (np.ndarray): 4x4 matrix

        """

        if self._robot[:5] == 'icub' or self._robot[:5] == 'iCub':
            self._icub_base_T_robot_base = np.eye(4)

        else:
            # rotation of pi/2 around z
            icub_quat_robot = np.array([0., 0., 1., 0.])

            self._icub_base_T_robot_base = np.eye(4)
            icub_R_robot = tr.quaternion_to_matrix(icub_quat_robot)
            self._icub_base_T_robot_base[:3] = np.append(icub_R_robot, np.array([0., 0., 0.]).T, axis=1)

        return self._icub_base_T_robot_base

    def _compute_superquadric(self, camera_data):
        # Check point cloud validity
        if camera_data.pc_img.getNumberPoints() < self.cfg['sq_model']['minimum_points']:
            print("the point cloud is unvalid. Only {} points."
                  .format(camera_data.pc_img.getNumberPoints()))
            return False

        # Compute superquadrics
        self._superqs = sb.vector_superquadric(self._sq_estimator.computeSuperq(camera_data.pc_img))

        return np.size(self._superqs, 0) >= 0

    def _compute_grasp_poses(self, sq, n_candidates=1):
        # Check a superquadric model exists
        if np.size(sq, 0) is 0:
            print("No superquadric given")
            return False

        # --- Estimate grasp poses --- #
        self._grasp_res = self._grasp_estimator.computeGraspPoses(sq)

        # Checks
        if np.size(self._grasp_res.grasp_poses, 0) is 0:
            return False

        gp = self._grasp_res.grasp_poses[0]
        if np.linalg.norm((gp.position[0][0], gp.position[0][1], gp.position[0][2])) == 0.:
            return False

        # --- Estimate pose cost --- #
        # TODO: Compute pose hat
        # ...

        # Refine pose cost
        self._grasp_estimator.refinePoseCost(self._grasp_res)

        # ---fill self._grasp_poses --- #
        # order grasp poses by cost
        ordered_grasp_poses = list(self._grasp_res.grasp_poses)
        ordered_grasp_poses.sort(key=lambda x: x.cost)

        for i in range(0, min(n_candidates, len(ordered_grasp_poses))):
            gp = ordered_grasp_poses[i]

            robot_base_T_robot_gp = self._transform_grasp(gp)

            # fill Grasp6D()
            grasp_6d = Grasp6D()

            grasp_6d.position = robot_base_T_robot_gp[:3, 3]
            grasp_6d.orientation = robot_base_T_robot_gp[:3, :3]
            grasp_6d.width = 0.0
            grasp_6d.ref_frame = 'base_link'
            grasp_6d.score = gp.cost

            self._grasp_poses.append(grasp_6d)

        self._best_grasp = self._grasp_poses[0]

        return True

    def _transform_grasp(self, gp: sb.GraspPoses):
        # --- transform grasp pose from icub to robot base coordinates --- #
        # robot_T_gp = robot_T_icub * icub_T_gp

        robot_base_T_icub_base = np.linalg.inv(self._icub_base_T_robot_base)

        icub_gp_ax = [gp.axisangle[0][0], gp.axisangle[0][1], gp.axisangle[0][2],
                      gp.axisangle[0][3]]
        icub_gp_pos = [gp.position[0][0], gp.position[0][1], gp.position[0][2]]

        icub_base_T_icub_gp = np.eye(4)
        icub_base_R_icub_gp = tr.quaternion_to_matrix(tr.axis_angle_to_quaternion(icub_gp_ax))
        icub_base_T_icub_gp[:3] = np.append(icub_base_R_icub_gp, np.array([icub_gp_pos]).T, axis=1)

        robot_base_T_icub_gp = np.matmul(robot_base_T_icub_base, icub_base_T_icub_gp)

        # --- transform grasp pose from icub hand ref frame to robot hand ref frame --- #
        robot_base_T_robot_gp = np.matmul(robot_base_T_icub_gp, self._icub_hand_T_robot_hand)

        return robot_base_T_robot_gp






