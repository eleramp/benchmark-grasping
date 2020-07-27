#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import math
import os
import time

import numpy as np
import rospy
from benchmark_grasping.base.grasp import Grasp6D

class CameraData:
    rgb_img = None
    depth_img = None
    pc_img = None
    seg_img = None
    bounding_box = None
    intrinsic_params = None
    extrinsic_params = {'position': (0.0)*3, 'rotation': (0.0)*9}


class BaseGraspPlanner(object):
    """The base class for grasp planners

    """
    def __init__(self, cfg:dict):
        """
        Parameters
        ----------
        cfg : dict
            Dictionary of configuration parameters.
        """
        self.cfg = cfg
        self._grasp_poses = []
        self._best_grasp = None

    def reset(self):
        self.grasp_poses = []

    def plan_grasp(self, camera_data, n_candidates=1):
        """Grasp Planner
            Compute candidate grasp poses

        Args:
            camera_data (obj): `CameraData`. Contains the data (img, camera params) necessary to compute the grasp poses
            n_candidates (int): number of candidate grasp poses to return

        Raises:
            NotImplementedError: [description]
        """

        raise NotImplementedError

    def visualize(self):
        """Plot the grasp poses
        """

        pass

    @property
    def grasp_poses(self):
        return self._grasp_poses

    @grasp_poses.setter
    def grasp_poses(self, grasp_poses:list):
        if len(grasp_poses) is 0:
            self._grasp_poses = []

        elif type(grasp_poses[0]) is not Grasp6D:
            raise ValueError('Invalid grasp type. Must be `benchmark_grasping.grasp.Grasp6D`')

        self._grasp_poses = grasp_poses

    @property
    def best_grasp(self):
        return self._best_grasp

    @best_grasp.setter
    def best_grasp(self, best_grasp:Grasp6D):
        if type(best_grasp) is not Grasp6D:
            raise ValueError('Invalid grasp type. Must be `benchmark_grasping.grasp.Grasp6D`')

        self._best_grasp = best_grasp

