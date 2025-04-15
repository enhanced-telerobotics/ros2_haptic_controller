#!/usr/bin/env python
import os
import time
import pygame
import threading
import cv2
import pickle
import numpy as np
import pandas as pd
from numpy import pi as PI
from tqdm import tqdm
from PIL import Image, ImageDraw
from datetime import datetime
import time
from scipy.spatial.transform import Rotation as R

import pybullet as p
import pybullet_data
from surrol.utils.pybullet_utils import (
    step,
    step_real,
    get_joints,
    get_link_name,
    reset_camera,
    forward_kinematics,
    inverse_kinematics,
)

from surrol.utils.robotics import (
    get_matrix_from_pose_2d,
    get_pose_2d_from_matrix,
    get_matrix_from_euler
)

from surrol.const import ROOT_DIR_PATH, ASSET_DIR_PATH

from surrol.robots.psm import Psm, Psm1, Psm2
from robots.stereo_ecm import StereoEcm as Ecm
import utils.utils as utils

from pyOpenHaptics.hd_device import HapticDevice
import pyOpenHaptics.hd as hd
from dataclasses import dataclass, field
from pyOpenHaptics.hd_callback import hd_callback


def dh_transform(theta, d, a, alpha):
    """
    Calculate the transformation matrix from DH parameters.

    Parameters:
    - theta: rotation angle around the previous z-axis (in radians)
    - d: distance along the previous z-axis
    - a: length of the common normal (distance along the previous x-axis)
    - alpha: angle about the common normal, from the previous z-axis to the new z-axis (in radians)

    Returns:
    - 4x4 NumPy array representing the transformation matrix
    """
    # Define the transformation matrix according to DH parameters
    transform_matrix = np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha),
         np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -
         np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0,              np.sin(alpha),                 np.cos(
            alpha),                 d],
        [0,              0,
            0,                             1]
    ])

    return transform_matrix


def get_transform(joints, gimbals):
    T = dh_transform(-joints[0], 110, 0, -np.radians(90))
    T = np.dot(T,
               dh_transform(-joints[1], 0, 133.35, 0))
    T = np.dot(T,
               dh_transform(-joints[2]+np.radians(180), 0, 0, np.radians(90)))
    T = np.dot(T,
               dh_transform(-gimbals[0]+np.radians(180), 133.35, 0, np.radians(90)))
    T = np.dot(T,
               dh_transform(-gimbals[1]+np.radians(-90), 0, 0, np.radians(90)))
    T = np.dot(T,
               dh_transform(gimbals[2]+np.radians(90), 0, 0, np.radians(90)))

    return T


def ctype_to_array(mat) -> np.ndarray:

    np_mat = np.array([
        [mat[0][0], mat[1][0], mat[2][0], mat[3][0]],
        [mat[0][1], mat[1][1], mat[2][1], mat[3][1]],
        [mat[0][2], mat[1][2], mat[2][2], mat[3][2]],
        [mat[0][3], mat[1][3], mat[2][3], mat[3][3]],
    ])

    return np_mat


def draw_axes(transform_matrix, length=0.1):
    """
    绘制以变换矩阵为中心的三维坐标轴，并返回线条的ID列表。
    :param transform_matrix: 4x4 变换矩阵。
    :param length: 坐标轴的长度。
    :return: 包含三条线条ID的列表。
    """
    # 提取平移和旋转部分
    position = transform_matrix[:3, 3]
    rotation_matrix = transform_matrix[:3, :3]

    # 计算坐标轴的末端点
    x_end = position + rotation_matrix[:, 0] * length
    y_end = position + rotation_matrix[:, 1] * length
    z_end = position + rotation_matrix[:, 2] * length

    # 绘制坐标轴并保存ID
    line_ids = []
    line_ids.append(p.addUserDebugLine(
        position, x_end, [1, 0, 0], lineWidth=2))  # X轴为红色
    line_ids.append(p.addUserDebugLine(
        position, y_end, [0, 1, 0], lineWidth=2))  # Y轴为绿色
    line_ids.append(p.addUserDebugLine(
        position, z_end, [0, 0, 1], lineWidth=2))  # Z轴为蓝色

    return line_ids


def clear_axes(line_ids):
    """
    清除由 draw_axes 函数绘制的线条。
    :param line_ids: 由 draw_axes 返回的线条ID列表。
    """
    for line_id in line_ids:
        p.removeUserDebugItem(line_id)


@dataclass
class DeviceState:
    button: bool = False
    position: list = field(default_factory=list)
    joints: list = field(default_factory=list)
    gimbals: list = field(default_factory=list)
    force: list = field(default_factory=list)


@hd_callback
def state_callback():
    global device_state
    transform = hd.get_transform()
    joints = hd.get_joints()
    gimbals = hd.get_gimbals()
    device_state.position = [transform[3][0],
                             -transform[3][1],
                             transform[3][2]]
    device_state.transform = ctype_to_array(transform)
    device_state.joints = [joints[0], joints[1], joints[2]]
    device_state.gimbals = [gimbals[0], gimbals[1], gimbals[2]]
    hd.set_force(device_state.force)
    button = hd.get_buttons()
    device_state.button = True if button == 1 else False


def main():
    client = p.connect(p.GUI)
    p.setRealTimeSimulation(1)
    scaling = 1.
    fps = 60

    p.setGravity(0, 0, -9.81)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    time_step = 1.0 / fps
    p.setTimeStep(time_step, physicsClientId=client)
    reset_camera(yaw=90.0, pitch=-30.0, dist=0.82 * scaling,
                 target=(-0.05 * scaling, 0, 0.36 * scaling))

    POSE_PSM1 = ((0.05, 0.24, 0.8524), (0, 0, -(90 + 20) / 180 * np.pi))
    POSE_TABLE = ((0.5, 0, 0.001), (0, 0, 0))

    p.loadURDF("plane.urdf", [0, 0, -0.001], globalScaling=1)
    psm = Psm1(POSE_PSM1[0],
               p.getQuaternionFromEuler(POSE_PSM1[1]),
               scaling=scaling)
    psm.reset_joint((0, 0, 0.10, 0, 0, 0))

    # ecm = Ecm((0.15, 0.0, 0.8524),
    #           scaling=scaling,
    #           stereo=False)
    # ecm.reset_joint((0, 0.6, 0.04, 0))

    p.loadURDF(os.path.join(ASSET_DIR_PATH, 'table/table.urdf'),
               np.array(POSE_TABLE[0]) * scaling,
               p.getQuaternionFromEuler(POSE_TABLE[1]),
               globalScaling=scaling)

    pre_transform = get_transform(
        device_state.joints, np.array(device_state.gimbals))
    line_ids = []

    # while not device_state.button:
    while True:
        clear_axes(line_ids)  # TODO: Debug
        current_transform = get_transform(
            device_state.joints, np.array(device_state.gimbals))
        diff_transform = current_transform - pre_transform

        world_pose = psm.pose_rcm2world(psm.get_current_position())

        if device_state.button:
            world_pose[:3, 3] += diff_transform[:3, 3]/1000
        world_pose[:3, :3] = current_transform[:3, :3]

        rcm_pose = psm.pose_world2rcm(world_pose)

        psm.move(rcm_pose)
        step()
        pre_transform = current_transform.copy()

        print(np.array(device_state.joints), np.array(
            device_state.gimbals), end='\r')
        line_ids = draw_axes(world_pose)  # TODO: Debug


if __name__ == "__main__":
    np.set_printoptions(precision=4)
    device_state = DeviceState()
    device = HapticDevice(device_name="Default Device",
                          callback=state_callback)
    time.sleep(0.2)
    main()
    device.close()
