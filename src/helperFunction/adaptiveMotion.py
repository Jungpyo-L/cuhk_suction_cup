#!/usr/bin/env python3

import os
import datetime
import numpy as np
import re
from geometry_msgs.msg import PoseStamped
from .mathUtils import (
    rotation_from_quaternion,
    create_transform_matrix,
    quaternion_from_matrix,
    normalize,
    hat,
)
from scipy.spatial.transform import Rotation as Rot
import scipy


class adaptMotionHelp(object):
    def __init__(self, dP_threshold=10, dw=15, P_vac=-500, d_lat=5, d_z=1.5e-3):
        # for first performance test dw=15, d_lat = 0.5e-2, d_z= 1.5e-3
        # original dw = 3, d_lat = 0.1e-2, d_z = 0.3e-3
        self.dP_threshold = dP_threshold
        self.dw = dw * np.pi / 180.0

        self.P_vac = P_vac
        self.d_lat = d_lat
        self.d_z_normal = d_z

        # for Brownian motion (initial condition)
        self.x0 = 0
        self.BM_step = 0
        self.BM_x = 0
        self.BM_y = 0

        # for tansformation matrix to calculate relative haptic location
        self.T = create_transform_matrix(np.eye(3), [0, 0, 0])

    def get_ObjectPoseStamped_from_T(self, T):
        thisPose = PoseStamped()
        thisPose.header.frame_id = "base_link"
        R = T[0:3, 0:3]
        quat = quaternion_from_matrix(R)
        position = T[0:3, 3]
        [
            thisPose.pose.position.x,
            thisPose.pose.position.y,
            thisPose.pose.position.z,
        ] = position
        [
            thisPose.pose.orientation.x,
            thisPose.pose.orientation.y,
            thisPose.pose.orientation.z,
            thisPose.pose.orientation.w,
        ] = quat

        return thisPose

    def get_Tmat_from_Pose(self, PoseStamped):
        quat = [
            PoseStamped.pose.orientation.x,
            PoseStamped.pose.orientation.y,
            PoseStamped.pose.orientation.z,
            PoseStamped.pose.orientation.w,
        ]
        translate = [
            PoseStamped.pose.position.x,
            PoseStamped.pose.position.y,
            PoseStamped.pose.position.z,
        ]
        return self.get_Tmat_from_PositionQuat(translate, quat)

    def get_Tmat_from_PositionQuat(self, Position, Quat):
        rotationMat = rotation_from_quaternion(Quat)
        T = create_transform_matrix(rotationMat, Position)
        return T

    def get_PoseStamped_from_T_initPose(self, T, initPoseStamped):
        T_now = self.get_Tmat_from_Pose(initPoseStamped)
        targetPose = self.get_ObjectPoseStamped_from_T(np.matmul(T_now, T))
        return targetPose

    def get_Tmat_TranlateInBodyF(self, translate=[0.0, 0.0, 0.0]):
        return create_transform_matrix(np.eye(3), translate)

    def get_Tmat_TranlateInZ(self, direction=1):
        offset = [0.0, 0.0, np.sign(direction) * self.d_z_normal]
        # if step:
        #     offset = [0.0, 0.0, np.sign(direction)*step]
        return self.get_Tmat_TranlateInBodyF(translate=offset)

    def get_Tmat_TranlateInY(self, direction=1):
        offset = [0.0, np.sign(direction) * self.d_lat, 0.0]
        # if step:
        #     offset = [0.0, 0.0, np.sign(direction)*step]
        return self.get_Tmat_TranlateInBodyF(translate=offset)

    def get_Tmat_TranlateInX(self, direction=1):
        offset = [np.sign(direction) * self.d_lat, 0.0, 0.0]
        # if step:
        #     offset = [0.0, 0.0, np.sign(direction)*step]
        return self.get_Tmat_TranlateInBodyF(translate=offset)

    def get_Tmat_alignSuction(self, P_array, weightVal=1.0):
        dw = self.dw
        dP_threshold = self.dP_threshold

        P0, P1, P2, P3 = P_array

        PW = (P3 + P2) / 2
        PE = (P1 + P0) / 2
        PN = (P1 + P2) / 2
        PS = (P0 + P3) / 2

        # pressure differentials
        dP_WE = PW - PE  # 0 deg
        dP_SN = PS - PN  # 90 deg
        dP_NW_SE = P2 - P0  # 45 deg
        dP_SW_NE = P3 - P1  # -45 deg

        # always reset variables for rotation axis / pressure gradient
        a = 0
        b = 0
        theta = 0

        # if above threshold
        if abs(dP_WE) > dP_threshold:
            a += dP_WE

        # if above threshold
        if abs(dP_SN) > dP_threshold:
            b -= dP_SN

        # if either is above threshold, the increase dw
        if abs(dP_SN) > dP_threshold or abs(dP_WE) > dP_threshold:
            theta = dw

        # if dP_SN > dP_threshold or dP_WE > dP_threshold:
        #   print("decreased theta")
        #   theta += dw
        #   if dP_SN*dP_WE < 0:
        #     theta += dw
        # if dP_SN < -dP_threshold or dP_WE < -dP_threshold:
        #   print("increased theta")
        #   theta -= dw
        #   if dP_SN*dP_WE > 0:
        #     theta -= dw

        # rotation axis definition from a and bBM_step
        rot_axis = np.array([a, b, 0])
        norm = np.linalg.norm(rot_axis)

        # if vector != 0, go to next pressure iteration
        if norm == 0:
            # skip to checking normal force and grasp condition
            # continue
            T = np.eye(4)
            pass  # it seems it should be pass rather than continue

        else:  # if good, add 1 deg
            rot_axis = rot_axis / norm

            # publish the next target pose
            # print("theta: ", theta)

            omega_hat = hat(rot_axis)
            Rw = scipy.linalg.expm(weightVal * theta * omega_hat)

            T = create_transform_matrix(Rw, [0, 0, 0])

        # should check normal force is high enough, if not, adjust
        # print('dP_WE: ', dP_WE)     # Px
        # print('dP_SN: ', dP_SN)     # Py
        # print('dP_NW_SE: ', dP_NW_SE)
        # print('dP_SW_NE: ', dP_SW_NE)
        # print("a: ", a)
        # print("b: ", b)

        return T

    def get_Tmat_lateralMove(self, P_array, weightVal=1.0):
        d_lat = self.d_lat
        dP_threshold = self.dP_threshold

        P0, P1, P2, P3 = P_array

        PW = (P3 + P2) / 2
        PE = (P1 + P0) / 2
        PN = (P1 + P2) / 2
        PS = (P0 + P3) / 2

        # pressure differentials
        dP_WE = PW - PE  # 0 deg
        dP_SN = PS - PN  # 90 deg
        dP_NW_SE = P2 - P0  # 45 deg
        dP_SW_NE = P3 - P1  # -45 deg

        dx_lat = 0.0
        dy_lat = 0.0

        # added by Jungpyo (220909)
        r_p = np.zeros(2)
        r_p[0] = -dP_WE  # x-axis: from east to west
        r_p[1] = -dP_SN  # y-axis: from north to south
        r_p = r_p / np.linalg.norm(r_p)


        if abs(dP_WE) > dP_threshold:
            dy_lat = r_p[0] * d_lat * weightVal

        if abs(dP_SN) > dP_threshold:
            dx_lat = r_p[1] * d_lat * weightVal

        T = self.get_Tmat_TranlateInBodyF([dx_lat, dy_lat, 0.0])
        return T

    def get_Tmat_lateralMove_Random(self, d_lat = 5):
        theta = np.random.rand() * 360 * np.pi / 180
        dx_lat = np.cos(theta) * d_lat
        dy_lat = np.sin(theta) * d_lat

        T = self.get_Tmat_TranlateInBodyF([dx_lat, dy_lat, 0.0])
        return T

    def get_Tmat_lateralMove_BM(self):
        dx_lat = self.BM_x[self.BM_step + 1] - self.BM_x[self.BM_step]
        dy_lat = self.BM_y[self.BM_step + 1] - self.BM_y[self.BM_step]

        T = self.get_Tmat_TranlateInBodyF([dx_lat, dy_lat, 0.0])
        return T

    def get_BM(self, n_step=125 * 15, sigma=0.03):
        w = np.ones(n_step) * self.x0

        for i in range(1, n_step):
            # Sampling from the NOrmal distribution
            yi = np.random.normal()
            # Weiner process
            w[i] = w[i - 1] + (yi / np.sqrt(n_step)) * sigma

        return w

    def get_Tmats_Suction(self, weightVal):
        P_array = [20, 0, 0, 20]
        T_align = self.get_Tmat_alignSuction(P_array, weightVal=weightVal)
        T_later = self.get_Tmat_lateralMove(P_array, weightVal=1.0 - weightVal)
        return T_later, T_align

    def get_Tmats_freeRotation(self, a, b):
        rot_axis = np.array([a, b, 0])
        norm = np.linalg.norm(rot_axis)
        theta = self.dw

        # if vector != 0, go to next pressure iteration
        if norm == 0:
            T_align = np.eye(4)
            pass  # it seems it should be pass rather than continue

        else:  # if good, add 1 deg
            rot_axis = rot_axis / norm

            omega_hat = hat(rot_axis)
            Rw = scipy.linalg.expm(theta * omega_hat)

            T_align = create_transform_matrix(Rw, [0, 0, 0])

        T_later = np.eye(4)
        return T_later, T_align

    def get_Tmat_alignTorque(self, T_array):
        dw = self.dw
        # dP_threshold = self.dP_threshold
        # T_threshold = 0.1
        # T_threshold = 0.005
        # T_threshold = 0.003
        T_threshold = 0.008

        Tx, Ty = T_array

        # always reset variables for rotation axis / pressure gradient
        a = 0
        b = 0
        theta = 0

        # if above threshold
        if abs(Ty) > T_threshold:
            b += Ty

        # if above threshold
        if abs(Tx) > T_threshold:
            a += Tx

        # if either is above threshold, then increase dw
        if abs(Tx) > T_threshold or abs(Ty) > T_threshold:
            theta = dw

        # rotation axis definition from a and b
        rot_axis = np.array([a, b, 0])
        norm = np.linalg.norm(rot_axis)

        # if vector != 0, go to next pressure iteration
        if norm == 0:
            # skip to checking normal force and grasp condition
            # continue
            T = np.eye(4)
            pass  # it seems it should be pass rather than continue

        else:  # if good, add 1 deg
            rot_axis = rot_axis / norm

            # publish the next target pose
            # print("theta: ", theta)

            omega_hat = hat(rot_axis)
            Rw = scipy.linalg.expm(theta * omega_hat)

            T = create_transform_matrix(Rw, [0, 0, 0])

        # should check normal force is high enough, if not, adjust
        # print('dP_WE: ', dP_WE)     # Px
        # print('dP_SN: ', dP_SN)     # Py
        # print('dP_NW_SE: ', dP_NW_SE)
        # print('dP_SW_NE: ', dP_SW_NE)
        # print("a: ", a)
        # print("b: ", b)

        return T

    def get_T_array_cup_old(self, T_array, F_array, quat):
        Tx = T_array[0]
        Ty = T_array[1]
        Fx = F_array[0]
        Fy = F_array[1]
        dd = 0 / 1000  # -2, 0, 2
        d = 0.0938 + 14 / 1000 + dd
        # d = 0.09

        Rot = rotation_from_quaternion(quat)
        # print(Rot)
        x_axis = Rot[0, 0:3]
        y_axis = Rot[1, 0:3]
        z_axis = Rot[2, 0:3]
        # print(x_axis)

        MGD = 0.02
        Tx_cg = np.dot(y_axis, np.cross(z_axis, MGD * np.array([0, 0, 1])))
        Ty_cg = np.dot(x_axis, np.cross(z_axis, MGD * np.array([0, 0, 1])))

        MG = 0.26 * 9.81
        F_cg = MG * np.matmul(np.linalg.inv(Rot), np.array([0, 0, -1]))

        Fcx = Fx - F_cg[0]
        Fcy = Fy - F_cg[1]

        T_array_cup = [Tx + d * Fcy - Tx_cg, Ty - d * Fcx - Ty_cg]

        return T_array_cup

    def get_T_array_cup(self, T_array, F_array, quat):
        Tx = T_array[0]
        Ty = T_array[1]
        Fx = F_array[0]
        Fy = F_array[1]
        dd = 0 / 1000  # -2, 0, 2
        d = 100 / 1000 + dd
        # d = 0.09

        Rot = rotation_from_quaternion(quat)
        # print(Rot)
        x_axis = Rot[0, 0:3]
        y_axis = Rot[1, 0:3]
        z_axis = Rot[2, 0:3]
        # print(x_axis)

        MGD = 0.017
        Tx_cg = np.dot(y_axis, np.cross(z_axis, MGD * np.array([0, 0, -1])))
        Ty_cg = np.dot(x_axis, np.cross(z_axis, MGD * np.array([0, 0, -1])))

        MG = 0.264 * 9.81
        F_cg = MG * np.matmul(np.linalg.inv(Rot), np.array([0, 0, -1]))

        Fcx = Fx - F_cg[0]
        Fcy = Fy - F_cg[1]

        T_array_cup = [Tx + d * Fcy - Tx_cg, Ty - d * Fcx - Ty_cg]

        return T_array_cup

    def get_Tmats_from_controller(self, P_array, T_array_cup, controller_str, altFlag):
        # ["FTR","W1","W2","W3","W4","W5","PRLalt"]
        if controller_str == "FTR":
            # self.dw = 0.3 * np.pi / 180.0
            T_align = self.get_Tmat_alignTorque(T_array_cup)
            T_later = np.eye(4)

        elif controller_str == "NON":
            T_align = np.eye(4)
            T_later = np.eye(4)

        elif controller_str == "BML":
            if self.BM_step == 0:
                self.BM_x = self.get_BM()
                self.BM_y = self.get_BM()
                print("self.BM_x:", self.BM_x)
                print("self.BM_y:", self.BM_y)
            T_align = np.eye(4)
            T_later = self.get_Tmat_lateralMove_BM()
            self.BM_step += 1

        elif "W" in controller_str:
            if controller_str == "W1":
                weightVal = 0.0
            if controller_str == "W2":
                weightVal = 0.25
            if controller_str == "W3":
                weightVal = 0.5
            if controller_str == "W4":
                weightVal = 0.75
            if controller_str == "W5":
                weightVal = 1.0

            T_align = self.get_Tmat_alignSuction(P_array, weightVal=weightVal)
            T_later = self.get_Tmat_lateralMove(P_array, weightVal=1.0 - weightVal)

        elif "FTRPL" in controller_str:
            T_align = self.get_Tmat_alignTorque(T_array_cup)
            T_later = self.get_Tmat_lateralMove(P_array, weightVal=1.0)

        elif controller_str == "PRLalt":
            if altFlag:
                n_step

            T_align = self.get_Tmat_alignSuction(P_array, weightVal=weightVal)
            T_later = self.get_Tmat_lateralMove(P_array, weightVal=1.0 - weightVal)

        return T_later, T_align

    def get_Tmat_axialMove(self, F_normal, F_normalThres):
        if F_normal > -F_normalThres[0]:
            # print("should be pushing towards surface in cup z-dir")
            T_normalMove = self.get_Tmat_TranlateInZ(direction=1)
        elif F_normal < -F_normalThres[1]:
            # print("should be pulling away from surface in cup z-dir")
            T_normalMove = self.get_Tmat_TranlateInZ(direction=-1)
        else:
            T_normalMove = np.eye(4)
        return T_normalMove

    def intersection_between_cup_box(
        self, T_targetPose, BoxTopLeftCorner_meter, BoxBottomRightCorner_meter
    ):
        positionVec = T_targetPose[3, 0:3]
        rayVec = -T_targetPose[
            2, 0:3
        ]  # z axis is aligned to downward suction cup axis.
        rayCoefficients = np.zeros(4)
        rayCoefficients[0] = (BoxTopLeftCorner_meter[0] - positionVec[0]) / rayVec[0]
        rayCoefficients[1] = (BoxTopLeftCorner_meter[1] - positionVec[1]) / rayVec[1]
        rayCoefficients[2] = (BoxBottomRightCorner_meter[0] - positionVec[0]) / rayVec[
            0
        ]
        rayCoefficients[3] = (BoxBottomRightCorner_meter[1] - positionVec[1]) / rayVec[
            1
        ]

        closestCoef = np.amin(rayCoefficients[rayCoefficients > 0])
        intersecPoint = positionVec + closestCoef * rayVec
        return intersecPoint

    def get_Tmat_path(self, T_array, count):
        dw = self.dw
        # dP_threshold = self.dP_threshold
        # T_threshold = 0.1
        T_threshold = 0.01

        Tx, Ty = T_array

        # always reset variables for rotation axis / pressure gradient
        a = 0
        b = 0
        theta = 0

        # # if above threshold
        # if abs(Ty) > T_threshold:
        #     a += Tx

        # # if above threshold
        # if abs(Tx) > T_threshold:
        #     b += Ty

        # # if either is above threshold, then increase dw
        # if abs(Tx) > T_threshold or abs(Ty) > T_threshold:
        #     theta = dw

        a = 1
        b = 1
        # b = -np.cos(np.deg2rad(count*5.5))

        # a = np.cos(5*count/180*np.pi)
        # b = np.sin(5*count/180*np.pi)

        theta = dw

        # rotation axis definition from a and b
        rot_axis = np.array([a, b, 0])
        norm = np.linalg.norm(rot_axis)

        # if vector != 0, go to next pressure iteration
        if norm == 0:
            # skip to checking normal force and grasp condition
            # continue
            T = np.eye(4)
            pass  # it seems it should be pass rather than continue

        else:  # if good, add 1 deg
            rot_axis = rot_axis / norm

            # publish the next target pose
            # print("theta: ", theta)

            omega_hat = hat(rot_axis)
            Rw = scipy.linalg.expm(theta * omega_hat)

            T = create_transform_matrix(Rw, [0, 0, 0])

        # should check normal force is high enough, if not, adjust
        # print('dP_WE: ', dP_WE)     # Px
        # print('dP_SN: ', dP_SN)     # Py
        # print('dP_NW_SE: ', dP_NW_SE)
        # print('dP_SW_NE: ', dP_SW_NE)
        # print("a: ", a)
        # print("b: ", b)

        return T

    def getGoalPosestampedFromGQCNN(self, T, thisPose, initEndEffPosestamped):
        # From pose Information of GQCNN, get the transformed orientation of the UR10

        initEndEffectorPosition = [
            initEndEffPosestamped.pose.position.x,
            initEndEffPosestamped.pose.position.y,
            initEndEffPosestamped.pose.position.z,
        ]
        initEndEffectorQuat = [
            initEndEffPosestamped.pose.orientation.x,
            initEndEffPosestamped.pose.orientation.y,
            initEndEffPosestamped.pose.orientation.z,
            initEndEffPosestamped.pose.orientation.w,
        ]

        deltaPosition = np.matmul(
            T,
            (
                np.array(
                    [thisPose.position.x, thisPose.position.y, thisPose.position.z, 1]
                )
            ),
        )
        goalRobotPosition = deltaPosition[0:3] + np.array(initEndEffectorPosition)

        # orient
        thisQuat = [
            thisPose.orientation.x,
            thisPose.orientation.y,
            thisPose.orientation.z,
            thisPose.orientation.w,
        ]

        r_pose_from_cam = Rot.from_quat(thisQuat)
        axis_in_cam = r_pose_from_cam.as_matrix()
        # print(axis_in_cam)
        targetVec = axis_in_cam[:, 0]  # rotated x is the target vector

        R_N_cam = T[0:3, 0:3]
        targetSuctionAxisVec_N = np.matmul(R_N_cam, targetVec)
        # print(targetSuctionAxisVec_N)

        # to us, z axis of the the tool0 should be the

        r_currOrient_RobotEff = Rot.from_quat(initEndEffectorQuat)
        currSuctionAxisVec_N = r_currOrient_RobotEff.as_matrix()[:, 2]

        rotAxis = np.cross(currSuctionAxisVec_N, targetSuctionAxisVec_N)
        rotAxis /= np.linalg.norm(rotAxis)
        angleBtwTwo = np.arccos(np.dot(currSuctionAxisVec_N, targetSuctionAxisVec_N))

        rotAxis_in_BodyF = r_currOrient_RobotEff.apply(rotAxis, inverse=True)
        r_RotOrient = Rot.from_mrp(rotAxis_in_BodyF * np.tan(angleBtwTwo / 4))

        r_targetOrient_RobotEff = r_currOrient_RobotEff * r_RotOrient

        targetOrient_quat = r_targetOrient_RobotEff.as_quat()

        T_pose = self.get_Tmat_from_PositionQuat(goalRobotPosition, targetOrient_quat)
        return self.get_ObjectPoseStamped_from_T(T_pose)
        # return goalRobotPosition, targetOrient_quat, targetSuctionAxisVec_N

    def getGoalPosestampedFromCam(self, T, thisPose, initEndEffPosestamped):
        # From pose Information of GQCNN, get the transformed orientation of the UR10

        initEndEffectorPosition = [
            initEndEffPosestamped.pose.position.x,
            initEndEffPosestamped.pose.position.y,
            initEndEffPosestamped.pose.position.z,
        ]
        initEndEffectorQuat = [
            initEndEffPosestamped.pose.orientation.x,
            initEndEffPosestamped.pose.orientation.y,
            initEndEffPosestamped.pose.orientation.z,
            initEndEffPosestamped.pose.orientation.w,
        ]

        targetPosition = np.array([thisPose[0], thisPose[1], thisPose[2], 1])
        goalRobotPosition = np.matmul(T, targetPosition)
        goalRobotPosition[0:3] = goalRobotPosition[0:3] + np.array(
            initEndEffectorPosition
        )

        T_pose = self.get_Tmat_from_PositionQuat(
            goalRobotPosition[0:3], np.array(initEndEffectorQuat)
        )
        return self.get_ObjectPoseStamped_from_T(T_pose)
