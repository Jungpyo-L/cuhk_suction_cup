#!/usr/bin/env python3

# Authors: Jungpyo Lee

# imports
try:
    import rospy
    import tf

    ros_enabled = True
except:
    print("Couldn't import ROS.  I assume you're running this on your laptop")
    ros_enabled = False

import os, sys
import numpy as np
import time

from cuhk_suction_cup.srv import Enable

from geometry_msgs.msg import Pose
from libnachi.msg import SegmentationInfo
from helperFunction.SuctionP_callback_helper import P_CallbackHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.adaptiveMotion import adaptMotionHelp
from helperFunction.nachiHelper import NachiController
from helperFunction.modbus_control import ModbusController
import argparse


class HapticSearchSync(object):
    def __init__(self, arg):
        rospy.init_node("suction_run")
        self.nachi_help = NachiController()
        self.grasp_info = SegmentationInfo()
        self.modbus_controller = ModbusController()
        # experimental parameters
        self.P_help = P_CallbackHelp()  # it deals with subscription.
        rospy.sleep(0.5)
        self.file_help = fileSaveHelp()
        self.adapt_help = adaptMotionHelp(d_lat=5)

        # Set data logger

        rospy.sleep(1)
        self.file_help.clearTmpFolder()
        self.use_dataloader = True  # clear the temporary fold    print("Wait for the data_logger to be enabled")
        if self.use_dataloader:
            rospy.wait_for_service("data_logging")
            self.dataLoggerEnable = rospy.ServiceProxy("data_logging", Enable)
            #
            self.dataLoggerEnable(False)  # reset Data Logger just in caseer
        print("Start sampling")
        rospy.sleep(0.3)
        self.end_workspace_limit = 950
        self.timeLimit = 15
        self.waiting_point_y = -330
        self.args = arg
        self.waiting_point = [313, -55, 120]
        self.p_check = []
        while True:
            input("start_search")
            self.start_search()

    def is_vacuum(self):
        pressure_bias = self.P_help.four_pressure - self.P_help.PressureOffset
        average_bias = sum(pressure_bias) / 4
        print("pressure_bias", pressure_bias)
        print("average_bias", average_bias)
        print("self.p_check", self.p_check)
        if (
                average_bias < self.adapt_help.P_vac
                or any(pressure_bias) < 2 * self.adapt_help.P_vac
                or (all(np.array(self.p_check) < -1600) and average_bias < -3000)
        ):
            return True
        else:
            return False

    def start_search(self):
        if self.use_dataloader:
            self.dataLoggerEnable(True)
        self.modbus_controller.open_gas()
        self.P_help.startSampling()
        rospy.sleep(0.3)
        try:
            self.P_help.setNowAsOffset()
            rospy.sleep(0.1)
        except:
            print("set now as offset failed, but it's okay")
        item_location = [self.nachi_help.tip_state.pose[0], self.nachi_help.tip_state.pose[1],
                         self.nachi_help.tip_state.pose[2]]
        try:
            waiting_point = [item_location[0], item_location[1], -10 + 15]
            self.nachi_help.move_robot_target_pose_sync(waiting_point)
            input("start going down")
            # Target pose which is 15 mm above a PCB. For a test, use 50 mm above just in case

            suction_flag = False
            start_time = time.time()
            iteration = 1
            target_pose = [waiting_point[0], waiting_point[1], waiting_point[2]]
            while not suction_flag:
                # move down
                target_pose[2] += -15
                print("iteration: ", iteration)
                print("move down")

                self.nachi_help.move_robot_target_pose_sync(target_pose)

                rospy.sleep(0.05)
                self.p_check = self.P_help.four_pressure - self.P_help.PressureOffset
                print("P_check", self.p_check)
                # move up
                target_pose[2] += +15
                if target_pose[1] > 200:
                    self.modbus_controller.open_gas()
                    break
                self.nachi_help.move_robot_target_pose_sync(target_pose)
                print("move up")

                # Check vacuum & move to next pose
                self.nachi_help.update_iteration(iteration)

                if self.is_vacuum():
                    print(f"Suction Engage Succeed from {iteration} touch")
                    suction_flag = True
                    self.args.elapsedTime = start_time
                    self.nachi_help.move_robot_target_pose_sync(self.waiting_point)

                    ## go to the bin

                    # rospy.sleep(1)
                    target_relative_joint_pose = [110, 0, 0, 0, 0, 0]
                    self.nachi_help.move_robot_relative_target_joint_pose(
                        target_relative_joint_pose
                    )
                    # self.nachi_help.move_robot_relative_target_joint_pose(target_relative_joint_pose)
                    # self.nachi_help.move_robot_relative_target_joint_pose(target_relative_joint_pose)
                    # self.nachi_help.move_robot_relative_target_joint_pose(target_relative_joint_pose)
                    target_relative_joint_pose = [-110, 0, 0, 0, 0, 0]
                    self.modbus_controller.close_gas()
                    time.sleep(0.1)
                    # rospy.sleep(1)
                    self.nachi_help.move_robot_relative_target_joint_pose(
                        target_relative_joint_pose
                    )
                    # self.nachi_help.move_robot_relative_target_joint_pose(target_relative_joint_pose)
                    # self.nachi_help.move_robot_relative_target_joint_pose(target_relative_joint_pose)
                    # self.nachi_help.move_robot_relative_target_joint_pose(target_relative_joint_pose)
                    self.nachi_help.move_robot_target_pose_sync(self.waiting_point)
                    break
                elif time.time() - start_time > self.timeLimit:
                    self.args.timeOverFlag = True
                    break
                else:
                    self.adapt_help.T = self.adapt_help.get_Tmat_lateralMove(
                        self.p_check
                    )
                    target_pose[0] += self.adapt_help.T[1, 3]
                    target_pose[1] += self.adapt_help.T[0, 3]
                    iteration += 1
                    print("target_pose", self.adapt_help.T)
                    # nachi_help.move_robot_target_pose_sync(targetPose)

            self.nachi_help.move_robot_target_pose_sync(self.waiting_point)
            # Save args
            self.args.suctionFlag = suction_flag
            self.args.iteration = iteration
            self.args.timeLimit = self.timeLimit
            # Save Init data
            if self.use_dataloader:
                self.dataLoggerEnable(False)  # start data logging
                # args = parser.parse_args()
                self.file_help.saveDataParams(
                    self.args, appendTxt="mode_" + str(self.args.mode)
                )
                self.file_help.clearTmpFolder()
                self.P_help.stopSampling()
            self.modbus_controller.close_gas()

            rospy.sleep(0.5)

            print("==========Suction Pressure Test complete!==========")

        except rospy.ROSInterruptException:
            return
        except KeyboardInterrupt:
            return


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        type=str,
        help="label of system mode (stationary or belt)",
        default="stationary",
    )
    parser.add_argument(
        "--step", type=int, help="step size of haptic search", default=5
    )
    args = parser.parse_args()
    haptic_search_synv = HapticSearchSync(args)
    rospy.spin()
