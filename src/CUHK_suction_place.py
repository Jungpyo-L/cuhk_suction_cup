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
import argparse


class HapticSearchSync(object):
    def __init__(self, arg):
        rospy.init_node("suction_run")
        self.nachi_help = NachiController()
        self.grasp_info = SegmentationInfo()
        # experimental parameters
        self.P_help = P_CallbackHelp()  # it deals with subscription.
        rospy.sleep(0.5)
        self.file_help = fileSaveHelp()
        self.adapt_help = adaptMotionHelp(d_lat=5)

        # Set data logger
        print("Wait for the data_logger to be enabled")
        rospy.wait_for_service("data_logging")
        self.dataLoggerEnable = rospy.ServiceProxy("data_logging", Enable)

        self.dataLoggerEnable(False)  # reset Data Logger just in case
        rospy.sleep(1)
        self.file_help.clearTmpFolder()  # clear the temporary folder
        print("Start sampling")
        # start sampling pressure , bias both
        self.P_help.startSampling()
        rospy.sleep(0.3)
        self.end_workspace_limit = 950
        self.timeLimit = 15
        self.waiting_point_y = -330
        self.args = arg
        self.waiting_point = [313, -65, 120]
        self.grasp_info_subscriber = rospy.Subscriber(
            "nachi_left", SegmentationInfo, self.start_search
        )

    def start_search(self, msg):
        self.dataLoggerEnable(True)
        rospy.sleep(0.3)
        try:
            self.P_help.setNowAsOffset()
            rospy.sleep(0.1)
        except:
            print("set now as offset failed, but it's okay")
        grasp_info = msg
        item_location = [
            grasp_info.object_pose.pose.position.x * 1000,
            grasp_info.object_pose.pose.position.y * 1000 + 30,
            grasp_info.object_pose.pose.position.z * 1000,
        ]
        try:
            waiting_point = [item_location[0], self.waiting_point_y, 5 + 15]
            self.nachi_help.move_robot_target_pose_sync(waiting_point)

            # Target pose which is 15 mm above a PCB. For a test, use 50 mm above just in case
            waiting_distance = self.waiting_point_y - item_location[1]

            suction_flag = False
            iteration = 1
            while (
                    self.nachi_help.conveyor_value - grasp_info.Register_value
            ) < waiting_distance:
                continue
            current_value = self.nachi_help.conveyor_value
            target_pose = [waiting_point[0], waiting_point[1], waiting_point[2] - 15]

            # move down
            target_pose[2] += -15
            target_pose[1] = (
                    target_pose[1] + self.nachi_help.conveyor_value - current_value
            )
            current_value = self.nachi_help.conveyor_value
            print("iteration: ", iteration)
            print("move down")

            self.nachi_help.move_robot_target_pose_sync(target_pose)

            # move up
            target_pose[2] += +15
            target_pose[1] = (
                    target_pose[1] + self.nachi_help.conveyor_value - current_value
            )

            self.nachi_help.move_robot_target_pose_sync(target_pose)
            print("move up")

            # place_position = [35, 353, 119]
            # self.nachi_help.move_robot_target_pose_sync(place_position)
            self.nachi_help.move_robot_target_pose_sync(self.waiting_point)

            ## go to the bin
            bin_pose = Pose()
            bin_pose.position.x = 60.0
            # rospy.sleep(1)
            self.nachi_help.relative_joint_movement_publisher.publish(bin_pose)
            rospy.sleep(1.5)
            bin_pose.position.x = -30.0
            # rospy.sleep(1)
            self.nachi_help.relative_joint_movement_publisher.publish(bin_pose)
            self.nachi_help.move_robot_target_pose_sync(self.waiting_point)


            # Save args
            self.args.suctionFlag = suction_flag
            self.args.iteration = iteration
            self.args.timeLimit = self.timeLimit
            # Save Init data
            self.dataLoggerEnable(False)  # start data logging
            # args = parser.parse_args()
            self.file_help.saveDataParams(
                self.args, appendTxt="mode_" + str(self.args.mode)
            )
            self.file_help.clearTmpFolder()
            self.P_help.stopSampling()
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
