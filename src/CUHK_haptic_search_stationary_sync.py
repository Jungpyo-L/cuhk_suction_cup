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

rospy.init_node("suction_run")
nachi_help = NachiController()
grasp_info = SegmentationInfo()
# experimental parameters
timeLimit = 15
# Setup helper functions
P_help = P_CallbackHelp()  # it deals with subscription.
rospy.sleep(0.5)
file_help = fileSaveHelp()
adpt_help = adaptMotionHelp(d_lat=5)
end_workspacelimit = 950

# Set data logger
print("Wait for the data_logger to be enabled")
rospy.wait_for_service("data_logging")
dataLoggerEnable = rospy.ServiceProxy("data_logging", Enable)
dataLoggerEnable(False)  # reset Data Logger just in case
rospy.sleep(1)
file_help.clearTmpFolder()  # clear the temporary folder

print("Start sampling")
# start sampling pressure , bias both
P_help.startSampling()
rospy.sleep(0.3)

dataLoggerEnable(True)  # start data logging


def init():
    grasp_info_subscriber = rospy.Subscriber(
        "nachi_left", SegmentationInfo, start_search
    )


def start_search(msg):
    # set biases now
    try:
        P_help.setNowAsOffset()
        rospy.sleep(0.1)
    except:
        print("set now as offset failed, but it's okay")
    grasp_info = msg
    item_location = [grasp_info.object_pose.pose. position.x * 1000, grasp_info.object_pose.pose.position.y * 1000 + 30,
                     grasp_info.object_pose.pose.position.z * 1000]
    try:
        waiting_point_y = -330
        waiting_point = [item_location[0], waiting_point_y, 5 + 15]
        nachi_help.move_robot_target_pose_sync(waiting_point)

        # Target pose which is 15 mm above a PCB. For a test, use 50 mm above just in case
        waiting_distance = waiting_point_y - item_location[1]

        suctionFlag = False
        pressureCheckFlag = False
        vacuumCheckFlag = False
        startTime = time.time()
        iteration = 1
        P_vac = adpt_help.P_vac
        while (nachi_help.conveyor_value - grasp_info.Register_value) < waiting_distance:
            continue
        current_value = nachi_help.conveyor_value
        target_pose = [waiting_point[0], waiting_point[1], waiting_point[2] - 15]
        while not suctionFlag:
            # move down
            target_pose[2] += -15
            target_pose[1] = target_pose[1] + nachi_help.conveyor_value - current_value
            current_value = nachi_help.conveyor_value
            print("iteration: ", iteration)
            print("move down")
            if target_pose[1] > 200:
                break
            nachi_help.move_robot_target_pose_sync(target_pose)

            # rospy.sleep(0.05)
            P_check = P_help.four_pressure

            # move up
            target_pose[2] += +15
            target_pose[1] = target_pose[1] + nachi_help.conveyor_value - current_value
            current_value = nachi_help.conveyor_value
            if target_pose[1] > 200:
                break
            nachi_help.move_robot_target_pose_sync(target_pose)
            print("move up")

            # Check vacuum & move to next pose
            # rospy.sleep(0.05)
            P = P_help.four_pressure
            nachi_help.update_iteration(iteration)
            if all(np.array(P) < P_vac):
                print(f"Suction Engage Succeed from {iteration} touch")
                suctionFlag = True
                args.elapsedTime = startTime
                break
            elif time.time() - startTime > timeLimit:
                args.timeOverFlag = True
                break
            else:
                adpt_help.T = adpt_help.get_Tmat_lateralMove(P_check)
                target_pose[0] += adpt_help.T[0, 3]
                target_pose[1] += adpt_help.T[1, 3]
                iteration += 1
                # nachi_help.move_robot_target_pose_sync(targetPose)

        Waiting_point = [313, -65, 120]
        nachi_help.move_robot_target_pose_sync(Waiting_point)
        # Save args
        args.suctionFlag = suctionFlag
        args.iteration = iteration
        args.timeLimit = timeLimit
        # Save Init data
        dataLoggerEnable(False)  # start data logging
        # args = parser.parse_args()
        file_help.saveDataParams(args, appendTxt="mode_" + str(args.mode))
        file_help.clearTmpFolder()
        P_help.stopSampling()
        rospy.sleep(0.5)

        print("==========Suction Pressure Test complete!==========")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


def main(args):
    init()
    rospy.spin()


if __name__ == "__main__":
    import argparse

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
    main(args)
