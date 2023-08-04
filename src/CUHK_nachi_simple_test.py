#!/usr/bin/env python3

# Authors: Jungpyo Lee

# imports
try:
  import rospy
  import tf
  ros_enabled = True
except:
  print('Couldn\'t import ROS.  I assume you\'re running this on your laptop')
  ros_enabled = False

import os, sys
import numpy as np

from cuhk_suction_cup.srv import Enable

from geometry_msgs.msg import Pose

from helperFunction.SuctionP_callback_helper import P_CallbackHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.adaptiveMotion import adaptMotionHelp
from helperFunction.nachiHelper import NachiController



def main(args):

  # controller node
  rospy.init_node('suction_run')

  # Setup helper functions
  P_help = P_CallbackHelp() # it deals with subscription.
  rospy.sleep(0.5)
  file_help = fileSaveHelp()
  adpt_help = adaptMotionHelp(d_lat = 0.005)
  nachi_help = NachiController()

  # Set data logger
  print("Wait for the data_logger to be enabled")
  rospy.wait_for_service('data_logging')
  dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)
  dataLoggerEnable(False) # reset Data Logger just in case
  rospy.sleep(1)
  file_help.clearTmpFolder()        # clear the temporary folder

  print("Start sampling")
  # start sampling pressure , bias both
  P_help.startSampling()
  rospy.sleep(0.3)

  dataLoggerEnable(True) # start data logging
  
  # set biases now
  try:
    P_help.setNowAsOffset()
    rospy.sleep(0.1)
  except:
    print("set now as offset failed, but it's okay")

  
  try:
    i = 0
    # while i < 10:
    #   # start data logger
    #   P_help.startSampling()
    #   rospy.sleep(0.1)


    #   P_array = P_help.four_pressure

    #   #  Find to above the next search location wrt tool coordinate
    #   T_later = adpt_help.get_Tmat_lateralMove(P_array)
    #   print("T_later")
    #   print(T_later)
    #   # targetPoseStamped = adpt_help.get_PoseStamped_from_T_initPose(T_later, measuredCurrPose)
    #   # targetSearchPoseStamped = adpt_help.get_PoseStamped_from_T_initPose(T_offset, targetPoseStamped)
    #   # rtde_help.goToPose(targetSearchPoseStamped, speed=args.speed, acc=args.acc)
    #   i += 1

    # Basic test for robot arm
    disengagePose = [313, -65, 100]
    input("please enter to go disengagePose")
    nachi_help.move_robot_target_pose(disengagePose)
    speed, pose = nachi_help.get_tip_state()
    print("pose: ", pose[0:3])
    print("speed: ", speed)
    rospy.sleep(0.5)
    speed, pose = nachi_help.get_tip_state()
    print("pose: ", pose)
    print("speed: ", speed)
    rospy.sleep(0.5)
    speed, pose = nachi_help.get_tip_state()
    print("pose: ", pose)
    print("speed: ", speed)

    disengagePose2 = [313, -65, 15]
    input("please enter to go disengagePose2")
    nachi_help.move_robot_target_pose(disengagePose2)
    speed, pose = nachi_help.get_tip_state()
    print("pose: ", pose)
    print("speed: ", speed)
    rospy.sleep(0.5)
    speed, pose = nachi_help.get_tip_state()
    print("pose: ", pose)
    print("speed: ", speed)
    rospy.sleep(0.5)
    speed, pose = nachi_help.get_tip_state()
    print("pose: ", pose)
    print("speed: ", speed)

    input("please enter to move down")
    nachi_help.move_robot_down()
    speed, pose = nachi_help.get_tip_state()
    print("pose: ", pose)

    input("please enter to move up")
    nachi_help.move_robot_up()
    speed, pose = nachi_help.get_tip_state()
    print("pose: ", pose)

    input("please enter to move +x")
    nachi_help.move_robot_x()
    speed, pose = nachi_help.get_tip_state()
    print("pose: ", pose)

    input("please enter to move +y")
    nachi_help.move_robot_y()
    speed, pose = nachi_help.get_tip_state()
    print("pose: ", pose)

      
    # Save Init data
    dataLoggerEnable(False) # start data logging
    file_help.saveDataParams(args, appendTxt='PCB_'+str(args.PCB))
    file_help.clearTmpFolder()
    P_help.stopSampling()
    rospy.sleep(0.5)

    print("==========Suction Pressure Test complete!==========")

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--PCB', type=str, help='label of test PCB from 0 to 1', default='0')
  args = parser.parse_args()
  main(args)
