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
import time

from cuhk_suction_cup.srv import Enable

from geometry_msgs.msg import Pose

from helperFunction.SuctionP_callback_helper import P_CallbackHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.adaptiveMotion import adaptMotionHelp
from helperFunction.nachiHelper import NachiController



def main(args):

  # controller node
  rospy.init_node('suction_run')
  
  # experimental parameters
  timeLimit = 10

  # Setup helper functions
  P_help = P_CallbackHelp() # it deals with subscription.
  rospy.sleep(0.5)
  file_help = fileSaveHelp()
  adpt_help = adaptMotionHelp(d_lat = args.step)
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

    ##In case 1
    input("please enter to go disengagePose")
    # nachi_help.move_robot_target_pose(disengagePose)
    # V3 = 1
    nachi_help.robotStart()
    disEngagePose = [313, -65, 200]
    nachi_help.move_robot_target_pose_sync(disEngagePose)

    targetPose = [313, -65, -10 +50] # Target pose which is 15 mm above a PCB. For a test, use 50 mm above just in case
    input("please enter to go start haptic search")
    nachi_help.move_robot_target_pose_sync(targetPose)

    suctionFlag = False
    pressureCheckFlag = False
    vacuumCheckFlag = False
    startTime = time.time()
    iteration = 1
    P_vac = adpt_help.P_vac

    while !suctionFlag:
      # move down
      targetPose[2] += -15
      nachi_help.move_robot_target_pose_sync(targetPose)
      rospy.sleep(0.05)
      P_check = P_help.four_pressur

      # move up
      targetPose[2] += +15
      nachi_help.move_robot_target_pose_sync(targetPose)

      # Check vacuum & move to next pose
      P = P_help.four_pressur
      if all(np.array(P)<P_vac):
            print(f"Suction Engage Succeed from {iteration} touch")
        suctionFlag = True
        args.elapsedTime = startTime
        break
      elif time.time()-startTime > timeLimit:
        args.timeOverFlag = True
        break
      else:
        adpt_help.T = adpt_help.get_Tmat_lateralMove(P)
        targetPose[0] += adpt_help.T[0,3]
        targetPose[1] += adpt_help.T[1,3]
        iternation += 1
        nachi_help.move_robot_target_pose_sync(targetPose)

    # Save args
    args.suctionFlag = suctionFlag
    args.iteration = iteration
    args.timeLimit = timeLimit
    # Save Init data
    dataLoggerEnable(False) # start data logging
    file_help.saveDataParams(args, appendTxt='mode_'+str(args.mode))
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
  parser.add_argument('--mode', type=str, help='label of system mode (stationary or belt)', default='stationary')
  parser.add_argument('--step', type=int, help='step size of haptic search', default=5)
  args = parser.parse_args()
  main(args)
