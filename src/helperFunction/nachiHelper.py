#!/usr/bin/env python3

import sys
import rospy
import curses
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float32

from cuhk_suction_cup.msg import cmdToRobot
from cuhk_suction_cup.msg import iterationPacket

from libnachi.srv import nachiGetTCPState
from libnachi.msg import TipState
from libnachi.msg import cmdToPC
from libnachi.msg import SegmentationInfo


class NachiController(object):
    def __init__(self):
        super(NachiController).__init__()
        self._start_follow = False
        self.move_pose_publisher = rospy.Publisher(f"tcp_pose", Pose, queue_size=50)
        self.move_pose_sync_publisher = rospy.Publisher(
            f"tcp_pose_sync", Pose, queue_size=50
        )
        self.coordinate_system_publisher = rospy.Publisher(
            f"coordinate_system", String, queue_size=1
        )
        self.pose_method_publisher = rospy.Publisher(
            "pose_method", String, queue_size=1
        )
        self.suction_signal_publisher = rospy.Publisher(
            "suction_signal", Bool, queue_size=1
        )
        self.iteration_publisher = rospy.Publisher(
            "iterationPacket", iterationPacket, queue_size=1
        )
        self.relative_joint_movement_publisher = rospy.Publisher(
            "relative_joint_movement", Pose, queue_size=50
        )
        self.coordinate_system_msg = String()
        self.pose_method_msg = String()
        self.direction_list = ["x+", "x-", "y+", "y-", "z+", "z+", "w+", "w-"]
        self.tip_state_subscriber = rospy.Subscriber(
            "TipState", TipState, self.get_tip_state
        )
        self.robot_state_subscriber = rospy.Subscriber(
            "RunningState", Bool, self.get_robot_state
        )
        self.conveyor_value_subscriber = rospy.Subscriber(
            "ConveyorValue", Float32, self.get_conveyor_value
        )
        # self.grasp_info_subscriber = rospy.Subscriber("/sorting_line/nachi_left", SegmentationInfo, self.get_grasp_info)
        self.tip_state = TipState()
        self.robot_state = Bool()
        self.iteration = iterationPacket()
        self.grasp_info = SegmentationInfo()
        self.conveyor_value = 0

        # Robot state
        self.robotCMD_Pub = rospy.Publisher("cmdToRobot", cmdToRobot, queue_size=10)
        self.cmdToRobot = cmdToRobot()
        self.cmd_IDLE = 0
        self.cmd_START = 1
        self.cmd_GRASPING = 2
        self.cmd_UPDATING = 3
        self.cmd_FINISHING = 4
        self.cmd_COMPLETEPRESSURE = 5

        # PC state
        rospy.Subscriber("cmdToPC", cmdToPC, self.callback_statePC)
        self.statePC = 0

        # C++ sync state
        self.sync = 0

    # send CMD to Robot

    def robotIdle(self):
        self.cmdToRobot.cmdInput = self.cmd_IDLE
        self.cmdToRobot.header.stamp = rospy.Time.now()
        self.robotCMD_Pub.publish(self.cmdToRobot)

    def robotStart(self):
        self.cmdToRobot.cmdInput = self.cmd_START
        self.cmdToRobot.header.stamp = rospy.Time.now()
        self.robotCMD_Pub.publish(self.cmdToRobot)

    def robotGrasping(self):
        self.cmdToRobot.cmdInput = self.cmd_GRASPING
        self.cmdToRobot.header.stamp = rospy.Time.now()
        self.robotCMD_Pub.publish(self.cmdToRobot)

    def robotUpdating(self):
        self.cmdToRobot.cmdInput = self.cmd_UPDATING
        self.cmdToRobot.header.stamp = rospy.Time.now()
        self.robotCMD_Pub.publish(self.cmdToRobot)

    def robotFinishing(self):
        self.cmdToRobot.cmdInput = self.cmd_FINISHING
        self.cmdToRobot.header.stamp = rospy.Time.now()
        self.robotCMD_Pub.publish(self.cmdToRobot)

    def completePressureCheck(self):
        self.cmdToRobot.cmdInput = self.cmd_COMPLETEPRESSURE
        self.cmdToRobot.header.stamp = rospy.Time.now()
        self.robotCMD_Pub.publish(self.cmdToRobot)

    # call back for PC state
    def callback_statePC(self, data):
        self.statePC = data.cmdInput

    # call back function
    def get_tip_state(self, msg):
        """
        Get the current tip 's state including TcpSpeed and pose.
        :return:
        """
        self.tip_state = msg

    def get_robot_state(self, msg):
        """
        Get the robot's state
        :return:
        bool
        """
        # print(msg.data)
        self.robot_state = msg.data

    def get_grasp_info(self, msg):
        """
        Get the robot's state
        :return:
        bool
        """
        # print(msg.data)
        self.grasp_info = msg

    def get_conveyor_value(self, msg):
        """
        Get the robot's state
        :return:
        bool
        """
        # print(msg.data)
        self.conveyor_value = msg.data

    def update_iteration(self, iteration):
        self.iteration.data = iteration
        self.iteration.header.stamp = rospy.Time.now()
        self.iteration_publisher.publish(self.iteration)

    def _keyboard_control(self):
        """
        The robot will move with the keyboard input.
        [w','s','a','d','q','e','[',']'] is corresponding to
        ["x+", "x-", "y+", "y-", "z+", "z+", "w+", "w-"]

        """
        self.pose_method_msg.data = "relative"
        self.pose_method_publisher.publish(self.pose_method_msg)
        rospy.loginfo("Please enter one of the keys from 'w','s','a','d','q' and 'e'.")
        while not rospy.is_shutdown():
            keycode = input()
            target_location = Pose()
            if keycode == "w":
                target_location.position.x += 10
                rospy.loginfo("Move 0.01 meter along the x direction.")
            if keycode == "s":
                target_location.position.x -= 10
                rospy.loginfo("Move 0.01 meter along the minus x direction.")
            if keycode == "a":
                target_location.position.y += 10
                rospy.loginfo("Move 0.01 meter along the y direction.")
            if keycode == "d":
                target_location.position.y -= 10
                rospy.loginfo("Move 0.01 meter along the minus y direction.")
            if keycode == "[":
                target_location.position.z += 10
                rospy.loginfo("Move 0.01 meter along the z direction.")
            if keycode == "]":
                target_location.position.z -= 10
                rospy.loginfo("Move 0.01 meter along the minus z direction.")
            if keycode == "q":
                target_location.orientation.w += 10
                rospy.loginfo("Move 0.01 meter along the w direction.")
            if keycode == "e":
                target_location.orientation.w -= 10
                rospy.loginfo("Move 0.01 meter along the minus w direction.")
            self.move_pose_publisher.publish(target_location)
            rospy.sleep(0.05)

    def move_robot(self, direction, step, coordinate_system="base"):
        """
        Args:
            direction: '+','-'
            step
            coordinate_system: 'base', 'end_effector'

        Returns:

        """
        self.pose_method_msg.data = "relative"
        self.pose_method_publisher.publish(self.pose_method_msg)
        self.coordinate_system_msg.data = coordinate_system
        self.coordinate_system_publisher.publish(self.coordinate_system_msg)
        target_location = Pose()
        if direction not in self.direction_list:
            rospy.logwarn(
                'direction should be one of ["x+", "x-", "y+", "y-", "z+", "z+" "w+" "w-"]'
            )
        if direction == "x+":
            target_location.position.x += step
            rospy.loginfo("Move 0.01 meter along the x + direction.")
        if direction == "x-":
            target_location.position.x -= step
            rospy.loginfo("Move 0.01 meter along the x - direction.")
        if direction == "y+":
            target_location.position.y += step
            rospy.loginfo("Move 0.01 meter along the y + direction.")
        if direction == "y-":
            target_location.position.y -= step
            rospy.loginfo("Move 0.01 meter along the y - direction.")
        if direction == "z+":
            target_location.position.z += step
            rospy.loginfo("Move 0.01 meter along the z - direction.")
        if direction == "z-":
            target_location.position.z -= step
            rospy.loginfo("Move 0.01 meter along the z - direction.")
        if direction == "w+":
            target_location.orientation.w += 10
            rospy.loginfo("Move 0.01 meter along the w direction.")
        if direction == "w-":
            target_location.orientation.w -= 10
            rospy.loginfo("Move 0.01 meter along the minus w direction.")
        self.move_pose_publisher.publish(target_location)
        rospy.sleep(0.05)

    def move_robot_target_pose(self, pose, orientation=[0, 0, 180, 1]):
        """
        Control the robot's end_effector moving to the target pose.
        The pose can accept Pose msg or list[x,y,z,rx,ry,rz,w] as inputs.
        Args:
            pose (Pose|list):
            orientatin: [0, 0, 180] y-direction is flipped
        Returns:

        """
        # [303.738,-275.794,184.377,-134.656,-1.603]
        if not isinstance(pose, list) or isinstance(pose, Pose):
            rospy.logwarn(
                "The pose only accept ros Pose msg or list [x,y,z,rx,ry,rz,w] as inputs."
            )
        target_location = Pose()
        if isinstance(pose, list):
            target_location.position.x = pose[0]
            target_location.position.y = pose[1]
            target_location.position.z = pose[2]
            target_location.orientation.x = orientation[0]
            target_location.orientation.y = orientation[1]
            target_location.orientation.z = orientation[2]
            target_location.orientation.w = orientation[3]
        else:
            target_location = pose
        self.pose_method_msg.data = "absolute"
        self.pose_method_publisher.publish(self.pose_method_msg)
        self.move_pose_publisher.publish(target_location)

    def move_robot_target_pose_sync(self, pose, orientation=[0, 0, 180, 1]):
        """
        Control the robot's end_effector moving to the target pose.
        The pose can accept Pose msg or list[x,y,z,rx,ry,rz,w] as inputs.
        Args:
            pose (Pose|list):
            orientatin: [0, 0, 180] y-direction is flipped
        Returns:

        """
        if not isinstance(pose, list) or isinstance(pose, Pose):
            rospy.logwarn(
                "The pose only accept ros Pose msg or list [x,y,z,rx,ry,rz,w] as inputs."
            )
        target_location = Pose()
        if isinstance(pose, list):
            target_location.position.x = pose[0]
            target_location.position.y = pose[1]
            target_location.position.z = pose[2]
            target_location.orientation.x = orientation[0]
            target_location.orientation.y = orientation[1]
            target_location.orientation.z = orientation[2]
            target_location.orientation.w = orientation[3]
        else:
            target_location = pose
        self.pose_method_msg.data = "absolute"
        self.pose_method_publisher.publish(self.pose_method_msg)
        self.move_pose_sync_publisher.publish(target_location)

        # rospy.sleep(0.05)
        pose_diff_norm = np.linalg.norm(
            np.array(pose[0:3]) - np.array(self.tip_state.pose[0:3])
        )

        while pose_diff_norm > 1:
            pose_diff_norm = np.linalg.norm(
                np.array(pose[0:3]) - np.array(self.tip_state.pose[0:3])
            )
            continue

        # while self.tip_state.speed > 1:
        #     continue
        # while self.robot_state:
        #     continue
        # other method: get sync from the C++

    def move_robot_lateral(self, T, coordinate_system="base"):
        """
        move robot to lateral direction from the pressure reading
        Args:
            T: trasnformation matrix （4 by 4)
            coordinate_system：reference coordinate

        Returns:
        """
        self.coordinate_system_msg.data = coordinate_system
        self.coordinate_system_publisher.publish(self.coordinate_system_msg)
        target_location = Pose()

        target_location.position.x += T[1, 3]
        target_location.position.y += T[2, 3]
        self.pose_method_msg.data = "relative"
        self.pose_method_publisher.publish(self.pose_method_msg)
        self.move_pose_publisher.publish(target_location)

    def move_robot_up(self, step=15, coordinate_system="base"):
        """
        move robot to up for jumping haptic search
        Args:
            step: step size for the movement
            coordinate_system：reference coordinate

        Returns:
        """
        self.coordinate_system_msg.data = coordinate_system
        self.coordinate_system_publisher.publish(self.coordinate_system_msg)
        target_location = Pose()

        target_location.position.z += step
        self.pose_method_msg.data = "relative"
        self.pose_method_publisher.publish(self.pose_method_msg)
        self.move_pose_publisher.publish(target_location)

    def move_robot_down(self, step=15, coordinate_system="base"):
        """
        move robot to down for jumping haptic search
        Args:
            step: step size for the movement
            coordinate_system：reference coordinate

        Returns:
        """
        self.coordinate_system_msg.data = coordinate_system
        self.coordinate_system_publisher.publish(self.coordinate_system_msg)
        target_location = Pose()

        target_location.position.z -= step
        self.pose_method_msg.data = "relative"
        self.pose_method_publisher.publish(self.pose_method_msg)
        self.move_pose_publisher.publish(target_location)

    def move_robot_x(self, step=5, coordinate_system="base"):
        self.coordinate_system_msg.data = coordinate_system
        self.coordinate_system_publisher.publish(self.coordinate_system_msg)
        target_location = Pose()

        target_location.position.x += step
        self.pose_method_msg.data = "relative"
        self.pose_method_publisher.publish(self.pose_method_msg)
        self.move_pose_publisher.publish(target_location)

    def move_robot_y(self, step=5, coordinate_system="base"):
        self.coordinate_system_msg.data = coordinate_system
        self.coordinate_system_publisher.publish(self.coordinate_system_msg)
        target_location = Pose()

        target_location.position.y += step
        self.pose_method_msg.data = "relative"
        self.pose_method_publisher.publish(self.pose_method_msg)
        self.move_pose_publisher.publish(target_location)

    # utilities
    def quaternion_multiply(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return (
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2,
            w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2,
        )

    def getPoseObj(self, goalPosition, setOrientation):
        Pose = PoseStamped()

        Pose.header.frame_id = "base"
        Pose.pose.orientation.x = setOrientation[0]
        Pose.pose.orientation.y = setOrientation[1]
        Pose.pose.orientation.z = setOrientation[2]
        Pose.pose.orientation.w = setOrientation[3]

        Pose.pose.position.x = goalPosition[0]
        Pose.pose.position.y = goalPosition[1]
        Pose.pose.position.z = goalPosition[2]

        return Pose

    def getRotVector(self, goalPose):
        qx = goalPose.pose.orientation.x
        qy = goalPose.pose.orientation.y
        qz = goalPose.pose.orientation.z
        qw = goalPose.pose.orientation.w
        # q = [0, 0, 0, 1] # calculate during the calibration
        # print("goalPose.pose.orientation (converted): ", goalPose.pose.orientation)
        # print("q: ", q)
        # r = R.from_quat(self.quaternion_multiply([qx, qy, qz, qw], q))
        r = R.from_quat([qx, qy, qz, qw])
        Rx, Ry, Rz = r.as_rotvec()
        return Rx, Ry, Rz

    def getTransformedPose(self, goalPose):
        T_mat = np.matmul(
            np.linalg.inv(self.transformation), adpt_help.get_Tmat_from_Pose(goalPose)
        )
        pose = adpt_help.get_ObjectPoseStamped_from_T(T_mat)
        return pose

    def getTransformedPoseInv(self, goalPose):
        T_mat = np.matmul(self.transformation, adpt_help.get_Tmat_from_Pose(goalPose))
        pose = adpt_help.get_ObjectPoseStamped_from_T(T_mat)
        return pose

    def getTCPPose(self, pose):
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z
        Rx, Ry, Rz = self.getRotVector(pose)
        return [x, y, z, Rx, Ry, Rz]
