#!/usr/bin/env python3
import os
import time
from enum import Enum

import rospy

from std_msgs.msg import Header

from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput

from extend_msgs.msg import GripperControl, GripperResponse


FINGER_FORCE = 150
FINGER_SPEED = 255

class Robotiq3FGripperMode(Enum):
    BASIC = 0
    PINCH = 1
    WIDE = 2
    SCISSOR = 3

class Robotiq3FGripperControlNode:
    def __init__(self, mode=Robotiq3FGripperMode.BASIC):
        # Initializing the publishers
        self.pub_robotiq_control = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput, queue_size=1)
        self.pub_gripper_command_republisher = rospy.Publisher('extend_gripper_republished_command', GripperControl, queue_size=1)
        self.pub_gripper_response = rospy.Publisher('extend_gripper_response', GripperResponse, queue_size=1)
        self.current_mode = mode
        self.joint_command_values = [0.0] * 3  # Assuming 3 joints for the gripper


    def reset_gripper(self):
        gripper_control_msg = Robotiq3FGripperRobotOutput()
        self.pub_robotiq_control.publish(gripper_control_msg)

    def activate_gripper(self):
        gripper_control_msg = Robotiq3FGripperRobotOutput()
        gripper_control_msg.rACT = 1
        self.pub_robotiq_control.publish(gripper_control_msg)

    def mode_select(self, mode):
        gripper_control_msg = Robotiq3FGripperRobotOutput()
        gripper_control_msg.rACT = 1
        if mode == Robotiq3FGripperMode.BASIC:
            gripper_control_msg.rMOD = 0
        elif mode == Robotiq3FGripperMode.PINCH:
            gripper_control_msg.rMOD = 1
        elif mode == Robotiq3FGripperMode.WIDE:
            gripper_control_msg.rMOD = 2
        elif mode == Robotiq3FGripperMode.SCISSOR:
            gripper_control_msg.rMOD = 3
        else:
            raise ValueError("Invalid gripper mode selected")
        self.pub_robotiq_control.publish(gripper_control_msg)

    def gripper_command_publish(self):
        gripper_control_msg = Robotiq3FGripperRobotOutput()
        gripper_control_msg.rACT = 1
        gripper_control_msg.rGTO = 1
        gripper_control_msg.rATR = 0
        gripper_control_msg.rGLV = 0
        gripper_control_msg.rICS = 0

        gripper_control_msg.rSPA = FINGER_SPEED
        gripper_control_msg.rFRA = FINGER_FORCE

        gripper_control_msg.rSPB = FINGER_SPEED
        gripper_control_msg.rFRB = FINGER_FORCE

        gripper_control_msg.rSPC = FINGER_SPEED
        gripper_control_msg.rFRC = FINGER_FORCE

        gripper_control_msg.rPRS = 0
        gripper_control_msg.rSPS = 0
        gripper_control_msg.rFRS = 0


        if self.current_mode == Robotiq3FGripperMode.BASIC:
            gripper_control_msg.rMOD = 0
            gripper_control_msg.rICF = 1
            gripper_control_msg.rPRA = int(255 * self.joint_command_values[0] /70.0)  #Joint command values are in the range of 0-70 degrees for the fingers
            gripper_control_msg.rPRB = int(255 * self.joint_command_values[1] /70.0)
            gripper_control_msg.rPRC = int(255 * self.joint_command_values[2] /70.0)

        elif self.current_mode == Robotiq3FGripperMode.PINCH:
            gripper_control_msg.rMOD = 1
            gripper_control_msg.rPRA = int(255 * self.joint_command_values[0] /70.0) # In pinch mode individual finger control is disabled.

        elif self.current_mode == Robotiq3FGripperMode.WIDE:
            gripper_control_msg.rMOD = 2
            gripper_control_msg.rICF = 1
            gripper_control_msg.rPRA = int(255 * self.joint_command_values[0] /70.0)
            gripper_control_msg.rPRB = int(255 * self.joint_command_values[1] /70.0)
            gripper_control_msg.rPRC = int(255 * self.joint_command_values[2] /70.0)

        elif self.current_mode == Robotiq3FGripperMode.SCISSOR:
            gripper_control_msg.rMOD = 3
            # Map analog input value 0-1 to 0-255 for scissor mode
            gripper_control_msg.rPRA = int(255 * self.joint_command_values[0] /70.0) # In scissor mode individual finger control is disabled.

        else:
            raise ValueError("Invalid gripper mode selected, cannot publish gripper command")

        self.pub_robotiq_control.publish(gripper_control_msg)

    def vr_gripper_command_callback(self, msg):
        if len(msg.handJointValues) > 0 and len(msg.handJointValues) == 3:
            self.joint_command_values = msg.handJointValues
        else:
            self.joint_command_values = [msg.gripperAnalog.data] * 3  # Use analog value for all joints if joint values are not provided

        header = Header()
        header.seq = 0
        header.frame_id = ""
        header.stamp = rospy.Time.now()

        pub_gripper_command_republisher_data = GripperControl()
        pub_gripper_command_republisher_data = msg
        pub_gripper_command_republisher_data.header = header
        self.pub_gripper_command_republisher.publish(pub_gripper_command_republisher_data)

        # Fetching the Gripper Response Joint States and Force
        pub_gripper_response_data = GripperResponse()
        pub_gripper_response_data.header = header
        self.pub_gripper_response.publish(pub_gripper_response_data)

def main():
    gripper_mode = os.getenv("GRIPPER_MODE", "BASIC")

    # Creating the ros node
    rospy.init_node("ur_robotiq_gripper")
    gripper_control_node = Robotiq3FGripperControlNode(mode=Robotiq3FGripperMode[gripper_mode.upper()])

    time.sleep(0.5)
    # Reset the Gripper
    gripper_control_node.reset_gripper()
    time.sleep(1)
    # Activate the Gripper
    gripper_control_node.activate_gripper()

    time.sleep(0.5)
    # Set the Gripper Mode
    gripper_control_node.mode_select(mode=Robotiq3FGripperMode[gripper_mode.upper()])

    # Subscribe to Digital Gripper Data Stream from Unity
    rospy.Subscriber("extend_gripper_command", GripperControl, gripper_control_node.vr_gripper_command_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
