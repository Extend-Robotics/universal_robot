#!/usr/bin/env python3
import os
import time

import rospy
from std_msgs.msg import Header
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput
from extend_msgs.msg import GripperControl, GripperResponse
from extend_msgs.srv import GetString, GetStringResponse

FINGER_FORCE = 150
FINGER_SPEED = 255
ROBOTIQ_3F_GRIPPER_MODE_ENUM_DICT = {"BASIC": 0, "PINCH": 1, "WIDE": 2, "SCISSOR": 3}


class Robotiq3FGripperControlNode:
    def __init__(self, mode):
        # Initializing the publishers
        self.robotiq_control_pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput, queue_size=1)
        self.gripper_command_republisher = rospy.Publisher('extend_gripper_republished_command', GripperControl, queue_size=1)
        self.gripper_response_pub = rospy.Publisher('extend_gripper_response', GripperResponse, queue_size=1)
        rospy.Service('robotiq_3f_current_mode', GetString, self.current_mode_provider)
        self.current_mode = mode
        self.joint_command_values = [0.0] * 3  # 3 Active joints for the gripper

    def current_mode_provider(self, request):
        response = GetStringResponse()
        # Providing the current gripper mode string
        response.data = self.current_mode
        return response

    def reset_gripper(self):
        gripper_control_msg = Robotiq3FGripperRobotOutput()
        self.robotiq_control_pub.publish(gripper_control_msg)

    def activate_gripper(self):
        gripper_control_msg = Robotiq3FGripperRobotOutput()
        gripper_control_msg.rACT = 1
        self.robotiq_control_pub.publish(gripper_control_msg)

    def mode_select(self):
        gripper_control_msg = Robotiq3FGripperRobotOutput()
        gripper_control_msg.rACT = 1
        gripper_control_msg.rMOD = ROBOTIQ_3F_GRIPPER_MODE_ENUM_DICT[self.current_mode]
        self.robotiq_control_pub.publish(gripper_control_msg)

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

        gripper_control_msg.rMOD = ROBOTIQ_3F_GRIPPER_MODE_ENUM_DICT[self.current_mode]

        # Joint command values are in the range of 0-70 degrees for the fingers
        gripper_control_msg.rPRA = int(255 * self.joint_command_values[0] /70.0)
        if ROBOTIQ_3F_GRIPPER_MODE_ENUM_DICT[self.current_mode] in (0, 2):
            gripper_control_msg.rICF = 1
            gripper_control_msg.rPRB = int(255 * self.joint_command_values[1] / 70.0)
            gripper_control_msg.rPRC = int(255 * self.joint_command_values[2] / 70.0)

        self.robotiq_control_pub.publish(gripper_control_msg)

    def vr_gripper_command_callback(self, msg):
        if len(msg.handJointValues) > 0 and len(msg.handJointValues) == 3:
            self.joint_command_values = msg.handJointValues
        else:
            raise ValueError("Invalid hand joint values received.")

        header = Header()
        header.seq = 0
        header.frame_id = ""
        header.stamp = rospy.Time.now()

        gripper_command_republisher_data = GripperControl()
        gripper_command_republisher_data = msg
        gripper_command_republisher_data.header = header
        self.gripper_command_republisher.publish(gripper_command_republisher_data)

        # Fetching the Gripper Response Joint States and Force
        gripper_response_data = GripperResponse()
        gripper_response_data.header = header
        self.gripper_response_pub.publish(gripper_response_data)
        self.gripper_command_publish()

def main():
    gripper_mode = os.getenv("gripperMode", "BASIC").upper()
    if gripper_mode not in ROBOTIQ_3F_GRIPPER_MODE_ENUM_DICT:
        raise ValueError(f"Invalid gripper mode \"{gripper_mode}\" selected. Please select the correct gripper mode")

    # Creating the ros node and class instance
    rospy.init_node("ur_robotiq_gripper")
    gripper_control_node = Robotiq3FGripperControlNode(mode=gripper_mode)

    # Reset the Gripper
    gripper_control_node.reset_gripper()
    time.sleep(1)

    # Activate the Gripper
    gripper_control_node.activate_gripper()
    time.sleep(0.5)

    # Set the Gripper Mode
    gripper_control_node.mode_select()
    time.sleep(0.5)

    # Subscribe to Digital Gripper Data Stream from Unity
    rospy.Subscriber("extend_gripper_command", GripperControl, gripper_control_node.vr_gripper_command_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
