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
    NUM_ACTIVE_JOINTS = 3
    FINGER_POSITION_REGISTER_MAX = 255
    MAX_FINGER_ANGLE_DEGREES = 70.0

    def __init__(self, mode):
        self.robotiq_control_pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput, queue_size=1)
        self.gripper_command_republisher = rospy.Publisher('extend_gripper_republished_command', GripperControl, queue_size=1)
        self.gripper_response_pub = rospy.Publisher('extend_gripper_response', GripperResponse, queue_size=1)
        rospy.Service('robotiq_3f_current_mode', GetString, self.current_mode_provider)
        self.current_mode = mode
        self.joint_command_values = [0.0] * self.NUM_ACTIVE_JOINTS
        self.analog_command_value = 0.0

    def current_mode_provider(self, _):
        response = GetStringResponse()
        response.data = self.current_mode
        return response

    def reset_gripper(self):
        gripper_control_msg = Robotiq3FGripperRobotOutput()
        # rACT = 0 is the reset command for the Robotiq 3F gripper, resets it to its initial state.
        # Making the assignmenent explicitly here for clarity, even though the default value of rACT is 0
        gripper_control_msg.rACT = 0
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

        if self.current_mode in ("BASIC", "WIDE"):
            # For BASIC and WIDE modes, joint command values are in the range of 0-70 degrees for the fingers
            gripper_control_msg.rICF = 1
            gripper_control_msg.rPRA = int(self.FINGER_POSITION_REGISTER_MAX * self.joint_command_values[0] / self.MAX_FINGER_ANGLE_DEGREES)
            gripper_control_msg.rPRB = int(self.FINGER_POSITION_REGISTER_MAX * self.joint_command_values[1] / self.MAX_FINGER_ANGLE_DEGREES)
            gripper_control_msg.rPRC = int(self.FINGER_POSITION_REGISTER_MAX * self.joint_command_values[2] / self.MAX_FINGER_ANGLE_DEGREES)
        else:
            # For PINCH and SCISSOR modes, all fingers move together based on the analog value between 0-1
            gripper_control_msg.rICF = 0
            gripper_control_msg.rPRA = int(self.FINGER_POSITION_REGISTER_MAX * self.analog_command_value)
        self.robotiq_control_pub.publish(gripper_control_msg)

    def vr_gripper_command_callback(self, msg):
        if self.current_mode in ("PINCH", "SCISSOR"):
            self.analog_command_value = msg.gripperAnalog.data
            if not 0.0 <= self.analog_command_value <= 1.0:
                raise ValueError("Gripper analog command value must be between 0 and 1 for PINCH and SCISSOR modes.")
        else:
            if len(msg.handJointValues) == self.NUM_ACTIVE_JOINTS:
                value_error = []
                self.joint_command_values = msg.handJointValues
                for i, value in enumerate(self.joint_command_values):
                    if not 0.0 <= value <= self.MAX_FINGER_ANGLE_DEGREES:
                        value_error.append((f"Joint Command {value} recieved for Index {i} is outside [0.0, {self.MAX_FINGER_ANGLE_DEGREES}]"))
                if value_error:
                    raise ValueError("\n".join(value_error))
            else:
                raise ValueError("Invalid hand joint values received.")

        header = Header()
        header.seq = 0
        header.frame_id = ""
        header.stamp = rospy.Time.now()

        gripper_command_republisher_data = msg
        gripper_command_republisher_data.header = header
        self.gripper_command_republisher.publish(gripper_command_republisher_data)

        # Data collection always requires a response to be published, even if it's empty
        # Data collection config is based on arm models and does not consider the attached gripper type.
        # Therefore, till we distinguish between different grippers we need to publish the response.
        gripper_response_data = GripperResponse()
        gripper_response_data.header = header
        self.gripper_response_pub.publish(gripper_response_data)
        self.gripper_command_publish()


def main():
    gripper_mode = os.getenv("gripperMode", "BASIC").upper()
    if gripper_mode not in ROBOTIQ_3F_GRIPPER_MODE_ENUM_DICT:
        raise ValueError(f"Invalid gripper mode \"{gripper_mode}\" selected. Valid modes are: {list(ROBOTIQ_3F_GRIPPER_MODE_ENUM_DICT.keys())}")

    rospy.init_node("ur_robotiq_gripper")
    gripper_control_node = Robotiq3FGripperControlNode(mode=gripper_mode)

    gripper_control_node.reset_gripper()
    time.sleep(1)

    gripper_control_node.activate_gripper()
    time.sleep(0.5)

    gripper_control_node.mode_select()
    time.sleep(0.5)

    rospy.Subscriber("extend_gripper_command", GripperControl, gripper_control_node.vr_gripper_command_callback)
    rospy.spin()


if __name__ == '__main__':
    main()
