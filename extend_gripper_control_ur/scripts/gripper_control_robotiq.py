#!/usr/bin/env python3

import rosnode
import rospy
import sys
import copy
import rospkg
import extend_msgs
from extend_msgs.msg import GripperControl
from std_msgs.msg import String
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
import time

#Creating the ros node and service client
rospy.init_node("ur_robotiq_gripper")

def initialize():
    pubRobotiqControl = rospy.Publisher('Robotiq2FGripperRobotOutput',Robotiq2FGripper_robot_output,queue_size=0)
    return pubRobotiqControl

def dataCallback(msg):
    gripperControlMsg = Robotiq2FGripper_robot_output()
    gripperControlMsg.rACT = 1
    gripperControlMsg.rGTO = 1
    gripperControlMsg.rSP = 255
    gripperControlMsg.rFR = 150
    gripperControlMsg.rPR = int(255 * msg.gripperAnalog.data)
    pubRobotiqControl.publish(gripperControlMsg)

if __name__ == '__main__':
    pubRobotiqControl = initialize()
    time.sleep(0.5)
    #Reset the Gripper
    gripperControlMsg = Robotiq2FGripper_robot_output()
    pubRobotiqControl.publish(gripperControlMsg)

    time.sleep(1)
    #Activate the Gripper
    gripperControlMsg.rACT = 1
    gripperControlMsg.rGTO = 1
    gripperControlMsg.rSP = 255
    gripperControlMsg.rFR = 150
    pubRobotiqControl.publish(gripperControlMsg)

    #Subscribe to Digital Gripper Data Stream from Unity
    rospy.Subscriber("extend_gripper_command", GripperControl, dataCallback)
    rospy.spin()
