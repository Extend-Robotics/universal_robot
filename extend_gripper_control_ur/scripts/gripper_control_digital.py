#!/usr/bin/env python3

import rosnode
import rospy
import sys
import copy
import rospkg
import extend_msgs
from extend_msgs.msg import GripperControl
from std_msgs.msg import String
import ur_msgs.srv

#Creating the ros node and service client
rospy.init_node("ur_gripper")
rospy.wait_for_service("ur_hardware_interface/set_io")

def dataCallback(msg):
    if(msg.gripperDigital.data):
        gripperValue = 1
    else:
        gripperValue = 0
    gripperPin = msg.gripperPin.data
    gripperControl = rospy.ServiceProxy("ur_hardware_interface/set_io", ur_msgs.srv.SetIO)
    gripperAction = gripperControl(1,gripperPin,gripperValue)
    
if __name__ == '__main__': 
    #Subscribe to Digital Gripper Data Stream from Unity   
    rospy.Subscriber("extend_gripper_command", GripperControl, dataCallback)
    rospy.spin() 
