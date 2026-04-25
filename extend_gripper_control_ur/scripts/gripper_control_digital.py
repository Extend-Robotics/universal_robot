#!/usr/bin/env python3

import rosnode
import rospy
import sys
import copy
import rospkg
import extend_msgs
from extend_msgs.msg import GripperControl, GripperResponse
from std_msgs.msg import Header
import ur_msgs.srv
import os



def initialize():
    #Initialize the Modbus service and the response publisher
    pubGripperCommandRepublisher = rospy.Publisher('extend_gripper_republished_command',GripperControl,queue_size=1)
    pubGripperResponse = rospy.Publisher('extend_gripper_response',GripperResponse,queue_size=1)
    return pubGripperCommandRepublisher,pubGripperResponse

def dataCallback(msg):
    if(msg.gripper_digital.data):
        gripperValue = 1
    else:
        gripperValue = 0
    gripper_pin = msg.gripper_pin.data
    gripperControl = rospy.ServiceProxy(setIOServiceName, ur_msgs.srv.SetIO)
    gripperAction = gripperControl(1,gripper_pin,gripperValue)
    header = Header()
    header.seq = 0
    header.frame_id = ""
    header.stamp = rospy.Time.now()

    pubGripperCommandRepublisherData = GripperControl()
    pubGripperCommandRepublisherData = msg
    pubGripperCommandRepublisherData.header = header
    pubGripperCommandRepublisher.publish(pubGripperCommandRepublisherData)

    #Fetching the Gripper Response Joint States and Force
    pubGripperResponseData = GripperResponse()
    pubGripperResponseData.header = header
    pubGripperResponse.publish(pubGripperResponseData)

    
if __name__ == '__main__': 
    #Creating the ros node and service client
    rospy.init_node("ur_gripper")
    setIOServiceName = os.environ['ROS_NAMESPACE']  + "/ur_hardware_interface/set_io"

    rospy.wait_for_service(setIOServiceName)
    (pubGripperCommandRepublisher,pubGripperResponse) = initialize()   

    #Subscribe to Digital Gripper Data Stream from Unity  
    rospy.Subscriber("extend_gripper_command", GripperControl, dataCallback, queue_size=1)
    rospy.spin() 
