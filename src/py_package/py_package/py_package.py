#!/bin/python3
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped

from std_msgs.msg import String

def Concatenate(message):
    new_data = message.data + "World"
    message.data = new_data
    return message

def ComputeVelocity(poseStamped, twist):
    start = time.time()
    # print(dir(message))
    # print(type(message))

    print(poseStamped)
    print(twist)
    
    twistStamped = TwistStamped()
    twistStamped.header = poseStamped.header
    twistStamped.twist = twist

    print(twistStamped)

    end = time.time()
    print("Function internal call time: {}".format((end - start)*1000000))
    return twistStamped