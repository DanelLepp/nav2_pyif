#!/bin/python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped

from std_msgs.msg import String

def Concatenate(message):
    new_data = message.data + "World"
    message.data = new_data
    return message

def ComputeVelocity(message):
    print("ComputeVelocity call")
    print(message)
    print(dir(message))
    print(type(message))
    x = message.pose.position.x
    print("x succeeded")
    y = message.pose.position.y
    z = message.pose.position.z
    print("recv PoseStamped, x={}, y={}, z={}".format(x, y, z))
    return message