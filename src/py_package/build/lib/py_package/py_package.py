#!/bin/python3
import rclpy
from rclpy.node import Node

from geometry_msgs import PoseStamped, Twist, TwistStamped

from std_msgs.msg import String

def Concatenate(message):
    new_data = message.data + "World"
    message.data = new_data
    return message

def ComputeVelocity(message):
    x = message.pose.position
    y = message.pose.position
    z = message.pose.position
    print("recv PoseStamped, x={}, y={}, z={}".format(x, y, z))
    return message