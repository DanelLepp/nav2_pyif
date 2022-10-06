#!/bin/python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

def Concatenate(message):
    print("in python")
    new_data = message.data + "World"
    return new_data