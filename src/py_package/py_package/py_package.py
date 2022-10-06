#!/bin/python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

def Concatenate(message):
    new_data = message.data + "World"
    message.data = new_data
    return message