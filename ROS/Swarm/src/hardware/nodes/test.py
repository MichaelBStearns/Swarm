#!/usr/bin/env python3
# license removed for brevity
import rospy
import math
import copy
import tf
import numpy as np
from Pose import geometry_msgs
from os import system

class Pheromone():
    def __init__(self):
        rospy.init_node('World', anonymous=True)
        self.sub1 = rospy.Subscriber('/Robot', geometry_msgs.Pose, self.callback)
        # pub_drive = rospy.Publisher('/cmd_vel', std_msgs.msg.Bool, queue_size=10)
        print("init")
        self.init = rospy.Rate(1)
        self.init.sleep()

    def callback(self, data):
        print(data)


if __name__ == '__main__':

    scent = Pheromone()

    rate = rospy.Rate(40) # 40hz
    while not rospy.is_shutdown():
        # _ = system('clear')


        rate.sleep()
