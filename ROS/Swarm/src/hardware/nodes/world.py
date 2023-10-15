#!/usr/bin/env python3
# license removed for brevity
import rospy
import math
import copy
import tf
import numpy as np
import geometry_msgs
import std_msgs
from swarm_msgs.msg import Grid                  #http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
from swarm_msgs.msg import Plus                  #http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
from os import system

class Pheromone():
    def __init__(self):
        rospy.init_node('World', anonymous=True)
        self.sub_pher = rospy.Subscriber('/Pheromones_Write', Plus, self.callback)
        self.pub_pher = rospy.Publisher('/Pheromones_Read', Grid , queue_size=10)
        self.msg = Grid()
        self.init = rospy.Rate(1)
        self.init.sleep()
        self.init.sleep()
        self.init.sleep()
        self.widthOfSquare = 1
        self.heightOfSquare = self.widthOfSquare

    widthOfSquare = 1
    heightOfSquare = 1
    gridY = 0
    gridX = 0
    active = 0

    def callback(self, data):
        x = data.pose.position.x
        y = data.pose.position.y
        self.name = data.header.frame_id
        self.active = data.header.stamp.secs
        [self.gridX, self.gridY] = self.convert_to_grid(x,y)

    def talker(self):
        strength_max = 99        # number of seconds the pheromone lasts for
        strength_mid = 89
        strength_low = 79
        self.msg.column[0].row[0].pheromones[1] = 1
        if(self.active == 1):
            if(self.name == "Blue"):        # sets high strength in middle, mid strength adjacent, low strength at diagonals
                self.msg.column[self.gridY+0].row[self.gridX+0].pheromones[0] = strength_max
                self.msg.column[self.gridY+0].row[self.gridX+1].pheromones[0] = strength_mid
                self.msg.column[self.gridY+1].row[self.gridX+1].pheromones[0] = strength_low
                self.msg.column[self.gridY+1].row[self.gridX+0].pheromones[0] = strength_mid
                self.msg.column[self.gridY+1].row[self.gridX-1].pheromones[0] = strength_low
                self.msg.column[self.gridY+0].row[self.gridX-1].pheromones[0] = strength_mid
                self.msg.column[self.gridY-1].row[self.gridX-1].pheromones[0] = strength_low
                self.msg.column[self.gridY-1].row[self.gridX+0].pheromones[0] = strength_mid
                self.msg.column[self.gridY-1].row[self.gridX+1].pheromones[0] = strength_low
            if(self.name == "Charlie"):
                self.msg.column[self.gridY+0].row[self.gridX+0].pheromones[1] = strength_max
                self.msg.column[self.gridY+0].row[self.gridX+1].pheromones[1] = strength_mid
                self.msg.column[self.gridY+1].row[self.gridX+1].pheromones[1] = strength_low
                self.msg.column[self.gridY+1].row[self.gridX+0].pheromones[1] = strength_mid
                self.msg.column[self.gridY+1].row[self.gridX-1].pheromones[1] = strength_low
                self.msg.column[self.gridY+0].row[self.gridX-1].pheromones[1] = strength_mid
                self.msg.column[self.gridY-1].row[self.gridX-1].pheromones[1] = strength_low
                self.msg.column[self.gridY-1].row[self.gridX+0].pheromones[1] = strength_mid
                self.msg.column[self.gridY-1].row[self.gridX+1].pheromones[1] = strength_low
            if(self.name == "Delta"):
                self.msg.column[self.gridY+0].row[self.gridX+0].pheromones[2] = strength_max
                self.msg.column[self.gridY+0].row[self.gridX+1].pheromones[2] = strength_mid
                self.msg.column[self.gridY+1].row[self.gridX+1].pheromones[2] = strength_low
                self.msg.column[self.gridY+1].row[self.gridX+0].pheromones[2] = strength_mid
                self.msg.column[self.gridY+1].row[self.gridX-1].pheromones[2] = strength_low
                self.msg.column[self.gridY+0].row[self.gridX-1].pheromones[2] = strength_mid
                self.msg.column[self.gridY-1].row[self.gridX-1].pheromones[2] = strength_low
                self.msg.column[self.gridY-1].row[self.gridX+0].pheromones[2] = strength_mid
                self.msg.column[self.gridY-1].row[self.gridX+1].pheromones[2] = strength_low
            if(self.name == "Echo"):
                self.msg.column[self.gridY+0].row[self.gridX+0].pheromones[3] = strength_max
                self.msg.column[self.gridY+0].row[self.gridX+1].pheromones[3] = strength_mid
                self.msg.column[self.gridY+1].row[self.gridX+1].pheromones[3] = strength_low
                self.msg.column[self.gridY+1].row[self.gridX+0].pheromones[3] = strength_mid
                self.msg.column[self.gridY+1].row[self.gridX-1].pheromones[3] = strength_low
                self.msg.column[self.gridY+0].row[self.gridX-1].pheromones[3] = strength_mid
                self.msg.column[self.gridY-1].row[self.gridX-1].pheromones[3] = strength_low
                self.msg.column[self.gridY-1].row[self.gridX+0].pheromones[3] = strength_mid
                self.msg.column[self.gridY-1].row[self.gridX+1].pheromones[3] = strength_low
            if(self.name == "Foxtrot"):
                self.msg.column[self.gridY+0].row[self.gridX+0].pheromones[4] = strength_max
                self.msg.column[self.gridY+0].row[self.gridX+1].pheromones[4] = strength_mid
                self.msg.column[self.gridY+1].row[self.gridX+1].pheromones[4] = strength_low
                self.msg.column[self.gridY+1].row[self.gridX+0].pheromones[4] = strength_mid
                self.msg.column[self.gridY+1].row[self.gridX-1].pheromones[4] = strength_low
                self.msg.column[self.gridY+0].row[self.gridX-1].pheromones[4] = strength_mid
                self.msg.column[self.gridY-1].row[self.gridX-1].pheromones[4] = strength_low
                self.msg.column[self.gridY-1].row[self.gridX+0].pheromones[4] = strength_mid
                self.msg.column[self.gridY-1].row[self.gridX+1].pheromones[4] = strength_low

        self.pub_pher.publish(self.msg)


    def decay(self):
        for i in range(10):
            for j in range(10):
                for k in range(5):
                    if(self.msg.column[i].row[j].pheromones[k] > 0):
                        self.msg.column[i].row[j].pheromones[k] -= 1




    def convert_to_grid(self, x, y):    #TODO convert so gives correctly rounded numbers
        xConv = x/self.widthOfSquare
        yConv = y/self.heightOfSquare

        return [int(xConv), int(yConv)]


if __name__ == '__main__':

    scent = Pheromone()

    hz = 40
    i = 0

    rate = rospy.Rate(hz) # 40hz
    while not rospy.is_shutdown():
        # _ = system('clear')
        scent.talker()
        if(i % hz == 0):   #once per second 
            scent.decay()

        i += 1
        rate.sleep()
