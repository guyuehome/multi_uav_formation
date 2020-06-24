#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import sys, select, termios, tty
from numpy import *
import operator
from os import listdir
import time

twist= Twist()
pose=PoseStamped()
#Create Publisher

rospy.init_node('formation_publish')
r=rospy.Rate(1)
UAV=rospy.get_param('~UAV')
seq=rospy.get_param('~seq')
print(UAV+'command/pose')
print(seq)
num=seq
pose_pub = rospy.Publisher(UAV+'command/pose',PoseStamped,queue_size=1)

def __init__():
        pose.pose.position.x=0
        pose.pose.position.y=0
        pose.pose.position.z=0
        
def file2matrix(filename):
    fr = open(filename)
    numberOfLines = len(fr.readlines())         #get the number of lines in the file
    returnMat = zeros((numberOfLines,3))        #prepare matrix to return
    classLabelVector = []                       #prepare labels return   
    fr = open(filename)
    index = 0
    for line in fr.readlines():
        line = line.strip()
        listFromLine = line.split('\t')
        classLabelVector.append(int(listFromLine[0]))
        returnMat[index,:] = listFromLine[1:4]
        index += 1
    return returnMat,numberOfLines 
    
def odomCallback(msg):
    global num
    if msg.position.x>=pose.pose.position.x-0.1 and num<numberOfLines:
        num += 6
        time.sleep(0.01)
        pose.pose.position.x=dataMat[num][0]
        pose.pose.position.y=dataMat[num][1]*2
        pose.pose.position.z=dataMat[num][2]*0.5
        pose_pub.publish(pose)
       
#if __name__=="__main__":


#UAV='UAV1'

__init__();
dataMat,numberOfLines = file2matrix("/home/zdz/source/data/pos_data.txt")
#Create subscriber
odom_sub = rospy.Subscriber('odometry_sensor1/pose',Pose, odomCallback,queue_size = 1)

    
# spin
rospy.spin()


