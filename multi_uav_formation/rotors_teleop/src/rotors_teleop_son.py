#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import sys, select, termios, tty

twist= Twist()
pose=PoseStamped()
#Create Publisher
pose_pub = rospy.Publisher('command/pose',PoseStamped,queue_size=1)

def velocityCallback(msg):
    twist.linear.x=msg.linear.x
    twist.linear.y=msg.linear.y
    twist.linear.z=msg.linear.z

def odomCallback(msg):
    pose.pose.position.x=msg.position.x+twist.linear.x
    pose.pose.position.y=msg.position.y+twist.linear.y
    pose.pose.position.z=msg.position.z+twist.linear.z
    pose_pub.publish(pose)

    
#if __name__=="__main__":

rospy.init_node('motion_controller')
#Create subscriber
vel_sub = rospy.Subscriber('command/vel',Twist,velocityCallback,queue_size = 1)
odom_sub = rospy.Subscriber('odometry_sensor1/pose',Pose, odomCallback,queue_size = 1)

    
# spin
rospy.spin()


