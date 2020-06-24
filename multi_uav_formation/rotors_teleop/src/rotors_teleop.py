#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

import sys, select, termios, tty
# some paramters

msg = """

Control rotors!
---------------------------
Moving around:
        w    e 
   a    s    d
             c
]/[ : increase/decrease only linear speed by 10%
space key, k : force stop
anything else : stop smoothly
CTRL-C to quit
"""

moveBindings = {
        'w':(1,0,0),
        's':(-1,0,0),
        'a':(0,-1,0),    
        'd':(0,1,0),
        'e':(0,0,1),
        'c':(0,0,-1),
           }
speedBindings={
        '[':0.9,
        ']':1.1,
          }
# 获取键值
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

#获取行进速度和转向速度
def vels(speed):
    return "currently:\tspeed %s " % (speed)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('rotors_teleop')
    vel_pub = rospy.Publisher('/UAV/command/vel',Twist,   queue_size=1)

    vx = 0
    vy = 0
    vz = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    speed = 0.2
    target_speed = 0
    target_turn = 0
    control_speed_x = 0
    control_speed_y = 0
    control_speed_z = 0
    control_turn = 0
    move_style = 0
    laser_open = 0
    gun_fire = 0
    gun_pitch_angle = 185
    gun_yaw_angle=185
    mask = 0
    try:
        #打印提示信息
        print msg
        #打印速度和转向值
        print vels(speed)
        while(1):
            #获取键值
            key = getKey()
            # 运动控制方向键（1：正方向，-1负方向）da
            if key in moveBindings.keys():
                vx = moveBindings[key][0]
                vy = moveBindings[key][1]
                vz = moveBindings[key][2]
                count = 0
                
            # 速度修改键
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key]  # 线速度增加0.1倍
                count = 0
                print vels(speed)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
                
            # 停止键
            elif key == ' ' or key == 'k' :
                vx = 0
                vy = 0
                vz = 0

            else:
                '''
                count = count + 1
                if count > 30:
                    x = 0
                    y = 0
                    th = 0
                    gun_fire = 0
                '''
                if (key == '\x03'):
                    break

            target_speed_x = speed * vx
            target_speed_y = speed * vy
            target_speed_z = speed * vz
            
            # 速度限位，防止速度增减过快
            if target_speed_x > control_speed_x:
                control_speed_x = min( target_speed_x, control_speed_x + 10 )
            elif target_speed_x < control_speed_x:
                control_speed_x = max( target_speed_x, control_speed_x - 10 )
            else:
                control_speed_x = target_speed_x
        
            if target_speed_y > control_speed_y:
                control_speed_y = min( target_speed_y, control_speed_y + 10 )
            elif target_speed_y < control_speed_y:
                control_speed_y = max( target_speed_y, control_speed_y - 10 )
            else:
                control_speed_y = target_speed_y
                
            if target_speed_z > control_speed_z:
                control_speed_z = min( target_speed_z, control_speed_z + 10 )
            elif target_speed_z < control_speed_z:
                control_speed_z = max( target_speed_z, control_speed_z - 10 )
            else:
                control_speed_z = target_speed_z
            
            # 创建并发布twist消息
            
            twist= Twist()
            twist.linear.x = control_speed_x; 
            twist.linear.y = control_speed_y; 
            twist.linear.z = control_speed_z;
            vel_pub.publish(twist)

    except:
        print("error")

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        vel_pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
