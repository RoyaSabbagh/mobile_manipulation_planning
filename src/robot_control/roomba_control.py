#!/usr/bin/env python

import rospy
from pam.msg import Roomba_cmd_vel
from breezycreate2 import Robot
import time

# Create a Create2. This will automatically try to connect to your robot over serial
bot = Robot()



def callback(data):
    if 1 > data.Radius > 0:
        radius = (1-data.Radius)*(-500)

    elif -1 < data.Radius < 0:
        radius = (1+data.Radius)*500
        
    elif data.Radius == 1:
        radius = -1
    
    elif data.Radius == -1:
        radius = 1
               
    else:
        radius = 32767
    bot.setcurlSpeed(data.Linear*(-250), radius)
    rospy.loginfo(radius)        
        
def roomba_control():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('roomba_control', anonymous=True)

    rospy.Subscriber('Roomba_vel', Roomba_cmd_vel, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    roomba_control()
