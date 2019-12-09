#!/usr/bin/env python

import rospy
from pam.msg import Gripper_mode
from motor_current_drive import *
import RPI.GPIO as GPIO


Enable_pin = 3
Direction_pin = 4
PWM_pin = 12

GPIO.setmode(GPIO.BCM)
GPIO.setup(Enable_pin, GPIO.OUT)
GPIO.setup(Direction_pin, GPIO.OUT)
GPIO.setup(PWM_pin, GPIO.OUT)
p = GPIO.PWM(PWM_pin, 40)  # channel=12 frequency=50Hz

Direction = 1
GPIO.output(Direction_pin, Direction)

p.start(0)

def ON():
        Enable = 1
        GPIO.output(Enable_pin, Enable)
        time.sleep(2)
        dc = 40
        p.ChangeDutyCycle(dc)
        time.sleep(2)
        
def OFF():
        Enable = 0
        GPIO.output(Enable_pin, Enable)
        time.sleep(2)
        
def CLOSE():
        dc = 80
        p.ChangeDutyCycle(dc)
        time.sleep(2)
        
def OPEN():
        dc = 20
        p.ChangeDutyCycle(dc)
        time.sleep(2)
        dc = 40
        p.ChangeDutyCycle(dc)
        time.sleep(2)

def STAY_PUT(dc):
        p.ChangeDutyCycle(dc)
        time.sleep(2)

def callback(data,gripper):
    if data.mode == "Close":
        rospy.loginfo("closing...")
        CLOSE()
    elif data.mode == "Open":
        rospy.loginfo("opening")
        OPEN()
    elif data.mode == "Off":
        rospy.loginfo("Gripper off")
        OFFs()
    else:
        rospy.loginfo("staying put...")
        STAY_PUT(40)
           
def gripper_control():
    ON()
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('gripper_control', anonymous=True)

    rospy.Subscriber('gripper_mode', Gripper_mode, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    gripper_control()
