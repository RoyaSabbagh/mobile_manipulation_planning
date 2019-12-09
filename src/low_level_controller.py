#!/usr/bin/env python

import rospy
from pam.msg import Roomba_cmd_vel
from pam.msg import Gripper_mode
from pam.msg import controlInput
'Low level control to send commands to the robot.'


def callback(data):
	global last_mode
        if data.mode == 0: # Start of the program, or infeasible times
            Gripper_mode = "Stay_put"
            Roomba_Linear = 0
            Roomba_Radius = 0

        elif data.mode == 1:  # PAM moves toward the object
            Gripper_mode = "Open"
            Roomba_Linear, Roomba_Radius = data.desired_input

        elif data.mode == 2:  # PAM is close to a leg and moves to grasp it
            Gripper_mode = "Open"
            Roomba_Linear, Roomba_Radius = data.desired_input

        elif data.mode == 3:  # Do the manipulatuion when grasped a leg
            Gripper_mode = "Close"
            Roomba_Linear, Roomba_Radius = data.desired_input

	elif data.mode == 4:  # Repositioning to avoid getting stuck
            Gripper_mode = "Open"
            Roomba_Linear, Roomba_Radius = data.desired_input

	elif data.mode == 5:  # Success
		Gripper_mode = "Open"
		Roomba_Linear, Roomba_Radius = data.desired_input

	pub1.publish(Gripper_mode)
        Roomba_vel=Roomba_cmd_vel()
        Roomba_vel.Linear = Roomba_Linear
        Roomba_vel.Radius = Roomba_Radius
        rospy.loginfo(Roomba_vel)
        pub2.publish(Roomba_vel)

if __name__ == '__main__':
    rospy.init_node('low_level_controller', anonymous=True)
    pub1 = rospy.Publisher('gripper_mode', Gripper_mode, queue_size=1)
    pub2 = rospy.Publisher('Roomba_vel', Roomba_cmd_vel, queue_size=1)
    rospy.Subscriber('controlInput', controlInput, callback)
    rospy.spin()
