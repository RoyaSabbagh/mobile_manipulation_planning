#!/usr/bin/env python


import rospy
import message_filters
from pam.msg import feedback, force_reading
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from time import time
from Examples_real import environment
'Reading feedback from motion capture and publishing it as feedback massage.'

def call_pos(walker, PAM, bed, blue_chair, gray_chair, cart):
        # publishing to "feedback" for manipulation planning
        global walker_state_old
        r = rospy.Rate(30)
        quaternion_walker = [walker.pose.orientation.x, walker.pose.orientation.y, walker.pose.orientation.z, walker.pose.orientation.w]
        (theta_walker,psi_walker,phi_walker) = euler_from_quaternion(quaternion_walker)
        new_time = time()
        dt = new_time - walker_state_old[3]
        xdot_walker = (walker.pose.position.x+1 - walker_state_old[0])/dt
        ydot_walker = (walker.pose.position.y+1 - walker_state_old[1])/dt
        phidot_walker = (phi_walker - walker_state_old[2])/dt
        walker_state_old = [walker.pose.position.x+1, walker.pose.position.y+1, phi_walker, new_time]
        fb = feedback()
        fb.walker_state.x = walker.pose.position.x+1
        fb.walker_state.y = walker.pose.position.y+1
        fb.walker_state.phi = phi_walker
        fb.walker_state.xdot = xdot_walker
        fb.walker_state.ydot = ydot_walker
        fb.walker_state.phidot = phidot_walker

        global PAM_state_old
        quaternion_PAM = [PAM.pose.orientation.x, PAM.pose.orientation.y, PAM.pose.orientation.z, PAM.pose.orientation.w]
        (theta_PAM,psi_PAM,phi_PAM) = euler_from_quaternion(quaternion_PAM)
        xdot_PAM = (PAM.pose.position.x+1 - PAM_state_old[0])/dt
        ydot_PAM = (PAM.pose.position.y+1 - PAM_state_old[1])/dt
        phidot_PAM = (phi_PAM - PAM_state_old[2])/dt
        PAM_state_old = [PAM.pose.position.x+1, PAM.pose.position.y+1, phi_PAM, new_time]
        fb.PAM_state.x = PAM.pose.position.x+1
        fb.PAM_state.y = PAM.pose.position.y+1
        fb.PAM_state.phi = phi_PAM
        fb.PAM_state.xdot = xdot_PAM
        fb.PAM_state.ydot = ydot_PAM
        fb.PAM_state.phidot = phidot_PAM

        global bed_state_old
        quaternion_bed = [bed.pose.orientation.x, bed.pose.orientation.y, bed.pose.orientation.z, bed.pose.orientation.w]
        (theta_bed,psi_bed,phi_bed) = euler_from_quaternion(quaternion_bed)
        xdot_bed = (bed.pose.position.x+1 - bed_state_old[0])/dt
        ydot_bed = (bed.pose.position.y+1 - bed_state_old[1])/dt
        phidot_bed = (phi_bed - bed_state_old[2])/dt
        bed_state_old = [bed.pose.position.x+1, bed.pose.position.y+1, phi_bed, new_time]
        fb.bed_state.x = bed.pose.position.x+1
        fb.bed_state.y = bed.pose.position.y+1
        fb.bed_state.phi = phi_bed
        fb.bed_state.xdot = xdot_bed
        fb.bed_state.ydot = ydot_bed
        fb.bed_state.phidot = phidot_bed

        global blue_chair_state_old
        quaternion_blue_chair = [blue_chair.pose.orientation.x, blue_chair.pose.orientation.y, blue_chair.pose.orientation.z, blue_chair.pose.orientation.w]
        (theta_blue_chair,psi_blue_chair,phi_blue_chair) = euler_from_quaternion(quaternion_blue_chair)
        xdot_blue_chair = (blue_chair.pose.position.x+1 - blue_chair_state_old[0])/dt
        ydot_blue_chair = (blue_chair.pose.position.y+1 - blue_chair_state_old[1])/dt
        phidot_blue_chair = (phi_blue_chair - blue_chair_state_old[2])/dt
        blue_chair_state_old = [blue_chair.pose.position.x+1, blue_chair.pose.position.y+1, phi_blue_chair, new_time]
        fb.blue_chair_state.x = blue_chair.pose.position.x+1
        fb.blue_chair_state.y = blue_chair.pose.position.y+1
        fb.blue_chair_state.phi = phi_blue_chair
        fb.blue_chair_state.xdot = xdot_blue_chair
        fb.blue_chair_state.ydot = ydot_blue_chair
        fb.blue_chair_state.phidot = phidot_blue_chair

        global gray_chair_state_old
        quaternion_gray_chair = [gray_chair.pose.orientation.x, gray_chair.pose.orientation.y, gray_chair.pose.orientation.z, gray_chair.pose.orientation.w]
        (theta_gray_chair,psi_gray_chair,phi_gray_chair) = euler_from_quaternion(quaternion_gray_chair)
        xdot_gray_chair = (gray_chair.pose.position.x+1 - gray_chair_state_old[0])/dt
        ydot_gray_chair = (gray_chair.pose.position.y+1 - gray_chair_state_old[1])/dt
        phidot_gray_chair = (phi_gray_chair - gray_chair_state_old[2])/dt
        gray_chair_state_old = [gray_chair.pose.position.x+1, gray_chair.pose.position.y+1, phi_gray_chair, new_time]
        fb.gray_chair_state.x = gray_chair.pose.position.x+1
        fb.gray_chair_state.y = gray_chair.pose.position.y+1
        fb.gray_chair_state.phi = phi_gray_chair
        fb.gray_chair_state.xdot = xdot_gray_chair
        fb.gray_chair_state.ydot = ydot_gray_chair
        fb.gray_chair_state.phidot = phidot_gray_chair

        global cart_state_old
        quaternion_cart = [cart.pose.orientation.x, cart.pose.orientation.y, cart.pose.orientation.z, cart.pose.orientation.w]
        (theta_cart,psi_cart,phi_cart) = euler_from_quaternion(quaternion_cart)
        xdot_cart = (cart.pose.position.x+1 - cart_state_old[0])/dt
        ydot_cart = (cart.pose.position.y+1 - cart_state_old[1])/dt
        phidot_cart = (phi_cart - cart_state_old[2])/dt
        cart_state_old = [cart.pose.position.x+1, cart.pose.position.y+1, phi_cart, new_time]
        fb.cart_state.x = cart.pose.position.x+1
        fb.cart_state.y = cart.pose.position.y+1
        fb.cart_state.phi = phi_cart
        fb.cart_state.xdot = xdot_cart
        fb.cart_state.ydot = ydot_cart
        fb.cart_state.phidot = phidot_cart

        env = environment()
        env.example_2()
        fb.walker_goal = env.object_goal
        pub.publish(fb)

if __name__ == '__main__':
    walker_state_old = [0, 0, 0, 0]
    PAM_state_old = [0, 0, 0, 0]
    bed_state_old = [0, 0, 0, 0]
    blue_chair_state_old = [0, 0, 0, 0]
    gray_chair_state_old = [0, 0, 0, 0]
    cart_state_old = [0, 0, 0, 0]
    try:
        # *** read feedbacks here ***
        rospy.init_node('Feedback', anonymous=True)
        pub = rospy.Publisher('feedback', feedback, queue_size=1)
        walker_sub = message_filters.Subscriber('/vrpn_client_node/walker/pose', PoseStamped)
        PAM_sub = message_filters.Subscriber('/vrpn_client_node/pam/pose', PoseStamped)
        bed_sub = message_filters.Subscriber('/vrpn_client_node/bed/pose', PoseStamped)
        gray_chair_sub = message_filters.Subscriber('/vrpn_client_node/gray_chair/pose', PoseStamped)
        blue_chair_sub = message_filters.Subscriber('/vrpn_client_node/blue_chair/pose', PoseStamped)
        cart_sub = message_filters.Subscriber('/vrpn_client_node/cart/pose', PoseStamped)
        ts = message_filters.ApproximateTimeSynchronizer([walker_sub,PAM_sub, bed_sub, blue_chair_sub, gray_chair_sub, cart_sub], 2, 0.05)
        ts.registerCallback(call_pos)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
