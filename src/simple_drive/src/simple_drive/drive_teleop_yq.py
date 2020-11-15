#!/usr/bin/python

import rospy
import subprocess

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID

class DriveTeleop:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher("teleop/cmd_vel", Twist, queue_size=1)
        self.goal_cancel_pub = rospy.Publisher("move_base/cancel", GoalID, queue_size=1)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.on_joy)

    def on_joy(self, data):
        # Set speed ratio using d-pad
        twist = Twist()
        if data.axes[7] == 1: # moving forward
            twist.linear.x = 1
        elif data.axes[7] == -1:
            twist.linear.x = -1
        if data.axes[6] == 1: # rotating left
            twist.angular.z = 1
        elif data.axes[6] == -1:
            twist.angular.z = -1

        # Stop
        if data.buttons[1]: # B button
            twist.linear.x = 0
            twist.angular.z = 0

        # Publish Twist
        self.cmd_vel_pub.publish(twist)

        # Cancel move base goal
        if data.buttons[2]: # X button
            rospy.loginfo('Cancelling move_base goal')
            cancel_msg = GoalID()
            self.goal_cancel_pub.publish(cancel_msg)

def main():
    rospy.init_node("drive_teleop")
    controller = DriveTeleop()
    rospy.spin()