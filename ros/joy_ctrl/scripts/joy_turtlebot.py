#!/usr/bin/env python
# encoding: utf-8
import time
import rospy
import getpass
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyCtrl:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.user_name = getpass.getuser()
        self.linear_speed = 0
        self.angular_speed = 0
        self.Joy_state = False
        self.velocity = Twist()
        self.rate = rospy.Rate(20)
        self.Joy_time = time.time()
        self.linear_speed_limit = rospy.get_param('~linear_speed_limit', 0.22)
        self.angular_speed_limit = rospy.get_param('~angular_speed_limit', 2.0)
        self.pub_cmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_Joy = rospy.Subscriber('/joy', Joy, self.buttonCallback)

    def buttonCallback(self, joy_data):
        if not isinstance(joy_data, Joy): return
        self.Joy_time = time.time()
        # rospy.loginfo(joy_data)
        # rospy.loginfo(joy_data.axes)
        if self.user_name == "jetson":
            self.linear_speed = joy_data.axes[1] * self.linear_speed_limit
            self.angular_speed = joy_data.axes[2] * self.angular_speed_limit
        else:
            self.linear_speed = joy_data.axes[1] * self.linear_speed_limit
            self.angular_speed = joy_data.axes[3] * self.angular_speed_limit
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.pub_cmdVel.publish(twist)

    def cancel(self):
        self.pub_cmdVel.unregister()
        self.sub_Joy.unregister()


if __name__ == '__main__':
    rospy.init_node('joy_turtlebot')
    joy = JoyCtrl()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('exception')
