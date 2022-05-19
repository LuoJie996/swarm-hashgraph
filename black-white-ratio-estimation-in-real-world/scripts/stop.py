#!/usr/bin/env python
from geometry_msgs.msg import Twist
import rospy
from std_msgs.msg import Int32MultiArray
import time,random


def main():
    rospy.init_node('avoid_obstacle', anonymous = False)
    velocity = 0
    angular_speed = 0
    twist = Twist()
    twist.linear.x = velocity
    pub_stop = rospy.Publisher('mobile_base/cmd_vel_stop', Twist, queue_size = 1)    
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        pub_stop.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    main()
