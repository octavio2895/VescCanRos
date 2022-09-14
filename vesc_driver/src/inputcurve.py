#!/usr/bin/env python
import math
import rospy
from std_msgs.msg import Float64
import time

pub_speed = rospy.Publisher('motor_b/speed', Float64, queue_size=10)
pub_pos = rospy.Publisher('motor_b/position', Float64, queue_size=10)


rospy.init_node('inputcurve', anonymous=True)

rate = rospy.Rate(10)

i = 0


while not rospy.is_shutdown():
    pos_sp = 90*math.sin(time.time()*1)+180
    vel_sp = 30*math.cos(time.time()*1)
    rospy.loginfo("I publish:")
    rospy.loginfo(pos_sp)
    rospy.loginfo(vel_sp)
    pub_speed.publish(vel_sp)
    pub_pos.publish(pos_sp)
    rate.sleep()
    i = i + 1
