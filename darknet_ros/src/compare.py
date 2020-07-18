#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64

from geometry_msgs.msg import Pose
global x
global y
global pub
def input1(data):
	global x
	x = data.linear.x
	print x

def input2(data):
	global y
	global pub
	global x
	y = data.orientation.x
	print y
	if(x == y):
		print "okk"
		pub.publish(data)

def compare():
	global pub
	rospy.init_node('compare1',anonymous=True)
	rospy.Subscriber('/id1',Twist, input1)
	rospy.Subscriber('/darknet_ros/points',Pose, input2)
	pub = rospy.Publisher('/check',Pose ,queue_size=10)
	rospy.spin()

if __name__=='__main__':
	compare()
