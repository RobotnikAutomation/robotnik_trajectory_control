#! /usr/bin/env python

import roslib; roslib.load_manifest('robotnik_trajectory_control')
import rospy
import actionlib
import time
import sys

from actionlib_msgs.msg import *

from control_msgs.msg import *

if __name__ == '__main__':
	rospy.init_node('gripper_command_test')
	client = actionlib.SimpleActionClient('rt_gripper_exe/gripper_action', GripperCommandAction)
	client.wait_for_server()
	
	if len(sys.argv) < 2:
		position = 1.0
		print 'Selected position by default %f'%position
	else:
		position = float(sys.argv[1])
		print 'Selected position %f'%position
	
	goal = GripperCommandGoal()
	#goal.command.header.stamp = rospy.Time()
	
	goal.command.position = position
	goal.command.max_effort = 0.0
	
	rospy.loginfo('sending command')
	# Fill in the goal here
	client.send_goal(goal)
	rospy.loginfo('waiting for result')
	while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
		rospy.loginfo('waiting for result. state = %s'%client.get_state())
			
	
