#! /usr/bin/env python
'''
	Program to send and test the trajectories 

'''


import roslib; roslib.load_manifest('robotnik_torso_control')
import rospy
import actionlib
import time

from actionlib_msgs.msg import *
from trajectory_msgs.msg import *

from control_msgs.msg import *

if __name__ == '__main__':
	rospy.init_node('follow_trajectory_real_test')
	client = actionlib.SimpleActionClient('rt_traj_exe/follow_joint_trajectroy', FollowJointTrajectoryAction)
	client.wait_for_server()
	
	option = 9
	
	goal = FollowJointTrajectoryGoal()
	goal.trajectory.header.stamp = rospy.Time()
	goal.trajectory.joint_names = ['left_arm_4_joint', 'left_arm_2_joint', 'left_arm_1_joint', 'left_arm_3_joint']
	tpoint1 = JointTrajectoryPoint()
	tpoint1.positions = [0, 0.0, 0.0, 0.0]
	#tpoint1.velocities = [0.1, 0.1, 0.1, 0.1]
	tpoint1.velocities = [0.05, 0.05, 0.05, 0.05]
	tpoint1.accelerations = [0.1, 0.1, 0.1, 0.1]
	tpoint1.time_from_start = rospy.Duration.from_sec(5.0)
	
	tpoint2 = JointTrajectoryPoint()
	tpoint2.positions = [0.3, 0.31, 0.32, 0.93]
	tpoint2.velocities = [0.05, 0.05, 0.05, 0.05]
	tpoint2.accelerations = [0.1, 0.11, 0.12, 0.13]
	tpoint2.time_from_start = rospy.Duration.from_sec(5.0)
	
	
	goal.trajectory.points = [tpoint1, tpoint2]
	
	# Sends 3 trajs
	if option == 1:
		
		rospy.loginfo('OPTION 1: sending trajectory 1')
		# Fill in the goal here
		client.send_goal(goal)
		rospy.loginfo('waiting for result')
		while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
			rospy.loginfo('waiting for result. state = %s'%client.get_state())
		
		
			
		print 'Result is %s'%client.get_result()

		time.sleep(0.1)
		
		tpoint1.positions = [0.5, 1.0, 0.5, 0.5]
		tpoint2.positions = [0.6, 1.1, 0.6, 0.6]

		rospy.loginfo('sending trajectory 2')
		# Fill in the goal here
		client.send_goal(goal)
		rospy.loginfo('waiting for result')
		while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
			rospy.loginfo('waiting for result. state = %s'%client.get_state())
		print 'Result is %s'%client.get_result()
		
		time.sleep(0.1)
		
		tpoint1.positions = [-0.5, -1.0, -0.5, -0.5]
		tpoint2.positions = [-0.6, -1.1, -0.5, -0.5]
		
		rospy.loginfo('sending trajectory 3')
		# Fill in the goal here
		client.send_goal(goal)
		rospy.loginfo('waiting for result')
		while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
			rospy.loginfo('waiting for result. state = %s'%client.get_state())
		print 'Result is %s'%client.get_result()
	# Sends 1 traj and owerwrite it
	elif option == 2:
		tpoint1.positions = [0.1, 0.2, 0, 0.0]
		tpoint2.positions = [0.2, 0.3, 0.1, 0.1]
		rospy.loginfo('OPTION 2: Overwritting trajectories')
		# Fill in the goal here
		client.send_goal(goal)
		time.sleep(2)
		tpoint1.positions = [-0.1, -0.2, 0, 0.0]
		tpoint2.positions = [-0.2, -0.3, -0.1, -0.1]
		rospy.loginfo('overwrite')
		client.send_goal(goal)
		
		while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
			rospy.loginfo('waiting for result. state = %s'%client.get_state())
	
	# Sends 1 traj and cancell it
	elif option == 3:
		tpoint1.positions = [0.1, 0.2, 0, 0.0]
		tpoint2.positions = [0.2, 0.3, 0.5, 0.6]
		rospy.loginfo('OPTION 3: Cancelling trajectories')
		# Fill in the goal here
		client.send_goal(goal)
		time.sleep(2)
		rospy.loginfo('cancel')
		client.cancel_goal()
		
	# 
	elif option == 4:
		goal = FollowJointTrajectoryGoal()
		goal.trajectory.header.stamp = rospy.Time()
		goal.trajectory.joint_names = ['left_arm_1_joint', 'left_arm_2_joint', 'left_arm_3_joint', 'left_arm_4_joint', 'left_arm_5_joint', 'left_arm_6_joint']
		tpoint0 = JointTrajectoryPoint()
		tpoint0.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		tpoint0.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		tpoint0.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		
		goal.trajectory.points = [tpoint0]
		client.send_goal(goal)
		
		while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
			rospy.loginfo('waiting for result. state = %s'%client.get_state())
		
		time.sleep(2)

		tpoint1 = JointTrajectoryPoint()
		#tpoint1.positions = [0.4, 0.5, 0.6, 0.7]
		#tpoint1.positions = [0.33, 0.44, 0.55, 0.66, 0.77, 0.88]
		tpoint1.positions = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		
		#tpoint1.velocities = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
		tpoint1.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		tpoint1.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		
		tpoint2 = JointTrajectoryPoint()
		tpoint2.positions = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
		tpoint2.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		tpoint2.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		
		tpoint3 = JointTrajectoryPoint()
		tpoint3.positions = [0.6, 0.6, 0.6, 0.6, 0.6, 0.6]
		tpoint3.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		tpoint3.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		
		goal.trajectory.points = [tpoint1, tpoint2, tpoint3]
		client.send_goal(goal)
		
		while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
			rospy.loginfo('waiting for result. state = %s'%client.get_state())

		time.sleep(2)
		print 'going back'
		
		goal.trajectory.points = [tpoint3, tpoint2, tpoint1]		
		client.send_goal(goal)

	elif option == 5:
		goal = FollowJointTrajectoryGoal()
		goal.trajectory.header.stamp = rospy.Time()
		goal.trajectory.joint_names = ['right_arm_1_joint', 'right_arm_2_joint', 'right_arm_3_joint', 'right_arm_4_joint', 'right_arm_5_joint', 'right_arm_6_joint']
		tpoint0 = JointTrajectoryPoint()
		tpoint0.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		tpoint0.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		tpoint0.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		
		goal.trajectory.points = [tpoint0]
		client.send_goal(goal)
		
		while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
			rospy.loginfo('waiting for result. state = %s'%client.get_state())
		
		time.sleep(2)

		tpoint1 = JointTrajectoryPoint()
		#tpoint1.positions = [0.4, 0.5, 0.6, 0.7]
		#tpoint1.positions = [0.33, 0.44, 0.55, 0.66, 0.77, 0.88]
		tpoint1.positions = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		
		#tpoint1.velocities = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
		tpoint1.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		tpoint1.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		
		tpoint2 = JointTrajectoryPoint()
		tpoint2.positions = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
		tpoint2.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		tpoint2.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		
		tpoint3 = JointTrajectoryPoint()
		tpoint3.positions = [0.6, 0.6, 0.6, 0.6, 0.6, 0.6]
		tpoint3.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		tpoint3.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		
		goal.trajectory.points = [tpoint1, tpoint2, tpoint3]
		client.send_goal(goal)
		
		while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
			rospy.loginfo('waiting for result. state = %s'%client.get_state())

		time.sleep(2)
		print 'going back'
		
		goal.trajectory.points = [tpoint3, tpoint2, tpoint1]		
		client.send_goal(goal)
		
	elif option == 6:
		goal = FollowJointTrajectoryGoal()
		goal.trajectory.joint_names = ['right_arm_1_joint', 'right_arm_2_joint', 'right_arm_3_joint', 'right_arm_4_joint', 'right_arm_5_joint', 'right_arm_6_joint', 'left_arm_1_joint', 'left_arm_2_joint', 'left_arm_3_joint', 'left_arm_4_joint', 'left_arm_5_joint', 'left_arm_6_joint']
		
		for i in range(5):
			print 'Iteration %s'%i
			
			tpoint0 = JointTrajectoryPoint()
			tpoint0.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
			tpoint0.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
			tpoint0.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
				
			goal.trajectory.points = [tpoint0]
			goal.trajectory.header.stamp = rospy.Time()
			client.send_goal(goal)
			
			while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
				rospy.loginfo('waiting for result. state = %s'%client.get_state())
			
			time.sleep(2)

			tpoint1 = JointTrajectoryPoint()
			tpoint1.positions = [-0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5]
			tpoint1.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
			tpoint1.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
			
			tpoint2 = JointTrajectoryPoint()
			tpoint2.positions = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
			tpoint2.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
			tpoint2.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
			
			tpoint3 = JointTrajectoryPoint()
			tpoint3.positions = [0.5, -0.5, 1.7, -0.5, 0.4, -1.57, -0.137, -0.642, 1.353, -0.548, 0.307, 1.89]
			tpoint3.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
			tpoint3.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
			
			goal.trajectory.points = [tpoint1, tpoint2, tpoint3]
			goal.trajectory.header.stamp = rospy.Time()
			client.send_goal(goal)
			
			while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
				rospy.loginfo('waiting for result. state = %s'%client.get_state())

			time.sleep(2)
			print 'going back'
			
			goal.trajectory.points = [tpoint3, tpoint2, tpoint1]
			goal.trajectory.header.stamp = rospy.Time()		
			client.send_goal(goal)
			
			time.sleep(5)
	elif option == 7:
		goal = FollowJointTrajectoryGoal()
		goal.trajectory.header.stamp = rospy.Time()
		goal.trajectory.joint_names = ['head_pan_joint', 'head_tilt_joint']
		tpoint0 = JointTrajectoryPoint()
		tpoint0.positions = [1.3, 0.5]
		tpoint0.velocities = [0.1, 0.1]
		tpoint0.accelerations = [0.1, 0.1]
		
		goal.trajectory.points = [tpoint0]
		client.send_goal(goal)
		
		while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
			rospy.loginfo('waiting for result. state = %s'%client.get_state())
		
		time.sleep(2)
		tpoint0.positions = [-1.3, 0.5]
		client.send_goal(goal)
		
		while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
			rospy.loginfo('waiting for result. state = %s'%client.get_state())
	
	elif option == 8:
		
		for i in range(10):
			print 'It %d'%i
			goal.trajectory.header.stamp = rospy.Time()
			goal.trajectory.joint_names = ['head_pan_joint', 'head_tilt_joint']
			tpoint0 = JointTrajectoryPoint()
			tpoint0.positions = [1.3, 0.6]
			tpoint0.velocities = [0.15, 0.15]
			tpoint0.accelerations = [0.1, 0.1]
			
			goal.trajectory.points = [tpoint0]
			client.send_goal(goal)
			
			while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
				rospy.loginfo('waiting for result. state = %s'%client.get_state())
			
			time.sleep(2)
			tpoint0.positions = [-1.3, 0.6]
			client.send_goal(goal)
			
			while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
				rospy.loginfo('waiting for result. state = %s'%client.get_state())
			
			
			goal = FollowJointTrajectoryGoal()
			goal.trajectory.joint_names = ['left_arm_1_joint', 'left_arm_2_joint', 'left_arm_3_joint', 'left_arm_4_joint', 'left_arm_5_joint', 'left_arm_6_joint', 'right_arm_1_joint', 'right_arm_2_joint', 'right_arm_3_joint', 'right_arm_4_joint', 'right_arm_5_joint', 'right_arm_6_joint', 'head_pan_joint', 'head_tilt_joint']
			
			
			tpoint0 = JointTrajectoryPoint()
			tpoint0.positions =     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0]
			tpoint0.velocities =    [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.15, 0.16]
			tpoint0.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
				
			goal.trajectory.points = [tpoint0]
			goal.trajectory.header.stamp = rospy.Time()
			client.send_goal(goal)
			
			while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
				rospy.loginfo('waiting for result. state = %s'%client.get_state())
			
			time.sleep(2)
			'''
			tpoint1 = JointTrajectoryPoint()
			tpoint1.positions = [-0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5]
			tpoint1.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
			tpoint1.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
			
			tpoint2 = JointTrajectoryPoint()
			tpoint2.positions = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
			tpoint2.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
			tpoint2.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
			'''
			tpoint3 = JointTrajectoryPoint()
			tpoint3.positions = [0.35, -0.5, 1.7, -0.5, 0.4, -1.57, -0.127, -0.642, 1.353, -0.548, 0.307, 1.89, 0.0, 0.6]
			tpoint3.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.15, 0.15]
			tpoint3.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
			
			goal.trajectory.points = [tpoint3]
			goal.trajectory.header.stamp = rospy.Time()
			client.send_goal(goal)
			
			while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
				rospy.loginfo('waiting for result. state = %s'%client.get_state())

			time.sleep(2)
			
			tpoint3 = JointTrajectoryPoint()
			tpoint3.positions = [0.0, -1.199, 1.070, 1.449, 0.831, 1.827, -0.055, -1.373, 0.890, -1.181, 1.270, 1.893, 0.0, 0.6]
			tpoint3.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.3, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
			tpoint3.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
			
			goal.trajectory.points = [tpoint3]
			goal.trajectory.header.stamp = rospy.Time()
			client.send_goal(goal)
			
			while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
				rospy.loginfo('waiting for result. state = %s'%client.get_state())
			
			
			time.sleep(2)
			
			tpoint3 = JointTrajectoryPoint()
			tpoint3.positions = [-1.46, -1.725, 0, 0, 0, 0, -0.055, -1.373, 0.890, -1.181, 1.270, 1.893, 0.0, 0.6]
			tpoint3.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
			tpoint3.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
			
			goal.trajectory.points = [tpoint3]
			goal.trajectory.header.stamp = rospy.Time()
			client.send_goal(goal)
			
			while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
				rospy.loginfo('waiting for result. state = %s'%client.get_state())


			time.sleep(2)
			
			tpoint3 = JointTrajectoryPoint()
			tpoint3.positions = [-1.46, -1.725, 0, 0, 0, 0, 1.5, -1.76, 0, 0, 0, 0, 0.0, 0.6]
			tpoint3.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
			tpoint3.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
			
			goal.trajectory.points = [tpoint3]
			goal.trajectory.header.stamp = rospy.Time()
			client.send_goal(goal)
			
			while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
				rospy.loginfo('waiting for result. state = %s'%client.get_state())


			time.sleep(2)
			
			print 'going back'
			
			goal.trajectory.points = [tpoint0]
			goal.trajectory.header.stamp = rospy.Time()		
			client.send_goal(goal)
			
			time.sleep(5)	
	elif option == 9:
		
		goal = FollowJointTrajectoryGoal()
		goal.trajectory.header.stamp = rospy.Time()
		goal.trajectory.joint_names = ['waist_joint']
		tpoint0 = JointTrajectoryPoint()
		tpoint0.positions = [1.0]
		tpoint0.velocities = [0.2]
		tpoint0.accelerations = [0.2]
		print 'Sending traj to %s'%goal.trajectory.joint_names
		goal.trajectory.points = [tpoint0]
		client.send_goal(goal)
		
		while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
			rospy.loginfo('waiting for result. state = %s'%client.get_state())
		
		time.sleep(2)
		tpoint0.positions = [0.0]
		client.send_goal(goal)
		
		while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
			rospy.loginfo('waiting for result. state = %s'%client.get_state())

	exit()
