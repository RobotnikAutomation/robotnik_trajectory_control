#!/usr/bin/env python
"""
	Software License Agreement (BSD License)

	Copyright (c) 2015 Robotnik Automation SLL. All Rights Reserved.

	Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

	2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
	   in the documentation and/or other materials provided with the distribution.

	3. The name of the author may not be used to endorse or promote products derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY ROBOTNIK AUTOMATION SLL "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
	AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
	OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
	AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
	EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
import roslib; roslib.load_manifest('robotnik_trajectory_control')
import rospy
import actionlib
from trajectory_msgs.msg import *
from control_msgs.msg import *
from actionlib_msgs.msg import *
from gripper_interface import StandardGripperInterface

class Rb1GripperInterface(StandardGripperInterface):
	"""
		Class to set the position of the Gripper
		Moves the joint through a FollowTrajectoryAction
	"""
	def __init__(self, joints, action_service):
		self.joints = joints
		self.action_service_name = action_service
		self.min_opening = rospy.get_param('~min_opening', 0.0)
		self.min_opening_joint_val_f1 = rospy.get_param('~min_opening_joint_val_f1', 0.0)
		self.min_opening_joint_val_f2 = rospy.get_param('~min_opening_joint_val_f2', -1.0)
		self.max_opening_joint_val_f1 = rospy.get_param('~max_opening_joint_val_f1', 1.0)
		self.max_opening_joint_val_f2 = rospy.get_param('~max_opening_joint_val_f2', 0.0)
		self.max_opening = rospy.get_param('~max_opening', 0.10)
		self.center = rospy.get_param('~center', 0.0)

		self.op_f1 = (self.max_opening - self.min_opening)/2.0
		self.op_f2 = self.op_f1
		self.range_f1 = self.max_opening_joint_val_f1 - self.min_opening_joint_val_f1
		self.range_f2 = self.max_opening_joint_val_f2 - self.min_opening_joint_val_f2

		self.as_client = actionlib.SimpleActionClient(self.action_service_name, FollowJointTrajectoryAction)

		if len(self.joints)!= 2:
			rospy.logerr('Rb1GripperInterface: incorrect number of joints %s'%str(self.joints))
			exit()
	
	def setup(self):
		"""
			Setups the component.
			Tries to connect the action server
			@return 0 if OK
			@return -1 if ERROR
		"""
		ret  = self.as_client.wait_for_server(timeout=rospy.Duration(2.0))
		if not ret:
			rospy.logerr('Rb1GripperInterface: Error waiting for server %s'%self.action_service_name)
			return -1
		
		return 0
		
		
	def setPositionGoal(self, position_goal):
		"""
			Sets the position of the gripper
			@param postion as GripperCommand
			@return 0 if it's send successfully
			@return -1 if ERROR		
		"""
		desired_position = position_goal.command.position
		if position_goal.command.position < self.min_opening:
			desired_position = self.min_opening
		elif position_goal.command.position > self.max_opening:
			desired_position = self.max_opening
		
		joints = self.convertGripperDistanceToJointValues(desired_position)
		
		#print 'Distance in rads = %s'%str(joints)
		
		goal = FollowJointTrajectoryGoal()
		goal.trajectory.header.stamp = rospy.Time()
		
		goal.trajectory.joint_names = [self.joints[0], self.joints[1]]
		tpoint1 = JointTrajectoryPoint()
		tpoint1.positions = [joints[0], joints[1]]
		tpoint1.velocities = [0.1, 0.1]
		tpoint1.accelerations = [0.1, 0.1]
		tpoint1.time_from_start = rospy.Duration.from_sec(5.0)
		goal.trajectory.points = [tpoint1]
		
		# Converts the desired position into a trajectory
		self.as_client.send_goal(goal)
		
		return 0
		
		
	def isGoalReached(self):
		"""
			Returns 0 if the position Goal is reached
			Returns -1 if the position hasn't been reached yet
			Returns -2 if it has been an error
		"""
		
		#result = self.as_client.get_result()
		#print 'isGoalReached: Result = %s'%str(result)
		state = self.as_client.get_state()
		#print 'isGoalReached: state = %s'%str(state)
		
		if state == GoalStatus.LOST or state == GoalStatus.PREEMPTED or state == GoalStatus.ABORTED:
			return -2
		if state == GoalStatus.PENDING or state == GoalStatus.ACTIVE:
			return -1			
		if state == GoalStatus.SUCCEEDED:
			return 0
		
		return 0	
		
		
	def cancelGoal(self):
		"""
			Cancels the current position
		"""
		self.as_client.cancel_goal()
		
		return 0
	
	def convertGripperDistanceToJointValues(self, distance):
		"""
			Converts the distance between gripper fingers in values of the joints linked with them
			@return [j1,j2] as a list of the joint values to reach this distance
		"""
		d_2 = distance/2.0
		j1 = self.min_opening_joint_val_f1 + d_2*(self.range_f1/self.op_f1)
		j2 = self.min_opening_joint_val_f2 + d_2*(self.range_f2/self.op_f2)
		
		return [j1, j2]
		
		
