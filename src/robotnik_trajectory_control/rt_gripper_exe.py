#!/usr/bin/env python
"""
	@file rt_gripper_exe.py
	
	ROS Component t
	
	Subscribes:
		- 
		
	Publishes:
		- 
		
	Actions:
		- GripperCommandAction
	
	@author: Robotnik Automation
	
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
from control_msgs.msg import *
from actionlib_msgs.msg import *

from gripper_interfaces.rb1_gripper_interface import *


WAITING_STATE = -1
IDLE_STATE = 0
ACTION_STATE = 1


class GripperCommandActionController:
	"""
		Class to manage the execution the action service GripperCommand
	"""	
	def __init__(self):
		self.state = WAITING_STATE
		
		try:
			self.name = rospy.get_param('~name', default = 'gripper')
			self.model = rospy.get_param('~model', default = 'rb1_gripper1')
			self.joints = rospy.get_param('~joints', default = ['j1', 'j2'])
			self.action_service_client = rospy.get_param('~action_service', default = '')
			self.desired_freq = rospy.get_param('~desired_freq', default = 10.0)
			
		except rospy.ROSException, e:
			rospy.logerr('%s: error getting params %s'%(rospy.get_name(), e))
			exit()

		## AVAILABLE MODELS
		if self.model == 'rb1_gripper1':
			self.gripper = Rb1GripperInterface(self.joints, self.action_service_client)
		else:
			rospy.logerr('%s: Gripper model %s does not exist'%(rospy.get_name(), self.model))
			exit()

		self.gripper_command_action_server = actionlib.SimpleActionServer('~gripper_action', GripperCommandAction, None, False)
		

	def controlLoop(self):
		"""
			Runs the control loop
		"""
		t_sleep = 1.0/self.desired_freq
		
		while not rospy.is_shutdown():
			
			if self.state == WAITING_STATE:
				if self.gripper.setup() == 0:
					rospy.loginfo('%s: Gripper setup OK'%rospy.get_name())
					self.state = IDLE_STATE
					t_sleep = 1.0/self.desired_freq
				else:
					t_sleep = 2.0
								
			elif self.state == IDLE_STATE:
				#print 'IDLE'
				if self.gripper_command_action_server.is_new_goal_available():
					if self.processNewGoal(self.gripper_command_action_server.accept_new_goal()) == 0:
						self.state = ACTION_STATE
				
			elif self.state == ACTION_STATE:
				#print 'ACTION'
				if self.gripper_command_action_server.is_preempt_requested():
					self.gripper.cancelGoal()
					self.gripper_command_action_server.set_preempted()
					self.state = IDLE_STATE
				else:
					state = self.gripper.isGoalReached()
					if state == 0:						
						self.gripper_command_action_server.set_succeeded()
						self.state = IDLE_STATE
					elif state == -2:
						self.gripper_command_action_server.set_aborted()
						self.state = IDLE_STATE
					else:
						rospy.loginfo('%s: executing gripper action'%rospy.get_name())
			
			rospy.sleep(t_sleep)
				
		
	def processNewGoal(self, new_goal):
		"""
			Process new action goal
		"""
		rospy.loginfo('%s: New goal -> position =  %.2f, max_effort = %.2f'%(rospy.get_name(), new_goal.command.position, new_goal.command.max_effort))	
		ret = self.gripper.setPositionGoal(new_goal)
		#print 'ret = %d'%ret
		
		return 0
			
			
	def start(self):
		"""
			Starts the action server and runs spin
		"""	
		try:
			rospy.loginfo('%s: starting server'%rospy.get_name())
			self.gripper_command_action_server.start()
			self.controlLoop()
		except rospy.ROSInterruptException:
			rospy.loginfo('%s: Bye!'%rospy.get_name())


def main():

	rospy.init_node('robotnik_gripper_controller')
		
	gripper_node = GripperCommandActionController()
	

	gripper_node.start()
	

	
if __name__=='__main__':
	main()
	exit()
	
