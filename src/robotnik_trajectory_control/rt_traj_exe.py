#!/usr/bin/env python
"""
	@file rt_traj_exe.py
	
	ROS Component to follow trajectories communicating with different ROS component controllers
	Every controlled component has to provide the command topic type JointState
	
	Subscribes:
		- /joint_states
		
	Publishes:
		- controller_x/joint_states
		
	Actions:
		- FollowJointTrajectoryAction
	
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
import wx
import xml.dom.minidom
from sensor_msgs.msg import JointState
from math import pi
from threading import Thread, Timer
import string
import time

#from ipa_canopen_command_interface import *
from interfaces.device_command_interface import DeviceCommandInterface
from interfaces.gazebo_command_interface import GazeboPositionCommandInterface

#from robotnik_trajectory_msgs.msg import State, Actions
from robotnik_msgs.msg import State
from robotnik_trajectory_control.msg import Actions
from robotnik_trajectory_control.msg import SubcomponentState
from control_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robotnik_trajectory_control.msg import State as ControlState
from robotnik_trajectory_control.srv import TrajExecActions, GetJointsLimits
import std_msgs.msg as std_msgs

DEFAULT_FREQ = 100.0
MAX_FREQ = 500.0

# Substates for READY_STATE
IDLE_SUBSTATE = 23
ACTIVE_SUBSTATE = 24
PAUSE_SUBSTATE = 25
CANCEL_SUBSTATE = 26
# Substates for INIT_STATE
INITIALIZE_COMPONENT = 33
INITIALIZE_SUBCOMPONENTS = 34
# 
DEFAULT_ERROR_JOINT_POSITION = 0.0009 # rads
#self.error_joint_position_ = 0.000872665
DEFAULT_JOINT_READ_STATE_TIMEOUT = 0.1 # seconds
# Once the trajectory is initialized, there's a timeout if no joint variation is received
DEFAULT_JOINT_NO_MOVEMENT_TIMEOUT = 2.0 #seconds
# joint distance to change of trajectory point
DEFAULT_JOINT_LOOKAHEAD = 0.09

def get_param(name, value=None):
	'''
		Function from joint_state_subscriber
		@param name: name of the param toget
		@return the param or the default value
	'''
	
	private = "~%s" % name
	if rospy.has_param(private):
		return rospy.get_param(private)
	elif rospy.has_param(name):
		return rospy.get_param(name)
	else:
		return value

# Represents an internal Trajectory folliwng point
#
class TrajectoryGoalExecution:
	'''
	Contains the info and data necessary to perform the trajectory execution
	'''
	_fields_ = [ ("joint_name", string),          
				 ("joint_traj_points", JointTrajectoryPoint), 
				 ("joint_params",  None),  # Ej. {'value': 0, 'upper': 3.14, 'lower': -3.14, 'last_update_time': time.time(), 'joint_update_freq': Hz} params from robot_description
				 ("joint_state",  None),  # Ej. {'position': 0.0, 'velocity': 0.0, 'effort': 0.0} values extracted from /joint_state
				 ("current_point", int),
				 ("current_time", float),   
				 ("finished", bool)]     

	def __init__(self, desired_freq, joint_read_state_timeout):
		self.current_point = 0
		self.current_time = 0.0
		self.finished = False
		self.len_points = 0	# number of points
		self.diff_position = 1000 # difference between current and current desired position 
		if desired_freq <= 0:
			desired_freq  = 1.0
		self.joint_state_timeout = joint_read_state_timeout	# Sets timeout that will be trigger if joint_state value is not received.
		if self.joint_state_timeout <= 0:
			self.joint_state_timeout = DEFAULT_JOINT_READ_STATE_TIMEOUT
		
	def updateLen(self):
		'''
			Updates the value of length points
		'''
		self.len_points = len(self.joint_traj_points.positions)
			
	def finish(self):
		'''
			Sets the flag finished to True to indicate that this joint has reached its position
		'''
		self.finished = True
		
	def morePoints(self):
		'''
			Checks whether or not there are more points in the trajectory
			@return True if there are, False otherwise
		'''
		if self.current_point < self.len_points - 1:
			return True
		else:
			return False
	
	def updateDiffPosition(self, diff):
		self.diff_position = diff
		
	def nextPoint(self):
		'''
			Move to next point in the array and updates the diff position variable
		'''
		if self.morePoints():
			self.current_point = self.current_point + 1
			self.updateDiffPosition(abs(self.joint_state['position'] - self.joint_traj_points.positions[self.current_point]))
			self.diff_position = 1000			
	
	def isJointUpdated(self):
		'''
			@return: True if the value is updated at the correct frequency, False otherwise
		'''
		if self.joint_state['last_update_time'] is not None:
			#diff = (rospy.Time.now() - self.joint_state['last_update_time']).to_sec() 
			diff = time.time() - self.joint_state['last_update_time']
			if diff > self.joint_state_timeout:
				#print 'Joint %s, diff = %.3f, max = %.3f'%(self.joint_name, diff, self.joint_state_timeout)
				return False	# There's a timeout
			else:
				return True
			
		else:
			return False # No time saved, so there's a timeout

		
	
# Class to simulate the trajectory execution
class TrajExec:
	'''
		Offers the action service FollowJointTrajectoryAction
	'''
		
	def __init__(self, args):
		
		self.node_name = rospy.get_name().replace('/','')
		self.desired_freq = args['desired_freq'] 
		self.error_joint_position_ = args['error_joint_position']
		self.joint_read_state_timeout = args['joint_read_state_timeout']
		self.joint_no_movement_timeout = args['joint_no_movement_timeout']
		self.ignore_acceleration = args['ignore_acceleration']
		self.joint_lookahead_ = args['joint_lookahead']
		# Checks value of freq
		if self.desired_freq <= 0.0 or self.desired_freq > MAX_FREQ:
			rospy.loginfo('%s::init: Desired freq (%f) is not possible. Setting desired_freq to %f'%(self.node_name,self.desired_freq, DEFAULT_FREQ))
			self.desired_freq = DEFAULT_FREQ
	
	
		self.real_freq = 0.0
		
		# Saves the state of the component
		self.state = State.INIT_STATE
		# Saves the previous state
		self.previous_state = State.INIT_STATE
		# Substate of READY STATE
		self.ready_substate = IDLE_SUBSTATE
		# Substate for INIT STATE
		self.init_substate = INITIALIZE_COMPONENT
		
		# flag to control the initialization of the component
		self.initialized = False
		# flag to control the initialization of ROS stuff
		self.ros_initialized = False
		# flag to control that the control loop is running
		self.running = False
		# Variable used to control the loop frequency
		self.time_sleep = 1.0 
		# State msg to publish
		self.msg_state = ControlState()
		# Timer to publish state
		self.publish_state_timer = 0.2
		
		self.t_publish_state = Timer(self.publish_state_timer, self.publishROSstate)
		
		# Dict to save the actions request. It's based on Actions.msg definition
		# It's set in actionServiceCb
		'''self.requested_actions = {Actions.PAUSE: False, Actions.CONTINUE: False, Actions.CANCEL: False, Actions.INIT: False, Actions.INIT_ARMS: False, 
		Actions.INIT_HEAD: False, Actions.INIT_WAIST: False, Actions.INIT_RIGHT_GRIPPER: False, Actions.INIT_LEFT_GRIPPER: False, Actions.INIT_FINISHED: False,
		Actions.RECOVER: False}'''
		self.requested_actions = {Actions.PAUSE: False, Actions.CONTINUE: False, Actions.CANCEL: False}
			
	def setup(self):
		'''
			Initializes de hand
			@return: True if OK, False otherwise
		'''
		self.initialized = True
		
		return 0
		
	def rosSetup(self):
		'''
			Creates and inits ROS components
		'''
		if self.ros_initialized:
			return 0
		
		# Gets and saves robot description	
		description = get_param('robot_description')
		
		if description is None:
			rospy.logerr('%s::rosSetup: robot description not published'%self.node_name)
			
			return -1
			
		robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
		self.free_joints = {}
		self.joint_list = [] # for maintaining the original order of the joints
		self.dependent_joints = get_param("dependent_joints", {})
		
		# Find all non-fixed joints
		for child in robot.childNodes:
			if child.nodeType is child.TEXT_NODE:
				continue
			if child.localName == 'joint':
				jtype = child.getAttribute('type')
				if jtype == 'fixed':
					continue
				name = child.getAttribute('name')
				if jtype == 'continuous':
					minval = -pi
					maxval = pi
				else:
					limit = child.getElementsByTagName('limit')[0]
					minval = float(limit.getAttribute('lower'))
					maxval = float(limit.getAttribute('upper'))

				if name in self.dependent_joints:
					continue
				if minval > 0 or maxval < 0:
					zeroval = (maxval + minval)/2
				else:
					zeroval = 0

				joint = {'min':minval, 'max':maxval, 'zero':zeroval, 'value':zeroval }
				self.free_joints[name] = joint
				self.joint_list.append(name)
				# Sets these values into ROS param server
				rospy.set_param('/joints/%s'%(name), joint)

		source_list = get_param("source_list", [])
		self.sources = []
		for source in source_list:
			self.sources.append(rospy.Subscriber(source, JointState, self.source_cb))
		
		# Gets and process device commands interface groups
		try:
			if rospy.search_param('groups'): 
				groups = rospy.get_param('groups')
			else:
				rospy.logerr('%s::rosSetup: Param groups not found!'%self.node_name)
				return -1
		except rospy.ROSException, e:
			rospy.logerr('%s::rosSetup %s'%(self.node_namee, e))
			return -1
		
		# array with all the command interfaces created
		self.command_interfaces_dict = {}
		
		for i in groups:
			type_group = groups[i]['type']
			# Schunk arm
			if type_group == 'ipa_can_open':
				d = IpaCANOpenCommandInterface( args = groups[i])
			# BHand
			elif type_group == 'bhand':
				d = BHandCommandInterface( args = groups[i])
			# WSG50
			elif type_group == 'wsg50_gripper':
				d = WSG50CommandInterface( args = groups[i])
			# PW70
			elif type_group == 'pw70':
				d = PW70CommandInterface( args = groups[i])
			# Gazebo Position
			elif type_group == 'gazebo_position':
				d = GazeboPositionCommandInterface(args = groups[i])
			# Others
			else:
				d = DeviceCommandInterface( args = groups[i])
			
			if d.setup() == 0:
				if self.command_interfaces_dict.has_key(d.name):
					rospy.logerr('%s::rosSetup: Error on the interface name %s:%s already exist. Discarded'%(self.node_name, d.type, d.name))
				else:
					self.command_interfaces_dict[d.name] = d
			else:
				rospy.logerr('%s::rosSetup: Error on Command Interface setup. %s:%s'%(self.node_name, type_group, d.name))
		
		# If there are no command interfaces setup is aborted		
		if len(self.command_interfaces_dict) == 0:
			rospy.logerr('%s::rosSetup: No device command interfaces defined'%(self.node_name))	
			return -1		
			
		# dict to save the current joint state value. Easier to access than standard JointState msg
		# {'joint_1':{'position': 0.0, 'velocity': 0.0, 'effort'}
		self.joint_state = {}
		# Copy of joint state, used to the position increment
		# It will have extra params to allow the control
		# Ex. {'position': float(joint['value']), 'velocity': 0.0, 'effort': 0.0, 'last_update_time': None, 'command_interface': None}
		self.desired_joint_state = {}
		t_now = time.time()
		# Inits joint states
		for (name,joint) in self.free_joints.items():
			self.joint_state[name] = {'position': float(joint['value']), 'velocity': 0.0, 'effort': 0.0, 'last_update_time': t_now, 'joint_update_freq': 0.0}
			self.desired_joint_state[name] = {'position': float(joint['value']), 'velocity': 0.0, 'effort': 0.0, 'last_update_time': None, 'command_interface': None}
			# For state publish
			#self.msg_state.controlled_joints.append(name)
			#self.msg_state.update_freq_joints.append(0.0)
			
		# Link every Command Interface with its configured joints
		for interface in self.command_interfaces_dict:
			for joint in self.command_interfaces_dict[interface].joint_names:
				# Check that the joint name exists
				if self.desired_joint_state.has_key(joint):
					# Assigns the name of the interface inside the command_interfaces_dict object
					print 'rosSetup: linking joint %s with interface %s'%(joint, interface)
					self.desired_joint_state[joint]['command_interface'] = interface
				else:
					rospy.logerr('%s::rosSetup: Joint name %s on %s:%s command interface is not defined'%(self.node_name, joint, self.command_interfaces_dict[interface].type, self.command_interfaces_dict[interface].name))	
					return -1	
		
		# Structure to save the current trajectory being followed
		# field 'points' contains an array of 'TrajectoryGoalExecution'(one per each joint involved on the execution),made from the last FollowTrajectoryAction goal received
		# field 'finished' indicates whether the trajectory is finished or not
		self.current_follow_traj_goal = { 'points': [], 'finished': True}
				
		
		# Publishers
		self.joint_state_subscriber = rospy.Subscriber('joint_states', JointState, self.receiveJointStateCb)
		self._state_publisher = rospy.Publisher('%s/state'%self.node_name, ControlState)
		
		# Services
		self._actions_service = rospy.Service('%s/actions'%self.node_name, TrajExecActions, self.actionsServiceCb)
		# Action Server
		self.follow_traj_action_server = actionlib.SimpleActionServer('%s/follow_joint_trajectory'%self.node_name, FollowJointTrajectoryAction, None, False)
		self._get_joints_limits_service = rospy.Service('%s/get_joints_limits'%self.node_name, GetJointsLimits, self.getJointsLimitsServiceCb)
		
		self.ros_initialized = True
		
		self.publishROSstate()
		
		return 0
		
		
	def shutdown(self):
		'''
			Shutdowns device
			@return: 0 if it's performed successfully, -1 if there's any problem or the component is running
		'''
		if self.running or not self.initialized:
			return -1
		rospy.loginfo('%s::shutdown'%self.node_name)
		
		# Cancels current timers
		self.t_publish_state.cancel()
		
		
		
			
		self.initialized = False
		
		return 0
	
	
	def rosShutdown(self):
		'''
			Shutdows all ROS components
			@return: 0 if it's performed successfully, -1 if there's any problem or the component is running
		'''
		if self.running or not self.ros_initialized:
			return -1
		
		rospy.loginfo('%s::rosShutdown'%self.node_name)
		# cancels any ongoing trajectory, 
		if not self.current_follow_traj_goal['finished']:
			# Cancels current trajectory
			self.cancelCurrentTraj()
			self.follow_traj_action_server.set_aborted()
		
		# Shutdowns all command interfaces
		for interface in self.command_interfaces_dict:
			self.command_interfaces_dict[interface].shutdown()
		
		# Performs ROS topics & services shutdown
		self._state_publisher.unregister()
		self.joint_state_subscriber.unregister()
		
		self.ros_initialized = False
		
		return 0
			
	
	def stop(self):
		'''
			Creates and inits ROS components
		'''
		self.running = False
		
		return 0
	
	
	def start(self):
		'''
			Runs ROS configuration and the main control loop
			@return: 0 if OK
		'''
		
		if self.running:
			return 0
			
		self.running = True
		
		self.controlLoop()
		
		return 0
	
	
	def controlLoop(self):
		'''
			Main loop of the component
			Manages actions by state
		'''
		
		while self.running and not rospy.is_shutdown():
			t1 = time.time()
			
			if self.state == State.INIT_STATE:
				self.initState()
				
			elif self.state == State.STANDBY_STATE:
				self.standbyState()
				
			elif self.state == State.READY_STATE:
				self.readyState()
				
			elif self.state == State.EMERGENCY_STATE:
				self.emergencyState()
				
			elif self.state == State.FAILURE_STATE:
				self.failureState()
				
			elif self.state == State.SHUTDOWN_STATE:
				self.shutdownState()
				
			self.allState()
			
			t2 = time.time()
			tdiff = (t2 - t1)
			
			
			t_sleep = self.time_sleep - tdiff
			
			try:
				
				if t_sleep > 0.0:
					rospy.sleep(t_sleep)
				else:
					rospy.logerr('%s::controlLoop: not reaching desired freq (%f)'%(self.node_name, self.desired_freq))
				
				t3= time.time()
				self.real_freq = 1.0/(t3 - t1)
			except rospy.exceptions.ROSInterruptException, e:
				rospy.loginfo(e)
				
			
		
		self.running = False
		# Performs component shutdown
		self.shutdownState()
		# Performs ROS shutdown
		self.rosShutdown()
		rospy.loginfo('%s::controlLoop: exit control loop'%self.node_name)
		
		return 0
		
	
	
	def rosPublish(self):
		'''
			Publish topics at standard frequency
		'''
		
		return 0
		
	
	
	def initState(self):
		'''
			Actions performed in init state
		'''
		
		# Initializes the component
		if self.init_substate == INITIALIZE_COMPONENT:
			
			if not self.initialized:
				self.setup()
			
			if not self.ros_initialized:
				self.rosSetup()
				
			if self.initialized and self.ros_initialized:
				rospy.loginfo('%s:InitState: Starting FollowJointTraj action server'%self.node_name)
				self.follow_traj_action_server.start()
				self.switchToState(State.STANDBY_STATE)
		
		
		"""
				# Moving substate
				self.init_substate = INITIALIZE_SUBCOMPONENTS
		
		# Initializes subcomponents		
		elif self.init_substate == INITIALIZE_SUBCOMPONENTS:
			# Depending on the signal received it will initialize each component
			if self.requested_actions[Actions.INIT_ARMS]:
				#rospy.loginfo('%s:InitState: Inititializing arms'%self.node_name)
				self.command_interfaces_dict['right_arm'].initialize()
				self.command_interfaces_dict['left_arm'].initialize()
				
				self.requested_actions[Actions.INIT_ARMS] = False
			
			if self.requested_actions[Actions.INIT_HEAD]:
				#rospy.loginfo('%s:InitState: Inititializing head'%self.node_name)
				self.command_interfaces_dict['head'].initialize()
				self.requested_actions[Actions.INIT_HEAD] = False
			
			if self.requested_actions[Actions.INIT_WAIST]:
				#rospy.loginfo('%s:InitState: Inititializing waist'%self.node_name)
				self.command_interfaces_dict['waist'].initialize()
				self.requested_actions[Actions.INIT_WAIST] = False
			
			if self.requested_actions[Actions.INIT_RIGHT_GRIPPER]:
				#rospy.loginfo('%s:InitState: Inititializing right gripper'%self.node_name)
				self.command_interfaces_dict['right_gripper'].initialize()
				self.requested_actions[Actions.INIT_RIGHT_GRIPPER] = False
			
			if self.requested_actions[Actions.INIT_LEFT_GRIPPER]:
				#rospy.loginfo('%s:InitState: Inititializing left gripper'%self.node_name)
				self.requested_actions[Actions.INIT_LEFT_GRIPPER] = False
				self.command_interfaces_dict['left_gripper'].initialize()
				
			if self.requested_actions[Actions.INIT_FINISHED]:
				self.requested_actions[Actions.INIT_FINISHED] = False
				self.switchToState(State.STANDBY_STATE)
			
			if self.follow_traj_action_server.is_new_goal_available():
				#rospy.loginfo('%s:InitState: Received goal while in INIT. Aborted'%self.node_name)
				goal = self.follow_traj_action_server.accept_new_goal()
				self.follow_traj_action_server.set_aborted()
			
		if self.requested_actions[Actions.RECOVER]:
			self.requested_actions[Actions.RECOVER] = False
			self.recoverComponents()
		"""
				
		return
	
	def standbyState(self):
		'''
			Actions performed in standby state
		'''
		self.switchToState(State.READY_STATE)
		
		return
	
	
	def checkJointsMovement(self, joint_name, desired_pos, current_pos, diff):
		'''
			Checks that the joints are moving to the desired position.
			In case that the movement is stuck, it'll detect it
		'''
		
		
		#self.check_joints_movement_table = {'joint_xx':{'desired_pos': 1.2, 'current_pos': 1.1, 'diff': 0.1, 't_last_diff': time}}
		
		# Check that the structure exists
		if hasattr(self, 'check_joints_movement_table'):
			if self.check_joints_movement_table.has_key(joint_name):
				last_diff = self.check_joints_movement_table[joint_name]['diff']
				
				if abs(desired_pos - current_pos) > self.error_joint_position_ and last_diff == diff:
					
					# Firs time detected
					if self.check_joints_movement_table[joint_name]['t_last_diff'] is None:
						self.check_joints_movement_table[joint_name]['t_last_diff'] = time.time()
					else:
					# Check time without variation
						t_diff = time.time() - self.check_joints_movement_table[joint_name]['t_last_diff']
						#print 'checking'
						if t_diff >= self.joint_no_movement_timeout:
							rospy.logerr('%s:checkJointsMovement: Following error on joint %s. %.3f secs without variation on position (current = %f, desired = %f)'%(self.node_name, joint_name, t_diff, current_pos, desired_pos))
							
							# ERROR EN FOLLOWING
							return False
						#else:
							#self.check_joints_movement_table[joint_name]['diff'] = diff	
							#self.check_joints_movement_table[joint_name]['t_last_diff'] = time.time()	
				else:
					self.check_joints_movement_table[joint_name]['diff'] = diff	
					self.check_joints_movement_table[joint_name]['t_last_diff'] = time.time()	
			# insert the joint in the table
			else:
				self.check_joints_movement_table[joint_name] = {'diff': diff, 't_last_diff': None}
		else:
			# First call
			self.check_joints_movement_table = {}
			self.check_joints_movement_table[joint_name] = {'diff': diff, 't_last_diff': None}
	
		return True
	
	def resetCheckJointsMovement(self):
		'''
			Resets the timeout checkout 
		'''
		if hasattr(self, 'check_joints_movement_table'):
			self.check_joints_movement_table = {}
		
		
	def readyState(self):
		'''
			Actions performed in ready state
		'''
		#
		# ACTIVE 
		if self.ready_substate == ACTIVE_SUBSTATE:
			# variable to control the end of the trajectory
			traj_finish = True
		
			# go through all the trajectory points and updates desired references
			for i in self.current_follow_traj_goal['points']:
				# check the flag of finished (active when reaching the position)
				if not i.finished:
					
					# Checks that it's receiving the current value of joint states
					# A delay in the reception of controlled joints is not allowed
					if not i.isJointUpdated():
						rospy.logerr('%s:readyState: ERROR. joint_state of %s is not being received'%(self.node_name, i.joint_name))
						rospy.loginfo('%s:readyState: switching from ACTIVE to CANCEL substate'%self.node_name)
						self.ready_substate = CANCEL_SUBSTATE
						return 
					# diff between current joint position and desired value
					diff = abs(i.joint_state['position'] - i.joint_traj_points.positions[i.current_point])
					# TEST: Checking that the joints are moving	
					if not self.checkJointsMovement(i.joint_name, i.joint_traj_points.positions[i.current_point], i.joint_state['position'], diff):
						rospy.logerr('%s:readyState: ERROR executing the trajectory. Joint %s cannot reach the position'%(self.node_name, i.joint_name))
						rospy.loginfo('%s:readyState: switching from ACTIVE to CANCEL substate'%self.node_name)
						self.ready_substate = CANCEL_SUBSTATE
						return 
					
					# There are more points to follow
					if i.morePoints():
						
						#print 'joint_state = %.3f'%i.joint_state['position']	
						# if the difference is greater than the last cycle or is lower than the error joint pos,  we change the point objective  
						if diff > i.diff_position:
							print 'joint %s, point %d reached (%.3f). Diff = %.3lf  (last diff = %.3lf)'%(i.joint_name, i.current_point, i.joint_state['position'], diff, i.diff_position)	
							i.nextPoint()
						elif diff <= self.joint_lookahead_:
							#print 'joint %s, point %d: reached'%(i.joint_name, i.current_point)	
							print 'joint %s, point %d reached (%.3f). Diff = %.3lf (error = %.3lf)'%(i.joint_name, i.current_point, i.joint_state['position'], diff, self.error_joint_position_)	
							i.nextPoint()
											
						else:
							i.updateDiffPosition(diff)						
						traj_finish = False
					else:
						
						#print 'joint_state = %.3f'%i.joint_state['position']
						diff = abs(i.joint_state['position'] - i.joint_traj_points.positions[i.current_point])
						
						# TEST: Checking that the joints are moving	
						if not self.checkJointsMovement(i.joint_name, i.joint_traj_points.positions[i.current_point], i.joint_state['position'], diff):
							rospy.logerr('%s:readyState: ERROR executing the trajectory. Joint %s cannot reach the position'%(self.node_name, i.joint_name))
							rospy.loginfo('%s:readyState: switching from ACTIVE to CANCEL substate'%self.node_name)
							self.ready_substate = CANCEL_SUBSTATE
							return
							
						# look for the end of trajectory
						if diff > i.diff_position or diff <= self.error_joint_position_:
							print 'joint %s, end point %d reached (%.3f). Diff = %.3lf, diff_pos = %.3lf'%(i.joint_name, i.current_point, i.joint_state['position'],  diff, i.diff_position)
							i.finish()
						else:
							traj_finish = False
				
			# send position and velocity refs to each joint / component
			if self.sendCommands() != 0:
				rospy.logerr('%s:readyState: error executing the trajectory'%self.node_name)
				rospy.loginfo('%s:readyState: switching from ACTIVE to CANCEL substate'%self.node_name)
				self.ready_substate = CANCEL_SUBSTATE
			 
			
			self.current_follow_traj_goal['finished'] = traj_finish

			
			if self.current_follow_traj_goal['finished']:
				self.follow_traj_action_server.set_succeeded()
				self.ready_substate = IDLE_SUBSTATE
				self.resetCheckJointsMovement()
				rospy.loginfo('%s:readyState: switching from ACTIVE to IDLE substate'%self.node_name)
			
		#
		# PAUSE	
		elif self.ready_substate == PAUSE_SUBSTATE:
			
			pass
			
		#
		# CANCEL
		elif self.ready_substate == CANCEL_SUBSTATE:
			rospy.loginfo('%s:readyState: cancelling trajectory'%self.node_name)
			# Stops current movements
			self.cancelCurrentTraj()
			self.follow_traj_action_server.set_preempted()
			self.ready_substate = IDLE_SUBSTATE
		
		#
		# Actions in all substates
		if self.follow_traj_action_server.is_new_goal_available():
			#if self.ready_substate == ACTIVE_SUBSTATE:
				# CANCEL?
			#	rospy.loginfo('%s:readyState: New goal available when substate is active'%self.node_name)
				
			if self.ready_substate == IDLE_SUBSTATE:
				goal = self.follow_traj_action_server.accept_new_goal()
				rospy.loginfo('%s:readyState: New goal available'%self.node_name)
				#print goal
				if self.processNewGoal(goal) == 0:
					self.ready_substate = ACTIVE_SUBSTATE
					rospy.loginfo('%s:readyState: switching from IDLE to ACTIVE substate'%self.node_name)
				else:
					self.follow_traj_action_server.set_aborted()
		
		if self.follow_traj_action_server.is_preempt_requested():
			if self.ready_substate == ACTIVE_SUBSTATE:
				# If there's an action running we'll replace the action avoiding to cancel and stop movement
				if self.follow_traj_action_server.is_new_goal_available():
					goal = self.follow_traj_action_server.accept_new_goal()
					#rospy.loginfo('%s:readyState: New goal available while in ACTIVE substate'%self.node_name)
					#print goal
					if self.processNewGoal(goal) != 0:
						self.ready_substate = CANCEL_SUBSTATE	
						rospy.logerr('%s:readyState: Error processing new goal in ACTIVE substate. Switching from ACTIVE to CANCEL substate'%self.node_name)
				else:
					self.ready_substate = CANCEL_SUBSTATE	
					rospy.loginfo('%s:readyState: preempted requested. Switching from ACTIVE to CANCEL substate'%self.node_name)
		'''if self.ready_substate == ACTIVE_SUBSTATE:
				self.ready_substate = CANCEL_SUBSTATE	
				rospy.loginfo('%s:readyState: preempted requested. Switching from ACTIVE to CANCEL substate'%self.node_name)
		'''
		# 
		# Checks for required service actions
		i = self.getPendingActions()
		while i is not None:
			if i == Actions.PAUSE:
				if self.ready_substate == ACTIVE_SUBSTATE:
					rospy.loginfo('%s:readyState: switching from ACTIVE to PAUSE substate'%self.node_name)
					self.pauseCurrentTraj()
					self.ready_substate = PAUSE_SUBSTATE
			elif i == Actions.CONTINUE:
				if self.ready_substate == PAUSE_SUBSTATE:
					rospy.loginfo('%s:readyState: switching from PAUSE to ACTIVE substate'%self.node_name)
					self.ready_substate = ACTIVE_SUBSTATE
			elif i == Actions.CANCEL:
				if self.ready_substate == PAUSE_SUBSTATE or self.ready_substate == ACTIVE_SUBSTATE:
					rospy.loginfo('%s:readyState: switching to CANCEL substate'%self.node_name)
					self.ready_substate = CANCEL_SUBSTATE
			"""elif i == Actions.INIT:
					rospy.loginfo('%s:readyState: switching to INIT state'%self.node_name)
					self.switchToState(State.INIT_STATE)
			
			if i == Actions.RECOVER:
				self.recoverComponents()
				self.requested_actions[Actions.RECOVER] = False
			"""	
			i = self.getPendingActions()
				
		return
	
	def shutdownState(self):
		'''
			Actions performed in shutdown state 
		'''
		if self.shutdown() == 0:
			self.switchToState(State.INIT_STATE)
		
		return
	
	def emergencyState(self):
		'''
			Actions performed in emergency state
		'''
		
		return
	
	def failureState(self):
		'''
			Actions performed in failure state
		'''
		
			
		return
	
	def switchToState(self, new_state):
		'''
			Performs the change of state
		'''
		if self.state != new_state:
			self.previous_state = self.state
			self.state = new_state
			rospy.loginfo('%s::switchToState: %s'%(self.node_name, self.stateToString(self.state)))
			if self.previous_state == State.INIT_STATE:
				self.time_sleep = 1.0 / self.desired_freq
		
		return
		
	def allState(self):
		'''
			Actions performed in all states
		'''
		if self.ros_initialized:
			self.rosPublish()
		
		return
	
	
	def stateToString(self, state):
		'''
			@param state: state to convert
			@type state: State
			@returns the equivalent string of the state
		'''
		if state == State.INIT_STATE:
			return 'INIT_STATE'
				
		elif state == State.STANDBY_STATE:
			return 'STANDBY_STATE'
			
		elif state == State.READY_STATE:
			return 'READY_STATE'
			
		elif state == State.EMERGENCY_STATE:
			return 'EMERGENCY_STATE'
			
		elif state == State.FAILURE_STATE:
			return 'FAILURE_STATE'
			
		elif state == State.SHUTDOWN_STATE:
			return 'SHUTDOWN_STATE'
		else:
			return 'UNKNOWN_STATE'
	
	
	def substateToString(self, substate):
		'''
			@param substate: substate to switch to
			@type substate: int
			@returns the equivalent string of the substate
		'''
		if substate == ACTIVE_SUBSTATE:
			return 'ACTIVE_SUBSTATE'
				
		elif substate == PAUSE_SUBSTATE:
			return 'PAUSE_SUBSTATE'
			
		elif substate == CANCEL_SUBSTATE:
			return 'CANCEL_SUBSTATE'
			
		elif substate == IDLE_SUBSTATE:
			return 'IDLE_SUBSTATE'
			
		elif substate == INITIALIZE_COMPONENT:
			return 'INITIALIZE_COMPONENT'
			
		elif substate == INITIALIZE_SUBCOMPONENTS:
			return 'INITIALIZE_SUBCOMPONENTS'
			
		else:
			return 'UNKNOWN_SUBSTATE'
			
	
	def source_cb(self, msg):

		for i in range(len(msg.name)):
			name = msg.name[i]
			position = msg.position[i]
			if name in self.free_joints:
				joint = self.free_joints[name]
				joint['value'] = position
		
	def publishROSstate(self):
		'''
			Publish the State of the component at the desired frequency
		'''
		self.msg_state.state.state = self.state
		self.msg_state.state.state_description = self.stateToString(self.state)
		self.msg_state.state.desired_freq = self.desired_freq
		self.msg_state.state.real_freq = self.real_freq
		
		if self.state == State.INIT_STATE:
			self.msg_state.substate = self.substateToString(self.init_substate)
		if self.state == State.READY_STATE:
			self.msg_state.substate = self.substateToString(self.ready_substate)
			
		#Subcomponents state
		components_array = []
		for i in self.command_interfaces_dict:
			subcomponent = SubcomponentState()
			subcomponent.name = i	
			subcomponent.type = self.command_interfaces_dict[i].type
			subcomponent.state = self.command_interfaces_dict[i].getState()
			subcomponent.state_description = self.stateToString(subcomponent.state)
			'''subcomponent.type = self.command_interfaces_dict[i]['type']
			subcomponent.state = self.command_interfaces_dict[i]['state']
			subcomponent.state_description = self.stateToString(self.command_interfaces_dict[i]['state'])
			'''
			components_array.append(subcomponent)
		
		self.msg_state.components = components_array	
		
		joints_freq = []
		joint_names = []
		for joint in self.joint_state:
			joint_names.append(joint)
			joints_freq.append('%.2f'%self.joint_state[joint]['joint_update_freq'])
		
		self.msg_state.update_freq_joints = joints_freq
		self.msg_state.controlled_joints = joint_names		
			
		self._state_publisher.publish(self.msg_state)
		
		self.t_publish_state = Timer(self.publish_state_timer, self.publishROSstate)
		self.t_publish_state.start()
	
		
	
	def processNewGoal(self, new_goal):
		'''
			Process new trajectory goal
			@param new_goal: new trajectory goal
			@type new_goal: FollowJointTrajectoryGoal
			@return: 0 if OK, -1 if ERROR
		'''	
		# trajectory_msgs/JointTrajectoryPoint
		# Header header
		# string[] joint_names
		# JointTrajectoryPoint[] points
		new_traj = []
		
		
		if len(new_goal.trajectory.joint_names) == 0 or len(new_goal.trajectory.points) == 0:
			rospy.logerr('%s:processNewGoal: error on new goal parameters. Joint names or points wrong size'%self.node_name)
			return -1
		
		#[ ("joint_name", string),          
		#("joint_traj_points", JointTrajectoryPoint), 
		# ("joint_params",  None),          
		# ("current_point",    int),
		# ("current_time", float)
		num_of_joints = len(new_goal.trajectory.joint_names)
		for i in range(num_of_joints):
				
			#print 'joint %s'%new_goal.trajectory.joint_names[i]
			if new_goal.trajectory.joint_names[i] in self.free_joints:
				
				
				n = TrajectoryGoalExecution(self.desired_freq, self.joint_read_state_timeout)
				n.joint_name = new_goal.trajectory.joint_names[i] # name of the joint
				n.joint_state = self.joint_state[n.joint_name]	# joint_state of this joint
				n.joint_params = self.free_joints[n.joint_name] # joint params of this joint
				n.joint_traj_points = JointTrajectoryPoint()
				
				for points in new_goal.trajectory.points:
					# Checks that the format of the msg is correct
					if len(points.positions) != num_of_joints:
						rospy.logerr('%s:processNewGoal: positions have different length than the number of joints'%self.node_name)	
						return -1
					if len(points.velocities) != num_of_joints:
						rospy.logerr('%s:processNewGoal: velocities have different length than the number of joints'%self.node_name)	
						return -1
					if len(points.accelerations) != num_of_joints:
						rospy.logerr('%s:processNewGoal: accelerations have different length than the number of joints'%self.node_name)	
						return -1
					
					# Saves the related position, vel, acc of this joint and put it together in one array
					n.joint_traj_points.positions.append(points.positions[i])
					n.joint_traj_points.velocities.append(abs(points.velocities[i]))
					n.joint_traj_points.accelerations.append(abs(points.accelerations[i]))
				
				n.joint_traj_points.time_from_start = points.time_from_start
				n.updateLen()
				
				#n.joint_traj_points = new_goal.trajectory.points[i] # desired trajectory
				
				#self.joint_state[n.joint_name]['position'] = 2.0
				#print 'name = %s, current value = %f, points = %s'%(n.joint_name, n.joint_params['value'], n.joint_traj_points)
				#print 'name = %s, points = %s, vel = %s'%(n.joint_name, n.joint_traj_points.positions,  n.joint_traj_points.velocities)
				new_traj.append(n)
			else:
				rospy.logerr('%s:processNewGoal: joint name %s is not correct'%(self.node_name, new_goal.trajectory.joint_names[i]))
		
		#self.current_follow_traj_goal = { 'points': None, 'finished': False}
		self.current_follow_traj_goal['points'] = new_traj
		self.current_follow_traj_goal['finished'] = False

		
		#print 'New GOAL processed:'
		#print self.current_follow_traj_goal
		
		return 0
		
	
	def sendCommands(self):
		'''
			Sends the position/velocity commands to every joint
			@return 0 if OK, -1 otherwise
		'''
		# Saves the active interfaces where it'll send commands
		active_interfaces = []
		
		# go through all the trajectory points.
		for point in self.current_follow_traj_goal['points']:
			# If it hasn't passed through this point yet
			if not point.finished:
				direction = 0
				# Only takes in account the joints of this trajectory
				self.desired_joint_state[point.joint_name]['position'] = point.joint_traj_points.positions[point.current_point]
				# target pos - current
				diff_pos = self.desired_joint_state[point.joint_name]['position'] - self.joint_state[point.joint_name]['position']
				
				#print 'joint %s: target pos = %.3f, current_pos = %.3f, diff = %.3f'%(point.joint_name, self.desired_joint_state[point.joint_name]['position'], self.joint_state[point.joint_name]['position'], diff_pos)
				
				if diff_pos < 0.0:
					direction = -1.0
				else:
					direction = 1.0
				
				
				t = 1.0/self.real_freq
				a = point.joint_traj_points.accelerations[point.current_point]
				desired_velocity = direction * point.joint_traj_points.velocities[point.current_point]
				if desired_velocity == 0.0:
					desired_velocity = direction * 0.017
				
				# ignores the acceleration 
				if self.ignore_acceleration:
					self.desired_joint_state[point.joint_name]['velocity'] = desired_velocity	
				else:
					# Control of velocity/acceleration
					diff_velocity = desired_velocity - self.joint_state[point.joint_name]['velocity']
					# inc_vel = a*t
					inc_vel = t * a
					
					#print 'joint %s, desired_vel = %.4f, current_vel = %.4f, inc_vel = %.4f'%(point.joint_name, desired_velocity, self.joint_state[point.joint_name]['velocity'],inc_vel)
					if diff_velocity > 0.0:
						# positive acc
						# v = v_current + a*t
						#self.desired_joint_state[point.joint_name]['velocity'] = self.joint_state[point.joint_name]['velocity'] + inc_vel
						self.desired_joint_state[point.joint_name]['velocity'] = self.desired_joint_state[point.joint_name]['velocity'] + inc_vel
						if self.desired_joint_state[point.joint_name]['velocity'] - direction*point.joint_traj_points.velocities[point.current_point] > 0.0:
							self.desired_joint_state[point.joint_name]['velocity'] = desired_velocity
							#print 'MAX'
				
					elif diff_velocity < 0.0:
						#self.desired_joint_state[point.joint_name]['velocity'] = self.joint_state[point.joint_name]['velocity'] - inc_vel
						self.desired_joint_state[point.joint_name]['velocity'] = self.desired_joint_state[point.joint_name]['velocity'] - inc_vel
						if self.desired_joint_state[point.joint_name]['velocity'] - direction*point.joint_traj_points.velocities[point.current_point] < 0.0:
							self.desired_joint_state[point.joint_name]['velocity'] = desired_velocity
							#print 'MIN'
					
					else:
						self.desired_joint_state[point.joint_name]['velocity'] = desired_velocity		
					
				#print 'joint %s, desired_instant_vel = %.4f'%(point.joint_name, self.desired_joint_state[point.joint_name]['velocity'])
				'''
				# NO ACCEL CONTROL
				if diff_pos < 0.0:
					self.desired_joint_state[point.joint_name]['velocity'] = -point.joint_traj_points.velocities[point.current_point]
				else:
					self.desired_joint_state[point.joint_name]['velocity'] = point.joint_traj_points.velocities[point.current_point]
				'''
					
				#print '%s, sending point %d [pos = %f, vel = %f]'%(point.joint_name, point.current_point, self.desired_joint_state[point.joint_name]['position'], self.desired_joint_state[point.joint_name]['velocity'])
			else:
				#print '%s set zero vel'%point.joint_name
				self.desired_joint_state[point.joint_name]['velocity'] = 0.0
				
				
			
			# Prepares the value to send it to the Controller device
			interface_name = self.desired_joint_state[point.joint_name]['command_interface']
			# Mark this interface as active
			if interface_name not in active_interfaces:
				active_interfaces.append(interface_name)
			
			if interface_name is not None:
				self.command_interfaces_dict[interface_name].setDesiredJointValue( point.joint_name, [ self.desired_joint_state[point.joint_name]['position'], self.desired_joint_state[point.joint_name]['velocity'], self.desired_joint_state[point.joint_name]['effort'] ])
				#print 'Setting joint %s value of %s'%(point.joint_name, interface_name)
			else:
				rospy.logerr('%s:sendCommands: Joint %s set to follow a trajectory without any defined command interface'%(self.node_name, point.joint_name))
				return -1
			
		
		# Send commands to the active Commands Interfaces of this trajectory
		for interface_name in active_interfaces:
			# print 'Sending command to %s interface'%interface_name
			
			# Check the state before sending the command
			if self.command_interfaces_dict[interface_name].getState() == State.READY_STATE:
				self.command_interfaces_dict[interface_name].sendCommand()
			else:
				rospy.logerr('%s:sendCommands: Interface %s not READY. Cancelling trajectory'%(self.node_name, interface_name))
				return -1
	
		return 0
		
	def updateJoints(self):
		'''
			Updates the value of the joints depending on the set velocity and position 
			Used for simulation
		'''
		t_now = time.time()
		
		for point in self.current_follow_traj_goal['points']:
			if self.desired_joint_state[point.joint_name]['velocity'] != 0.0:
				
				if self.desired_joint_state[point.joint_name]['last_update_time'] is not None:
					t_diff = t_now - self.desired_joint_state[point.joint_name]['last_update_time']
					self.joint_state[point.joint_name]['velocity'] = self.desired_joint_state[point.joint_name]['velocity']
					# increments the proportional part of the position coming from velocity * time increment
					self.joint_state[point.joint_name]['position'] = self.joint_state[point.joint_name]['position'] + self.joint_state[point.joint_name]['velocity'] * t_diff
					#print 'Increment joint %s to position %f. T diff = %f, velocity = %f'%(i, self.joint_state[point.joint_name]['position'], t_diff, self.joint_state[point.joint_name]['velocity'])
				self.desired_joint_state[point.joint_name]['last_update_time'] = t_now
			else:
				self.desired_joint_state[point.joint_name]['last_update_time'] = None
				
		'''
		for i in self.desired_joint_state:
			if self.desired_joint_state[i]['velocity'] != 0.0:
				
				if self.desired_joint_state[i]['last_update_time'] is not None:
					t_diff = t_now - self.desired_joint_state[i]['last_update_time']
					self.joint_state[i]['velocity'] = self.desired_joint_state[i]['velocity']
					# increments the proportional part of the position coming from velocity * time increment
					self.joint_state[i]['position'] = self.joint_state[i]['position'] + self.joint_state[i]['velocity'] * t_diff
					#print 'Increment joint %s to position %f. T diff = %f, velocity = %f'%(i, self.joint_state[i]['position'], t_diff, self.joint_state[i]['velocity'])
				self.desired_joint_state[i]['last_update_time'] = t_now
			else:
				self.desired_joint_state[i]['last_update_time'] = None
			'''	
		#print self.joint_state
	
	def cancelCurrentTraj(self):
		'''
			Cancels the current trajectory 
		'''
		rospy.loginfo('%s:cancelCurrentTraj: cancelling current trajectory'%(self.node_name))
		for point in self.current_follow_traj_goal['points']:
			# Set every position as finished
			point.finish()
			# Sets the desired speed to zero
			self.desired_joint_state[point.joint_name]['velocity'] = 0.0
				
		self.sendCommands()
		self.resetCheckJointsMovement()
	
	def pauseCurrentTraj(self):
		'''
			Pause the current trajectory 
		'''
		rospy.loginfo('%s:pauseCurrentTraj: pausing current trajectory'%(self.node_name))
		for point in self.current_follow_traj_goal['points']:
			# Sets the desired speed to zero
			self.desired_joint_state[point.joint_name]['velocity'] = 0.0
				
		self.sendCommands()
		
		
	def actionsServiceCb(self, req):
		'''
			ROS service to perform the actions defined in robotnik_trajectory_control.msg.Actions.msg 
			@param req: Required action
			@type req: robotnik_trajectory_control/TrajExecActions
		'''
		if req.action == Actions.PAUSE:
			rospy.loginfo('%s:actionsServiceCb: PAUSE action received'%self.node_name)
			self.requested_actions[Actions.PAUSE] = True
		elif req.action == Actions.CONTINUE:
			rospy.loginfo('%s:actionsServiceCb: CONTINUE action received'%self.node_name)
			self.requested_actions[Actions.CONTINUE] = True
		elif req.action == Actions.CANCEL:
			rospy.loginfo('%s:actionsServiceCb: CANCEL action received'%self.node_name)
			self.requested_actions[Actions.CANCEL] = True
		elif req.action == Actions.INIT:
			#rospy.loginfo('%s:actionsServiceCb: INIT action received'%self.node_name)
			self.requested_actions[Actions.INIT] = True
		elif req.action == Actions.INIT_ARMS:
			#rospy.loginfo('%s:actionsServiceCb: INIT_ARMS action received'%self.node_name)
			self.requested_actions[Actions.INIT_ARMS] = True
		elif req.action == Actions.INIT_HEAD:
			#rospy.loginfo('%s:actionsServiceCb: INIT_HEAD action received'%self.node_name)
			self.requested_actions[Actions.INIT_HEAD] = True
		elif req.action == Actions.INIT_WAIST:
			#rospy.loginfo('%s:actionsServiceCb: INIT_WAIST action received'%self.node_name)
			self.requested_actions[Actions.INIT_WAIST] = True
		elif req.action == Actions.INIT_RIGHT_GRIPPER:
			#rospy.loginfo('%s:actionsServiceCb: INIT_RIGHT_GRIPPER action received'%self.node_name)
			self.requested_actions[Actions.INIT_RIGHT_GRIPPER] = True
		elif req.action == Actions.INIT_LEFT_GRIPPER:
			#rospy.loginfo('%s:actionsServiceCb: INIT_LEFT_GRIPPER action received'%self.node_name)
			self.requested_actions[Actions.INIT_LEFT_GRIPPER] = True
		elif req.action == Actions.INIT_FINISHED:
			#rospy.loginfo('%s:actionsServiceCb: INIT_FINISHED action received'%self.node_name)
			self.requested_actions[Actions.INIT_FINISHED] = True
		elif req.action == Actions.RECOVER:
			rospy.loginfo('%s:actionsServiceCb: RECOVER action received'%self.node_name)
			self.requested_actions[Actions.RECOVER] = True
		else:
			rospy.loginfo('%s:actionsServiceCb: unknown service'%self.node_name)
		
		return True
	
	
	def getPendingActions(self):
		'''
			Search amongst all defined and available actions and return its ID in case that is active. This action will set to false
			@return action id (int) in case of active actions
			@return None otherwise
		'''
		for i in self.requested_actions:
			if self.requested_actions[i]:
				self.requested_actions[i] = False
				return i
		
		return None
		
		
	def resetPendingActions(self):
		'''
			Resets the all pending requested actions
		'''
		for i in self.requested_actions:
			self.requested_actions[i] = False
	
	def receiveJointStateCb(self, msg):
		'''
			Callback for joint states topic
			@param msg: received message
			@type msg: sensor_msgs/JointState
		'''
		
		for i in range(len(msg.name)):
			joint_name = msg.name[i]
			if self.joint_state.has_key(joint_name):
				try:
					self.joint_state[joint_name]['position'] = float(msg.position[i])
				except IndexError, e:
					rospy.logerr('%s:receiveJointStateCb: position index %d out of range: %s'%(self.node_name, i, e))
				try:
					self.joint_state[joint_name]['velocity'] =  float(msg.velocity[i])
				except IndexError, e:
					rospy.logerr('%s:receiveJointStateCb: velocity index %d out of range: %s'%(self.node_name, i, e))
				try:
					self.joint_state[joint_name]['effort'] =  float(msg.effort[i])
				except IndexError, e:
					rospy.logerr('%s:receiveJointStateCb: effort index %d out of range: %s'%(self.node_name, i, e))
				t_now = time.time()
				self.joint_state[joint_name]['joint_update_freq'] =  1.0/ (t_now - self.joint_state[joint_name]['last_update_time'])
				self.joint_state[joint_name]['last_update_time'] =  t_now #rospy.Time.now() #msg.header.stamp
				
				#if joint_name == 'head_pan_joint':
				#	print '%s updated to %s!'%(joint_name,self.joint_state[joint_name])
	
	
	def recoverComponents(self):
		'''
			Send the comand to try to recover every component from FAILURE STATE
		'''
		
		for i in self.command_interfaces_dict:
			rospy.loginfo('%s:recoverComponents: Recovering component %s'%(self.node_name, i))
			self.command_interfaces_dict[i].recover()
	
	
	def	getJointsLimitsServiceCb(self, req):
		'''
			Callback for service get joint limits
			@returns all the available joints of the model along with theirs max. and min. limits
		'''	
		joints_array = []
		max_array = []
		min_array = []
		for i in self.free_joints:
			joints_array.append(i)
			max_array.append(self.free_joints[i]['max'])
			min_array.append(self.free_joints[i]['min'])
		
		return joints_array, max_array, min_array		
		
def main():

	rospy.init_node("rt_traj_exe")
	
	
	_name = rospy.get_name().replace('/','')
	
	arg_defaults = {
	  'topic_state': 'state',
	  'desired_freq': DEFAULT_FREQ,
	  'error_joint_position': DEFAULT_ERROR_JOINT_POSITION,
	  'joint_read_state_timeout': DEFAULT_JOINT_READ_STATE_TIMEOUT,
	  'joint_no_movement_timeout': DEFAULT_JOINT_NO_MOVEMENT_TIMEOUT,
	  'ignore_acceleration': False,
	  'joint_lookahead': DEFAULT_JOINT_LOOKAHEAD
	}
	
	args = {}
	
	for name in arg_defaults:
		try:
			if rospy.search_param(name): 
				args[name] = rospy.get_param('%s/%s'%(_name, name)) # Adding the name of the node, because the para has the namespace of the node
			else:
				args[name] = arg_defaults[name]
			#print name
		except rospy.ROSException, e:
			rospy.logerr('%s: %s'%(e, _name))
			
	
	rtte_node = TrajExec(args)
	
	rospy.loginfo('%s: starting'%(_name))

	rtte_node.start()


if __name__ == "__main__":
	main()
	exit()
