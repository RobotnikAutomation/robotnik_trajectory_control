#!/usr/bin/env python

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

from robotnik_msgs.msg import State
from robotnik_trajectory_control.msg import State as ControlState
from robotnik_trajectory_control.msg import SubcomponentState
from control_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robotnik_trajectory_control.msg import Actions
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
ERROR_JOINT_POS = 0.0009 # rads
#ERROR_JOINT_POS = 0.000872665

def get_param(name, value=None):
	'''
		Function from joint_state_publisher
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
				 ("joint_params",  None),  # Ej. {'value': 0, 'upper': 3.14, 'lower': -3.14} params from robot_description
				 ("joint_state",  None),  # Ej. {'position': 0.0, 'velocity': 0.0, 'effort': 0.0} values extracted from /joint_state
				 ("current_point", int),
				 ("current_time", float),   
				 ("finished", bool)]     

	def __init__(self, desired_freq):
		self.current_point = 0
		self.current_time = 0.0
		self.finished = False
		self.len_points = 0	# number of points
		self.diff_position = 1000 # difference between current and current desired position 
		if desired_freq <= 0:
			desired_freq  = 1.0
		self.joint_state_timeout = 1/ (desired_freq/2.0)	# Sets timeout that will be trigger if joint_state value is not received. We apply a reduction of the main frequency
	
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
	
	def getLastPointPosition(self):
		'''
			@returns the last point of the list or None if there aren't points
		'''
		if self.len_points > 0:
			return self.joint_traj_points.positions[self.len_points - 1]
			
			
			
	
# Class to simulate the trajectory execution
class SimTrajExec:
	'''
		Simulates join_state_publisher
		Offers the action service FollowJointTrajectoryAction
	'''
		
	def __init__(self, args):
		
		self.node_name = rospy.get_name().replace('/','')
		self.desired_freq = args['desired_freq'] 
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
		
		self._state_publisher = rospy.Publisher('%s/state'%self.node_name, ControlState)
		self._actions_service = rospy.Service('%s/actions'%self.node_name, TrajExecActions, self.actionsServiceCb)
		self._get_joints_limits_service = rospy.Service('%s/get_joints_limits'%self.node_name, GetJointsLimits, self.getJointsLimitsServiceCb)
		
		
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
		
		self.joint_state_publisher = rospy.Publisher('joint_states', JointState)

		# Creating Action Server
		self.follow_traj_action_server = actionlib.SimpleActionServer('%s/follow_joint_trajectory'%self.node_name, FollowJointTrajectoryAction, None, False)
		
		self.ros_initialized = True
		
		# dict to save the current joint state value 
		# {'joint_1':{'position': 0.0, 'velocity': 0.0, 'effort'}
		self.joint_state = {}
		# Copy of joint state but used to simulate the position increment
		self.desired_joint_state = {}
		# Inits joint states
		for (name,joint) in self.free_joints.items():
			self.joint_state[name] = {'position': float(joint['value']), 'velocity': 0.0, 'effort': 0.0}
			self.desired_joint_state[name] = {'position': float(joint['value']), 'velocity': 0.0, 'effort': 0.0, 'last_update_time': None}
		
		
		# Structure to save the current trajectory being followed
		# field 'points' contains an array of 'TrajectoryGoalExecution'(one per each joint involved on the execution),made from the last FollowTrajectoryAction goal received
		# field 'finished' indicates whether the trajectory is finished or not
		self.current_follow_traj_goal = { 'points': None, 'finished': False}
		
		"""
		# Gets and process device commands interface groups
		try:
			if rospy.search_param('groups'): 
				groups = rospy.get_param('groups')
			else:
				rospy.logerr('%s::rosSetup: Param groups not found!'%self.node_name)
				return -1
		except rospy.ROSException, e:
			rospy.logerror('%s::rosSetup %s'%(self.node_namee, e))
			return -1
		
		# array with all the command interfaces created
		self.command_interfaces_dict = {}
		
		for i in groups:
			type_group = groups[i]['type']
			name = groups[i]['name']
			
			if self.command_interfaces_dict.has_key(name):
				rospy.logerr('%s::rosSetup: Error on the interface name %s:%s already exist. Discarded'%(self.node_name, type_group, name))
			else:
				self.command_interfaces_dict[name] = {'type': type_group, 'state': State.INIT_STATE}
		
		# If there are no command interfaces setup is aborted		
		if len(self.command_interfaces_dict) == 0:
			rospy.logerr('%s::rosSetup: No device command interfaces defined'%(self.node_name))	
			return -1
		"""
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
		
		# Performs ROS topics & services shutdown
		self._state_publisher.unregister()
		self.joint_state_publisher.unregister()
		self._actions_service .shutdown()
		
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
			
			if t_sleep > 0.0:
				rospy.sleep(t_sleep)
			
			t3= time.time()
			self.real_freq = 1.0/(t3 - t1)
		
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
		# Publish joint state
		msg = JointState()
		msg.header.stamp = rospy.Time.now()


		for i in self.joint_state:
			msg.name.append(str(i))
			msg.position.append(self.joint_state[i]['position'])
			msg.velocity.append(self.joint_state[i]['velocity'])
			msg.effort.append(self.joint_state[i]['effort'])
		
		'''
		# Add Free Joints
		for (name,joint) in self.free_joints.items():
			msg.name.append(str(name))
			msg.position.append(joint['value'])

		# Add Dependent Joints
		for (name,param) in self.dependent_joints.items():
			parent = param['parent']
			baseval = self.free_joints[parent]['value']
			value = baseval * param.get('factor', 1)

			msg.name.append(str(name))
			msg.position.append(value)
		'''
		
		self.joint_state_publisher.publish(msg)
		
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
				# Moving substate
				#self.init_substate = INITIALIZE_SUBCOMPONENTS
		
		# Initializes subcomponents		
		"""
		elif self.init_substate == INITIALIZE_SUBCOMPONENTS:
			# Depending on the signal received it will initialize each component
			if self.requested_actions[Actions.INIT_ARMS]:
				# Simulates the Initialization
				self.command_interfaces_dict['left_arm']['state'] = State.READY_STATE
				self.command_interfaces_dict['right_arm']['state'] = State.READY_STATE
				
				rospy.loginfo('%s:InitState: Inititializing arms'%self.node_name)
				self.requested_actions[Actions.INIT_ARMS] = False
			
			if self.requested_actions[Actions.INIT_HEAD]:
				# Simulates the Initialization
				self.command_interfaces_dict['head']['state'] = State.READY_STATE
				rospy.loginfo('%s:InitState: Inititializing head'%self.node_name)
				self.requested_actions[Actions.INIT_HEAD] = False
			
			if self.requested_actions[Actions.INIT_WAIST]:
				# Simulates the Initialization
				self.command_interfaces_dict['waist']['state'] = State.READY_STATE
				rospy.loginfo('%s:InitState: Inititializing waist'%self.node_name)
				self.requested_actions[Actions.INIT_WAIST] = False
			
			if self.requested_actions[Actions.INIT_RIGHT_GRIPPER]:
				# Simulates the Initialization
				self.command_interfaces_dict['right_gripper']['state'] = State.READY_STATE
				rospy.loginfo('%s:InitState: Inititializing right gripper'%self.node_name)
				self.requested_actions[Actions.INIT_RIGHT_GRIPPER] = False
			
			if self.requested_actions[Actions.INIT_LEFT_GRIPPER]:
				# Simulates the Initialization
				self.command_interfaces_dict['left_gripper']['state'] = State.READY_STATE
				rospy.loginfo('%s:InitState: Inititializing left gripper'%self.node_name)
				self.requested_actions[Actions.INIT_LEFT_GRIPPER] = False
				
			if self.requested_actions[Actions.INIT_FINISHED]:
				self.requested_actions[Actions.INIT_FINISHED] = False
				self.switchToState(State.STANDBY_STATE)
			
			if self.follow_traj_action_server.is_new_goal_available():
				rospy.loginfo('%s:InitState: Received goal while in INIT. Aborted'%self.node_name)
				goal = self.follow_traj_action_server.accept_new_goal()
				self.follow_traj_action_server.set_aborted()
			"""
			
		return
	
	def standbyState(self):
		'''
			Actions performed in standby state
		'''
		self.switchToState(State.READY_STATE)
		
		return
	
	def readyState(self):
		'''
			Actions performed in ready state
		'''
		#
		# ACTIVE 
		if self.ready_substate == ACTIVE_SUBSTATE:
			# variable to control the end of the trajectory
			traj_finish = True
		
			# go through all the trajectory points
			for i in self.current_follow_traj_goal['points']:
				# check the flag of finished (active when reaching the position)
				if not i.finished:
					# diff between current joint position and desired value
					diff = abs(i.joint_state['position'] - i.joint_traj_points.positions[i.current_point])
						
					# there are more points to follow
					if i.morePoints():
						# if the difference is greater than the last cycle or is lower than the error joint pos,  we change the point objective  
						if diff > i.diff_position or diff <= ERROR_JOINT_POS:
							#print 'joint %s, point %d: reached'%(i.joint_name, i.current_point)	
							i.nextPoint()
											
						else:
							i.updateDiffPosition(diff)						
						traj_finish = False
					else:
						# look for the end of trajectory
						if diff > i.diff_position or diff <= ERROR_JOINT_POS:
							#print 'joint %s, end point %d: reached'%(i.joint_name, i.current_point)
							i.finish()
						else:
							traj_finish = False
				
			# send position and velocity refs to each joint / component
			self.sendCommands()
			 
			# Simulates the joints update
			self.updateJoints()
			
			
			self.current_follow_traj_goal['finished'] = traj_finish

			
			if self.current_follow_traj_goal['finished']:
				# stops velocity
				self.pauseCurrentTraj()
				self.follow_traj_action_server.set_succeeded()
				self.ready_substate = IDLE_SUBSTATE
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
			"""
			elif i == Actions.INIT:
					rospy.loginfo('%s:readyState: switching to INIT state'%self.node_name)
					self.switchToState(State.INIT_STATE)
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
			
			self.resetPendingActions()
			
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
		"""for i in self.command_interfaces_dict:
			subcomponent = SubcomponentState()
			subcomponent.name = i	
			subcomponent.type = self.command_interfaces_dict[i]['type']
			subcomponent.state = self.command_interfaces_dict[i]['state']
			subcomponent.state_description = self.stateToString(self.command_interfaces_dict[i]['state'])
			components_array.append(subcomponent)
		"""
		self.msg_state.components = components_array
			
			
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
				
				
				n = TrajectoryGoalExecution(self.desired_freq)
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
				#print 'name = %s, %d points = %s\n vel = %s\n acc = %s\n'%(n.joint_name, len(n.joint_traj_points.positions), n.joint_traj_points.positions,  n.joint_traj_points.velocities, n.joint_traj_points.accelerations)
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
			Sends the position/velocity commands to every joints
			As it's in simulation, it will update joint_state values
		'''
		# go through all the trajectory points.
		for point in self.current_follow_traj_goal['points']:
			
			if not point.finished:
				direction = 0				
				# Only takes in account the joints of the current trajectory
				self.desired_joint_state[point.joint_name]['position'] = point.joint_traj_points.positions[point.current_point]
				
				diff = self.desired_joint_state[point.joint_name]['position'] - self.joint_state[point.joint_name]['position'] 
				
				if diff < 0.0:
					direction = -1.0
					#self.desired_joint_state[point.joint_name]['velocity'] = -point.joint_traj_points.velocities[point.current_point]
				else:
					direction = 1.0
					#self.desired_joint_state[point.joint_name]['velocity'] = point.joint_traj_points.velocities[point.current_point]
				
				
				# End condition: getting to the end position
				#dist_to_end = abs(point.getLastPointPosition() - self.joint_state[point.joint_name]['position'])
				'''if dist_to_end <= 0.035:
					if point.joint_traj_points.velocities[point.current_point] == 0.0:
						#print 'Joint %s, point %d, dist to end = %.4f, velocity = %.4f'%(point.joint_name, point.current_point, dist_to_end, point.joint_traj_points.velocities[point.current_point])
						t = dist_to_end/0.017
					else:
						t = dist_to_end/point.joint_traj_points.velocities[point.current_point]
					a = abs(0.017 - point.joint_traj_points.velocities[point.current_point]) / t
					desired_velocity = direction * 0.017
					#print 'Joint %s, dist to end = %.4f, velocity = %.4f'%(point.joint_name, dist_to_end, desired_velocity)
				else:'''
				
				t = 1.0/self.desired_freq
				#t = 1.0/self.real_freq
				a = point.joint_traj_points.accelerations[point.current_point]
				desired_velocity = direction * point.joint_traj_points.velocities[point.current_point]
				if desired_velocity == 0.0:
					desired_velocity = direction * 0.017
				
				# Control of velocity/acceleration
				diff_velocity = desired_velocity - self.joint_state[point.joint_name]['velocity']
				# inc_vel = a*t
				inc_vel = t * a
				if diff_velocity > 0.0:
					# positive acc
					# v = v_current + a*t
					#self.desired_joint_state[point.joint_name]['velocity'] = self.joint_state[point.joint_name]['velocity'] + inc_vel
					self.desired_joint_state[point.joint_name]['velocity'] = self.desired_joint_state[point.joint_name]['velocity'] + inc_vel
					if self.desired_joint_state[point.joint_name]['velocity'] - direction*point.joint_traj_points.velocities[point.current_point] > 0.0:
						self.desired_joint_state[point.joint_name]['velocity'] = desired_velocity
			
				elif diff_velocity < 0.0:
					#self.desired_joint_state[point.joint_name]['velocity'] = self.joint_state[point.joint_name]['velocity'] - inc_vel
					self.desired_joint_state[point.joint_name]['velocity'] = self.desired_joint_state[point.joint_name]['velocity'] - inc_vel
					if self.desired_joint_state[point.joint_name]['velocity'] - direction*point.joint_traj_points.velocities[point.current_point] < 0.0:
						self.desired_joint_state[point.joint_name]['velocity'] = desired_velocity
				
				else:
					self.desired_joint_state[point.joint_name]['velocity'] = desired_velocity		
				
				#print 'Joint %s, point %d, current vel = %.4f, desired vel = %.4f (inc_vel = %f)'%(point.joint_name, point.current_point, self.joint_state[point.joint_name]['velocity'], self.desired_joint_state[point.joint_name]['velocity'], inc_vel)
				#print '%s, sending point %d [pos = %f, vel = %f]'%(point.joint_name, point.current_point, self.desired_joint_state[point.joint_name]['position'], self.desired_joint_state[point.joint_name]['velocity'])
			else:
				self.desired_joint_state[point.joint_name]['velocity'] = 0.0
			
	
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
				self.joint_state[point.joint_name]['velocity'] = 0.0
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
				
		# Updates joint
		self.updateJoints()
	
	def pauseCurrentTraj(self):
		'''
			Pause the current trajectory 
		'''
		rospy.loginfo('%s:pauseCurrentTraj: pausing current trajectory'%(self.node_name))
		for point in self.current_follow_traj_goal['points']:
			# Sets the desired speed to zero
			self.desired_joint_state[point.joint_name]['velocity'] = 0.0
				
		# Updates joint
		self.updateJoints()
		
		
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
		"""
		elif req.action == Actions.INIT:
			rospy.loginfo('%s:actionsServiceCb: INIT action received'%self.node_name)
			self.requested_actions[Actions.INIT] = True
		elif req.action == Actions.INIT_ARMS:
			rospy.loginfo('%s:actionsServiceCb: INIT_ARMS action received'%self.node_name)
			self.requested_actions[Actions.INIT_ARMS] = True
		elif req.action == Actions.INIT_HEAD:
			rospy.loginfo('%s:actionsServiceCb: INIT_HEAD action received'%self.node_name)
			self.requested_actions[Actions.INIT_HEAD] = True
		elif req.action == Actions.INIT_WAIST:
			rospy.loginfo('%s:actionsServiceCb: INIT_WAIST action received'%self.node_name)
			self.requested_actions[Actions.INIT_WAIST] = True
		elif req.action == Actions.INIT_RIGHT_GRIPPER:
			rospy.loginfo('%s:actionsServiceCb: INIT_RIGHT_GRIPPER action received'%self.node_name)
			self.requested_actions[Actions.INIT_RIGHT_GRIPPER] = True
		elif req.action == Actions.INIT_LEFT_GRIPPER:
			rospy.loginfo('%s:actionsServiceCb: INIT_LEFT_GRIPPER action received'%self.node_name)
			self.requested_actions[Actions.INIT_LEFT_GRIPPER] = True
		elif req.action == Actions.INIT_FINISHED:
			rospy.loginfo('%s:actionsServiceCb: INIT_FINISHED action received'%self.node_name)
			self.requested_actions[Actions.INIT_FINISHED] = True
		else:
			rospy.loginfo('%s:actionsServiceCb: unknown service'%self.node_name)
		"""
		
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

	rospy.init_node("rt_sim_traj_exe")
	
	
	_name = rospy.get_name().replace('/','')
	
	arg_defaults = {
	  'topic_state': 'state',
	  'desired_freq': DEFAULT_FREQ,
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
			rospy.logerror('%s: %s'%(e, _name))
			
	
	rtste_node = SimTrajExec(args)
	
	rospy.loginfo('%s: starting'%(_name))

	rtste_node.start()


if __name__ == "__main__":
	main()
	exit()
