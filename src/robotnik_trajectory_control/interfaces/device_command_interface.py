#!/usr/bin/env python

import rospy
from threading import Thread
from sensor_msgs.msg import JointState
from rospy.exceptions import ROSException

from robotnik_msgs.msg import State
from std_srvs.srv import Empty

import time

#
# STANDARD INTERFACE
#
class DeviceCommandInterface():
	'''
		Class intended to communicate with controller devices by using a standard interface
	'''
	
	def __init__(self, args):
		'''
			Component initialization
			@param args: arguments to configure the component
			@type args: {name: string, command_topic: string, state_topic: string, joints: [string]}
		'''
		
		self.initialized = False
		
		if args.has_key('type'):
			self.type = args['type']
		else:
			self.type = 'DeviceCommandInterface'
			
		if args.has_key('name'):
			self.name = args['name']
		else:
			self.name = 'unnamed'
			rospy.logerr('%s:init: param name not found'%self.type)
		
		if args.has_key('command_topic'):
			self.command_topic = args['command_topic']
		else:
			self.command_topic = ''
			rospy.logerr('%s:init: param command_topic not found'%self.type)
		
		if args.has_key('state_topic'):
			self.state_topic = args['state_topic']
			print 'state topic =  %s'%(self.state_topic)
		
		else:
			self.state_topic = ''
			rospy.logerr('%s:init: param state_topic not found'%self.type)
		
		if args.has_key('joints'):
			self.joint_names = args['joints']
		else:
			self.joint_names = []
			rospy.logerr('%s:init: param joints not found'%self.type)
				
		self.joint_state = JointState()
		# Intermediate structure to save each pos, vel and effort value before sending the command to the component
		# Ej.: {'j1': [ 0, 0, 0 ]}
		self.joint_state_pointer = {} 
	
		# State of the component
		self.state = State.READY_STATE
	
	def setup(self):
		'''
			Initializes joint values, connect to command and state topics
			@return: 0 if OK, -1 otherwise
		'''	
		
		if len(self.joint_names) == 0:
			rospy.logerr('%s-%s:setup: no joints provided'%(self.type, self.name))
			return -1
		else:
			for i in range(len(self.joint_names)):
				self.joint_state.name.append(self.joint_names[i])
				self.joint_state.position.append(0)
				self.joint_state.velocity.append(0)
				self.joint_state.effort.append(0)
				# 
				self.joint_state_pointer[self.joint_names[i]] = [ 0, 0, 0]
				
		
		'''
		# TODO for each component
		if len(self.state_topic) > 0:			
			try:
				self.state_subscriber = rospy.Subscriber(self.state_topic, String, self.receiveStateCb)
			except ValueError, e:
				rospy.logerr('%s-%s:setup: Error connecting to topic  %s ->(%s)'%(self.type, self.name, self.state_topic, e))
		'''	
		
		if len(self.command_topic) > 0:			
			try:
				self.command_publisher = rospy.Publisher(self.command_topic, JointState)
				rospy.loginfo('%s-%s:setup: connecting to topic  %s'%(self.type, self.name, self.command_topic))
			except ValueError, e:
				rospy.logerr('%s-%s:setup: Error connecting to topic  %s ->(%s)'%(self.type, self.name, self.command_topic, e))
				return -1		
		else:
			rospy.logerr('%s-%s:setup: No command topic supplied.'%(self.type, self.name))
			return -1
				
		
		
		self.initialized = True
		
		return 0
	
	
		
	def setDesiredJointValue(self, name, value):
		'''
			Sets the joint value to desired value
			@param name: name of the joint
			@type name: string
			@param value: desired value of the joint
			@type value: array with the values of [position, velocity, effort]	
			@return: 0 if OK, -1 otherwise	
		'''
		if not self.initialized:
			rospy.logerr('%s-%s:setDesiredJointValue: interface not initialized correctly'%(self.type, self.name, name))	
			return -1
		
		
		if self.joint_state_pointer.has_key(name):
			
			if len(value) == 3:
				self.joint_state_pointer[name][0] = float(value[0])
				self.joint_state_pointer[name][1] = float(value[1])
				self.joint_state_pointer[name][2] = float(value[2])
			else:
				rospy.logerr('%s-%s:setDesiredJointValue: incorrect length of desired value %s'%(self.type, self.name, value))	
				return -1
				
		else:
			rospy.logerr('%s-%s:setDesiredJointValue: joint %s is not associated with this interface'%(self.type, self.name, name))	
			return -1
		
		return 0
		
	def sendCommand(self):
		'''
			Sends the current value of joint_state attribute to the controller
			@return: 0 if OK, -1 otherwise
		'''
		if not self.initialized:
			return -1
		
		# copy desired values into the joint_state structure	
		for i in range(len(self.joint_names)):
			self.joint_state.position[i] = self.joint_state_pointer[self.joint_names[i]][0]
			self.joint_state.velocity[i] = self.joint_state_pointer[self.joint_names[i]][1]
			self.joint_state.effort[i] = self.joint_state_pointer[self.joint_names[i]][2]
			
		#self.joint_state.position=[]	
		self.joint_state.header.stamp = rospy.Time.now()
		#rospy.loginfo('%s-%s:sendCommand: sending command (pos = %s) to %s'%(self.type, self.name, self.joint_state.position, self.command_topic))	
		self.command_publisher.publish(self.joint_state)
		
		return 0
		
	
	def getState(self):
		'''
			Gets the state of the controller interface
			@return: the state of the component based on robotnik_trajectory_msgs.msg.State
		'''
		
		return self.state 
		
	def stop(self):
		'''
			Stops any movement
			@return: 0 if OK, -1 otherwise
		'''
		# Sets velocity to 0.0
		for name in self.joint_names:
			self.joint_state_pointer[name][1] = 0.0
			
		self.sendCommand()
		
		return 0
		
	def receiveStateCb(self, msg):
		'''
			Callback associated with the topic state
		'''
		
		pass

	def shutdown(self):
		'''
			Unregister ROS components
		'''
		
		if hasattr(self, 'command_publisher'):
			self.command_publisher.unregister()
			
		if hasattr(self, 'state_subscriber'):
			self.state_subscriber.unregister()
		
		self.initialized = False
		
		
	def initialize(self):
		'''
			Initializes the component
		'''
		self.state = State.READY_STATE

	def recover(self):
		'''
			Recovers the component
		'''
		pass


