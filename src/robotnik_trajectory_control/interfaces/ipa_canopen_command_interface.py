#!/usr/bin/env python

import rospy
from threading import Thread
from sensor_msgs.msg import JointState
from brics_actuator.msg import JointVelocities, JointValue
from rospy.exceptions import ROSException

from robotnik_trajectory_msgs.msg import State

from std_srvs.srv import Empty

from cob_srvs.srv import Trigger
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


			
#
# IPA CANOPEN Interface
#		
class IpaCANOpenCommandInterface(DeviceCommandInterface):
	'''
		Command interface to interact with Schunk LWA4P (Powerball) arm
	'''
	def __init__(self, args):
		DeviceCommandInterface.__init__(self, args)
		
		if args.has_key('init_service'):
			self._init_service_name = args['init_service']
			#print 'IpaCANOpenCommandInterface: init_service = %s'%self._init_service_name
		else:
			self._init_service_name = ''
		
		if args.has_key('recovery_service'):
			self._recover_service_name = args['recovery_service']
			#print 'IpaCANOpenCommandInterface: recover_service = %s'%self._recover_service_name
		else:
			self._recover_service_name = ''	
		
		# Joint state is not used. Instead it will use the interface for BricsActuator
		self.joint_velocities = JointVelocities()
		# Flag active when the initialization service is called
		self.initializing = False
		# Flag active when the recovery service is called
		self.recovering = False
	
	
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
				j = JointValue()
				j.joint_uri = self.joint_names[i]
				j.value = 0.0
				j.unit = 'rad'
				self.joint_velocities.velocities.append(j)
				
				self.joint_state_pointer[self.joint_names[i]] = [ 0, 0, 0]
				
		
		# SUBSCRIBERS
		# IPA state
		#self.state_subscriber = rospy.Subscriber(self.state_topic, IpaCanOpenState, self.receiveStateCb)
		#print '**********IPA setup*********'
		# PUBLISHERS
		if len(self.command_topic) > 0:			
			try:
				self.command_publisher = rospy.Publisher(self.command_topic, JointVelocities)
			except ValueError, e:
				rospy.logerr('%s-%s:setup: Error connecting to topic  %s ->(%s)'%(self.type, self.name, self.command_topic, e))
				return -1		
		else:
			rospy.logerr('%s-%s:setup: No command topic supplied.'%(self.type, self.name))
			return -1
				
		# IPA SERVICES
		try:
			self._service_init = rospy.ServiceProxy(self._init_service_name, Trigger)
		except ValueError, e:
			rospy.logerr('%s-%s:setup: Error connecting service (%s)'%(self.type, self.name, e))
		try:
			self._service_recover = rospy.ServiceProxy(self._recover_service_name, Trigger)
		except ValueError, e:
			rospy.logerr('%s-%s:setup: Error connecting service (%s)'%(self.type, self.name, e))
			
		
		self.initialized = True
		
		return 0
	
	
	def sendCommand(self):
		'''
			Sends the current value of joint_state attribute to the controller
			@return: 0 if OK, -1 otherwise
		'''
		if not self.initialized:
			return -1
		
		t = rospy.Time.now()
		# copy desired values into the joint_state structure	
		for i in range(len(self.joint_names)):
			# Only needs velocity
			self.joint_velocities.velocities[i].value = self.joint_state_pointer[self.joint_names[i]][1]
			self.joint_velocities.velocities[i].timeStamp = t

		#rospy.loginfo('%s-%s:sendCommand: sending command %s to %s'%(self.type, self.name, self.joint_velocities.velocities ,self.command_topic))	
		self.command_publisher.publish(self.joint_velocities)
		
		return 0
	
	
	def receiveStateCb(self, msg):
		'''
			Callback associated with the topic state
		'''
		pass
		
	
	def initialize_call(self):
		'''
			Calls the initialization service
			Blocking call
		'''
		try:
			ret = self._service_init()	
				
			if ret.success.data:
				rospy.loginfo('%s-%s:initialize_call: Initialized successfully'%(self.type, self.name))		
			else:
				rospy.logerr('%s-%s:initialize_call: Error on Initialize'%(self.type, self.name))
			
		except rospy.ServiceException, e:
				rospy.logerr('%s-%s:initialize_call: Error calling service: %s'%(self.type, self.name, e))
		
		self.initializing = False
		
	def initialize(self):
		'''
			Initializes the component
		'''
		'''
		self.initializing = False
		'''
		if not self.initializing:	
			self.initializing = True
			rospy.loginfo('%s-%s:initialize: Starting initialization'%(self.type, self.name))	
			self.thread_init = Thread(target = self.initialize_call)
			self.thread_init.start()
	
	
	def recover_call(self):
		'''
			Calls the recover service
			Blocking call
		'''
		try:
			ret = self._service_recover()	
				
			if ret.success.data:
				rospy.loginfo('%s-%s:recover_call: Recovered successfully'%(self.type, self.name))		
			else:
				rospy.logerr('%s-%s:recover_call: Error on Recover'%(self.type, self.name))
			
		except rospy.ServiceException, e:
				rospy.logerr('%s-%s:recover_call: Error calling service: %s'%(self.type, self.name, e))
		
		self.recovering = False
		
	def recover(self):
		'''
			Recovers the component
		'''
		'''
		self.initializing = False
		'''
		if not self.recovering:	
			self.recovering = True
			rospy.loginfo('%s-%s:recover: Starting recover'%(self.type, self.name))	
			self.thread_recover = Thread(target = self.recover_call)
			self.thread_recover.start()
	
	
	
	def shutdown(self):
		'''
			Shutdowns connections
		'''
		DeviceCommandInterface.shutdown(self)
		
		self._service_init.close()
		self._service_recover.close()
	
