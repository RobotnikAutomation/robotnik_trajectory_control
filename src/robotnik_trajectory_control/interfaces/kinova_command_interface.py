#!/usr/bin/env python

import rospy
from threading import Thread
from sensor_msgs.msg import JointState
from rospy.exceptions import ROSException

from robotnik_msgs.msg import State

from std_srvs.srv import Empty

from kinova_msgs.msg import JointVelocity 
import time
import math

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
				rospy.loginfo('%s-%s:setup: connecting to topic  %s'%(self.type, self.name, self.command_topic))
				self.command_publisher = rospy.Publisher(self.command_topic, JointVelocity)
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
# Kinova Interface
#		
class KinovaCommandInterface(DeviceCommandInterface):
	'''
		Command interface to interact with Schunk LWA4P (Powerball) arm
	'''
	def __init__(self, args):
		DeviceCommandInterface.__init__(self, args)
		
###XXX		if args.has_key('init_service'):
###XXX			self._init_service_name = args['init_service']
###XXX			#print 'KinovaCommandInterface: init_service = %s'%self._init_service_name
###XXX		else:
###XXX			self._init_service_name = ''
###XXX		
###XXX		if args.has_key('recovery_service'):
###XXX			self._recover_service_name = args['recovery_service']
###XXX			#print 'KinovaCommandInterface: recover_service = %s'%self._recover_service_name
###XXX		else:
###XXX			self._recover_service_name = ''	
		
		# Joint state is not used. Instead it will use the interface for BricsActuator
		self.joint_velocities = JointVelocity()
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
###XXX				j = JointValue()
###XXX				j.joint_uri = self.joint_names[i]
###XXX				j.value = 0.0
###XXX				j.unit = 'rad'
###XXX				self.joint_velocities.velocities.append(j)
				
				self.joint_state_pointer[self.joint_names[i]] = [ 0, 0, 0]
				
		
		# SUBSCRIBERS
		# KINOVA state
		#self.state_subscriber = rospy.Subscriber(self.state_topic, KinovaState, self.receiveStateCb)
		#print '**********Kinova setup*********'
		# PUBLISHERS
		if len(self.command_topic) > 0:			
			try:
				rospy.loginfo('%s-%s:setup: connecting to topic  %s (second)'%(self.type, self.name, self.command_topic))
				self.command_publisher = rospy.Publisher(self.command_topic, JointVelocity)
			except ValueError, e:
				rospy.logerr('%s-%s:setup: Error connecting to topic  %s ->(%s)'%(self.type, self.name, self.command_topic, e))
				return -1		
		else:
			rospy.logerr('%s-%s:setup: No command topic supplied.'%(self.type, self.name))
			return -1
				
		# Kinova SERVICES
###XXX		try:
###XXX			self._service_init = rospy.ServiceProxy(self._init_service_name, Trigger)
###XXX		except ValueError, e:
###XXX			rospy.logerr('%s-%s:setup: Error connecting service (%s)'%(self.type, self.name, e))
###XXX		try:
###XXX			self._service_recover = rospy.ServiceProxy(self._recover_service_name, Trigger)
###XXX		except ValueError, e:
###XXX			rospy.logerr('%s-%s:setup: Error connecting service (%s)'%(self.type, self.name, e))
			
		
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
###XXX		for i in range(len(self.joint_names)):
###XXX			# Only needs velocity
###XXX			self.joint_velocities.velocities[i].value = self.joint_state_pointer[self.joint_names[i]][1]
###XXX			self.joint_velocities.velocities[i].timeStamp = t
		self.joint_velocities.joint1 = math.degrees(self.joint_state_pointer[self.joint_names[0]][1])
#		rospy.loginfo('%s-%s:sendCommand: sending command degrees: %f radians: %f' %(self.type, self.name, math.degrees(self.joint_state_pointer[self.joint_names[0]][1]), self.joint_state_pointer[self.joint_names[0]][1]))
		self.joint_velocities.joint2 = math.degrees(self.joint_state_pointer[self.joint_names[1]][1])
		self.joint_velocities.joint3 = math.degrees(self.joint_state_pointer[self.joint_names[2]][1])
		self.joint_velocities.joint4 = math.degrees(self.joint_state_pointer[self.joint_names[3]][1])
		if len(self.joint_names) > 4:
			self.joint_velocities.joint5 = math.degrees(self.joint_state_pointer[self.joint_names[4]][1])
			self.joint_velocities.joint6 = math.degrees(self.joint_state_pointer[self.joint_names[5]][1])
		
		#rospy.loginfo('%s-%s:sendCommand: sending command %s to %s'%(self.type, self.name, self.joint_velocities.velocities ,self.command_topic))	
		self.command_publisher.publish(self.joint_velocities)
		
		return 0
	
	
	def receiveStateCb(self, msg):
		'''
			Callback associated with the topic state
		'''
		pass
		
	
	def initialize_call(self):
###XXX		'''
###XXX			Calls the initialization service
###XXX			Blocking call
###XXX		'''
###XXX		try:
###XXX			ret = self._service_init()	
###XXX				
###XXX			if ret.success.data:
###XXX				rospy.loginfo('%s-%s:initialize_call: Initialized successfully'%(self.type, self.name))		
###XXX			else:
###XXX				rospy.logerr('%s-%s:initialize_call: Error on Initialize'%(self.type, self.name))
###XXX			
###XXX		except rospy.ServiceException, e:
###XXX				rospy.logerr('%s-%s:initialize_call: Error calling service: %s'%(self.type, self.name, e))
###XXX		
###XXX		self.initializing = False
		pass
		
	def initialize(self):
###XXX		'''
###XXX			Initializes the component
###XXX		'''
###XXX		'''
###XXX		self.initializing = False
###XXX		'''
###XXX		if not self.initializing:	
###XXX			self.initializing = True
###XXX			rospy.loginfo('%s-%s:initialize: Starting initialization'%(self.type, self.name))	
###XXX			self.thread_init = Thread(target = self.initialize_call)
###XXX			self.thread_init.start()
		pass	
	
	def recover_call(self):
###XXX		'''
###XXX			Calls the recover service
###XXX			Blocking call
###XXX		'''
###XXX		try:
###XXX			ret = self._service_recover()	
###XXX				
###XXX			if ret.success.data:
###XXX				rospy.loginfo('%s-%s:recover_call: Recovered successfully'%(self.type, self.name))		
###XXX			else:
###XXX				rospy.logerr('%s-%s:recover_call: Error on Recover'%(self.type, self.name))
###XXX			
###XXX		except rospy.ServiceException, e:
###XXX				rospy.logerr('%s-%s:recover_call: Error calling service: %s'%(self.type, self.name, e))
###XXX		
###XXX		self.recovering = False
		pass
		
	def recover(self):
		'''
			Recovers the component
		'''
		'''
		self.initializing = False
		'''
###XXX		if not self.recovering:	
###XXX			self.recovering = True
###XXX			rospy.loginfo('%s-%s:recover: Starting recover'%(self.type, self.name))	
###XXX			self.thread_recover = Thread(target = self.recover_call)
###XXX			self.thread_recover.start()
		pass	
	
	
	def shutdown(self):
		'''
			Shutdowns connections
		'''
		DeviceCommandInterface.shutdown(self)
		
###XXX		self._service_init.close()
###XXX		self._service_recover.close()
	
