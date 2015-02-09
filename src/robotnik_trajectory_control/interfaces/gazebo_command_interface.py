import rospy
from device_command_interface import DeviceCommandInterface
from std_msgs.msg import Float64
from robotnik_msgs.msg import State
							
#
# Interface for Gazebo controllers 
#		
class GazeboPositionCommandInterface(DeviceCommandInterface):
	'''
		Command interface to interact with Gazebo controllers
	'''
	def __init__(self, args):
		DeviceCommandInterface.__init__(self, args)
		
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
				
		
		if len(self.command_topic) > 0:			
			try:
				self.command_publisher = rospy.Publisher(self.command_topic, Float64)
				rospy.loginfo('%s-%s:setup: connecting to topic  %s'%(self.type, self.name, self.command_topic))
			except ValueError, e:
				rospy.logerr('%s-%s:setup: Error connecting to topic  %s ->(%s)'%(self.type, self.name, self.command_topic, e))
				return -1		
		else:
			rospy.logerr('%s-%s:setup: No command topic supplied.'%(self.type, self.name))
			return -1
				
		
		
		self.initialized = True
		
		return 0
		
		
		self.initialized = True
		
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
		self.command_publisher.publish(self.joint_state.position[0])
		
		return 0
		
		
		return 0
		
	
