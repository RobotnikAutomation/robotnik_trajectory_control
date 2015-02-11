#!/usr/bin/env python
'''
	Publishes constant joint state values
'''


import rospy

from sensor_msgs.msg import JointState

DEFAULT_FREQ=10.0

def main():

	rospy.init_node("rt_static_joint_publihser")
	
	
	_name = rospy.get_name().replace('/','')
	
	arg_defaults = {
		'desired_freq': DEFAULT_FREQ
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
			
	
	# Process params
	freq = args['desired_freq']
	
	try:
		if rospy.search_param('fake_joints'): 
			fake_joints = rospy.get_param('fake_joints')
		else:
			rospy.logerr('Param fake_joints not found!')
			return -1
	except rospy.ROSException, e:
		rospy.logerror('%s'%(e))
		return -1
	
	# array with all the command interfaces created
	fake_joints_dict = {}
	
	for i in fake_joints:
		joint_value = fake_joints[i]['value']
		joint_name = fake_joints[i]['name']
		
		if fake_joints_dict.has_key(joint_name):
			rospy.logerr('Error on the joint name %s, already exists. Discarded'%(joint_name))
		else:
			fake_joints_dict[joint_name] = {'value': joint_value}
	
	# If there are no command interfaces setup is aborted		
	if len(fake_joints_dict) == 0:
		rospy.logerr('No joints defined')	
		return -1
	
	
	fake_joints = JointState()
	
	joint_state_publisher = rospy.Publisher('joint_states', JointState)
	
	for i in fake_joints_dict:
		fake_joints.name.append(i)
		fake_joints.position.append(fake_joints_dict[i]['value'])
		fake_joints.velocity.append(0.0)
		fake_joints.effort.append(0.0)
	
	"""
	fake_joints.name = ['waist_joint']
	fake_joints.position = [0]
	fake_joints.velocity = [0]
	fake_joints.effort = [0]
	"""
	
	t_sleep = 1/freq
	while not rospy.is_shutdown():
		fake_joints.header.stamp = rospy.Time.now()
		joint_state_publisher.publish(fake_joints)
		rospy.sleep(t_sleep)
	
	
	joint_state_publisher.unregister()

if __name__ == "__main__":
	main()
	exit()
