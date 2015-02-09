#!/usr/bin/env python
'''
	Publishes constant joint state values
'''


import rospy

from sensor_msgs.msg import JointState

def main():

	rospy.init_node("rt_static_joint_publihser")
	
	
	_name = rospy.get_name().replace('/','')
	
	arg_defaults = {
	  
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
			
	
	fake_joints = JointState()
	
	joint_state_publisher = rospy.Publisher('joint_states', JointState)
	
	fake_joints.name = ['waist_joint']
	fake_joints.position = [0]
	fake_joints.velocity = [0]
	fake_joints.effort = [0]
	
	freq = 10.0
	t_sleep = 1/freq
	while not rospy.is_shutdown():
		fake_joints.header.stamp = rospy.Time.now()
		joint_state_publisher.publish(fake_joints)
		rospy.sleep(t_sleep)
	
	
	joint_state_publisher.unregister()

if __name__ == "__main__":
	main()
	exit()
