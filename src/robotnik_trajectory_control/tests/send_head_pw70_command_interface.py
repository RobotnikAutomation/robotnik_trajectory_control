#!/usr/bin/env python

'''
	Program to test the Lwa4pCommandInterface, that interacts with the Powerball command interface

'''

import rospy

from device_command_interfaces import DeviceCommandInterface

def main():

	rospy.init_node("get_traj_params_test")
	
	
	
	args = {}
	
	groups = None

	try:
		if rospy.search_param('groups'): 
			groups = rospy.get_param('groups')
		else:
			rospy.logerr('Param groups not found!')
		#print name
	except rospy.ROSException, e:
		rospy.logerror('%s: %s'%(e, _name))
		
	
	d = None
	
	if groups:
		
		print 'Param groups:'
		print groups
		
		dci_array = []
		
		for i in groups:
			#args = {}
			#args['name'] = i
			type_group = groups[i]['type']
			
			if type_group == 'pw70':
				d = DeviceCommandInterface( args = groups[i])
				dci_array.append(d)
				
	if len(dci_array) > 0:
		
		for dci in dci_array:
			if dci.setup() == 0:
				print '%s: Setup OK'%dci.name	
				j = 0.2
				
				for i in dci.joint_names:
					if dci.setDesiredJointValue(i, [j, j, j]) != 0:
						print '%s: Error setting joint %s'%(dci.name	, i)
				
				wrong_joint = 'j12222'
				if dci.setDesiredJointValue(wrong_joint, [0, 0, 0]) != 0:
					print '%s: Error setting joint %s'%(dci.name, wrong_joint)							
			else:
				print '%s: Setup ERROR'%dci.name
				
		while not rospy.is_shutdown():	
			for dci in dci_array:
				if dci.sendCommand() != 0:
					print '%s: Error sending command'%(dci.name	)
				else:
					print '%s: Send command OK'%(dci.name	)
			rospy.sleep(0.1)
			
			#if rospy.is_shutdown_requested():
			#	for dci in dci_array:
			#		#stops movement
			#		print 'Stoping!'
			#		dci.stop()
		
		
	else:
		print 'No devices defined'
			

if __name__ == "__main__":
	main()
