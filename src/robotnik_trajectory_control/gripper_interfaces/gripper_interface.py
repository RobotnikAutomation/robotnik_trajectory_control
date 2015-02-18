#!/usr/bin/env python
"""
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

class StandardGripperInterface:
	"""
			Abstract class to set the position of the Gripper	
	"""
	def setup(self):
		"""
			Setups the component.
			Tries to connect the action server
			@return 0 if OK
			@return -1 if ERROR
		"""
		raise NotImplementedError("StandardGripperInterface::setup: Should have implemented this" )
		
		
	def setPositionGoal(self, position_goal):
		"""
			Sets the position of the gripper
			@param postion as GripperCommand
			@return 0 if it's send successfully
			@return -1 if ERROR		
		"""
		raise NotImplementedError("StandardGripperInterface::setPositionGoal: Should have implemented this" )
		
		
	def isGoalReached(self):
		"""
			Returns 0 if the position Goal is reached
			Returns -1 if the position hasn't been reached yet
			Returns -2 if it has been an error
		"""
		
		raise NotImplementedError("StandardGripperInterface::isGoalReached: Should have implemented this" )
		
		
	def cancelGoal(self):
		"""
			Cancels the current position
		"""
		
		raise NotImplementedError("StandardGripperInterface::cancelGoal: Should have implemented this" )
