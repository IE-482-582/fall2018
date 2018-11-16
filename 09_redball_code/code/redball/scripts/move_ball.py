#!/usr/bin/env python

# move_ball.py
# rosrun redball move_ball.py

import sys
import rospy
import random
import math

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState


# Define some Global Parameters
# ----------You may want to edit these----------
[MIN_X, MAX_X] 			= [-4.0, 6.0]		# [meters].  We're specifying the bounds of the playground.
[MIN_Y, MAX_Y] 			= [-2.7, 7.3]		# [meters]
[MIN_Z, MAX_Z] 			= [ 0.1, 2.0]		# [meters]

[MIN_SPEED, MAX_SPEED] 	= [ 0.1, 0.8]		# [m/s].  Specify the allowable range of speeds for the model.

MODEL_NAME				= 'red_ball_1'		# Name of the Gazebo model/object we're moving.

RATE 					= 2					# [Hz].  How frequently do we send commands?
# ----------------------------------------------

class moveBall():
	def __init__(self):

		rospy.init_node('move_ball', anonymous=True)
		rate = rospy.Rate(RATE) # 1hz
		
		# When the user quits, call the shutdown function:
		rospy.on_shutdown(self.shutdown)		
		
		# Wait for two services:
		# 1) get_model_state: Allows us to find out where the ball is initially.
		print 'Waiting for "/gazebo/get_model_state" service to become available...'
		rospy.wait_for_service('/gazebo/get_model_state')
		print 'DONE'

		# 2) set_model_state: Allows us to move the ball.  
		print 'Waiting for "/gazebo/set_model_state" service to become available...'
		rospy.wait_for_service('/gazebo/set_model_state')
		print 'DONE'

		# Define handlers to call each service:
		self.call_get_model_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
		self.call_set_model_state = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)

			
		# We want the model to be stationary initially
		(cmd, nextTime) = self.setStationary(MODEL_NAME)
		isMoving = False 		# We'll start off with a stationary model.


		while not rospy.is_shutdown():

			# Assume that we'renot going to publish a command.
			pubCmd = False
						
			if (rospy.Time.now() >= nextTime):
				# It's now time to publish a command
				pubCmd = True
				
				if (isMoving):
					# We want the model to be stationary now.
					(cmd, nextTime) = self.setStationary(MODEL_NAME)
					isMoving = False
				else:
					# We want the model to move now.	
					(cmd, nextTime) = self.setMove(MODEL_NAME)
					isMoving = True
			else:
				if (not isMoving):
					# If we're not moving, 
					# we want to keep publishing the same command
					# to keep the model in place.
					# NOTE: We turned off gravity, and the model tends to float away.
					pubCmd = True

			if (pubCmd):
				# Issue the service request to move the model:
				myResponse = self.call_set_model_state(cmd)
				# print myResponse
	
				# myResponse.success will be 'True' if everything is OK.
				# Otherwise, print an error message:
				if (not myResponse.success):
					print 'Error:'
					print myResponse.status_message
				
			
			rate.sleep()


	def whereAmI(self, model_name):
		# This function finds the position of a model, as determined by Gazebo.

		# 1) Call Gazebo Service:
		modelLoc = self.call_get_model_state(model_name, '')
		# print modelLoc

		# 2) Grab just the info we need:
		[x, y, z] = [modelLoc.pose.position.x, modelLoc.pose.position.y, modelLoc.pose.position.z]
		# print x, y, z
		
		# 3) Return this info:
		return (x, y, z)

	def setStationary(self, model_name):
		# Where is the model right now?
		(x, y, z) = self.whereAmI(model_name)
		print 'NOW:', x, y, z
		
		# How long do we want this model to stay still?
		duration = random.randint(3,5)		# 3, 4, or 5 seconds

		# At what time should we stop executing this command?
		nextTime = rospy.Time.now() + rospy.Duration(duration)

		# Initialize an empty 'ModelState' message.
		# We have to send this type of message to the service to get the model to move.
		cmd = ModelState()

		# Add some details to the message:
		cmd.model_name 		= model_name
		cmd.reference_frame	= 'world'
		cmd.pose.position.x	= min(max(x, MIN_X), MAX_X)
		cmd.pose.position.y	= min(max(y, MIN_Y), MAX_Y)
		cmd.pose.position.z	= min(max(z, MIN_Z), MAX_Z)
		# NOTE:  The linear accel. has been set to zero.

		# print cmd
		
		return (cmd, nextTime)

	def setMove(self, model_name):
		# Where is the model right now?
		(xNow, yNow, zNow) = self.whereAmI(model_name)

		# Where do you want it to go?
		(xGoal, yGoal, zGoal) = (random.uniform(MIN_X, MAX_X), random.uniform(MIN_Y, MAX_Y), random.uniform(MIN_Z, MAX_Z))
		print 'GOAL:', xGoal, yGoal, zGoal
		
		# How far is it from the current location to the goal location?
		dist = math.sqrt( (xNow-xGoal)**2 + (yNow-yGoal)**2 + (zNow-zGoal)**2 )
		
		# How fast should the model move?
		speed = random.uniform(MIN_SPEED, MAX_SPEED)
		
		# How long will it take to get to the goal location?
		duration = dist/speed		

		# At what time should we stop executing this command?
		nextTime = rospy.Time.now() + rospy.Duration(duration)
		
		# Initialize an empty 'ModelState' message.
		# We have to send this type of message to the service to get the model to move.
		cmd = ModelState()

		# Add some details to the message:
		cmd.model_name 		= model_name
		cmd.reference_frame	= 'world'
		cmd.pose.position.x	= xNow
		cmd.pose.position.y	= yNow
		cmd.pose.position.z	= zNow
		cmd.twist.linear.x	= (xGoal - xNow) / duration
		cmd.twist.linear.y	= (yGoal - yNow) / duration
		cmd.twist.linear.z	= (zGoal - zNow) / duration

		# print cmd
		
		return (cmd, nextTime)
		
		 
	def shutdown(self):
		rospy.loginfo("Shutting down the node...")
		rospy.sleep(1)

	

if __name__ == '__main__':
	try:
		moveBall()
	except rospy.ROSInterruptException:
		pass
