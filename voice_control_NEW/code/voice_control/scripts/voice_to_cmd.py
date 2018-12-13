#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class Listener():
	def __init__(self):
		# Initialize our node:
		rospy.init_node('listener', anonymous=True)

		# Set the shutdown function
		rospy.on_shutdown(self.shutdown)		

		# Define the publish rate:
		rate = rospy.Rate(5) 	# [Hz]

		# Define our publisher:
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)

		# Initialize with an empty Twist message:
		self.twist = Twist()
		
		# Subscribe to the voice commands
		rospy.Subscriber("voice", String, self.voice_callback)
		
		while not rospy.is_shutdown():

			self.cmd_vel_pub.publish(self.twist)
		
			rate.sleep()
			

	def voice_callback(self, msg):
		print(msg.data)

		if (msg.data == 'stop'):
			self.twist 				= Twist()
		elif (msg.data == 'forward'):
			# self.twist 			= Twist()
			self.twist.linear.x 	= 0.5
		elif (msg.data == 'backward'):
			# self.twist 			= Twist()
			self.twist.linear.x 	= -0.25
		elif (msg.data == 'left'):
			# self.twist 			= Twist()
			self.twist.angular.z 	= 0.5
		elif (msg.data == 'right'):
			# self.twist 			= Twist()
			self.twist.angular.z 	= -0.5
		elif (msg.data == 'faster'):
			self.twist.linear.x 	*= 1.25
			self.twist.angular.z 	*= 1.25
		elif (msg.data == 'slower'):
			self.twist.linear.x 	*= 0.75
			self.twist.angular.z 	*= 0.75
		else:
			print('Unknown Command: %s' % (msg.data)) 


	def shutdown(self):
		rospy.loginfo("Shutting down the Cmd node...")
			
		rospy.sleep(1)


if __name__ == '__main__':
	try:
		Listener()
	except rospy.ROSInterruptException:
		pass

