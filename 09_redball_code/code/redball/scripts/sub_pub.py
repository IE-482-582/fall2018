#!/usr/bin/env python

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

import math
import time

class Follower:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		
		cv2.namedWindow("Original", 1)
		'''
		cv2.namedWindow("RedMask", 1)
		cv2.namedWindow("HSV", 1)
		cv2.namedWindow("Visor", 1)
		'''

		self.image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage)		
		
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)


	def image_callback(self, msg):
		
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
	
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		
		h, w, d = hsv.shape
		
		# UNcomment the following line to see the dimensions of the image:
		# print(h, w, d)

		
		# Define the range (rows) for our mask:
		# FIXME:  You'll probably want to adjust this.
		search_top = 0 # 3*h/4
		search_bot = 3*h/4 + 20

						
		
		# Red Mask
		# FIXME:  YOU WILL NEED TO CHANGE THESE VALUES!
		lower_red = np.array([  0, 220,  50]) 
		upper_red = np.array([  0, 255, 255])

		red_mask = cv2.inRange(hsv, lower_red, upper_red)
		red_mask[0:search_top, 0:w] = 0
		red_mask[search_bot:h, 0:w] = 0		
		masked_for_red = cv2.bitwise_and(image, image, mask=red_mask)
			    
		

		# print red_mask.shape
		# print red_mask[133,268]
		# print max(red_mask[:,:])
		print red_mask.max()
		
		M = cv2.moments(red_mask)
		if M['m00'] > 0:
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			cv2.circle(image, (cx, cy), 20, (0,255,), -1)

			print M['m00']


		
		# See https://www.pyimagesearch.com/2014/07/21/detecting-circles-images-using-opencv-hough-circles/
		# detect circles in the image
		circles = cv2.HoughCircles(red_mask, cv2.HOUGH_GRADIENT, 4.5, 40)
		
		print circles
		 
		# ensure at least some circles were found
		if circles is not None:
			# convert the (x, y) coordinates and radius of the circles to integers
			circles = np.round(circles[0, :]).astype("int")
		 
			# loop over the (x, y) coordinates and radius of the circles
			for (x, y, r) in circles:
				# draw the circle in the output image, then draw a rectangle
				# corresponding to the center of the circle
				cv2.circle(red_mask, (x, y), r, (200, 255, 200), 4)
				cv2.rectangle(red_mask, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
				
				# Find the area of the circle:
				area = math.pi * r**2
				print 'Area = ', area, ' Radius = ', r
				
			# show the output image
			#cv2.imshow('img', red_mask)
			# cv2.imshow("output", np.hstack([image, output]))
			# cv2.waitKey(0)		



		# Display the original image:
		cv2.imshow("Original", image )
		
		'''
		# Display the yellow-filtered image:
		cv2.imshow("RedMask", red_mask ) 
		
		# Display the HSV image:
		cv2.imshow("HSV", hsv)
		
		# Display the visible portion of the original image:
		image_visor = image[search_top:search_bot, 0:w]
		cv2.imshow("Visor", image_visor)
		'''

		cv2.waitKey(3)


		#### Create CompressedIamge ####
		msg = CompressedImage()
		msg.header.stamp = rospy.Time.now()
		msg.format = "jpeg"
		msg.data = np.array(cv2.imencode('.jpg', masked_for_red)[1]).tostring()
		# Publish new image
		self.image_pub.publish(msg)			
		
rospy.init_node('color_filter')
follower = Follower()
rospy.spin()

