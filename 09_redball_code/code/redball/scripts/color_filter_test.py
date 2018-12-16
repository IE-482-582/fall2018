#!/usr/bin/env python

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
import math

class Follower:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("Original", 1)
		cv2.namedWindow("RedMask", 1)
		cv2.namedWindow("HSV", 1)
		cv2.namedWindow("Visor", 1)
		
		# Open two files for writing:
		# 1) We'll store data for the entire image (640x480):
		self.outFileBig = open("image_data_big.csv",'w')
		# 2) We'll also store data for just the small window used by our mask (640x???):
		self.outFileSmall = open("image_data_small.csv",'w')
		
		# Initialize a counter so we know how many frames we've seen:
		self.frameCount = 0

		# TESTING
		# See https://www.learnopencv.com/blob-detection-using-opencv-python-c/

		# Setup SimpleBlobDetector parameters.
		self.params = cv2.SimpleBlobDetector_Params()

		# Filter by Circularity
		self.params.filterByCircularity = True
		self.params.minCircularity = 0.1

		# Create a detector with the parameters
		ver = (cv2.__version__).split('.')
		print 'cv2 Version: ', ver
		if int(ver[0]) < 3 :
			self.detector = cv2.SimpleBlobDetector(self.params)
		else : 
			self.detector = cv2.SimpleBlobDetector_create(self.params)

		
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)

		
	def image_callback(self, msg):
		
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
	
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		
		h, w, d = hsv.shape
		
		# UNcomment the following line to see the dimensions of the image:
		# print(h, w, d)

		# Increment our frame counter:
		self.frameCount += 1
		
		# Define the range (rows) for our mask:
		# FIXME:  You'll probably want to adjust this.
		search_top = 0 # 3*h/4
		search_bot = 3*h/4 + 20

		# We're only going to save the 10th frame.
		# This will give the system a few frames to stabilize.
		if (self.frameCount == -10):
			# Loop over each pixel in the image
			for row in range(0,h):
				for col in range(0,w):
					myStr = "[%d; %d; %d], " % (hsv[row, col, 0], hsv[row, col, 1], hsv[row, col, 2])
					self.outFileBig.write(myStr)
					# Only write to the "small" file if we're in the narrow range of rows
					if (row in range(search_top, search_bot)):
						self.outFileSmall.write(myStr)
						
				self.outFileBig.write("\n")
				if (row in range(search_top, search_bot)):
					self.outFileSmall.write("\n")
				
			# We're done with the data files.  Close them now.
			self.outFileBig.close()
			self.outFileSmall.close()
			print("Data Files Written.")
						
			# Save the HSV images
			# 1) We'll first save the big image:
			cv2.imwrite("my_hsv_image_big.png", hsv)
			
			# 2) We'll now save the smaller image:
			hsv_small = hsv[search_top:search_bot, 0:w]
			cv2.imwrite("my_hsv_image_small.png", hsv_small)

			print("Images Written.")
						
		
		# Red Mask
		# FIXME:  YOU WILL NEED TO CHANGE THESE VALUES!
		lower_red = np.array([  0, 220,  50]) 
		upper_red = np.array([  0, 255, 255])

		red_mask = cv2.inRange(hsv, lower_red, upper_red)
		red_mask[0:search_top, 0:w] = 0
		red_mask[search_bot:h, 0:w] = 0		
		masked_for_red = cv2.bitwise_and(image, image, mask=red_mask)
			    
		
		
		
		
		'''		
		# Detect blobs.
		keypoints = self.detector.detect(red_mask)
		print keypoints
		 
		# Draw detected blobs as red circles.
		# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
		im_with_keypoints = cv2.drawKeypoints(red_mask, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
		 
		# Show keypoints
		cv2.imshow("Keypoints", im_with_keypoints)
		# cv2.waitKey(0)
		'''



		'''
		# See https://stackoverflow.com/questions/42203898/python-opencv-blob-detection-or-circle-detection
		# img = cv2.imread('rbv2g.jpg',0)
		img = cv2.medianBlur(red_mask,5)
		# cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
		
		circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,10,
		                            param1=50,param2=12,minRadius=10,maxRadius=200)
		
		# print circles		
		circles = np.uint16(np.around(circles))
		for i in circles[0,:]:
		    # draw the outer circle
		    cv2.circle(red_mask,(i[0],i[1]),i[2],(0,255,0),2)
		    # draw the center of the circle
		    cv2.circle(red_mask,(i[0],i[1]),2,(0,0,255),3)
		
		
		cv2.imshow('detected circles',red_mask)
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()		
		'''
		
		
		
		'''
		# See https://stackoverflow.com/questions/21104664/extract-all-bounding-boxes-using-opencv-python
		# Find the smallest rectangular bounding box around the filter:
		
		# im = cv2.imread('c:/data/ph.jpg')
		# gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
		contours,hierarchy, _ = cv2.findContours(red_mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
		print contours
		
		idx =0 
		for cnt in contours:
		    idx += 1
		    x,y,w,h = cv2.boundingRect(cnt)
		    roi=red_mask[y:y+h,x:x+w]
		    # cv2.imwrite(str(idx) + '.jpg', roi)
		    cv2.rectangle(red_mask,(x,y),(x+w,y+h),(200,0,0),2)
		 
		cv2.imshow('img',red_mask)
		'''
		
		
		
		'''
		# See https://docs.opencv.org/3.1.0/d4/d73/tutorial_py_contours_begin.html
		# ret,thresh = cv2.threshold(red_mask,127,255,0)
		im2, contours, hierarchy = cv2.findContours(red_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		cv2.drawContours(red_mask, contours, -1, (200,200,25), 3)		
		cv2.imshow('img', red_mask)
		
		print contours
		'''

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
			cv2.imshow('img', red_mask)
			# cv2.imshow("output", np.hstack([image, output]))
			# cv2.waitKey(0)		



		# Display the original image:
		cv2.imshow("Original", image )
		
		# Display the yellow-filtered image:
		cv2.imshow("RedMask", red_mask ) 
		
		# Display the HSV image:
		cv2.imshow("HSV", hsv)
		
		# Display the visible portion of the original image:
		image_visor = image[search_top:search_bot, 0:w]
		cv2.imshow("Visor", image_visor)


		cv2.waitKey(3)
				
			
	
rospy.init_node('color_filter')
follower = Follower()
rospy.spin()

