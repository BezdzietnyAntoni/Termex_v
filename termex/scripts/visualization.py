#!/usr/bin/env python
import rospy
import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int8MultiArray



class VisualizationVectors:
	def __init__(self):
		self.raw_image = None
		self.vector_matrix = None
		
		# get current image rescale and display vector on it 
		# TODO get image with the smame time stemp 
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/lepton_output", Image, self.callbackImage)
		
		# topic with matrx of vector rellocation dx, dy, corr coeff [0 255]
		self.vector_sub = rospy.Subscriber("/vector_matrix", Int8MultiArray, self.callbackVector)
		
		self.image_pub = rospy.Publisher("/image_vector", Image, queue_size=10)

		
	def callbackImage(self, data):
		# it will be use when image msg comming
		# in this example geting image when vector of rellocation comming
		try:
			self.raw_image = self.bridge.imgmsg_to_cv2(data,"mono16")
		except CvBridgeError as e:
			print(e)
		
		
	def callbackVector(self, data):
		# when vector message come
		self.vector_matrix = data
		self.prepareVisualization()
		
		
	def prepareVisualization(self):
		#its only for help
	    scale = 8
	    recal_image = self.recalculate(self.raw_image)
	    resized_image = self.rescale_image(recal_image, scale)
	    arrowed_image = self.add_arrows(resized_image, scale)
	    self.image_pub.publish(self.bridge.cv2_to_imgmsg(arrowed_image,"mono8"))
		
		
	def recalculate(self, data):
		"""
		:param data: numpy array
		:return:Array with values from 0-255 (8bits)
		"""

		minimum = np.min(data)
		maximum = np.max(data)
		difference = maximum - minimum

		data = (((data - minimum) / difference) * 255).astype('uint8')

		return data
		
		
	def rescale_image(self, image, scale):
		height, width = image.shape[:2]
		resize_image = cv.resize(image,(scale * width, scale * height),interpolation=cv.INTER_CUBIC)
		return resize_image
		
	
	def add_arrows(self, image, scale):
		size2Dmatrix = self.vector_matrix.layout.dim[1].stride 
		cols = self.vector_matrix.layout.dim[1].size
		rows = self.vector_matrix.layout.dim[0].size
		
		for i in range(size2Dmatrix):
			x_pos = i%cols
			y_pos = i//rows
			cv.arrowedLine(image, (x_pos*scale, y_pos*scale), ((x_pos+self.vector_matrix.data[i])*scale, (y_pos-self.vector_matrix.data[i+size2Dmatrix])*scale), (0,0,0), 1)
		return image
		
		
def main():
    vis = VisualizationVectors()
    rospy.init_node("Visualization", anonymous=True)
    rospy.spin()


if __name__ == '__main__':
    main()

		
