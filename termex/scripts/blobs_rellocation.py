#!/usr/bin/env python
import rospy
import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class BlobsRellocation:
	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/lepton_output", Image, self.callback)
		self.image_pub = rospy.Publisher("/image_vector", Image, queue_size=10)
		
		self.previous_image = None
		self.inProcess = False
		#its a vector of rellocation for single pixels size=(img_rows, img_cols)
		self.dx = None
		self.dy = None
		self.precision = None
		
		#its a help to draw delate
		self.image_to_view = None
		self.x_pos = None
		self.y_pos = None
		
		
		


	def callback(self, data):
	    try:
	        cv_image = self.bridge.imgmsg_to_cv2(data,"mono16")
	        self.handleNewRellocation(cv_image)
	    except CvBridgeError as e:
	        print(e)
	        
	        
	def handleNewRellocation(self, current_image):
	    #if still calculate
	    if self.inProcess:
	        return
	        
	    #if its first image save and return
	    if self.previous_image is None:
	        self.previous_image = current_image
	        return
	        
	    #set flag in process
	    self.inProcess = True

	    self.calculateRellocation(current_image)
	    
	    #its only for help
	    scale = 8
	    recal_image = self.recalculate(current_image)
	    resized_image = self.rescale_image(recal_image, scale)
	    arrowed_image = self.add_arrows(resized_image, scale)
	    self.image_pub.publish(self.bridge.cv2_to_imgmsg(arrowed_image,"mono8"))
	    
	    #after calculate and sending vector save current as prev
	    self.previous_image = current_image
	    self.inProcess = False
	    
	    
	def calculateRellocation(self, current_image):
		cols, rows = current_image.shape
		
		#its only for help now
		spread = 10.0
		self.dx = np.array([np.linspace(-spread, spread, rows), ]*cols).astype(int)
		self.dy = np.array([np.linspace(spread, -spread, cols), ]*rows).transpose().astype(int)
		self.dx = self.dx.reshape(self.dx.size)
		self.dy = self.dy.reshape(self.dy.size)
		
		self.x_pos = np.array([np.arange(rows), ]*cols).astype(int)
		self.y_pos = np.array([np.arange(cols), ]*rows).transpose().astype(int)
		self.x_pos = self.x_pos.reshape(self.x_pos.size)
		self.y_pos = self.y_pos.reshape(self.y_pos.size)
		
		
		
		
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
		for i in range(self.dx.size):
			cv.arrowedLine(image, (self.x_pos[i]*scale, self.y_pos[i]*8), ((self.x_pos[i]+self.dx[i])*scale, (self.y_pos[i]-self.dy[i])*scale), (0,0,0), 1)
		return image


def main():
    blobs_relo = BlobsRellocation()
    rospy.init_node("blobs_rellocation", anonymous=True)
    rospy.spin()


if __name__ == '__main__':
    main()
