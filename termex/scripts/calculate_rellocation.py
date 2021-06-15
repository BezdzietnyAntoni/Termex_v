#!/usr/bin/env python
import rospy
import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import MultiArrayDimension



class BlobsRellocation:
	def __init__(self):
		
		self.previous_image = None
		self.inProcess = False
		#its a vector of rellocation for single pixels size=(img_rows, img_cols)
		self.dx = None
		self.dy = None
		self.corr = None
		
		#create message
		self.matrix = Int8MultiArray()
		#customization message
		self.customizationVectorMessage()
		
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/lepton_output", Image, self.callback)
		self.vector_pub = rospy.Publisher("/vector_matrix", Int8MultiArray, queue_size=10)

	
	def customizationVectorMessage(self):
		# matrix size row*col*3(dx,dy,precision)
		self.matrix.layout.dim.append(MultiArrayDimension())
		self.matrix.layout.dim.append(MultiArrayDimension())
		self.matrix.layout.dim.append(MultiArrayDimension())
		self.matrix.layout.dim[0].label = "rows"
		self.matrix.layout.dim[0].size = 60
		self.matrix.layout.dim[0].stride = 60*80*3
		self.matrix.layout.dim[1].label = "cols"
		self.matrix.layout.dim[1].size = 80
		self.matrix.layout.dim[1].stride = 60*80
		self.matrix.layout.dim[2].label = "corr"
		self.matrix.layout.dim[2].size = 3
		self.matrix.layout.dim[2].stride = 3
		self.matrix.layout.data_offset = 0 
		

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

		#calculate rellocation
	    self.calculateRellocation(current_image)
	    self.createMessage()
	    self.vector_pub.publish(self.matrix)
	    
	    #after calculate and sending vector save current as prev
	    self.previous_image = current_image
	    self.inProcess = False
	    
	    
	def createMessage(self):
		self.dx = self.dx.reshape(-1)
		self.dy = self.dy.reshape(-1)
		self.corr = self.corr.reshape(-1)
		
		size = self.dx.size
		for i in range(size):
			self.matrix.data = np.concatenate([self.dx, self.dy, self.corr])
			"""
			self.matrix.data[i] = self.dx[i]
			self.matrix.data[i+size] = self.dy[i]
			self.matrix.data[i+2*size] = self.corr[i]
			"""
	    
	def calculateRellocation(self, current_image):
		cols, rows = current_image.shape
		
		#its only for help now
		spread = 10.0
		self.dx = np.array([np.linspace(-spread, spread, rows), ]*cols).astype(int)
		self.dy = np.array([np.linspace(spread, -spread, cols), ]*rows).transpose().astype(int)
		self.dx = self.dx.reshape(self.dx.size)
		self.dy = self.dy.reshape(self.dy.size)
		self.corr = np.random.randint(127, size=(rows,cols))


def main():
    blobs_relo = BlobsRellocation()
    rospy.init_node("blobs_rellocation", anonymous=True)
    rospy.spin()


if __name__ == '__main__':
    main()
