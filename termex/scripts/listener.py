#!/usr/bin/env python
import rospy
import numpy as np
import cv2 as cv
from std_msgs.msg import UInt16MultiArray


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "Get image")
    display(data)

    
    
def recalculate(data):
    """
    :param data: numpy array
    :return:Array with values from 0-255 (8bits)
    """

    minimum = np.min(data)
    maximum = np.max(data)
    difference = maximum - minimum

    data = (((data - minimum) / difference) * 255).astype('uint8')

    return data


def prepare_image(data):
    """
    :param data:
    :return: Image ready to display
    """
    row = data.layout.dim[0].size  # get row
    col = data.layout.dim[1].size  # get col
    data = np.array(data.data)  # change to data data

    return recalculate(data).reshape(row, col)



def rescale_image(image, scale):
    height, width = image.shape[:2]
    resize_image = cv.resize(image,
                             (scale * width, scale * height),
                             interpolation=cv.INTER_CUBIC)

    return resize_image


def display(data):
    image_in_8b = prepare_image(data)
    scaled_image = rescale_image(image_in_8b, 4)
    scaled_image_colormap = cv.applyColorMap(scaled_image,
                                             cv.COLORMAP_MAGMA)

    #cv.imshow("Termex", scaled_image)
    cv.imshow("Termex Map", scaled_image_colormap)
    
    
    
    

    
def listener():


    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("lepton_image", UInt16MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()
    
def main():

	listener()
	scaled_image_colormap = np.zeros((4*60,4*80)).astype('uint8')
	cv.imshow("Termex Map", scaled_image_colormap)
	while True:
		
		if cv.waitKey(1)==13:
			break
			

	cv.destroyAllWindows()
    
    
    

if __name__ == '__main__':
    main()
