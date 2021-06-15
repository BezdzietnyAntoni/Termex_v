import numpy as np
import cv2 as cv


class TermexCorr:
    def __init__(self, blob_size=40, distance_between_blobs=(6, 8)):
        # Image data
        self.image_rows = None
        self.image_cols = None

        # Data for corr
        self.blob_size = blob_size
        self.distance_between_blobs = distance_between_blobs
        self.blobs_position = None
        self.zero_mask = None

    def prepareCorrelationSetting(self, image):
        """
        Function have to use before use the class.
        :param image: Image with the same size
        :return: None
        """
        self.image_rows, self.image_cols = image.shape[:2]
        self.zero_mask = np.zeros((self.image_rows, self.image_cols), dtype=float)
        self._prepareBlobPosition()
        return len(self.blobs_position)

    def _prepareBlobPosition(self):
        row_mid_blob = self.image_rows // 2 - self.blob_size // 2
        col_mid_blob = self.image_cols // 2 - self.blob_size // 2

        row_first_blob = row_mid_blob % self.distance_between_blobs[0]
        col_first_blob = col_mid_blob % self.distance_between_blobs[1]

        blobs_position_row = np.arange(row_first_blob,
                                       self.image_rows - self.blob_size + 1,
                                       self.distance_between_blobs[0])
        blobs_position_col = np.arange(col_first_blob,
                                       self.image_cols - self.blob_size + 1,
                                       self.distance_between_blobs[1])

        self.blobs_position = [[row, col] for row in blobs_position_row for col in blobs_position_col]

    def calculateRelocation(self, current_image, previous_image):
        """
        Function calculate max value of phase corr for all blobs.
        :param current_image:
        :param previous_image:
        :return: vector n x 5
        """
        #current_image = self.recalculate(current_image)
        #previous_image = self.recalculate(previous_image)
        
        current_image = current_image.astype(float)

        # 5 is count of params row,col,new_row,new_col, corr
        blobs_shift = np.zeros((len(self.blobs_position), 5)).astype("int8")
        for i in range(len(self.blobs_position)):
            max_corr = self.findMaximalFit(current_image, previous_image, self.blobs_position[i])
            blobs_shift[i][0] = self.blobs_position[i][0]
            blobs_shift[i][1] = self.blobs_position[i][1]
            blobs_shift[i][2] = max_corr[0]
            blobs_shift[i][3] = max_corr[1]
            blobs_shift[i][4] = max_corr[2]

        return blobs_shift

    def findMaximalFit(self, current_image, previous_image, blob_position):
        """
        Function calculate max phase correlation, return shift of blob.
        :param current_image: current image getting to correlation
        :param previous_image: previous image getting to correlation
        :param blob_position: its left up corner of blob
        :return: tuple [y x z] (y and x) its shift, z factor corr in int
        """
        self.zero_mask[0:self.blob_size, 0:self.blob_size] = \
            previous_image[blob_position[0]:blob_position[0] + self.blob_size,
            blob_position[1]:blob_position[1] + self.blob_size]

        corr = cv.phaseCorrelate(self.zero_mask, current_image)

        max_corr_int8 = np.array([corr[0][1],
                                  corr[0][0],
                                  np.round(corr[1] / (1. / np.iinfo("int8").max))]).astype("int8")
                                  
        print(blob_position)
        print(max_corr_int8)
        
        return max_corr_int8
        
        
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
