import numpy as np
import cv2
import cv2
import numpy as np
import os
import math
from cv2 import imshow
from numpy import imag
import scipy.io
import time
from VideoProcessing.VideoProcessor import VideoProcessor
from VideoProcessing.FBError import FBError
start_time = time.time()
Images = []
FinalPoints = []

ImagesNames = []
path  = '/home/isam/HazenWork-Hasnain/Sample3/'
i=0
k=30
B = [1640,1700,183,211]
obj = VideoProcessor()
obj2 = FBError()
ImagesNames = obj.GetImagesName(path)

FinalPoints = obj2.FlowEstimation(ImagesNames,12)
#FinalPoints= np.array(FinalPoints)

Test = obj.GetFinalMatrix2(FinalPoints,ImagesNames)
Test = np.array(Test)

#test = np.array(FinalPoints[len(FinalPoints)-1])
scipy.io.savemat('/home/isam/HazenWork-Hasnain/TrackingGoodPoints/Experiment1/GoodPoints.mat', {'GoodPoints': Test})



print("--- %s seconds ---" % (time.time() - start_time))

#cv2.imshow('test',Images[1])
#cv2.waitKey()
