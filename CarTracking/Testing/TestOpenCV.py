import numpy as np
import cv2
import cv2
import numpy as np
import os
import math
from cv2 import imshow
from numpy import imag
import time
from VideoProcessing.VideoProcessor import VideoProcessor
from VideoProcessing.FBError import FBError
start_time = time.time()
Images = []
FinalPoints = []

ImagesNames = []
path  = '/home/isam/HazenWork-Hasnain/Sample/'
i=0
k=30
B = [1640,1700,183,211]
obj = VideoProcessor()
obj2 = FBError()
ImagesNames = obj.GetImagesName(path)

FinalPoints = obj2.FlowEstimation(ImagesNames,25)



print('length', len(FinalPoints))
#Images = obj.GetFrames(ImagesNames,i,k)
#while(i<k):

#cv2.rectangle(Images[1],(1640,183),(1700,211),(255,0,0),5)
#cv2.circle(Images[1],(1640,183), 3, (0,0,255),-1)
#cv2.imshow('Testing',Images[1])
 #   img = cv2.imread(path + str(i) + '.jpg')
#    print(img)
 #   if img is not None:
  #      Images.append(img)
   # i=i+1

#print(len(Images))
print("--- %s seconds ---" % (time.time() - start_time))

#cv2.imshow('test',Images[1])
#cv2.waitKey()
