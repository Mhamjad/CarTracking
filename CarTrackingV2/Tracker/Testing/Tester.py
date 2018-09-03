import time

import cv2

from Tracker.Tracking.Tracker import Tracker
from Tracker.VideoProcessing.VideoProcessor import VideoProcessor

start_time = time.time()
Images = []
FinalPoints = []

ImagesNames = []
path  = '/home/isam/HazenWork-Hasnain/Sample3/'
#path  = '/media/isam/CE0A03DD0A03C187/Hasnain-Hazenwork/sample4/'
i=0
k=30
B = [2155, 230, 2230, 260]
obj = VideoProcessor()

imagespath, status = obj.getimages_path(path)

tracker_obj = Tracker()

## By default some parameters of tracker
## geomatric_threshold ='2.0',forward_zeroflow_threshold='4.0',backward_zeroflow_threshold='3.0',descriptor_threshold='0.3'
## boxflow_method='ransac',ransac_method='median'
## max_corners=1000,qualitylevel=0.0001,min_distance=1,use_harrisdetector_boolean=True,k=0.001


d = tracker_obj.Tracking(imagespath,10)




#print('Images len', len(imagespath) , 'status', status)
#images = queue.Queue()
#grayimages = queue.Queue()

#images,grayimages,status =  obj.getframes(imagespath,0,10,images,grayimages)
#print(images.qsize())
print("--- %s seconds ---" % (time.time() - start_time))
cv2.waitKey()

