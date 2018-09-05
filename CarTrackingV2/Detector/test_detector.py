import queue

import cv2

from Detector.DetectorSSD import Detector as Detector
from Detector.undistort import undistort_2560
from Tracker.VideoProcessing.VideoProcessor import VideoProcessor

# 1. Create detector object
detector = Detector('/home/isam/HazenWork-Hasnain/PycharmProjects/CarTrackingV2/Detector/Gate_90.6.pth')

# 2. Read an image to be used for testing the detector
im = cv2.imread('/home/isam/HazenWork-Hasnain/PicsDataset/1.jpg', 1)

Images = []
FinalPoints = []

ImagesNames = []
path  = '/home/isam/HazenWork-Hasnain/Sample2/'
# 3. Run detector
obj = VideoProcessor()
ImagesNames,status = obj.getimages_path(path)
print(ImagesNames[0])
Images = queue.Queue()
grayimages =  queue.Queue()
Images,grayimages,status = obj.getframes(ImagesNames,0,40,Images,grayimages)
loop = Images.qsize()

imagecounter=0
print(loop)
while(imagecounter<=loop-1):
    Img = Images.get()
    got_boxes_flag, bbs = detector.detect(Img)
    undistorted_image = undistort_2560(Img)
    print('bbx',bbs[0])
    b =0
    while(b<=len(bbs)-1):
        Img = obj.draw_rectangles_detector(bbs[b],undistorted_image,imagecounter)
        b=b+1
    imagecounter= imagecounter+1

class Box:
    def getbox(self,img):
        detector = Detector('./Detector/Gate_90.6.pth')
        got_boxes_flag, bbs = detector.detect(Img)
        return bbs

#got_boxes_flag, bbs = detector.detect(im)
#for detection in bbs:
 #   print(detection)

