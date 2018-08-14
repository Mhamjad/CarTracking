import cv2
import numpy as np
from numpy.distutils.fcompiler import none

from VideoProcessing.VideoProcessor import VideoProcessor
from VideoProcessing.Ransac import Ransac
from VideoProcessing.Displacement import Displacement
import random

class FBError:





    def FlowEstimation(self,TotalImages,IntervalSize):
        obj = VideoProcessor()
        Ransac_Obj = Ransac()
        Displacement_Obj = Displacement()
        ImageCounter =0
        FinalFlowForward =[]
        FinalPoints=[]
        FlowForward =[]
        #B = [1640,184,1700,211]
        #Box = [1352, 117, 1412, 144] ## image 49
        #x1 = 1352
        #x2 = 1412
        #y1 = 117
        #y2 = 144
        #thefile = open('/home/isam/HazenWork-Hasnain/TrackingGoodPoints/Experiment1/TrackingGoodPoints', 'w')
        Box = [2155, 230, 2230, 260] ## image 49
        x1 = 2155
        x2 = 2230
        y1 = 230
        y2 = 260
        #FirstImagePoints = obj.GetPoints(B) ## Method which get all points inside a box
           # print(len(FirstImage))

        while(ImageCounter<len(TotalImages)-1):


            print('Working of counter ' , ImageCounter)
            Images = obj.GetFrames(TotalImages,ImageCounter,IntervalSize,  method = 'forward')
            #print('Images',Images)

            FirstImagePoints = obj.GetPoints(Box, Images[0], method='gCorners')
            FirstImagePoints = np.array(FirstImagePoints)



            if(len(FirstImagePoints)!= 0 or FirstImagePoints != None):
                OpticalFlowForward,TrackedFirstImagePoints = self.FindForwardPoints2(Images,FirstImagePoints)


            else:
                return FinalPoints


            #obj.DrawCirclesofFarwordPts(TrackedFirstImagePoints,TotalImages[ImageCounter],ImageCounter)

            if(len(OpticalFlowForward)!= 0 or OpticalFlowForward != None):

                OpticalFlowBackward,TrackedFirstImagePoints = self.FindBackWardPoints2(Images,OpticalFlowForward,TrackedFirstImagePoints)
            else:
                return FinalPoints


            if(len(OpticalFlowBackward)!= 0 or OpticalFlowBackward != None):

                GoodPoints = self.GoodPoints(TrackedFirstImagePoints,OpticalFlowForward,OpticalFlowBackward,Images,2,method='ssd')
                print('len of good points',len(GoodPoints))
            else:
                return FinalPoints
            #print('Len of GoodPoints ', len(GoodPoints))


            #if(len(GoodPoints)==0):
             #   GoodPoints =OpticalFlowForward
            GoodPoints=np.array(GoodPoints)

            if(len(GoodPoints)!= 0 or GoodPoints != None):

                TrackedPts  =self.FindForwardPoints(Images,GoodPoints,ImageCounter)
                TrackedPts1=np.array(TrackedPts)
                #print('shape',TrackedPts1.shape)
            else:
                #TrackedPts=obj.CreateGoodPointList(TrackedFirstImagePoints)
                #FinalPoints.append(TrackedPts)

                return FinalPoints



            FinalPoints.append(TrackedPts)










            FlowPoints = self.FlowPoints(GoodPoints,TotalImages[ImageCounter],TotalImages[ImageCounter+1])


            if(ImageCounter==len(TotalImages)-2):
                TrackedPts=obj.CreateGoodPointList(FlowPoints)
                FinalPoints.append(TrackedPts)

            Flow = self.Flow(GoodPoints,FlowPoints)


            Ransac_Flow = Ransac_Obj.Calculate_Ransac2(Flow)



            if(ImageCounter>=0):

                if(len(Ransac_Flow)>0):
                    fx1 = x1 + int(round(Ransac_Flow[0]))
                    fx2 = x2 + int(round(Ransac_Flow[0]))

                    fy1 = y1 + int(round(Ransac_Flow[1]))

                    fy2 = y2 + int(round(Ransac_Flow[1]))
                    B1 = [fx1,fy1,fx2,fy2]

      #
   #                 obj.DrawPoints3(B1,TotalImages[ImageCounter],ImageCounter)
    #                obj.DrawCircles(GoodPoints,TotalImages[ImageCounter],ImageCounter)

                    x1 =fx1
                    x2 =fx2

                    y1 =fy1
                    y2 =fy2
                else:
                    B1=Box
     #               obj.DrawPoints3(B1,TotalImages[ImageCounter],ImageCounter)
      #              obj.DrawCircles(GoodPoints,TotalImages[ImageCounter],ImageCounter)






                #print('Good points', len(GoodPoints))

                #print('Flow Points points', len(Flow))

                #GoodPoints = self.AddFlow(GoodPoints,Flow)
                #FirstImagePoints = GoodPoints


            Box=B1


            ImageCounter =ImageCounter+1

        return FinalPoints







    def FindForwardPoints2(self,Images,PreviousPoints):

        lk_params = dict( winSize  = (10,10),
                          maxLevel = 4,
                          criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))


        FirstImage = PreviousPoints
        FirstImagePoints= PreviousPoints
        PreviousPoints=np.array(PreviousPoints)
        a= 0
        p1=[]
        c=0
        while( a < len(Images)-1):
            next = a+1;


            previousImg = cv2.cvtColor(Images[a], cv2.COLOR_BGR2GRAY)
            #print('previousImg',previousImg)
            nextImg = cv2.cvtColor(Images[next], cv2.COLOR_BGR2GRAY)
            p1, st, err = cv2.calcOpticalFlowPyrLK(previousImg,nextImg, PreviousPoints.astype(np.float32), None, **lk_params)
            if(c==len(Images)-2):
                #print('Points before Zero Forward Flow' , len(p1))
                p1,FirstImagePoints = self.CalculateZeroFlow(p1,FirstImagePoints,4)
                #print('Points after Zero Forward Flow' , len(p1))
                p1=np.array(p1)
                c=0
                #FirstImage = p1
            if np.all(st==0):
                PreviousPoints = None
            else:

                PreviousPoints =p1
            c=c+1
            a=a+1

        return p1,FirstImagePoints


    def FindBackWardPoints2(self,Images,OpticalFlowLastImage,TrackedLastImage):

        lk_params = dict( winSize  = (10,10),
                          maxLevel = 4,
                          criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        a =len(Images)-1;
        p2=[]
        c=0
        FirstPoints = OpticalFlowLastImage
        LastImage=[]
        while(a > 0):
            previous=a-1
            img1Backward = cv2.cvtColor(Images[a], cv2.COLOR_BGR2GRAY)
            img2Backward = cv2.cvtColor(Images[previous], cv2.COLOR_BGR2GRAY)
            p2, st, err = cv2.calcOpticalFlowPyrLK(img1Backward, img2Backward, OpticalFlowLastImage.astype(np.float32), None, **lk_params)

            if(c==len(Images)-2):
                #print('Points before Zero Backward Flow' , len(p2))
                p2,LastImage = self.CalculateZeroFlowBackward(p2,FirstPoints,TrackedLastImage,3)
                #print('Points After Zero Backward Flow' , len(p2))

                p2=np.array(p2)
                c=0
                #FirstPoints = p2
            if np.all(st==0):
                OpticalFlowLastImage = None
            else:
                OpticalFlowLastImage = p2

            c=c+1
            a=a-1


        return p2, LastImage

    def FindForwardPoints(self,Images,PreviousPoints,ImageCounter):

        obj = VideoProcessor()
        lk_params = dict( winSize  = (10,10),
                          maxLevel = 3,
                          criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))



        a= 0
        TrackedGoodPoints=[]
        GoodPoints=obj.CreateGoodPointList(PreviousPoints)
        TrackedGoodPoints.append(GoodPoints[0])
        TrackedGoodPoints.append(GoodPoints[1])
        obj.DrawCircles(PreviousPoints,Images[a],ImageCounter,a)

        while( a < len(Images)-1):
            next = a+1;


            previousImg = cv2.cvtColor(Images[a], cv2.COLOR_BGR2GRAY)
            nextImg = cv2.cvtColor(Images[next], cv2.COLOR_BGR2GRAY)
            p1, st, err = cv2.calcOpticalFlowPyrLK(previousImg,nextImg, PreviousPoints.astype(np.float32), None, **lk_params)




            if np.all(st==0):
                PreviousPoints = None
            else:

                GoodPoints=obj.CreateGoodPointList(p1)
                TrackedGoodPoints.append(GoodPoints[0])
                TrackedGoodPoints.append(GoodPoints[1])
                obj.DrawCircles(p1,Images[next],ImageCounter,next)
                PreviousPoints =p1

            a=a+1

        return TrackedGoodPoints

    def ResversedTracking(self,Images,GoodPoints):
        GoodPoints = np.array(GoodPoints)
        p1 = self.FindForwardPoints(Images,GoodPoints)
        p2 = self.FindBackWardPoints(Images,p1)
        return p2


    def GoodPoints(self,FirstImagePoints,FarwordPoints,BackwardPoints,Images, Threshold, des_phi=0.3, method ='ssd'):

        if(method=='ssd'):
            j=0
            varGoodPoints =[]
            varFarwordPoints=[]
            while (j<len(BackwardPoints)):
                dist = np.linalg.norm(BackwardPoints[j]-FirstImagePoints[j])
                if(dist>Threshold):

                        #print(dist)
                    dist
                else:
                        #print(dist)
                    varGoodPoints.append(FirstImagePoints[j])
                    varFarwordPoints.append(FarwordPoints[j])

                j=j+1


            orb = cv2.ORB_create()
            FirstGrayImage = cv2.cvtColor(Images[0], cv2.COLOR_BGR2GRAY)
            LastGrayImage = cv2.cvtColor(Images[len(Images)-1], cv2.COLOR_BGR2GRAY)

            keyPoints_Good= self.KeyPoints(varGoodPoints)
            keyPoints_Farword= self.KeyPoints(varFarwordPoints)


            Goodkp,GoodptsDes = orb.compute(FirstGrayImage,keyPoints_Good)
            Farwordkp,FarwordptsDes = orb.compute(LastGrayImage,keyPoints_Farword)

            t=10000
            bf = cv2.BFMatcher()

# Match descriptors.
            print('GoodptsDes ',len(GoodptsDes))

            FinalGoodpts=[]

            numPts = GoodptsDes.shape[0]
            for i in range(numPts):
                a = GoodptsDes[i,:]
                a_ = FarwordptsDes[i,:]

                a = a / np.linalg.norm(a)
                a_ = a_ / np.linalg.norm(a_)

                ssd_distance = np.linalg.norm(a-a_)
                if ssd_distance < des_phi:
                    FinalGoodpts.append(varGoodPoints[i])

            return FinalGoodpts

#
#             a=0
#             matches = bf.knnMatch(GoodptsDes,FarwordptsDes, k=2)
#             print('matches len ',len(matches))
# # Apply ratio test
#             good = []
#             for m,n in matches:
#                 if m.distance < 0.8*n.distance:
#                     good.append([m])
#                     FinalGoodpts.append(varGoodPoints[a])
#                 a=a+1
#
#             print('good',len(good))
#             while(t<=len(GoodptsDes)-1):
#                 gddes_magnitude = np.linalg.norm(GoodptsDes[t])
#                 if(gddes_magnitude==0):
#                     gddes_magnitude=0.00001
#                 gdpt_des = (np.divide(GoodptsDes[t],gddes_magnitude))
#                 gdpt_des = np.array(gdpt_des)
#
#                 farworddes_magnitude = np.linalg.norm(FarwordptsDes[t])
#                 if(farworddes_magnitude==0):
#                     farworddes_magnitude=0.00001
#                 farword_des = (np.divide(FarwordptsDes[t],farworddes_magnitude))
#                 farword_des = np.array(farword_des)
#
#                 print(matches)
#                 ssd = np.linalg.norm((gdpt_des-farword_des))
#
#                 if(ssd<=0.7):
#                     FinalGoodpts.append(varGoodPoints[t])
#
#
#                 f=1000
#                 while(f<=len(FarwordptsDes)-1):
#
#
#
#                     pass
#
#                     #print('points shape', gdpt_des.shape, 'des shape',farword_des.shape)
#                     #ssd = np.linalg.norm((gdpt_des-farword_des))
#                     #if(product>=0.8):
#                     #    FinalGoodpts.append(varGoodPoints[t])
#                     #print('ssd of ', t , ' firstimage des with  ' , f , ' of last image des ' ,ssd)
#                     f=f+1
#
#                 t=t+1
#             return FinalGoodpts






    def KeyPoints(self,Points, size=1):
        keypoints=[]

        #print('in function',Points[0][0])
        for pt in Points:
            keypt = cv2.KeyPoint(pt[0], pt[1], size)
            keypoints.append(keypt)

        return keypoints

    def FlowPoints(self,GoodPoints,CurrentImagePath,NextImagePath):
        lk_params = dict( winSize  = (10,10),
                          maxLevel = 3,
                          criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        PreviousImage = cv2.imread(CurrentImagePath)
        NextImage = cv2.imread(NextImagePath)

        img1Backward = cv2.cvtColor(PreviousImage, cv2.COLOR_BGR2GRAY)
        img2Backward = cv2.cvtColor(NextImage, cv2.COLOR_BGR2GRAY)
        GoodPoints = np.array(GoodPoints)
        flow, st, err = cv2.calcOpticalFlowPyrLK(img1Backward, img2Backward, GoodPoints.astype(np.float32), None, **lk_params)
        if np.all(st==0):
            return None
        else:
            return flow

    def Flow(self,GoodPoints,FlowPoints):
        flow =[]

        a=0
        while(a<len(GoodPoints)-1):
            x = FlowPoints[a][0]-GoodPoints[a][0]
            y =FlowPoints[a][1]-GoodPoints[a][1]

            flow.append([x,y])
            a=a+1


        return flow

    def CalculateZeroFlow(self,CurrentImage,FirstImagePoints,Threshold):
        a=0
        Flow =[]
        FirstImage = []
        while(a<len(CurrentImage)-1):
            if(np.linalg.norm(CurrentImage[a]-FirstImagePoints[a])>Threshold):
                Flow.append(CurrentImage[a])
                FirstImage.append(FirstImagePoints[a])
            a=a+1
        if(len(Flow)==0):
            print('Calling Farword flow function')
            Threshold=Threshold-1
            self.CalculateZeroFlow(CurrentImage,FirstImagePoints,Threshold)
        else:

            return Flow , FirstImage

    def CalculateZeroFlowBackward(self,CurrentImage,PreviousImage,TrackedImagePoints,Threshold):
        a=0
        Flow =[]
        LastImage = []
        while(a<len(CurrentImage)-1):
            if(np.linalg.norm(CurrentImage[a]-PreviousImage[a])>Threshold):
                Flow.append(CurrentImage[a])
                LastImage.append(TrackedImagePoints[a])
            a=a+1

        if(len(Flow)==0):
            print('Calling Backword flow function')
            Threshold=Threshold-1
            self.CalculateZeroFlow(CurrentImage,PreviousImage,TrackedImagePoints,Threshold)
        else:

            return Flow,LastImage




