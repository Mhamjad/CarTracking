import cv2
import numpy as np
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
        FlowForward =[]
        B = [1640,184,1700,211]
        x1 = 1640
        x2 = 1700
        y1 = 184
        y2 = 211
        #FirstImagePoints = obj.GetPoints(B) ## Method which get all points inside a box
           # print(len(FirstImage))

        while(ImageCounter<len(TotalImages)-1):


            print('Working of counter ' , ImageCounter)
            FirstImagePoints = obj.GetPoints(B)
            FirstImagePoints = np.array(FirstImagePoints)
            #print(FirstImagePoints)
            Images = obj.GetFrames(TotalImages,ImageCounter,IntervalSize)
            #print('lenght of frames in every iteration ',len(Images))
            #print('First frame in every iteration',Images[0][1])
            #print('last frame in every iteration', Images[len(Images)-1][1])

            OpticalFlowForward = self.FindForwardPoints(Images,FirstImagePoints)


            #print('Forward',len(OpticalFlowForward))
                #LastImage = np.array(OpticalFlowForward)

            OpticalFlowBackward = self.FindBackWardPoints(Images,OpticalFlowForward)

            #print('Backward',len(OpticalFlowBackward))
            GoodPoints = self.GoodPoints(FirstImagePoints,OpticalFlowBackward)

            FlowPoints = self.FlowPoints(GoodPoints,TotalImages[ImageCounter],TotalImages[ImageCounter+1])

            #

            Flow = self.Flow(GoodPoints,FlowPoints)

            #print('First Good Point' , GoodPoints[0])
            #print('First FlowPoint' , FlowPoints[0])
            #print('First point flow' , Flow[0])

            Ransac_Flow = Ransac_Obj.Calculate_Ransac2(Flow)
            #Displace = Displacement_Obj.FindDisplacement(Flow)
            #print('Lenght Ransac_Flow',len(Ransac_Flow))
            #print('Ransac_Flow values',Displace)


            if(ImageCounter>=0):

                if(len(Ransac_Flow)>0):
                        fx1 = x1 + int(round(Ransac_Flow[0]))
                        fx2 = x2 + int(round(Ransac_Flow[0]))

                        fy1 = y1 + int(round(Ransac_Flow[1]))

                        fy2 = y2 + int(round(Ransac_Flow[1]))
                        B1 = [fx1,fy1,fx2,fy2]

  #
                        obj.DrawPoints3(B1,TotalImages[ImageCounter],ImageCounter)
                       # obj.DrawCircles(GoodPoints,TotalImages[ImageCounter],ImageCounter)

                        x1 =fx1
                        x2 =fx2

                        y1 =fy1
                        y2 =fy2





            #print('Good points', len(GoodPoints))

            #print('Flow Points points', len(Flow))

            #GoodPoints = self.AddFlow(GoodPoints,Flow)
            #FirstImagePoints = GoodPoints

            B=B1
            ImageCounter =ImageCounter+1

        return FinalFlowForward




    def FindForwardPoints(self,Images,PreviousPoints):

        lk_params = dict( winSize  = (25,25),
                          maxLevel = 4,
                          criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))



        a= 0
        p1=[]
        while( a < len(Images)-1):
            next = a+1;


            previousImg = cv2.cvtColor(Images[a], cv2.COLOR_BGR2GRAY)
            nextImg = cv2.cvtColor(Images[next], cv2.COLOR_BGR2GRAY)
            p1, st, err = cv2.calcOpticalFlowPyrLK(previousImg,nextImg, PreviousPoints.astype(np.float32), None, **lk_params)

            if np.all(st==0):
                PreviousPoints = None
            else:

                PreviousPoints =p1

            a=a+1

        return p1


    def FindBackWardPoints(self,Images,OpticalFlowLastImage):

        lk_params = dict( winSize  = (25,25),
                          maxLevel = 4,
                          criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        a =len(Images)-1;

        p2=[]
        while(a > 0):
            previous=a-1
            img1Backward = cv2.cvtColor(Images[a], cv2.COLOR_BGR2GRAY)
            img2Backward = cv2.cvtColor(Images[previous], cv2.COLOR_BGR2GRAY)
            p2, st, err = cv2.calcOpticalFlowPyrLK(img1Backward, img2Backward, OpticalFlowLastImage.astype(np.float32), None, **lk_params)
            if np.all(st==0):
                OpticalFlowLastImage = None
            else:
                OpticalFlowLastImage = p2

            a=a-1


        return p2




    def GoodPoints(self,FarwordPoints,BackwardPoints):
        j=0
        GoodPoints =[]
        while (j<len(BackwardPoints)):
            dist = np.linalg.norm(BackwardPoints[j]-FarwordPoints[j])
            if(dist >3):
                #print(dist)
                dist
            else:
                #print(dist)
                GoodPoints.append(BackwardPoints[j])

            j=j+1
        return GoodPoints

    def FlowPoints(self,GoodPoints,CurrentImagePath,NextImagePath):
        lk_params = dict( winSize  = (25,25),
                          maxLevel = 4,
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

    def CalculateDistance(self,FirstFrame,SecondFrame):

        avg=[]

        p1 = len(FirstFrame)-1
        p2 = len(SecondFrame)-1

        sumx=0
        sumy=0
        s=0
        if(p1< p2):
            fp =p1
        else:
            fp =p2
        while(s <= fp):



            j=fp
            distancepoint1 = FirstFrame[s]
            distancepoint2 = SecondFrame[s]
            #print('length of p1' , distancepoint1)
            #print('length of p2' , distancepoint2)
            sumx = sumx + int(distancepoint2[0]- distancepoint1[0])
            sumy = sumy + int(distancepoint2[1]- distancepoint1[1])
            s=s+1


        avgx = int(sumx /fp)
        avgy = int(sumy/fp)
        avg.append(avgx)
        avg.append(avgy)
        return avg

    def CalculateFlow(self,FirstPoint,SecondPoint,Number):

        avg=[]

        FirstPoint = FirstPoint[Number]
        x = int(FirstPoint[0]-SecondPoint[0])
        y = int(FirstPoint[1]-SecondPoint[1])
        avg.append(x)
        avg.append(y)


        return avg

    def GetRandomNumber(self,Range,TotalNumber):
        RandomList =[]
        while(len(RandomList)<TotalNumber):
            rand = random.randint(0,Range)
            if random not in RandomList:
                RandomList.append(rand)
            else:
                pass
        return  RandomList

    def GetPortion(self,List,RandomList):
        a =0
        ReturnList =[]



        while(a<len(RandomList)-1):

            print('Get Random Index',RandomList[a])
            ReturnList.append(List[RandomList[a]])
            a=a+1

        return ReturnList


    def AddFlow(self,GoodPoints,Flow):
        a=0
        while(a<len(GoodPoints)-1):
            GoodPoints[a][0] = GoodPoints[a][0]+Flow[a][0]
            GoodPoints[a][1] = GoodPoints[a][1]+Flow[a][1]
            a=a+1
        return GoodPoints


    def FarwordBackwardError(self,TotalImages,IntervalSize):
        obj = VideoProcessor()
        ImageCounter =0
        IntervalCounter =0
        FinalFlowForward =[]
        lk_params = dict( winSize  = (25,25),
                  maxLevel = 4,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        while(ImageCounter<=len(TotalImages)-1):

            print('Working')
            print(ImageCounter)
            Images = obj.GetFrames(TotalImages,ImageCounter,IntervalSize)
            Imagescount = len(Images) -1
            c=1
            OpticalFlowForward =[]
            OpticalFlowBackward =[]


            boxes =0
            BoundingBoxes =[1]  ## pass frame == imagecounter and detect boxes
            while(boxes <= len(BoundingBoxes)-1):
                B = [1640,1700,183,211]
                FirstImage = obj.GetPoints(B) ## Method which get all points inside a box
                FirstImage = np.array(FirstImage)
                while(c<= Imagescount):
                    #print('working.......')
                    z=c
                    img1Forward_gray = cv2.cvtColor(Images[z-1], cv2.COLOR_BGR2GRAY)
                    img2Forward_gray = cv2.cvtColor(Images[c], cv2.COLOR_BGR2GRAY)
                    p1, st, err = cv2.calcOpticalFlowPyrLK(img1Forward_gray, img2Forward_gray, FirstImage.astype(np.float32), None, **lk_params)
                    OpticalFlowForward.append(p1)
                    FirstImage = OpticalFlowForward[z-1]
                    c=c+1

                print(len(OpticalFlowForward))

        #print (Imagescount)
                while(Imagescount >=1):
                    #print('working.......')
                    #print(Imagescount)
                    z=Imagescount
                    img1Backward_gray = cv2.cvtColor(Images[Imagescount], cv2.COLOR_BGR2GRAY)
                    img2Backward_gray = cv2.cvtColor(Images[z-1], cv2.COLOR_BGR2GRAY)
                    p2, st, err = cv2.calcOpticalFlowPyrLK(img1Backward_gray, img2Backward_gray, OpticalFlowForward[z-1].astype(np.float32), None, **lk_params)
                    OpticalFlowBackward.append(p2)
                    Imagescount = Imagescount-1
                a=0
                b= len(OpticalFlowForward)-1
                while(a <1):
                    #print('working......')
                    pointsFortemp = OpticalFlowForward[a]
                    pointsBacktemp = OpticalFlowBackward[b]
                    points1final = []

                    j=0
                    k = len(pointsFortemp)-1
                    while (j<len(pointsFortemp)):
                        dist = np.linalg.norm(pointsFortemp[j] - pointsBacktemp[k])
                        if(dist >5):
                        #print(dist)
                            dist
                        else:
                            points1final.append(pointsFortemp[j])

                        j=j+1
                        k=k-1

                    FinalFlowForward.append(points1final)

                    a=a+1
                    b=b-1
                boxes=boxes+1;
                IntervalSize = IntervalSize+1
            ##Draw Boxes by giving coordinates to method
            ##IntervalCounter = IntervalSize
            ImageCounter =ImageCounter+1

        return FinalFlowForward



    def FarwordBackwardError2(self,TotalImages,IntervalSize):
        obj = VideoProcessor()
        Ransac_Obj = Ransac()
        ImageCounter =0
        FinalFlowForward =[]
        FlowForward =[]
        B = [1600,1720,175,220]
        x1 = 1600
        x2 = 1720
        y1 = 175
        y2 = 220
        FirstImagePoints = obj.GetPoints(B) ## Method which get all points inside a box
           # print(len(FirstImage))

        while(ImageCounter<len(TotalImages)-1):

            FirstImagePoints = np.array(FirstImagePoints)
            print('Working')
            Images = obj.GetFrames(TotalImages,ImageCounter,IntervalSize)





            OpticalFlowForward = self.FindForwardPoints(Images,FirstImagePoints)


            print('Forward',len(OpticalFlowForward))
                #LastImage = np.array(OpticalFlowForward)
            OpticalFlowBackward = self.FindBackWardPoints(Images,OpticalFlowForward)
            print('Backward',len(OpticalFlowBackward))
            FlowForward = self.GoodPoints(FirstImagePoints,OpticalFlowBackward)

            print('Good points',len(FlowForward))

            FinalFlowForward.append(FlowForward)
            FirstImagePoints = FlowForward
            if(ImageCounter>0):
                if(len(FinalFlowForward[ImageCounter-1])>0):




                    Random = self.GetRandomNumber(len(FinalFlowForward[ImageCounter])-1,40)

                    print('Random',Random)

                    Ransac_List = self.GetPortion(FinalFlowForward[ImageCounter],Random)
                    FirstFrame = self.GetPortion(FinalFlowForward[ImageCounter-1],Random)

                    SecondFrame = Ransac_Obj.Calculate_Ransac(Ransac_List)
                    BestFlow =SecondFrame[0]
                    Number =  SecondFrame[1]
                    print('firstframe',FirstFrame)
                    print('secondframe Best',BestFlow)
                    print('Point Number',Number)
                    avg= self.CalculateFlow(FirstFrame,BestFlow,Number)
                    fx1 = x1 + avg[0]
                    fx2 = x2 + avg[0]

                    fy1 = y1 + avg[1]

                    fy2 = y2 + avg[1]
                    B1 = [fx1,fy1,fx2,fy2]
                    print('Box',B1)
                    obj.DrawPoints3(B1,TotalImages[ImageCounter],ImageCounter)

                    x1 =fx1
                    x2 =fx2

                    y1 =fy1
                    y2 =fy2

                    print('AVg',avg)



            ImageCounter =ImageCounter+1


        return FinalFlowForward

