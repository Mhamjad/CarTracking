import cv2
import os
import glob
import numpy as np
import scipy.io as sci
class VideoProcessor:


    def GetImagesName(self, path):
        Names = []
        filenames = glob.glob(path + "*.jpg")
        #filenames.sort()
        for filename in os.listdir(path):
            name, ext = filename.split('.')
            name = int(name)
            name = chr(name)
            number = ord(name)
            Names.append(number)

        i=0
        Names = sorted(Names)
        while(i<=len(Names)-1):
            Names[i] = path + str(Names[i]) + '.jpg'
            i=i+1
        return Names

    def GetFrames(self,ListOfImages,FirstImage,IntervalSize, method = 'forward'):
        Images = []
        Status = -1



        if(method=='forward'):
            a= FirstImage+IntervalSize
            while(FirstImage <=a-1):
                if(FirstImage >len(ListOfImages)-1):
                    break
                else:
                    img = cv2.imread(ListOfImages[FirstImage])

                    if img is not None:
                            #Images.append((img,ListOfImages[FirstImage]))
                        Images.append(img)

                    else:
                        pass

                FirstImage=FirstImage+1

        if(method =='reversed'):

            a= FirstImage-IntervalSize

            while(FirstImage >=a):
                if(FirstImage<0):
                    break
                else:

                    img = cv2.imread(ListOfImages[FirstImage])

                    if img is not None:
                            #Images.append((img,ListOfImages[LastImage]))
                        Images.append(img)

                    else:
                        print('You are out of Images')
                        break
                FirstImage=FirstImage-1

        return Images


    def GetReversedFrames(self,ListOfImages,LastImage,NextStep):
        Images = []

        a= LastImage-NextStep

        while(LastImage >=a):
            if(LastImage<0):
                break
            else:

                img = cv2.imread(ListOfImages[LastImage])

                if img is not None:
                    #Images.append((img,ListOfImages[LastImage]))
                    Images.append(img)

                else:
                    print('You are out of Images')
                    break
            LastImage=LastImage-1
        return Images

    def GetPoints(self,BoundingBox, im, method='grid'):

        ImagePoints =[]

        if method=='grid':
            x_Start = BoundingBox[0]
            x_End = BoundingBox[2]

            y_Start = BoundingBox[1]
            y_End = BoundingBox[3]

            j = x_Start
            while(j <= x_End):
                i = y_Start
                while(i <= y_End):
                    temp =[]
                    temp.append(j)
                    temp.append(i)
                    ImagePoints.append(temp)


                    i= i+1

                j =j+1
            return ImagePoints

        elif method=='gCorners':

            im = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)

            box = BoundingBox
            xmin, ymin, xmax, ymax = BoundingBox

            patch = im[ymin:ymax, xmin:xmax]
            corners = cv2.goodFeaturesToTrack(patch, 1000, 0.0001, 1, useHarrisDetector=True, k=0.001)
            corners = corners.reshape( (-1,2) )
            corners[:,0] = corners[:,0]+xmin
            corners[:,1] = corners[:,1]+ymin

            ImagePoints = corners
            return ImagePoints

    def CreateGoodPointList(self,GoodPoints):
        n = len(GoodPoints)
        TrackedGoodPts = [0] * 2*(n)
        TrackedGoodPts= np.array(TrackedGoodPts)
        TrackedGoodPts= np.array(TrackedGoodPts)
        TrackedGoodPts=TrackedGoodPts.reshape((2,-1))
        a=0
        FirstPoint =[]
        SecondPoint =[]
        while(a<=n-1):
            FirstPoint.append(GoodPoints[a][0])
            SecondPoint.append(GoodPoints[a][1])
            a=a+1

        TrackedGoodPts[0,:] = FirstPoint
        TrackedGoodPts[1,:] = SecondPoint
        return TrackedGoodPts


    def GetFinalMatrix(self,TrackedPoints):
        a=0
        sum=0
        FinalMatrix=[]
        while(a<=len(TrackedPoints)-1):
            sum=sum+len(TrackedPoints[a][0])
            a=a+1
        FinalMatrix = [0] * 2*(len(TrackedPoints))*sum
        #print(len(FinalMatrix))
        FinalMatrix = np.array(FinalMatrix)
        l = 2*(len(TrackedPoints))
        FinalMatrix=FinalMatrix.reshape((l,sum))

        b= 0
        c=0
        Start=0
        #End = len(TrackedPoints[0][0])
        #print ('len',len(TrackedPoints[0][0]))
        #print ('len',TrackedPoints[0][1])
        #print('end',End)
        tempx=[]
        tempy=[]

        counter=0
        while(counter<=len(TrackedPoints)-1):
            tempx.append(TrackedPoints[counter][0])
            tempy.append(TrackedPoints[counter][1])
            counter=counter+1
        while(b<=len(FinalMatrix)-1):
            c=b+1
            print(b)
            t =Start
            End = len(tempx[b])

            while(t<=End-1):
                FinalMatrix[b][t] = tempx[t]
                FinalMatrix[c][t] = tempy[t]
                t=t+1
            b=b+2
            Start=End


            #counter=counter+1


        return FinalMatrix


    def GetFinalMatrix2(self,TrackedPoints,TotalImages):
        a=0
        sum=0
        FinalMatrix=[]
        while(a<=len(TrackedPoints)-1):
            sum=sum+len(TrackedPoints[a][0])
            print('sum',sum)
            a=a+1

        FinalMatrix = [0] * 2*(len(TotalImages))*sum
        #print(len(FinalMatrix))
        FinalMatrix = np.array(FinalMatrix)
        l = 2*(len(TotalImages))
        print('l',len(TrackedPoints))
        FinalMatrix=FinalMatrix.reshape((l,sum))
        print('final matrix', FinalMatrix.shape)

        b=0
        cstart=0
        c=0
        while(c<=len(TrackedPoints)-1): #len(TrackedPoints)-1):
            rend=len(TrackedPoints[c])+b
            cend = len(TrackedPoints[c][0])+cstart
            #print('Tracked Points',TrackedPoints[b])

            test =TrackedPoints[c]
            print('RStart',b,'Rend',rend,'Cstart',cstart, 'Cend' ,cend)
            print('test',len(test), len(test[0]))
            print('b',b)
            FinalMatrix[b:rend,cstart:cend]  =test  #print('Get Final Matrix', FinalMatrix[b:rend,cstart:cend]) #= TrackedPoints[b][:,:
            cstart=cend
            b=b+2
            c=c+1




        return FinalMatrix



    def DrawPoints(self,Boxes,Image):
        for Box in Boxes:
            cv2.rectangle(Image,(Box.x1,Box.y1),(Box.x2,Box.y2),(0,255,22),3)
        return Image

    def DrawPoints2(self,Boxes,Image):

        cv2.rectangle(Image,(Boxes[0],Boxes[1]),(Boxes[2],Boxes[3]),(0,255,22),3)
        return Image

    def DrawPoints3(self,OldBox,Image,number):




        Images = cv2.imread(Image,1)
        Images=cv2.rectangle(Images,(OldBox[0],OldBox[1]),(OldBox[2],OldBox[3]),(0,255,22),3)
        name = '/home/isam/HazenWork-Hasnain/Sample2_Results/25Frame/' + str(number) +'.png'
        cv2.imwrite(name,Images)


    def DrawCircles(self,Points,Image,number,Counter):
        #Images = cv2.imread(Image,1)
        a =0;

        if(len(Points)==0):
            pass
        else:

            while(a<=len(Points)-1):
                Images = cv2.circle(Image,(Points[a][0],Points[a][1]), 1, (0,0,255), 1)
                a=a+1

            newpath = '/home/isam/HazenWork-Hasnain/TrackingGoodPoints/Experiment1/Results/'+str(number)+'/'
            if not os.path.exists(newpath):
                os.makedirs(newpath)
            name = newpath + str(Counter) + '.png'

            cv2.imwrite(name,Images)


        #sci.savemat(newpath, mdict={str(number): Array})

    def DrawCirclesofFarwordPts(self,Points,Image,number):

        a =0;
        while(a<len(Points)-1):
            Images = cv2.circle(Image,(Points[a][0],Points[a][1]), 1, (0,0,255), 1)
            a=a+1

        name = '/home/isam/HazenWork-Hasnain/Sample2_GdPts/25Frame/' + str(number) +'Acircle.png'
        cv2.imwrite(name,Images)

    def DrawCirclesofBackwordPts(self,Points,Image,number):
        a =0;
        while(a<len(Points)-1):
            Images = cv2.circle(Image,(Points[a][0],Points[a][1]), 1, (0,0,255), 1)
            a=a+1

        name = '/home/isam/HazenWork-Hasnain/Sample2_GdPts/25Frame/' + str(number) +'Bcircle.png'
        cv2.imwrite(name,Images)






