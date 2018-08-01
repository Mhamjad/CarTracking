'''
Created on Jul 26, 2018

@author: Hashi
'''


while(ImageCounter<=len(ImagesNames)-1):

    print('Working')
    print(ImageCounter)
    boxes =0
    BoundingBoxes =[1]  ## pass frame == imagecounter and detect boxes
    Images = obj.GetFrames(ImagesNames,ImageCounter,20)
    print(len(Images))
    ImageCounter =      ImageCounter+1



import cv2
import numpy as np
import os
import math
from cv2 import imshow
from numpy import imag


Box1 =[]
Box1a =[]
Box2 =[]
Img1Points = []
temp =[]
temp1 =[]
Images = []
lk_params = dict( winSize  = (35,35),
                  maxLevel = 6,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
for filename in os.listdir('qq'):
        img = cv2.imread(os.path.join('qq',filename))
        if img is not None:
            Images.append(img)


Box1.append((540,450))
Box1.append((620,490))
Box1a.append((540,450))
Box1a.append((620,190))
cv2.rectangle(Images[0],Box1[0],Box1[1],(0,255,22),3)

#cv2.imshow('first Image',Images[0])
Start  = Box1[0]
End = Box1[1]

x_Start = Start[0]
x_End = End[0]

y_Start = Start[1]
y_End = End[1]

j = x_Start
while(j <= x_End):
    i = y_Start
    while(i <= y_End):
        temp =[]
        temp.append(j)
        temp.append(i)
        Img1Points.append(temp)


        i= i+1

    j =j+1

FirstImage = np.array(Img1Points)
#print(FirstImage)

Imagescount = len(Images) -1
c=1

OpticalFlowForward = []
OpticalFlowBackward= []


while(c<= Imagescount):
    print('working.......')
    z=c
    img1Forward_gray = cv2.cvtColor(Images[z-1], cv2.COLOR_BGR2GRAY)
    img2Forward_gray = cv2.cvtColor(Images[c], cv2.COLOR_BGR2GRAY)
    p1, st, err = cv2.calcOpticalFlowPyrLK(img1Forward_gray, img2Forward_gray, FirstImage.astype(np.float32), None, **lk_params)
    OpticalFlowForward.append(p1)
    FirstImage = OpticalFlowForward[z-1]
    c=c+1

print(len(OpticalFlowForward))

#print (Imagescount)
while(Imagescount >=0):
    print('working.......')
    print(Imagescount)
    z=Imagescount
    img1Backward_gray = cv2.cvtColor(Images[Imagescount], cv2.COLOR_BGR2GRAY)
    img2Backward_gray = cv2.cvtColor(Images[z-1], cv2.COLOR_BGR2GRAY)
    p2, st, err = cv2.calcOpticalFlowPyrLK(img1Backward_gray, img2Backward_gray, OpticalFlowForward[z-1].astype(np.float32), None, **lk_params)
    OpticalFlowBackward.append(p2)
    Imagescount = Imagescount-1

#print(len(OpticalFlowBackward))

## Ignore  points


FinalForwardPoints =[]
a=0
b= len(OpticalFlowForward)-1
while(a <= len(OpticalFlowForward)-1):
    print('working......')
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

    FinalForwardPoints.append(points1final)

    a=a+1
    b=b-1
x=0
distx=[]
disty=[]
while(x < len(FinalForwardPoints)-1):
    point1 =[]
    point2 =[]
    y=x+1
    point1 = FinalForwardPoints[x]
    point2 = FinalForwardPoints[y]

    print('working')
   # print(y)
   # print(point2)
    sumx=0
    sumy=0
    s=0
    p1 = len(point1)-1
    p2 = len(point2)-1
    fp=0
    if(p1< p2):
        fp =p1
    else:
        fp =p2
    while(s <= fp):
        print('length of p1' , len(point1))
        print('length of p2' , len(point2))
        distancepoint1 = point1[s]
        distancepoint2 = point2[s]
        sumx = sumx + int(distancepoint2[0]- distancepoint1[0])
        sumy = sumy + int(distancepoint2[1]- distancepoint1[1])
        s=s+1



    #distancepoint1 = point1[0]
    #distancepoint2 = point2[0]

    #tempx1 = int(distancepoint2[0]- distancepoint1[0])
    #tempx2 = int(distancepoint2last[0]- distancepoint1last[0])
    #tempy1 = int(distancepoint2[1]- distancepoint1[1])
    #tempy2 = int(distancepoint2last[1]- distancepoint1last[1])
    sumx = sumx / (len(point2))
    sumy = sumy / (len(point2))
    distx.append(int(sumx))
    disty.append(int(sumy))
    #disty.append(int(distancepoint2[1]- distancepoint1[1]))
    x=x+1

#print(len(distx))

imagecounter = 1

dx1 =540
dx2 = 620
dy1 = 450
dy2 = 490
fdx1 = dx1+distx[0]
fdx2 = dx2+distx[0]
fdy1 = dy1+disty[0]
fdy2 = dy2+disty[0]

print(distx)
print(len(distx))
print(disty)
print(len(disty))
while(imagecounter < (len(Images)-2)):
    Box2=[]
    Box2.append((fdx1,fdy1))
    Box2.append((fdx2,fdy2))

    cv2.rectangle(Images[imagecounter],Box2[0],Box2[1],(0,255,22),3)
    #print (imagecounter)
    fdx1 = fdx1+distx[imagecounter]
    fdx2 = fdx2+distx[imagecounter]
    fdy1 = fdy1+disty[imagecounter]
    fdy2 = fdy2+disty[imagecounter]

    name = 'trackedc' + str(imagecounter) +'.png'
    cv2.imwrite(name,Images[imagecounter])
    imagecounter= imagecounter+1

cv2.waitKey()
