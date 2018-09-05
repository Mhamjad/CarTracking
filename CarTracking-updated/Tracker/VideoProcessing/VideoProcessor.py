
import os
import glob
import numpy as np
from Detector.undistort import undistort_2560
import cv2
class VideoProcessor:
    def getimages_path(self, path):
            paths = []
            status =-1
            if(os.path.exists(path)):
                for filename in os.listdir(path):
                    name, ext = filename.split('.')
                    name = int(name)
                    name = chr(name)
                    number = ord(name)
                    paths.append(number)
            else:
                print('Given path not found')
                return paths,status


            i=0
            paths = sorted(paths)
            if(len(paths) ==0):
                print('No Image found of given path')
                return paths,status
            else:
                status =1
                while(i<=len(paths)-1):
                    paths[i] = path + str(paths[i]) + '.jpg'
                    i=i+1
                return paths , status

    def getframes(self,imagespath,firstimage_index,intervalsize,images,grayimages):


        status =-1
        if(firstimage_index<0 or intervalsize <0):
            print('Please enter positive index of firstimage and interval size')
            return images,grayimages,status

        a= firstimage_index + intervalsize
        if(a>=len(imagespath)-1):
            a= len(imagespath)-1


        if(images.empty()):
            while(firstimage_index <a):

                img = cv2.imread(imagespath[firstimage_index])
                img = undistort_2560(img)
                if img is not None:
                    status = 1
                    grayimg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

                    images.put(img)
                    grayimages.put(grayimg)
                else:
                    print('Out of images on given path list')
                    return images,grayimages,status
                firstimage_index=firstimage_index+1
        else:
            a= images.qsize()-1 + firstimage_index
            if(a>=len(imagespath)):
                status = 1
                images.get()
                grayimages.get()

            else:
                images.get()
                grayimages.get()

                img = cv2.imread(imagespath[a])
                img = undistort_2560(img)
                if img is not None:
                    status = 1
                    grayimg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
                    images.put(img)
                    grayimages.put(grayimg)

                else:
                    print('Out of images on given path list')
                    return images,grayimages,status

        return images,grayimages,status

    def inilization(self,boundingbox,grayimage,maxcorners,qualitylevel,mindistance,useharrisdetectorboolean,k,method='gCorners'):
        initialpoints =[]
        status =-1

        if method=='grid':
            box_counter = 0
            while(box_counter<=len(boundingbox)-1):
                x_Start = boundingbox[box_counter].x1
                x_End = boundingbox[box_counter].x2

                y_Start = boundingbox[box_counter].y1
                y_End = boundingbox[box_counter].y2

                j = x_Start
                while(j <= x_End):
                    i = y_Start
                    while(i <= y_End):
                        temp =[]
                        temp.append(j)
                        temp.append(i)
                        temp.append(box_counter)
                        initialpoints.append(temp)


                        i= i+1

                    j =j+1
                box_counter=box_counter+1
            if(len(initialpoints)>0):
                status =1
                initialpoints = np.array(initialpoints)
                return initialpoints, status
            else:
                initialpoints = np.array(initialpoints)
                return initialpoints, status

        if method=='gCorners':


            box_counter=0
            box=0
            finitialpoints=[]
            while(box_counter<=len(boundingbox)-1):

                if(boundingbox[box_counter].x1==None):


                    pass
                else:
                    xmin= int(boundingbox[box_counter].x1)
                    xmax = int(boundingbox[box_counter].x2)

                    ymin = int(boundingbox[box_counter].y1)
                    ymax = int(boundingbox[box_counter].y2)

                    if(xmin <0):
                        xmin =0

                    if(xmax <0):
                        xmax =0
                    if(ymin <0):
                        ymin =0
                    if(ymax <0):
                        ymax =0

                    patch = grayimage[ymin:ymax, xmin:xmax]
                    #print('patch',box_counter)
                    #self.DrawCirclesofPts(patch,grayimage,00000)
                    corners = cv2.goodFeaturesToTrack(patch, maxcorners, qualitylevel , mindistance, useHarrisDetector=useharrisdetectorboolean, k=k)
                    corners = corners.reshape( (-1,2) )
                    corners[:,0] = corners[:,0]+xmin
                    corners[:,1] = corners[:,1]+ymin
                    box_number = np.zeros(len(corners))
                    box_number[:] =  box
                    #box_number = box_number.reshape((-1,1))


                    initialpoints = np.column_stack((corners, box_number))

                    if(box==0):
                        finitialpoints = initialpoints
                        #print('final points',finitialpoints)
                    else:
                        #print('initial points',initialpoints)

                        finitialpoints = np.concatenate((finitialpoints,initialpoints), axis=0)
                    box=box+1
                box_counter=box_counter+1

            if(len(finitialpoints)>0):
                status =1


            return finitialpoints,status

    def update_box(self,boundingbox,flow):
        counter=0
        updatedBoxes=[]

        while(counter<=len(flow)-1):
            print('counter ',counter,boundingbox[counter] , flow[counter])
            if(flow[counter]==None):
                boundingbox[counter].x1 = None
                boundingbox[counter].y1 = None
                boundingbox[counter].x2 = None
                boundingbox[counter].y2 = None
            else:
                if(boundingbox[counter].x1==None):
                    boundingbox[counter].x1 = int(flow[counter][0])
                    boundingbox[counter].y1 = int(flow[counter][1])
                    boundingbox[counter].x2 = int(flow[counter][0])
                    boundingbox[counter].y2 = int(flow[counter][1])
                else:
                    boundingbox[counter].x1 = int(boundingbox[counter].x1 +flow[counter][0])
                    boundingbox[counter].y1 = int(boundingbox[counter].y1 +flow[counter][1])
                    boundingbox[counter].x2 = int(boundingbox[counter].x2 +flow[counter][0])
                    boundingbox[counter].y2 = int(boundingbox[counter].y2 +flow[counter][1])
            counter=counter+1
        return boundingbox

    def update_box2(self,boundingbox,flow):
        counter=0
        updatedBoxes=[]
        tempboundingbox =[]
        while(counter<=len(flow)-1):
            #print('counter ',counter,boundingbox[counter] , flow[counter])

            boundingbox[counter].x1 = int(boundingbox[counter].x1 +flow[counter][0])
            boundingbox[counter].y1 = int(boundingbox[counter].y1 +flow[counter][1])
            boundingbox[counter].x2 = int(boundingbox[counter].x2 +flow[counter][0])
            boundingbox[counter].y2 = int(boundingbox[counter].y2 +flow[counter][1])
            counter=counter+1
        tempboundingbox = boundingbox
        return tempboundingbox

    def update_box_list(self,boundingbox,flow):
        counter=0
        updatedBoxes=[]
        tempboundingbox =[]
        while(counter<=len(flow)-1):
            #print('counter ',counter,boundingbox[counter] , flow[counter])

            boundingbox[counter][0] = int(boundingbox[counter][0] +flow[counter][0])
            boundingbox[counter][1] = int(boundingbox[counter][1] +flow[counter][1])
            boundingbox[counter][2] = int(boundingbox[counter][2] +flow[counter][0])
            boundingbox[counter][3] = int(boundingbox[counter][3] +flow[counter][1])
            counter=counter+1

        return boundingbox
    def draw_rectangles(self,box,image,number):

        image=cv2.rectangle(image,(box[0],box[1]),(box[2],box[3]),(0,255,22),3)
        name = '/home/isam/HazenWork-Hasnain/Results/' + str(number) +'.png'
        cv2.imwrite(name,image)

    def draw_rectangles_detector(self,box,image,number,color='red'):

        if(color=='blue'):
            color_value =(255,44,66)
        if(color=='green'):
            color_value = (0,255,0)
        if(color=='red'):
            color_value = (0,0,255)


        if(len(box)==1 and box[0]== None):
            pass
        else:

            counter =0
            while(counter<=len(box)-1):
                #print('boxes',box[counter])
                if(box[counter].x1==None or box[counter]==None):
                    pass
                else:
                    image=cv2.rectangle(image,(box[counter].x1,box[counter].y1),(box[counter].x2,box[counter].y2),color_value,3)
                counter=counter+1
            #name = '/home/isam/HazenWork-Hasnain/Detected/' + str(number) +'.png'
            name = '/home/isam/HazenWork-Hasnain/Detected/' + str(number) +'.png'
            #name = '/media/isam/CE0A03DD0A03C187/Hasnain-Hazenwork/samples/Result2/' + str(number) +'.png'
            #name = '/media/isam/CE0A03DD0A03C187/Hasnain-Hazenwork/Results/DistanceThresholdPts/' + str(number) +'.png'

            cv2.imwrite(name,image)


    def DrawCirclesofPts(self,Points,image,number):

        a =0;
        while(a<len(Points)):
            image = cv2.circle(image,(int(Points[a][0]),int(Points[a][1])), 3, (0,0,255), -1)
            a=a+1

        name = '/home/isam/HazenWork-Hasnain/Detected/' + str(number) +'circle.png'
        cv2.imwrite(name,image)

    def DrawCirclesofPtstags(self,Points,image,number,tag):

        img = image
        a =0;
        while(a<len(Points)):
            img = cv2.circle(img,(int(Points[a][0]),int(Points[a][1])), 2, (0,255,0), -1)
            a=a+1

        #name = '/home/isam/HazenWork-Hasnain/Detected/' + str(number) + tag +'.png'
        name = '/home/isam/HazenWork-Hasnain/Detected/' + str(number) +'.png'
        #name = '/media/isam/CE0A03DD0A03C187/Hasnain-Hazenwork/samples/Result2/' + str(number) + tag +'.png'
        #name = '/media/isam/CE0A03DD0A03C187/Hasnain-Hazenwork/Results/DistanceThresholdPts/' + str(number) +'.png'

        cv2.imwrite(name,img)


    def DrawCirclesofPtstags2(self,Points,image,number,tag):

        img = image
        a =0;
        while(a<len(Points)):
            img = cv2.circle(img,(int(Points[0]),int(Points[1])), 2, (0,255,0), -1)
            a=a+1

        #name = '/home/isam/HazenWork-Hasnain/Detected/' + str(number) + tag +'.png'
        name = '/media/isam/CE0A03DD0A03C187/Hasnain-Hazenwork/Results/DescriptorMatcherPts/' + str(number) +'.png'
        #name = '/media/isam/CE0A03DD0A03C187/Hasnain-Hazenwork/Results/DistanceThresholdPts/' + str(number) +'.png'

        cv2.imwrite(name,img)
    def draw_rectangles_list(self,box,image,number,color='red'):

        if(color=='blue'):
            color_value =(255,44,66)
        if(color=='green'):
            color_value = (0,255,0)
        if(color=='red'):
            color_value = (0,0,255)


        if(len(box)==1 and box[0]== None):
            pass
        else:

            counter =0
            while(counter<=len(box)-1):
                #print('boxes',box[counter])
                if(box[counter]==None or box[counter]==None):
                    pass
                else:
                    image=cv2.rectangle(image,(box[counter][0],box[counter][1]),(box[counter][2],box[counter][3]),color_value,3)
                counter=counter+1
            #name = '/home/isam/HazenWork-Hasnain/Detected/' + str(number) +'.png'
            name = '/media/isam/CE0A03DD0A03C187/Hasnain-Hazenwork/Results/DescriptorMatcherPts/' + str(number) +'.png'
            #name = '/media/isam/CE0A03DD0A03C187/Hasnain-Hazenwork/Results/DistanceThresholdPts/' + str(number) +'.png'

            cv2.imwrite(name,image)
