import cv2
import os
import glob
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

    def GetFrames(self,ListOfImages,FirstImage,NextStep):
        Images = []

        a= FirstImage+NextStep
        while(FirstImage <=a-1):
            if(FirstImage >len(ListOfImages)-1):
                break
            else:
                img = cv2.imread(ListOfImages[FirstImage])

                if img is not None:
                    #Images.append((img,ListOfImages[FirstImage]))
                    Images.append(img)

                else:
                    print('You are out of Images')
                    break
            FirstImage=FirstImage+1
        return Images

    def GetPoints(self,BoundingBox):

        ImagePoints =[]
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
        name = '/home/isam/HazenWork-Hasnain/Experiment2-nomedian/' + str(number) +'.png'
        cv2.imwrite(name,Images)


    def DrawCircles(self,Points,Image,number):
        Images = cv2.imread(Image,1)
        a =0;
        while(a<len(Points)-1):
            Images = cv2.circle(Images,(Points[a][0],Points[a][1]), 1, (0,0,255), 1)
            a=a+1

        name = '/home/isam/HazenWork-Hasnain/GoodPoints/' + str(number) +'circle.png'
        cv2.imwrite(name,Images)






