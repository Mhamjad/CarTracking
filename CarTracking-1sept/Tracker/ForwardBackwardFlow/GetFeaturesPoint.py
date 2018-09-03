import cv2
import numpy as np

from Tracker.Ransac.RansacCalculations import RansacCalculations


class GetFeaturesPoint:

    def apply_filters(self,points_1st_image,points_1st_image_backward,points_k_image,grayimages,geomatric_threshold,descriptor_threshold):
        ransac_obj = RansacCalculations()
        status =-1

        ## apply geomatric distance filter

        geomatric_distance = np.linalg.norm(points_1st_image[:,0:2] - points_1st_image_backward[:,0:2],2,1)

        #print('geomatric_distance',geomatric_distance)

        temp_geomatric_distance = geomatric_distance < geomatric_threshold


        vargood_points_1stimage = points_1st_image[temp_geomatric_distance]
        vargood_points_kimage = points_k_image[temp_geomatric_distance]


        orb = cv2.ORB_create()
        keypoints_1st_image = self.KeyPoints(vargood_points_1stimage)
        keypoints_k_image = self.KeyPoints(vargood_points_kimage)


        kp_1stimage,descriptor_1stimage = orb.compute(grayimages[0],keypoints_1st_image)
        kp_kimage,descriptor_kimage = orb.compute(grayimages[len(grayimages)-1],keypoints_k_image)









        #
        # vargood_points_1stimage,vargood_points_kimage,descriptor_1stimage,descriptor_kimage = self.keypoints_to_points(kp_1stimage,kp_kimage,vargood_points_1stimage,vargood_points_kimage,descriptor_1stimage,descriptor_kimage)
        #
        # normalized_descriptor_1stimage = ransac_obj.normalized_to_1(descriptor_1stimage)
        # normalized_descriptor_kimage = ransac_obj.normalized_to_1(descriptor_kimage)
        #
        # normalized_descriptor_1stimage = np.array(normalized_descriptor_1stimage)
        # normalized_descriptor_kimage   = np.array(normalized_descriptor_kimage)



        descriptor_1stimage = np.array(descriptor_1stimage)
        descriptor_kimage = np.array(descriptor_kimage)
        #print(descriptor_1stimage)


        if(len(descriptor_1stimage)==0):
            final_goodpoints_1stimage = descriptor_1stimage
            print('No good point found after geo matric threshold')
            if(final_goodpoints_1stimage.size==0):
                return final_goodpoints_1stimage,status

        else:

            ssd_distance = np.sum( np.abs(descriptor_kimage - descriptor_1stimage) > 130, axis=1)/32 #np.linalg.norm(normalized_descriptor_1stimage - normalized_descriptor_kimage,2,1) #np.sum( np.abs(descriptor_kimage - descriptor_1stimage) > 160, axis=1)/32
            temp_ssd_distace= ssd_distance < descriptor_threshold


            final_goodpoints_1stimage = vargood_points_1stimage[temp_ssd_distace]

            if(final_goodpoints_1stimage.size==0):
                return final_goodpoints_1stimage,vargood_points_1stimage,status
            else:
                status =1
                return final_goodpoints_1stimage,vargood_points_1stimage,status









    def KeyPoints(self,Points):
        keypoints=[]
        p=0
        while(p<len(Points)):

            keypt = cv2.KeyPoint(Points[p][0], Points[p][1],1)
            keypoints.append(keypt)
            p=p+1



        return keypoints

    def keypoints_to_points(self,firstkp,lastkp,firstpoints,lastpoints ,des_first,des_last):
        if(len(firstkp)>=len(lastkp)):
            ln = lastkp
        else:
            ln= firstkp

        counter =0
        points_counter=0
        ffirstpoints =[]
        flastpoints =[]
        firstdes=[]
        lastdes=[]
        while(counter<=len(ln)-1):
            pt = ln[counter].pt
            if((firstpoints[points_counter,0:2]==pt).all() or (lastpoints[points_counter,0:2]==pt).all()):
                ffirstpoints.append(firstpoints[points_counter])
                flastpoints.append(lastpoints[points_counter])
                firstdes.append(des_first[counter])
                lastdes.append(des_last[counter])
                points_counter=points_counter+1
            else:
                points_counter=points_counter+1

            counter=counter+1

        ffirstpoints=np.array(ffirstpoints)
        flastpoints=np.array(flastpoints)
        firstdes=np.array(firstdes)
        lastdes= np.array(lastdes)
        return ffirstpoints,flastpoints,firstdes,lastdes

