
import numpy as np
import itertools
from Tracker.Ransac.RansacCalculations import RansacCalculations
import cv2

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


        #orb = cv2.ORB_create()
        sift = cv2.xfeatures2d.SIFT_create()
        keypoints_1st_image = self.KeyPoints(vargood_points_1stimage)
        keypoints_k_image = self.KeyPoints(vargood_points_kimage)


        kp_1stimage,descriptor_1stimage = sift.compute(grayimages[0],keypoints_1st_image)
        kp_kimage,descriptor_kimage = sift.compute(grayimages[len(grayimages)-1],keypoints_k_image)










            ## Method 1 to match descriptors


        vargood_points_1stimage,vargood_points_kimage,descriptor_1stimage,descriptor_kimage = self.keypoints_to_points(kp_1stimage,kp_kimage,vargood_points_1stimage,vargood_points_kimage,descriptor_1stimage,descriptor_kimage)

        normalized_descriptor_1stimage = ransac_obj.normalized_to_1(descriptor_1stimage)
        normalized_descriptor_kimage = ransac_obj.normalized_to_1(descriptor_kimage)

        normalized_descriptor_1stimage = np.array(normalized_descriptor_1stimage)
        normalized_descriptor_kimage   = np.array(normalized_descriptor_kimage)



        # descriptor_1stimage = np.array(descriptor_1stimage)
        # descriptor_kimage = np.array(descriptor_kimage)
        #print(descriptor_1stimage)


        if(len(descriptor_1stimage)==0):
            final_goodpoints_1stimage = descriptor_1stimage
            print('No good point found after geo matric threshold')
            if(final_goodpoints_1stimage.size==0):
                return final_goodpoints_1stimage,status

        else:
            #for i in range(len(normalized_descriptor_1stimage)):

            ssd_distance = np.linalg.norm(normalized_descriptor_1stimage - normalized_descriptor_kimage,2,1) #np.sum( np.abs(descriptor_kimage - descriptor_1stimage) > 160, axis=1)/32
                #ssd_distance.append(ssd_distance_value)
            #ssd_distance = np.array(ssd_distance)
            temp_ssd_distace= ssd_distance < descriptor_threshold


            final_goodpoints_1stimage = vargood_points_1stimage[temp_ssd_distace]


        ## Method 2 by using toturail on this link: https://jayrambhia.wordpress.com/2013/01/18/sift-keypoint-matching-using-python-opencv/




        # flann_params = dict(algorithm=1, trees=4)
        # flann = flann_Index(descriptor_1stimage, flann_params)
        # idx, dist = flann.knnSearch(descriptor_kimage, 1, params={})
        # del flann
        # dist = dist[:,0]/2500.0
        # dist = dist.reshape(-1,).tolist()
        # idx = idx.reshape(-1).tolist()
        # indices = range(len(dist))
        # indices.sort(key=lambda i: dist[i])
        # dist = [dist[i] for i in indices]
        # idx = [idx[i] for i in indices]
        # skp_final = []
        # for i, dis in itertools.izip(idx, dist):
        #     if dis < descriptor_threshold:
        #         skp_final.append(vargood_points_1stimage[i])
        #     else:
        #         break


            if(final_goodpoints_1stimage.size==0):
                return final_goodpoints_1stimage,vargood_points_1stimage,status
            else:
                status =1
                return final_goodpoints_1stimage,vargood_points_1stimage,status


    def apply_error_filter(self,points_1st_image,points_1st_image_backward,points_k_image,grayimages,geomatric_threshold,descriptor_threshold):
        ransac_obj = RansacCalculations()
        status =-1

        ## apply geomatric distance filter

        geomatric_distance = np.linalg.norm(points_1st_image[:,0:2] - points_1st_image_backward[:,0:2],2,1)

        #print('geomatric_distance',geomatric_distance)

        temp_geomatric_distance = geomatric_distance < geomatric_threshold


        vargood_points_1stimage = points_1st_image[temp_geomatric_distance]
        vargood_points_kimage = points_k_image[temp_geomatric_distance]



        if(vargood_points_1stimage.size==0):
            return vargood_points_1stimage
        else:
            status =1
            return vargood_points_1stimage






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

