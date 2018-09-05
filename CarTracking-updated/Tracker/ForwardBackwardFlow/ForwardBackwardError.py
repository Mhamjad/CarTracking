
import numpy as np
from Tracker.VideoProcessing.VideoProcessor import VideoProcessor
from Tracker.Ransac.RansacCalculations import RansacCalculations
import cv2


class ForwardBackwardError:

    def getforward_backwardflow(self,list_grayimages,points_1stimage,thresholdzeroflow,filtered_points_1stimage=None,direction='forward'):

        lk_params = dict( winSize  = (8,8),
                          maxLevel = 4,
                          criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))



        #featurepoints_1stimage = np.array(featurepoints_1stimage)
        status=0
        if(direction =='forward'):
            a=0
            tflow = points_1stimage[:,0:2]

            while(a<len(list_grayimages)-1):
                next = a+1
                current_img = list_grayimages[a]
                next_img = list_grayimages[next]
                flowpoints, st, err = cv2.calcOpticalFlowPyrLK(current_img,next_img, tflow.astype(np.float32), None, **lk_params)

                if(next==len(list_grayimages)-1):
                    flowpoints = np.column_stack((flowpoints, points_1stimage[:,2]))


                    filtered_points_kimage,filtered_points_1stimage,status = self.calculate_zeroflow(flowpoints,points_1stimage,thresholdzeroflow)

                if np.all(st==0):
                    tflow = None
                else:

                    tflow =flowpoints

                a=a+1
            if(status ==0):
                status = 1
                filtered_points_kimage  = points_1stimage
                filtered_points_1stimage = filtered_points_1stimage
                return filtered_points_kimage,filtered_points_1stimage,status
            if(status==-1):
                print('All points filtered out in zero flow threshold')
                return filtered_points_kimage,filtered_points_1stimage,status
            if(status== 1):
                return filtered_points_kimage,filtered_points_1stimage,status

        if(direction =='backward'):
            filtered_flow_points=[]
            filtered_points_at_given_image=[]
            filteredpoints_at_1st_image=[]
            temp_points_1stimage = points_1stimage
            tflow = points_1stimage[:,0:2]
            a=len(list_grayimages)-1
            status =0
            while(a>0):
                next = a-1
                current_img = list_grayimages[a]
                next_img = list_grayimages[next]

                flowpoints, st, err = cv2.calcOpticalFlowPyrLK(current_img,next_img, tflow.astype(np.float32), None, **lk_params)

                if(a==1):
                    flowpoints = np.column_stack((flowpoints, points_1stimage[:,2]))
                    filtered_flow_points,filtered_points_at_given_image,filteredpoints_at_1st_image,status = self.calculate_zeroflow(flowpoints,temp_points_1stimage,thresholdzeroflow,filtered_points_1stimage,direction)

                if np.all(st==0):

                    status =-1
                    return filtered_flow_points,filtered_points_at_given_image,filteredpoints_at_1st_image,status
                else:

                    tflow =flowpoints

                a=a-1

            if(status ==0):
                status = 1
                filtered_points_kimage  = points_1stimage
                filtered_points_1stimage = points_1stimage
                return filtered_points_kimage,filtered_points_1stimage,status

            if(status==-1):
                print('All points filtered out in zero flow threshold')
                return filtered_flow_points,filtered_points_at_given_image,filteredpoints_at_1st_image,status
            if(status== 1):
                return filtered_flow_points,filtered_points_at_given_image,filteredpoints_at_1st_image,status


    def calculate_zeroflow(self,points_at_kimage,points_at_1stimage,threshold,points_at_kimage_forward=None, method= 'forward'):



        difference = np.linalg.norm(points_at_kimage-points_at_1stimage,2,1)
        temp_difference = difference > threshold
        filtered_flow_points = points_at_kimage[temp_difference]
        filtered_points_at_given_image = points_at_1stimage[temp_difference]


        if(method=='backward'):
            filteredpoints_at_1st_image = points_at_kimage_forward[temp_difference]


        if((filtered_flow_points.size)==0 or (filtered_points_at_given_image.size)==0 ):
            status =-1

        else:
            status =1

        if(method =='forward'):
            return filtered_flow_points,filtered_points_at_given_image,status
        if(method =='backward'):
            return filtered_flow_points,filtered_points_at_given_image,filteredpoints_at_1st_image,status


    def calculate_boxflow(self,currentimage_gray,nextimage_gray,boxes,goodpoints,method,ransac_threshold,ransacmethod):
        lk_params = dict( winSize  = (10,10),
                          maxLevel = 4,
                          criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))


        finalflow=[]
        b=0
        while(b<=len(boxes)-1):
            temp_flow = goodpoints[:,2]==b
            temp_flowpoints = goodpoints[temp_flow]
            if(len(temp_flowpoints)==0):
                finalflow.append([None])
            else:
                ransac_obj = RansacCalculations()
                flowpoints, st, err = cv2.calcOpticalFlowPyrLK(currentimage_gray,nextimage_gray, temp_flowpoints[:,0:2].astype(np.float32), None, **lk_params)
                flow = flowpoints - temp_flowpoints[:,0:2]
                status =1
                if(method == 'median'):
                    flow_x=np.sort(flow[:,0])
                    flow_y=np.sort(flow[:,1])
                    flow_in_box = (np.median(flow_x),np.median(flow_y))
                    print('flow',flow_in_box)
                    #if(flow_in_box.size==0):
                    #    status = -1
                    finalflow.append(flow_in_box)
                if(method =='ransac'):
                    flow_in_box = ransac_obj.get_ransac(flow,ransac_threshold,ransacmethod)
                    if(flow_in_box.size==0):
                        status = -1
                    finalflow.append(flow_in_box)
            b=b+1
        return finalflow



    def getforward(self,list_grayimages,points_1stimage):

        lk_params = dict( winSize  = (8,8),
                          maxLevel = 4,
                          criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))



        #featurepoints_1stimage = np.array(featurepoints_1stimage)
        status=0

        a=0
        tflow = points_1stimage[:,0:2]

        while(a<len(list_grayimages)-1):
            next = a+1
            current_img = list_grayimages[a]
            next_img = list_grayimages[next]
            flowpoints, st, err = cv2.calcOpticalFlowPyrLK(current_img,next_img, tflow.astype(np.float32), None, **lk_params)

            if np.all(st==0):
                tflow = None
            else:

                tflow =flowpoints

            a=a+1
        return flowpoints





