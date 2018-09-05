import queue
import cv2
from Tracker.ForwardBackwardFlow.GetFeaturesPoint import GetFeaturesPoint

from Tracker.ForwardBackwardFlow.ForwardBackwardError import ForwardBackwardError
from Tracker.VideoProcessing.VideoProcessor import VideoProcessor
from Detector.DetectorSSD import Detector as Detector
from Detector.bbox import BBox as BBox
from Detector.undistort import undistort_2560




class Tracker:

    def Tracking(self,imagespath,intervalsize,geomatric_threshold =2.0,forward_zeroflow_threshold=5.0,backward_zeroflow_threshold=6.0,descriptor_threshold=0.3,ransac_threshold =0.8,boxflow_method='ransac',ransac_method='besthypothesis',max_corners=10000,qualitylevel=0.001,min_distance=1,use_harrisdetector_boolean=True,k=0.001):
        video_processor_obj = VideoProcessor()
        fb_flow_obj = ForwardBackwardError()
        filter_obj = GetFeaturesPoint()

        feature_point_everyframe =[]
        image_counter =0
        status =0
        images = queue.Queue()
        grayimages = queue.Queue()
        # exception= 'No point found'
        # ransac_exception = 'Flow Points not found'
        # exception_value = 'No points found'
        # ransac_exception_value = 'No flow points found'
        flow_in_box = []
        copy_images =[]
        copy_grayimages = []


        detector = Detector('/home/isam/HazenWork-Hasnain/PycharmProjects/CarTrackingV2/Detector/Gate_90.6.pth')
        c=0
        frame_checker =0
        while(image_counter < len(imagespath)-1):
            print('working on image number ',image_counter)

            ## get number of frames of given interval size
            if(frame_checker==0):
                images,grayimages,frames_status = video_processor_obj.getframes(imagespath,image_counter,intervalsize,images,grayimages)
                copy_images =  list(images.queue)
                copy_grayimages = list(grayimages.queue)

            if(c==0 or c==9):

                    got_boxes_flag, bbs = detector.detect(copy_images[0])

                    #video_processor_obj.draw_rectangles_detector(bbs,copy_images[0],image_counter, color='green')
                    c=0


            if(len(bbs)==1 and bbs[0]==None):
                pass
                # name = '/media/isam/CE0A03DD0A03C187/Hasnain-Hazenwork/Results4/' + str(image_counter) +'.png'
                #
                # cv2.imwrite(name,copy_images[0])
                # c=c+1
                # image_counter = image_counter+1
            else:



                #undistorted_image = undistort_2560(copy_images[0])
                #


                #bbs = bbs[0]
                #print('bbs',bbs.type)
                ## check frame status
                try:
                    # name = '/home/isam/HazenWork-Hasnain/Data/' + str(image_counter) +'.png'
                    #
                    # cv2.imwrite(name,copy_images[0])
                    if(frames_status ==1):

                        initial_points,intialpoints_status = video_processor_obj.inilization(bbs,copy_grayimages[0],max_corners,qualitylevel,min_distance,use_harrisdetector_boolean,k,method='gCorners')

                    else:
                        print('Exception called from frames_status ')
                        raise NoPointFound('No point found')




                    if(intialpoints_status == 1):

                        #video_processor_obj.DrawCirclesofPtstags(initial_points,copy_images[0],image_counter,'EdgeDetectorPoints')
                        points_at_kimage,filtered_points_at_1stimage,forward_status = fb_flow_obj.getforward_backwardflow(copy_grayimages,initial_points,forward_zeroflow_threshold,direction='forward')
                        #print('at k image forward', filtered_points_at_kimage.size)
                    else:
                        print('Exception called from intialpoints_status ')
                        video_processor_obj.draw_rectangles_detector(bbs,copy_images[0],image_counter, color='blue')
                        flow_in_box = [[None]] * len(bbs)
                        raise NoPointFound('No point found')







                    ## computer backward flow from points we got above.
                    if(forward_status == 1):

                        #video_processor_obj.DrawCirclesofPtstags(filtered_points_at_1stimage,copy_images[0],image_counter,'AfterForwardFilter')
                        filtered_points_at_counterimage_backward,filtered_points_at_kimage,filtered_points_at_counterimage,backward_status = fb_flow_obj.getforward_backwardflow(copy_grayimages,points_at_kimage,backward_zeroflow_threshold,filtered_points_at_1stimage,direction='backward')
                        #print('at k image backward', filtered_points_at_1stimage_backward.size)
                    else:
                        print('Exception called from forward_status ')
                        flow_in_box = [[None]] * len(bbs)
                        video_processor_obj.draw_rectangles_detector(bbs,copy_images[0],image_counter, color='blue')
                        raise NoPointFound('No point found')
                        #raise Exception(exception)










                    #video_processor_obj.DrawCirclesofPtstags(filtered_points_at_kimage,copy_images[0],image_counter,'Third')
                    if(backward_status == 1):

                        #video_processor_obj.DrawCirclesofPtstags(filtered_points_at_1stimage,copy_images[0],image_counter,'AfterBackwardFilter')

                        final_feature_points,threshold_goodpts,filter_status = filter_obj.apply_filters(filtered_points_at_counterimage,filtered_points_at_counterimage_backward,filtered_points_at_kimage,copy_grayimages,geomatric_threshold,descriptor_threshold)

                        #print('final_feature_points',final_feature_points)
                    else:
                        print('Exception called from backward_status ')
                        flow_in_box = [[None]] * len(bbs)
                        video_processor_obj.draw_rectangles_detector(bbs,copy_images[0],image_counter, color='blue')
                        raise NoPointFound('No point found')
                        #raise Exception(exception)


                    if(filter_status != 1):
                        print('Exception called from filter_status ')
                        flow_in_box = [[None]] * len(bbs)
                        video_processor_obj.draw_rectangles_detector(bbs,copy_images[0],image_counter, color='blue')
                        raise NoPointFound('No point found')
                        #raise Exception(exception)
                    # tracked_goodpoints =  fb_flow_obj.getforward(copy_grayimages,final_feature_points)
                    # c = len(copy_images)-1
                    # name = '/home/isam/HazenWork-Hasnain/Data/' + str(c) +'.png'

                    #cv2.imwrite(name,copy_images[len(copy_images)-1])


                    #video_processor_obj.DrawCirclesofPtstags(threshold_goodpts,copy_images[0],image_counter,'GoodPointsDistance')
                    #video_processor_obj.DrawCirclesofPtstags(final_feature_points,copy_images[0],image_counter,'gdpts160')

                    tracked_gdpts = fb_flow_obj.getforward(copy_grayimages,final_feature_points)
                    name = '/home/isam/HazenWork-Hasnain/Data/' + str(image_counter) +'.png'
                    #name = '/media/isam/CE0A03DD0A03C187/Hasnain-Hazenwork/Results4/' + str(number) +'.png'
                    cv2.imwrite(name,copy_images[0])

                    name = '/home/isam/HazenWork-Hasnain/Data/' + str(len(copy_images)) +'.png'
                    #name = '/media/isam/CE0A03DD0A03C187/Hasnain-Hazenwork/Results4/' + str(number) +'.png'
                    cv2.imwrite(name,copy_images[len(copy_images)-1])
                    #print('Good points',final_feature_points )
                    flow_in_box = fb_flow_obj.calculate_boxflow(copy_grayimages[0],copy_grayimages[1],bbs,final_feature_points,boxflow_method,ransac_threshold,ransac_method)



                    b=0
                    while(b<=len(flow_in_box)-1):
                        f =  flow_in_box[b]

                        if(f[0]==None):
                            print('Exception called from flow_in_box ')
                            raise NoPointFound('No point found')
                        else:
                            pass
                        b=b+1
                    #print('A',a)

                    c=c+1
                    bbs = video_processor_obj.update_box2(bbs,flow_in_box)
                    image_counter=image_counter+1
                    frame_checker =0
                    print('reached last step.')




                    ## get flow vector
                except NoPointFound:
                    print('Running Detector')
                    c=c+1
                    image_counter=image_counter+1

                    images,grayimages,frames_status = video_processor_obj.getframes(imagespath,image_counter,intervalsize,images,grayimages)
                    frame_checker=1
                    copy_images =  list(images.queue)
                    copy_grayimages = list(grayimages.queue)
                    got_boxes_flag, bbs_next = detector.detect(copy_images[0])


                    print('after flow',flow_in_box )
                    ## This part is to seprate out detected and not detected boxes.

                    b =0
                    temp_bbs =[]
                    detected_bbs=[]
                    #print(bbs,'bounding boxes')
                    while(b<=len(flow_in_box)-1):
                        f =  flow_in_box[b]

                        if(f[0]==None):
                            temp_bbs.append(bbs[b])
                        else:
                            detected_bbs.append(bbs[b])
                        b=b+1


                    ## This is to get boxes from next frames and find intersection of bxes
                    ftemp_bbs =[]
                    t_bbs=None

                    if(len(temp_bbs)>0):

                        ## this part is to get IOU ratio of boxes of current frames with next frame.
                        for i in range(len(temp_bbs)):
                            iou =-100000
                            for j in range(len(bbs_next)):
                                temp_iou =  BBox.get_iou(self,temp_bbs[i],bbs_next[j])
                                if(temp_iou >=iou):
                                    t_bbs=bbs_next[j]
                                    iou =  temp_iou

                            ftemp_bbs.append(t_bbs)

                        #video_processor_obj.draw_rectangles_detector(ftemp_bbs,copy_images[0],image_counter,color='red')
                        if(len(detected_bbs)>0):
                            tflow_in_box = list(filter([None].__ne__, flow_in_box))
                            #print(len(ftemp_bbs),'lenn')
                            #print(flow_in_box,'flow')
                            detected_bbs = video_processor_obj.update_box2(detected_bbs,tflow_in_box)
                            final_bbs=[]
                            a=0
                            ftemp_bbs_counter = 0
                            detected_bbs_counter=0
                            while(a<len(flow_in_box)):
                                f =  flow_in_box[a]

                                if(f[0]==None):
                                    final_bbs.append(ftemp_bbs[ftemp_bbs_counter])
                                    ftemp_bbs_counter=ftemp_bbs_counter+1
                                else:
                                    final_bbs.append(detected_bbs[detected_bbs_counter])
                                    detected_bbs_counter=detected_bbs_counter+1
                                a=a+1
                            bbs =final_bbs

                            #video_processor_obj.draw_rectangles_detector(ftemp_bbs,copy_images[0],image_counter,color='red')
                        else:
                            bbs = ftemp_bbs


                        #image_counter=image_counter+1











                #





            # flow_in_box = fb_flow_obj.calculate_boxflow(copy_grayimages[0],copy_grayimages[1],bbs,final_feature_points,boxflow_method,ransac_threshold,ransac_method)

            # b =0
            # temp_bbs =[]
            # detected_bbs=[]
            #     #print(bbs,'bounding boxes')
            # while(b<=len(flow_in_box)-1):
            #     f =  flow_in_box[b]
            #
            #     if(f[0]==None):
            #         temp_bbs.append(bbs[b])
            #     else:
            #         detected_bbs.append(bbs[b])
            #     b=b+1
            #
            # ftemp_bbs =[]
            # t_bbs=None
            # if(len(temp_bbs)>0):
            #     got_boxes_flag, boxes = detector.detect(copy_images[0])
            #     for i in range(len(temp_bbs)):
            #         iou =-1000
            #         for j in range(len(boxes)):
            #             temp_iou =  BBox.get_iou(self,temp_bbs[i],boxes[j])
            #             if(temp_iou >=iou):
            #                 t_bbs=boxes[j]
            #                 iou =  temp_iou
            #
            #         ftemp_bbs.append(t_bbs)
            #     tflow_in_box = list(filter([None].__ne__, flow_in_box))
            #     print(len(ftemp_bbs),'lenn')
            #     print(flow_in_box,'flow')
            #     detected_bbs = video_processor_obj.update_box2(detected_bbs,tflow_in_box)
            #     video_processor_obj.draw_rectangles_detector(ftemp_bbs,copy_images[0],image_counter,color='red')
            #     bbs = detected_bbs+ftemp_bbs
            #
            #     c=c+1
            #     image_counter=image_counter+1
            # else:
            #
            #     bbs = video_processor_obj.update_box2(bbs,flow_in_box)
            #
            #     print(bbs)
            #     #print('after flow',flow_in_box )
            #     c=c+1
            #     image_counter=image_counter+1


class NoPointFound(Exception):
    def __init__(self, message):

        # Call the base class constructor with the parameters it needs
        super().__init__(message)
