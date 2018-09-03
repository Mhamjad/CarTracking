import numpy as np
import random
class RansacCalculations:

    def get_ransac(self,flowpoints,threshold,method,samplesize = 20):
        normalized_points = self.normalized_to_1(flowpoints)


        if(samplesize > len(flowpoints)-1):
            samplesize = len(flowpoints)-1

        #print('sample size',samplesize)
        #print('len of flow points',len(flowpoints))
        random_sample = random.sample(range(0,len(flowpoints)-1),samplesize)
        #print('random sample',random_sample)
        inlairs =[]

        Status =-1
        flowpoint = np.array(flowpoints[0])
        for r in random_sample:
            #print('in method')
            hypothesis = normalized_points[r]
            dot_product = np.dot(normalized_points,hypothesis)
            temp_dot_product = dot_product > threshold
            temp_inlairs = flowpoints[temp_dot_product]
            #print('temp inlairs',len(temp_inlairs))
            if(len(temp_inlairs)>=len(inlairs)):

                inlairs= temp_inlairs
                if(method=='median'):
                    norm_inlairs = np.linalg.norm(inlairs,2,1)
                    norm_median = np.median(norm_inlairs)
                    flowpoint = norm_median * hypothesis
                if(method == 'besthypothesis'):
                    flowpoint = flowpoints[r]



        return flowpoint



    def normalized_to_1(self,points):
        n=0
        normalized_points=[]
        while(n < len(points)):
            n_magnitude = np.linalg.norm(points[n])
            if(n_magnitude==0):
                n_magnitude = 0.001
            normalized_points.append(np.divide(points[n],n_magnitude))
            n=n+1


        #normalized_points = np.divide(points,(np.linalg.norm(points)))
        return normalized_points


