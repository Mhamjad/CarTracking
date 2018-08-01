import numpy as np




class Ransac:

    def Calculate_Ransac(self,FlowPoints):


        Inliars =[]
        counter=0
        for i in FlowPoints:

            Hypothesis = i
            Hypothesis_Magnitude = np.linalg.norm(Hypothesis,ord=1)
            Hypothesis = np.divide(Hypothesis,Hypothesis_Magnitude)
            for j in FlowPoints:
                TempInliars =[]
                Temp = j
                Temp_Magnitude = np.linalg.norm(Temp)
                Temp = np.divide(Temp,Temp_Magnitude)

                #print('temp',Temp)
                print('Dot Product',round(np.dot(Hypothesis,Temp),3))
                if(round(np.dot(Hypothesis,Temp),3) < 0.9):
                    TempInliars.append(j)
                    print('In temp loop',j)
                else:
                    print(counter)
                    print('If not satisfied')

            if(len(TempInliars)>len(Inliars)):
                print('Inlairs of ',FlowPoints[counter], ' is equal to ' , len(TempInliars))
                Inliars = TempInliars
                GoodPoint=[]
                GoodPoint.append(i)
                GoodPoint.append(counter)
            counter =counter+1

        return GoodPoint


    def Calculate_Ransac2(self, FlowPoints):
        Inliars =[]
        counter=0
        NormalizedFlowPoints=[]
        n=0
        while(n < len(FlowPoints)-1):
            n_magnitude = np.linalg.norm(FlowPoints[n])
            NormalizedFlowPoints.append(np.divide(FlowPoints[n],n_magnitude))
            n=n+1


        #print('Lenght of Points', len(FlowPoints))
        for i in NormalizedFlowPoints:

            tempInlairs =[]
            tempOutlairs =[]
            temp=[]
            NormalizedFlowPoints = np.array(NormalizedFlowPoints)
            i=np.array(i)
            product = np.dot(NormalizedFlowPoints,i)
            #print('One Normalzied Points',i)

            #
            c=0
            for j in product:
               # print('Dot Product ',j)
                if(j>0.9999):

                    tempInlairs.append(i)
                    temp.append(FlowPoints[c])
                else:
                    tempOutlairs.append(i)
                c=c+1

            #print('Inlairs of ',counter , ' value is  ' , len(tempInlairs))
            #print('OutLairs of ',counter , ' value is  ' , len(tempOutlairs))

            if(len(tempInlairs)>len(Inliars)):
                Inliars = tempInlairs
                Flow=NormalizedFlowPoints[counter]
                Magnitude = self.TakeMagnitude(temp)

                #print('Magnitude',Magnitude)
                Median = np.median(Magnitude)
                Flow = np.multiply(Median,Flow)
                TFlow = FlowPoints[counter]

            counter =counter+1


        return TFlow


    def TakeMagnitude(self,Points):
        Magnitude=[]
        a=0
        while(a<=len(Points)-1):
            Magnitude.append(np.linalg.norm(Points[a]))
            a=a+1
        return Magnitude





