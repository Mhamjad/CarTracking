import numpy as np

class Displacement:

    def FindDisplacement(self,FirstFrame):
        a=0
        FinalDisplacement=[]

        FinalDisplacement.append(np.median(FirstFrame[0]))
        FinalDisplacement.append(np.median(FirstFrame[1]))


        return FinalDisplacement
