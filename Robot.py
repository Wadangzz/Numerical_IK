import numpy as np
from SE3lib import SE3

class SCARA():

    def __init__(self, L1, L2, L3):

        se3 = SE3()
        self.zero = np.array([[1, 0, 0, L2+L3],
                              [0, 1, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])

        self.B_tw = [[0, 0, 1 ,0, L2+L3 ,0],
                     [0, 0, 1, 0, L2, 0],
                     [0, 0, 1, 0, 0, 0],
                     [0, 0, 0, 0, 0, 1]]
        
        self.S_tw = (se3.adjoint(self.zero) @ np.array(self.B_tw).T).T.tolist()

        