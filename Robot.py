import numpy as np
from MyRobotMath import SE3

class SCARA():

    # SCARA 로봇의 Zero position, Body Twist, Space Twist, 초기값 추정 set
    def __init__(self, L1, L2, L3):

        se3 = SE3()

        self.preset = [[37, 0, 0, 0.8],  # 1사분면
                       [135, 0, 0, 1.0],  # 2사분면
                       [-135, 0, 0, 0.9], # 3사분면
                       [-65, 0, 0, 0.7], # 4사분면
                                            ]

        self.zero = np.array([[1, 0, 0, L2+L3],
                              [0, 1, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])

        self.B_tw = [[0, 0, 1 ,0, L2+L3 ,0],
                     [0, 0, 1, 0, L2, 0],
                     [0, 0, 1, 0, 0, 0],
                     [0, 0, 0, 0, 0, 1]]
        
        self.S_tw = (se3.adjoint(self.zero) @ np.array(self.B_tw).T).T.tolist()

        