import numpy as np
from MyRobotMath import SE3

se3 = SE3()

def select_init(desired, preset): # 목표 좌표에 해당하는 사분면 초기 상태 호출
    x, y = desired[0], desired[1]
    key = (x > 0, y > 0)
    mapping = {
        (True,   True): 0,
        (False,  True): 1,
        (False, False): 2,
        (True,  False): 3,
    }
    return preset[mapping[key]]

def deg2rad(value,joint): 
    """
    If revolute joint, convert degree to radian. 
    :param value: List of joint angle (degree)
    :param joint: List of joint
    :return: Numpy.array of joint angle (radian)
    """
    theta = np.array(value)
    for i in range(len(joint)):
        if joint[i] == 'R':
            theta[i] = np.deg2rad(theta[i])
    
    return theta

def rad2deg(value,joint):
    """
    If revolute joint, convert redian to degree. 
    :param value: List of joint angle (radian)
    :param joint: List of joint
    :return: Numpy.array of joint angle (degree)
    """
    theta = np.array(value)
    for i in range(len(joint)):
        if joint[i] == 'R':
           theta[i] = np.rad2deg(value[i])

    return theta

def theta_normalize(value,joint):
    """
    If revolute joint, normalize degree -180 to 180. 
    :param value: Numpy.Array of joint angle (degree)
    :param joint: List of joint
    :return: List of normalized joint angle 
    """
    theta = value.flatten().tolist()
    for i in range(len(value)):
        if joint[i] == 'R':
            if value[i] % 360 > 180:
                theta[i] = theta[i] % 360 - 360
            else:
                theta[i] = theta[i] % 360
    
    return theta

class SCARA():

    # SCARA 로봇의 Zero position, Body Twist, Space Twist, 초기값 추정 set
    def __init__(self, L1, L2, L3):

        self.preset = [[37, 0, 0, 0.8],  # 1사분면
                       [135, 0, 0, 1.0],  # 2사분면
                       [-135, 0, 0, 0.9], # 3사분면
                       [-65, 0, 0, 0.7] # 4사분면
                                            ]
        
        self.joint = ['R','R','R','P']

        self.zero = np.array([[1, 0, 0, L2+L3],
                              [0, 1, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])

        self.B_tw = [[0, 0, 1 ,0, L2+L3 ,0],
                     [0, 0, 1, 0, L2, 0],
                     [0, 0, 1, 0, 0, 0],
                     [0, 0, 0, 0, 0, 1]]
        
        self.S_tw = (se3.adjoint(self.zero) @ np.array(self.B_tw).T).T.tolist()

class BarretWAM():

    def __init__(self,L1,L2,L3,W):

        self.joint = ['R','R','R','R','R','R','R']

        self.preset = [[37, 45, 0, 3,0,0,0],  # 1사분면
                       [135, 1, 0, 3,0,0,0],  # 2사분면
                       [-135,1, 0, 3,0,0,0], # 3사분면
                       [-65, 1, 0, 3,0,0,0] # 4사분면
                                            ]

        self.zero = np.array([[1, 0, 0, 0],
                              [0, 1, 0, 0],
                              [0, 0, 1, L1+L2+L3],
                              [0, 0, 0, 1]])
        
        self.B_tw = [[0, 0, 1 ,0, 0 ,0],
                     [0, 1, 0, L1+L2+L3, 0, 0],
                     [0, 0, 1, 0, 0, 0],
                     [0, 1, 0, L2+L3, 0, W],
                     [0, 0, 1, 0, 0, 0],
                     [0, 1, 0, L3, 0, 0],
                     [0, 0, 1, 0, 0, 0]]

        self.S_tw = (se3.adjoint(self.zero) @ np.array(self.B_tw).T).T.tolist()