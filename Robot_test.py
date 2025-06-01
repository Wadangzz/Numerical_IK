import numpy as np
from dataclasses import dataclass
from MyRobotMath import SE3
from scipy.spatial.transform import Rotation as R


se3 = SE3()

@dataclass
class JointSpec:
    type: str  # 'R' or 'P'
    axis: str  # 'x', 'y', 'z'


class SCARA:
    # SCARA 로봇의 Zero position, Body Twist, Space Twist, 초기값 추정 set
    def __init__(self, L1, L2, L3, L4):
        
        self.joints = [
            JointSpec('R', 'z'),
            JointSpec('R', 'z'),
            JointSpec('P', 'z'),
            JointSpec('R', 'z'),
        ]

        # SCARA 로봇 링크 생성
        
        self.zero = np.array([[1, 0, 0, L2+L3],
                              [0, 1, 0, 0],
                              [0, 0, 1, 2],
                              [0, 0, 0, 1]])

        self.B_tw = [[0, 0, 1 ,0, L2+L3 ,0],
                     [0, 0, 1, 0, L2, 0],
                     [0, 0, 0, 0, 0, 1],
                     [0, 0, 1, 0, 0, 0]]
        
        self.S_tw = (se3.adjoint(self.zero) @ np.array(self.B_tw).T).T.tolist()

    def get_link_positions(self, matexps):
        
        positions = [T[:3, 3] for T in matexps]
        return positions


class BarretWAM:

    def __init__(self,L1,L2,L3,W):
        
        self.joints = [
            JointSpec('R', 'z'),
            JointSpec('R', 'y'),
            JointSpec('R', 'z'),
            JointSpec('R', 'y'),
            JointSpec('R', 'z'),
            JointSpec('R', 'y'),
            JointSpec('R', 'z')
        ]

        self.links = [
            RobotLink(0, self.joints[0]),  # L1: J1 회전 축 관절
            RobotLink(L1-W, self.joints[1], color='c'),  # L2: J1 관절에 의해 회전
            RobotLink(0, self.joints[2]),  # J2 회전 축 관절
            RobotLink(W, JointSpec(None,'xz'), color='c'),  # L3: J2 관절에 의해 회전
            RobotLink(W, self.joints[3], color='y'),  # L4: J3 직선 관절
            RobotLink(L4, self.joints[3], color='k'),
            RobotLink(-L4, self.joints[3], color='k')   # End-effector: J4 회전 관절
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