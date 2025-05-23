import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

class SE3:

    def skew(self, v):
        """
        Create a skew-symmetric matrix from a 3D vector.
        :param v: 3D vector (list)
        :return: 3x3 skew-symmetric matrix
        """
        return np.array([[0, -v[2], v[1]],
                        [v[2], 0, -v[0]],
                        [-v[1], v[0], 0]])
    
    def adjoint(self, T):
        """
        Compute the adjoint representation of a transformation matrix.
        :param T: 4x4 transformation matrix
        :return: Adjoint representation (6x6 matrix)
        """
        R = T[:3, :3]
        p = T[:3, 3]
        
        adj = np.zeros((6, 6))
        adj[:3, :3] = R
        adj[3:6, 3:6] = R
        adj[3:6, :3] = self.skew(p) @ R
        
        return adj
    
    def matexp(self, joint, degree, twist): # Matrix exponential
        """
        Compute the matrix exponential of a se(3) element.
        :param theta : Angle of rotation
        :param bodytwist 6D vector representing the se(3) element
        :return: 4x4 transformation matrix
        """
        if joint == 'R': # Revolute Joint 일 때
            theta = np.deg2rad(degree)
        elif joint == 'P': # Prismatic Joint 일 때
            theta = degree

        omega = twist[:3]
        v = twist[3:6]
        
        omega_hat = self.skew(omega)
        omega_hat_sq = np.dot(omega_hat, omega_hat)
        
        R = np.eye(3) + np.sin(theta)*omega_hat + (1 - np.cos(theta))*omega_hat_sq
        G_theta = np.eye(3)*theta + (1 - np.cos(theta))*omega_hat + (theta - np.sin(theta))*omega_hat_sq
        p = G_theta @ v
        
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = p
        
        return T
    
    def matsb(self,m,matexps):
        """
        Compute the forword kinematics transformation matrix (body axis).
        :param m : Initial transformation matrix (4x4)
        :param matexps : List of body transformation matrices (4x4)
        :return: 4x4 transformation matrix
        """
        T_sb = m
        for i in range(len(matexps)):
            T_sb = np.dot(T_sb, matexps[i]) # Forward kinematics to get the end-effector transformation matrix

        return T_sb
    
    def space_jacobian(self, matexps, v):
        """
        Compute the space Jacobian for a series of transformations.
        :param matexps: List of space transformation matrices (4x4)
        :param v: List of space twists (6D vectors)
        :return: Space Jacobian (6xN matrix)
        """
        j_s = np.zeros((6, len(matexps)))
        j_s[:, 0] = np.array(v[0]).reshape(6,)
        mul = np.eye(4)
        for i in range(1,len(matexps)):
            mul = np.dot(mul, matexps[i-1])
            j_s[:, i] = np.dot(self.adjoint(mul), np.array(v[i]).reshape(6,))
        
        return j_s

    def body_jacobian(self, m, matexps_b, matexps_s, v):
        """
        Compute the body Jacobian for a series of transformations.
        :param m: Initial transformation matrix (4x4)
        :param matexps_b: List of body transformation matrices (4x4)
        :param matexps_s: List of space transformation matrices (4x4)
        :param v: List of space twists (6D vectors)
        :return: Body Jacobian (6xN matrix)
        """
        T_sb = self.matsb(m,matexps_b)
        T_bs = np.linalg.inv(T_sb) # Inverse of the transformation matrix
        adj_bs = self.adjoint(T_bs)

        j_b = adj_bs @ self.space_jacobian(matexps_s, v)

        return j_b

    
    def pose_to_SE3(self, desired, degree = True):
        """
        Compute the desired transformation matrix.
        :param derised: List of disired position (x, y, z, roll, pitch, yaw)
        :param degree: Unit of axis (True = Degree, False = Radian)
        :return: Desired transformation matrix (4x4 matrix)
        """
        x, y, z = desired[0], desired[1], desired[2]
        j1 ,j2 ,j3 = desired[3], desired[4], desired[5]

        if degree:
            roll, pitch, yaw = np.deg2rad([j1, j2, j3])
        else:
            roll, pitch, yaw = j1, j2, j3
        
        rotmat = R.from_euler('zyx',[yaw,pitch,roll]).as_matrix()
       
        T = np.eye(4)
        T[:3,:3] = rotmat
        T[:3, 3] = [float(x), float(y), float(z)]

        return T
    
    def j_inv(self,j_b):
        """
        Compute pseudoinverse of the jacobian matrix.
        :param j_b : Body Jacobian (6xN matrix)
        :return : Pseudoinverse of the jacobian matrix.
        """
        # return np.linalg.inv(j_b.T @ j_b) @ j_b.T # Moore-Penrose 의사역행렬 (tall)
        return np.linalg.pinv(j_b) # 특이값 분해 기반 의사역행렬, square이면 그냥 역행렬
    
    def relativetwist(self, T_bd):
        """
        Compute the relative twist from the desired transformation matrix
        using matrix logarithm.
        :param T_bd : Desired transformation matrix (4x4)
        :return: 6D vector
        """
        T = np.zeros(6)
        R_bd = T_bd[:3,:3]
        p = T_bd[:3,3]
        trace = np.trace(R_bd)
        cos_theta = np.clip((trace-1)/2,-1.0,1.0) # 부동소수점으로 범위 초과하는거 방지
        theta_bd = np.acos(cos_theta) + np.random.normal(0,0.1) # theta_bd가 0이 되면 Nan 에러 발생, 가우시안 노이즈 추가
        omega_bd_hat = (1 / (2 * np.sin(theta_bd)) * (R_bd-R_bd.T))
        omega_bd_hat_sq = omega_bd_hat**2
        omega_bd = [omega_bd_hat[2][1],omega_bd_hat[0][2],omega_bd_hat[1][0]]
        v_bd = (np.eye(3)/theta_bd - 0.5*omega_bd_hat+(1/theta_bd-0.5/np.tan(0.5*theta_bd))*omega_bd_hat_sq) @ p
        T[:3] = omega_bd
        T[3:] = v_bd
        T = theta_bd*T
        return T
    
    def CurrenntAxis(self, T_sb):
        """
        Compute the current Euler angle
        :param T_sb : Current forword kinematics transformation matrix (body axis)
        :return : List of Euler angle roll,pitch,yaw (Degree)
        """
        R_31 = T_sb[2,0]

        if R_31**2 != 1:
            roll = np.atan2(T_sb[2,1],T_sb[2,2])
            pitch = np.atan2(-1*T_sb[2,0],np.sqrt(T_sb[0,0]**2+T_sb[1,0]**2))
            yaw = np.atan2(T_sb[1,0],T_sb[0][0])
        
        else:
            yaw = 0
            pitch = -R_31 * np.pi/2 
            roll = -R_31 * np.atan2(T_sb[0,1],T_sb[1,1])
        
        return np.rad2deg([roll, pitch, yaw]).tolist()




        
