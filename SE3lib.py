import numpy as np
import matplotlib.pyplot as plt

class SE3:

    def skew(self, v):

        """
        Create a skew-symmetric matrix from a 3D vector.
        :param v: 3D vector
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
    
    def space_jacobian(self, matexps, v):
        """
        Compute the space Jacobian for a series of transformations.
        :param matexps: List of transformation matrices (4x4)
        :param v: List of Space twists (6D vectors)
        :return: Space Jacobian (6xN matrix)
        """
        j_s = np.zeros((6, len(matexps)))
        j_s[:, 0] = np.array(v[0]).reshape(6,)
        mul = np.eye(4)
        for i in range(1,len(matexps)):
            mul = np.dot(mul, matexps[i-1])
            j_s[:, i] = np.dot(self.adjoint(mul), np.array(v[i]).reshape(6,))
        
        return j_s

    def body_jacobian(self, m, matexps, v):
        """
        Compute the body Jacobian for a series of transformations.
        :param m: Initial transformation matrix (4x4)
        :param matexps: List of transformation matrices (4x4)
        :param v: List of Body twists (6D vectors)
        :return: Body Jacobian (6xN matrix)
        """
        T_sb = m
        for i in range(len(matexps)):
            T_sb = np.dot(T_sb, matexps[i]) # Forward kinematics to get the end-effector transformation matrix
        
        T_bs = np.linalg.inv(T_sb) # Inverse of the transformation matrix
        adj_bs = self.adjoint(T_bs)

        j_b = adj_bs @ self.space_jacobian(matexps, v)

        return j_b

    def matexp(self, joint, degree, bodytwist): # body axis 기준 matrix exponential
        """
        Compute the matrix exponential of a se(3) element.
        :param theta : Angle of rotation
        :param bodytwist 6D vector representing the se(3) element
        :return: 4x4 transformation matrix
        """
        if joint == 'R':
            theta = np.deg2rad(degree)
        elif joint == 'P':
            theta = degree

        omega = bodytwist[:3]
        v = bodytwist[3:6]
        
        omega_hat = self.skew(omega)
        omega_hat_sq = np.dot(omega_hat, omega_hat)
        
        R = np.eye(3) + np.sin(theta)*omega_hat + (1 - np.cos(theta))*omega_hat_sq
        G_theta = np.eye(3)*theta + (1 - np.cos(theta))*omega_hat + (theta - np.sin(theta))*omega_hat_sq
        p = G_theta @ v
        
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = p
        
        return T