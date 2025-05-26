import numpy as np
import Robot
import matplotlib.pyplot as plt
from MyRobotMath import SE3

scara = Robot.SCARA(1,1.5,1.5)
se3 = SE3()


M = scara.zero
B = scara.B_tw
S = scara.S_tw
N = len(scara.joint)

desired = [0.6,0.4,0,0,0,-77] # 목표 자세의 (x,y,z,roll,pitch,yaw) / SCARA는 roll, pitch 없음
T_d = se3.pose_to_SE3(desired) # 목표 자세의 Tranformation Matrix
threshold = 1e-6 # 오차 범위
count = 0

init = Robot.select_init(desired, scara.preset) 

while True:

    matexps_b = []
    matexps_s = []

    count += 1 # 연산 횟수 증가

    for i in range(N):
        matexps_b.append(se3.matexp(scara.joint[i],init[i],B[i])) # Body Axis 기준 각 축의 Matrix Exponential
        matexps_s.append(se3.matexp(scara.joint[i],init[i],S[i])) # Space Axis 기준 각 축의 Matrix Exponential

    T_sb = se3.matsb(M,matexps_b) # Forward Kinematics 적용 변환행렬
    estimated = []
    for i in range(3):
        estimated.append(T_sb[i,3].item()) # 현재 x, y, z
    
    eulerAngles = se3.CurrenntAngles(T_sb)
    for eulerAngle in eulerAngles:
        estimated.append(eulerAngle) # Euler 각도 추정값

    pos_err = np.array(desired[:3]) - np.array(estimated[:3]) # x, y, z 오차

    T_bd = np.dot(np.linalg.inv(T_sb),T_d) # Relative Trasformation Matrix
    J_b = se3.body_jacobian(M,matexps_b,matexps_s,S) # Body Jacobian
    J_pseudo = se3.j_inv(J_b) # Jacobian의 역행렬 (또는 의사역행렬)
    V_bd = se3.relativetwist(T_bd) # Ralative Twist, 각도 오차

    theta = Robot.deg2rad(init,scara.joint)

    thetak = theta.reshape(N,1) + J_pseudo @ V_bd.reshape(6,1) # Newton Raphson Method
    thetak = Robot.rad2deg(thetak,scara.joint)

    # 각도 정규화 (-180~180)
    init = Robot.theta_normalize(thetak,scara.joint)

    if np.all(np.abs(pos_err) < threshold): # 오차가 임계값 이내면 break
        print(estimated)
        print(f"연산 횟수 : {count}, Joint Value : {init}")
        break

# time.sleep(0.001)