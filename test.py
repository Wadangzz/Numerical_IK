import numpy as np
import Robot
from MyRobotMath import SE3

scara = Robot.SCARA(2,1,1)
se3 = SE3()

def select_init(desired, preset):
    x, y = desired[0], desired[1]
    key = (x > 0, y > 0)
    mapping = {
        (True,   True): 0,
        (False,  True): 1,
        (False, False): 2,
        (True,  False): 3,
    }
    return preset[mapping[key]]

M = scara.zero
B = scara.B_tw
S = scara.S_tw

desired = [0.6,0.4,0,0,0,-77] # 목표 자세의 (x,y,z,roll,pitch,yaw)
T_d = se3.pose_to_SE3(desired) # 목표 자세의 Tranformation Matrix
threshold = 1e-4 # 오차 범위
count = 0

init = select_init(desired, scara.preset) # 목표 자세에 해당하는 사분면 초기 상태 호출

while True:

    count += 1 # 연산 횟수 증가

    eb_1 = se3.matexp('R',init[0],B[0])
    eb_2 = se3.matexp('R',init[1],B[1])
    eb_3 = se3.matexp('R',init[2],B[2])
    eb_4 = se3.matexp('P',init[3],B[3])

    matexps_b = [eb_1,eb_2,eb_3,eb_4] # Body Axis 기준 각 축의 Matrix Exponential

    es_1 = se3.matexp('R',init[0],S[0])
    es_2 = se3.matexp('R',init[1],S[1])
    es_3 = se3.matexp('R',init[2],S[2])
    es_4 = se3.matexp('P',init[3],S[3])

    matexps_s = [es_1,es_2,es_3,es_4] # Space Axis 기준 각 축의 Matrix Exponential

    T_sb = se3.matsb(M,matexps_b) # Forward Kinematics 적용 변환행렬
    currentPosition = []
    for i in range(3):
        currentPosition.append(T_sb[i,3].item()) # 현재 x, y, z

    eulerAngles = se3.CurrenntAxis(T_sb)
    # print(eulerAngles)
    for eulerAngle in eulerAngles:
        currentPosition.append(eulerAngle) # 현재 Euler 각도
    # print(currentPosition)

    relativePosition = np.array(desired) - np.array(currentPosition) # 목표와 추정값의 오차
    # print(relativePosition)

    T_bd = np.dot(np.linalg.inv(T_sb),T_d) # Relative Trasformation Matrix

    J_b = se3.body_jacobian(M,matexps_b,matexps_s,S) # Body Jacobian
    # print(J_b)
    J_pseudo = se3.j_inv(J_b) # Jacobian의 역행렬 (또는 의사역행렬)
    # print(J_pseudo)
    V_bd = se3.relativetwist(T_bd) # Ralative Twist

    theta = np.zeros(4)
    theta[:3], theta[3] = np.array(np.deg2rad(init[:3])), init[3]
    thetak = theta.reshape(4,1) + J_pseudo @ V_bd.reshape(6,1) # Newton Raphson Method
    thetak[:3] = np.rad2deg(thetak[:3])

    init = thetak.flatten().tolist()
    for i in range(3):
        if init[i] % 360 > 180:
            init[i] = init[i] % 360 - 360
        else:
            init[i] = init[i] % 360
    # print(sum(init[:3]))
    # print(init)

    if np.all(np.abs(relativePosition) < threshold): # 오차가 임계값 이내면 break
        print(currentPosition)
        print(f"연산 횟수 : {count}, 관절 각도 : {init}")
        break

# time.sleep(0.001)