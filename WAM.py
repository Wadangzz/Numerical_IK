import numpy as np
import matplotlib.pyplot as plt
import Robot
from MyRobotMath import SE3, quintic_time_scaling

wam = Robot.BarretWAM(55,30,6,4.5) # 링크 길이
se3 = SE3()

M = wam.zero
B = wam.B_tw
S = wam.S_tw
L = len(wam.joint)

desired = [25.5,10,55,10,-34,121.5] # 목표 자세의 (x,y,z,roll,pitch,yaw)
T_d = se3.pose_to_SE3(desired) # 목표 자세의 Tranformation Matrix
threshold = 1e-6 # 오차 범위
count = 0

init = Robot.select_init(desired, wam.preset) 

while True:

    matexps_b = []
    matexps_s = []

    count += 1 # 연산 횟수 증가

    for i in range(L):
        matexps_b.append(se3.matexp(wam.joint[i],init[i],B[i])) # Body Axis 기준 각 축의 Matrix Exponential
        matexps_s.append(se3.matexp(wam.joint[i],init[i],S[i])) # Space Axis 기준 각 축의 Matrix Exponential

    T_sb = se3.matsb(M,matexps_b) # Forward Kinematics 적용 변환행렬
    estimated = []
    for i in range(3):
        estimated.append(T_sb[i,3].item()) # 현재 x, y, z
    
    eulerAngles = se3.CurrenntAngles(T_sb)
    for eulerAngle in eulerAngles:
        estimated.append(eulerAngle) # Euler 각도 추정값

    pos_err = np.array(desired) - np.array(estimated) # x, y, z 오차

    T_bd = np.dot(np.linalg.inv(T_sb),T_d) # Relative Trasformation Matrix
    J_b = se3.body_jacobian(M,matexps_b,matexps_s,S) # Body Jacobian
    J_pseudo = se3.j_inv(J_b) # Jacobian의 역행렬 (또는 의사역행렬)
    V_bd = se3.relativetwist(T_bd) # Ralative Twist, 각도 오차

    theta = Robot.deg2rad(init,wam.joint)
    thetak = theta.reshape(L,1) + J_pseudo @ V_bd.reshape(6,1) # Newton Raphson Method
    thetak = Robot.rad2deg(thetak,wam.joint) # radian to degree
    init = Robot.theta_normalize(thetak,wam.joint) # 각도 정규화 (-180~180)

    if np.all(np.abs(pos_err) < threshold): # 오차가 임계값 이내면 break
        print(f"연산 횟수 : {count}")
        print(f"Joint Value : {init}")
        print(F"estimated Position : {estimated}")
        break

start = [0,0,0,0,0,0,0]
theta_start = np.array(start)
theta_end = np.array(init)

T = 2.0 # 이동 시간
N = 200 # 샘플링 갯수
time = np.linspace(0, T, N)

trajectory = []
velocity = []
acceleration = []

for i in range(N):
    t = i / N * T
    s, s_dot, s_ddot = quintic_time_scaling(t, T) # 5차 다항식 time scaling
    theta_desired = theta_start + s*(theta_end-theta_start)
    theta_dot = s_dot*(theta_end-theta_start)
    theta_ddot = s_ddot*(theta_end-theta_start)
    trajectory.append(theta_desired)
    velocity.append(theta_dot)
    acceleration.append(theta_ddot)

trajectory = np.array(trajectory).T  # shape = (4, N)
velocity = np.array(velocity).T
acceleration = np.array(acceleration).T

fig, axs = plt.subplots(1, 3, figsize=(15, 4))
fig.suptitle("BarretWAM Joint Trajectory", fontsize=16)

joint_names = ["θ1", "θ2", "θ3", "θ4", "θ5", "θ6", "θ7"]

# Position (Trajectory)
for i in range(L):
    axs[0].plot(time, trajectory[i], label=joint_names[i])

axs[0].set_xlabel("Time (s)")
axs[0].set_ylabel("Joint Angle (deg)")
axs[0].set_title("Joint Position Profile")
axs[0].legend()
axs[0].grid(True)

# Velocity
for i in range(L):
    axs[1].plot(time, velocity[i], label=joint_names[i])

axs[1].set_xlabel("Time (s)")
axs[1].set_ylabel("Joint Velocity (deg/s)")
axs[1].set_title("Joint Velocity Profile")
axs[1].legend()
axs[1].grid(True)

# Acceleration
for i in range(L):
    axs[2].plot(time, acceleration[i], label=joint_names[i])
axs[2].set_xlabel("Time (s)")

axs[2].set_ylabel("Joint Acceleration (deg/s²)")
axs[2].set_title("Joint Acceleration Profile")
axs[2].legend()
axs[2].grid(True)

plt.tight_layout()
plt.show()

# time.sleep(0.001)