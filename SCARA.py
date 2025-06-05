import numpy as np
import Robot
import MyRobotMath as math
import visual
from MyRobotMath import SE3

se3=SE3()
scara = Robot.SCARA(50,75,75) # 링크 + End effector 길이

M = scara.zero

desired = [-100,-74,65,0,0,90] # 목표 자세의 (x,y,z,roll,pitch,yaw) / SCARA는 roll, pitch 없음
initpos = M[:3,3].tolist()
init = [initpos[0],initpos[1],initpos[2],0,0,0]
theta_0 = [0,0,0,0]

X_s = math.joint_trajectory(init,desired,times=1.0,samples=100)

print(X_s[-1])
joint_trajectory = []

for i in range(len(X_s)):
    x0, y0 ,z0 = X_s[i][:3,3].flatten()
    roll0, pitch0, yaw0 = se3.CurrenntAngles(X_s[i])
    pos_d = [x0, y0, z0, roll0, pitch0, yaw0]
    theta = math.IK(scara,theta_0,pos_d)
    joint_trajectory.append(theta)
    theta_0 = theta

# end= math.IK(scara,init,desired)
# print(end)

# start = np.array(init)
# end = np.array(end)

# visual.plot_trajectory(start,d_theta,L)
visual.animate_robot(scara,joint_trajectory,samples=100)