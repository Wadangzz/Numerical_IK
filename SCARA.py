import numpy as np
import Robot
import MyRobotMath as math
import visual

scara = Robot.SCARA(50,75,75) # 링크 + End effector 길이

M = scara.zero
B = scara.B_tw
S = scara.S_tw
L = len(scara.joints)

desired = [10,-134,24,0,0,-135] # 목표 자세의 (x,y,z,roll,pitch,yaw) / SCARA는 roll, pitch 없음
init = [-153.83643734777988, -62.01562431857724, 10.0, -54.14793833364291]

end= math.IK(scara,init,desired)

start = np.array(init)
end = np.array(end)

d_theta, trajectory, _, _ = math.create_trajectory(start,end,times=0.5)

visual.plot_trajectory(start,d_theta,L)
visual.animate_robot(scara,trajectory)