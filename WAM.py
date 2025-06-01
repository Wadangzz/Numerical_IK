import numpy as np
import Robot
import MyRobotMath as math
import visual

wam = Robot.BarretWAM(55,30,6.0,4.5) # 링크 길이 L1, L2, L3, W

M = wam.zero
B = wam.B_tw
S = wam.S_tw
L = len(wam.joints)

desired = [40.5,0,59.5,45,70,0] # 목표 자세의 (x,y,z,roll,pitch,yaw)
init = [0,0,0,0,0,0,0]

end = math.IK(wam,init,desired)

start = np.array(init)
end = np.array(end)

d_theta, trajectory, _, _ = math.create_trajectory(start,end,times=1.0)

visual.plot_trajectory(start,d_theta,L)
visual.animate_robot(wam, trajectory, lim = 80)