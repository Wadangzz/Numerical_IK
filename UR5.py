import numpy as np
import Robot
import socket
import json
import threading
import time
import visual
import MyRobotMath as math

HOST = '127.0.0.1'
PORT = 5000

ur5 = Robot.UR5()

M = ur5.zero
B = ur5.B_tw
S = ur5.S_tw
L = len(ur5.joints)

desired = [350,-250,400,0,0,45] # x y z roll pitch yaw
init = [0,0,0,0,0,0]
# J1 J2 J3 J4 J5 J6
# init = [150.58620664045242, 91.2630905996635, -141.15415909989443, 116.27881380844283, -111.32304266645906, 23.429472188685242]

end = math.IK(ur5,init,desired)

start = np.array(init)
end = np.array(end)

d_theta, trajectory, _, _ = math.create_trajectory(start,end,times=1.0,samples=500)

visual.plot_trajectory(start,d_theta,L)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen(1)
    print(f"Waiting for Unity to connect on {HOST}:{PORT} ...")
    conn, addr = s.accept()
    print(f" Unity connected from {addr}")

    with conn:
        for angles in trajectory:
            # JSON 직렬화
            data = json.dumps(angles.tolist()).encode('utf-8')
            conn.sendall(data + b'\n')  # 한 줄 단위로 구분
            time.sleep(0.001) 
