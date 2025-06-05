import numpy as np
import Robot
import socket
import json
import threading
import time
import msvcrt
import MyRobotMath as math

HOST = '127.0.0.1'
PORT = 5000

ur5 = Robot.UR5()
M = ur5.zero

init = None
coodinate = ['x','y','z','roll','pitch','yaw']

def receive_loop(sock):
    global init
    with sock.makefile('r', encoding='utf-8') as f:
        for line in f:
            try:
                data = json.loads(line.strip())
                print("[DATA]", data)
                init = data
            except json.JSONDecodeError:
                print("[WARN] Invalid JSON")

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    print(f" Unity connected on {HOST}:{PORT}")

    angle_thread = threading.Thread(target=receive_loop,args=(s,))
    angle_thread.start()

    while init is None:
        print("[INFO] Waiting for joint angle data from Unity...")
        time.sleep(0.1)

    while True:

        desired = []
        for i, comp in enumerate(coodinate):
            desired.append(float(input(f"{comp} : ")))
        # init = [46.55746, 33.47226, -144.0964, 110.6242, 28.4425, 9.614345E-11]
        # J1 J2 J3 J4 J5 J6
        # init = [150.58620664045242, 91.2630905996635, -141.15415909989443, 116.27881380844283, -111.32304266645906, 23.429472188685242]

        end = math.IK(ur5,init,desired)

        if np.abs(end[1]) > 95:
            phi = sum(end[1:4])
            k_1 = ur5.links[2]+ur5.links[4]*np.cos(np.deg2rad(end[2]))
            k_2 = -ur5.links[4]*np.sin(np.deg2rad(end[2]))
            
            end[2] = -end[2]
            end[1] = end[1] -np.rad2deg(2*np.arctan2(k_2,k_1))
            end[3] = phi-(end[1]+end[2])

        start = np.array(init)
        end = np.array(end)

        trajectory = math.joint_trajectory(start,end,times=1.0,samples=100)

        # visual.plot_trajectory(start,d_theta,L)

        for angles in trajectory:
            # JSON 직렬화
            data = json.dumps(angles.tolist()).encode('utf-8')
            s.sendall(data + b'\n')  # 한 줄 단위로 구분
            time.sleep(0.001) 

        