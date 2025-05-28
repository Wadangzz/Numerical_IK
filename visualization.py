import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from MyRobotMath import quintic_time_scaling


class RobotLink:
    def __init__(self, length, joint_type='R', angle=0, z_offset=0, color = 'b'):
        self.length = length
        self.joint_type = joint_type  # 'R' for revolute, 'P' for prismatic
        self.angle = angle
        self.z_offset = z_offset
        self.start_point = np.array([0, 0, 0])
        self.end_point = np.array([0, 0, 0])
        self.color = color
        
    def update_position(self, start_point, angle, prismatic_offset=0):
        self.start_point = start_point
        self.angle = angle
        
        if self.joint_type == 'R':
            # 회전 관절의 경우
            rotation_matrix = np.array([
                [np.cos(angle), -np.sin(angle), 0],
                [np.sin(angle), np.cos(angle), 0],
                [0, 0, 1]
            ])
            self.end_point = self.start_point + rotation_matrix @ np.array([self.length, 0, 0])
        else:
            # 직선 관절의 경우
            self.end_point = self.start_point + np.array([0, 0, prismatic_offset])

class SCARAVisualizer:
    def __init__(self, L1, L2, L3, L4):
        self.fig = plt.figure(figsize=(7, 7))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # SCARA 로봇 링크 생성
        self.links = [
            RobotLink(L1, 'P'),  # L1: J1 회전 축 관절
            RobotLink(L2, 'R', color='c'),  # L2: J1 관절에 의해 회전
            RobotLink(20, 'P'),  # J2 회전 축 관절
            RobotLink(L3, 'R', color='m'),  # L3: J2 관절에 의해 회전
            RobotLink(0, 'P', color='y'),  # L4: J3 직선 관절
            RobotLink(L4, 'R', color='k'),
            RobotLink(-L4, 'R', color='k')   # End-effector: J4 회전 관절
        ]
        
    def update_angles(self, angles):
        current_point = np.array([0, 0, 0])
        
        # L1 (중앙 Z축)
        self.links[0].update_position(current_point, 0, self.links[0].length)
        current_point = self.links[0].end_point
        
        # L2 (J1 회전 관절)
        self.links[1].update_position(current_point, np.deg2rad(angles[0]))
        current_point = self.links[1].end_point

        self.links[2].update_position(current_point, 0, self.links[2].length)
        current_point = self.links[2].end_point
        
        # L3 (J2 회전 관절)
        self.links[3].update_position(current_point, np.deg2rad(angles[0]+angles[1]))
        current_point = self.links[3].end_point
        
        # L4 (J3 직선 관절)
        self.links[4].update_position(current_point, 0, -self.links[0].length-self.links[2].length+angles[2])
        current_point = self.links[4].end_point
        
        # End-effector (J4 회전 관절)
        self.links[5].update_position(current_point, np.deg2rad(angles[0]+angles[1]+angles[3]))
        self.links[6].update_position(current_point, np.deg2rad(angles[0]+angles[1]+angles[3]))

    def visualize(self):
        self.ax.clear()
        
        # 각 링크 그리기
        for i, link in enumerate(self.links):
            # 링크 그리기
            self.ax.plot([link.start_point[0], link.end_point[0]],
                        [link.start_point[1], link.end_point[1]],
                        [link.start_point[2], link.end_point[2]], 
                        link.color + '-', linewidth=2)
            
            # 관절 표시
            self.ax.scatter(link.start_point[0], link.start_point[1], link.start_point[2], 
                          color='red', s=10)
        
        # 엔드 이펙터 표시
        end_effector_point = self.links[4].end_point
        self.ax.scatter(end_effector_point[0], 
                       end_effector_point[1], 
                       end_effector_point[2], 
                       color='green', s=10)
        
        self.ax.set_xlim([-200, 200])
        self.ax.set_ylim([-200, 200])
        self.ax.set_zlim([0, 400])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        # self.ax.legend()
        plt.show()

# 사용 예시
if __name__ == "__main__":
    # SCARA 로봇 생성 (L1=50, L2=75, L3=75, L4=10)
    scara = SCARAVisualizer(50, 75, 75, 10)
    
    # 초기 자세와 목표 자세 설정
    # theta_start = np.array([0, 0, 0, 0])
    # theta_end = np.array([-135, 45.7, 63, -124])
    theta_start = np.array([-135, 45.7, 63, -124])
    theta_end = np.array([-188, -144, 30, -66])

    d_theta = theta_end - theta_start

    # 시간 설정
    T = 1.0
    N = 50  # 프레임 수
    # time = np.linspace(0, T, N)

    # 궤적 생성
    trajectory = []
    velocity = []
    acceleration = []

    for i in range(N):
        t = i / N * T
        s, s_dot, s_ddot = quintic_time_scaling(t, T)
        theta_desired = theta_start + s*(d_theta)
        theta_dot = s_dot*(d_theta)
        theta_ddot = s_ddot*(d_theta)
        trajectory.append(theta_desired)
        velocity.append(theta_dot)
        acceleration.append(theta_ddot)

    scara.ax.view_init(elev=20, azim=-65)
    
    trajectory_points = []
    ee_path_line = scara.ax.plot([], [], [], 'r--', linewidth=1.0)[0]

    lines = [scara.ax.plot([0, 0], [0, 0], [0, 0], link.color + '-', linewidth=2)[0] for link in scara.links]
    joints = [scara.ax.scatter(0, 0, 0, color='red', s=10) for _ in scara.links]
    ee = scara.ax.scatter(0, 0, 0, color='green', s=10)

    def update(frame):
        # 현재 프레임의 각도로 로봇 업데이트
        scara.update_angles(trajectory[frame].tolist())
        for i, link in enumerate(scara.links):
            lines[i].set_data([link.start_point[0], link.end_point[0]],
                            [link.start_point[1], link.end_point[1]])
            lines[i].set_3d_properties([link.start_point[2], link.end_point[2]])
            joints[i]._offsets3d = ([link.start_point[0]], [link.start_point[1]], [link.start_point[2]])

        end_effector = scara.links[4].end_point
        ee._offsets3d = ([end_effector[0]], [end_effector[1]], [end_effector[2]])

        trajectory_points.append(end_effector.copy())

        if len(trajectory_points) > 1:
            path = np.array(trajectory_points)
            ee_path_line.set_data(path[:, 0], path[:, 1])
            ee_path_line.set_3d_properties(path[:, 2])

        scara.ax.set_title(f"Time: {frame/N*T:.2f} s")
        # scara.ax.clear()

        # # 링크 그리기
        # for i, link in enumerate(scara.links):
        #     scara.ax.plot([link.start_point[0], link.end_point[0]],
        #                 [link.start_point[1], link.end_point[1]],
        #                 [link.start_point[2], link.end_point[2]],
        #                 link.color + '-', linewidth=2)
        #     scara.ax.scatter(link.start_point[0], link.start_point[1], link.start_point[2],
        #                     color='red', s=10)

        # # 엔드 이펙터 표시
        # end_effector_point = scara.links[6].end_point
        # scara.ax.scatter(end_effector_point[0], end_effector_point[1], end_effector_point[2],
        #                 color='green', s=10)

        # 그래프 설정
        scara.ax.set_xlim([-200, 200])
        scara.ax.set_ylim([-200, 200])
        scara.ax.set_zlim([0, 400])
        scara.ax.set_xlabel('X')
        scara.ax.set_ylabel('Y')
        scara.ax.set_zlabel('Z')
        scara.ax.set_title(f"Time: {frame/N*T:.2f} s")

    trajectory_points.clear()
    # 애니메이션 생성 및 실행
    ani = FuncAnimation(scara.fig, update, frames=N, interval = T*1000/N, repeat=False)
    ani.save('scara_animation2.gif', writer = 'pillow', fps = 20)
    plt.show()
    # angles = [-135, 120, 30, 45]
    # scara.update_angles(angles)
    
    # # SCARA 로봇 시각화
    # scara.visualize() 