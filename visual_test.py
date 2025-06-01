import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from MyRobotMath import quintic_time_scaling

class plot:

    def plot_trajectory(self, theta_start, d_theta, length, times=1.0, samples=400):

        T = times
        N = samples
        time = np.linspace(0, T, N)

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
        
        trajectory = np.array(trajectory).T  # shape = (4, N)
        velocity = np.array(velocity).T
        acceleration = np.array(acceleration).T

        fig, axs = plt.subplots(1, 3, figsize=(15, 4))
        fig.suptitle("SCARA Joint Trajectory", fontsize=16)

        joint_names = []
        for i in range(length):
            joint_names.append(f"J{i+1}")

        # Position (Trajectory)
        for i in range(length):
            axs[0].plot(time, trajectory[i], label=joint_names[i])

        axs[0].set_xlabel("Time (s)")
        axs[0].set_ylabel("J1,J2,J4 (deg) / J3 (cm)")
        axs[0].set_title("Joint Position Profile")
        axs[0].legend()
        axs[0].grid(True)

        # Velocity
        for i in range(length):
            axs[1].plot(time, velocity[i], label=joint_names[i])

        axs[1].set_xlabel("Time (s)")
        axs[1].set_ylabel("J1,J2,J4 (deg/s) / J3 (cm/s)") 
        axs[1].set_title("Joint Velocity Profile")
        axs[1].legend()
        axs[1].grid(True)

        # Acceleration
        for i in range(length):
            axs[2].plot(time, acceleration[i], label=joint_names[i])
        axs[2].set_xlabel("Time (s)")

        axs[2].set_ylabel("J1,J2,J4 (deg/s²) / J3 (cm/s²)")
        axs[2].set_title("Joint Acceleration Profile")
        axs[2].legend()
        axs[2].grid(True)

        plt.tight_layout()
        plt.show()

    def animate_scara(self,scara,trajectory,times=1.0,samples=50):

        fig = plt.figure(figsize=(7, 7))
        ax = fig.add_subplot(111, projection='3d')

        ax.view_init(elev=20, azim=-65)
        
        trajectory_points = []
        ee_path_line = ax.plot([], [], [], 'r--', linewidth=1.0)[0]

        lines = [ax.plot([0, 0], [0, 0], [0, 0], link.color + '-', linewidth=2)[0] for link in scara.links]
        joints = [ax.scatter(0, 0, 0, color='red', s=10) for _ in scara.links]
        ee = ax.scatter(0, 0, 0, color='green', s=10)

        def update(frame):
            # 현재 프레임의 각도로 로봇 업데이트
            scara.update_angles(trajectory[frame])
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

            ax.set_xlim([-200, 200])
            ax.set_ylim([-200, 200])
            ax.set_zlim([0, 400])
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            # scara.ax.set_title(f"Time: {frame/N*T:.2f} s")

        # 애니메이션 생성 및 실행
        ani = FuncAnimation(fig, update, frames=samples, interval = times*1000/samples, repeat=False)
        ani.save('animation.gif', writer = 'pillow', fps = 30)
        plt.show()


class RobotLink:
    def __init__(self, length, joint_type='R', angle=0, z_offset=0, color = 'b'):
        self.length = length
        self.joint_type = joint_type  # 'R' for revolute, 'P' for prismatic
        self.angle = angle
        self.z_offset = z_offset
        self.start_point = np.array([0, 0, 0])
        self.end_point = np.array([0, 0, 0])
        self.color = color
        
    def update_position(self, start_point, angle=0, prismatic=0):
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
            self.end_point = self.start_point + np.array([0, 0, prismatic])

class SCARAVisualizer():
    def __init__(self, L1, L2, L3, L4):
        self.fig = plt.figure(figsize=(7, 7))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # SCARA 로봇 링크 생성
        self.links = [
            RobotLink(L1, None),  # L1: J1 회전 축 관절
            RobotLink(L2, 'R', color='c'),  # L2: J1 관절에 의해 회전
            RobotLink(20, None),  # J2 회전 축 관절
            RobotLink(L3, 'R', color='m'),  # L3: J2 관절에 의해 회전
            RobotLink(0, 'P', color='y'),  # L4: J3 직선 관절
            RobotLink(L4, 'R', color='k'),
            RobotLink(-L4, 'R', color='k')   # End-effector: J4 회전 관절
        ]
        
    def update_angles(self, angles):
        current_point = np.array([0, 0, 0])
        
        # L1 (중앙 Z축)
        self.links[0].update_position(current_point, prismatic=self.links[0].length)
        current_point = self.links[0].end_point
        
        # L2 (J1 회전 관절)
        self.links[1].update_position(current_point, angle=np.deg2rad(angles[0]))
        current_point = self.links[1].end_point

        self.links[2].update_position(current_point, prismatic=self.links[2].length)
        current_point = self.links[2].end_point
        
        # L3 (J2 회전 관절)
        self.links[3].update_position(current_point, angle=np.deg2rad(angles[0]+angles[1]))
        current_point = self.links[3].end_point
        
        # L4 (J3 직선 관절)
        self.links[4].update_position(current_point, prismatic=-self.links[0].length-self.links[2].length+angles[2])
        current_point = self.links[4].end_point
        
        # End-effector (J4 회전 관절)
        self.links[5].update_position(current_point, angle=np.deg2rad(angles[0]+angles[1]+angles[3]))
        self.links[6].update_position(current_point, angle=np.deg2rad(angles[0]+angles[1]+angles[3]))

# 사용 예시
if __name__ == "__main__":
    # SCARA 로봇 생성 (L1=50, L2=75, L3=75, L4=10)
    scara = SCARAVisualizer(50, 75, 75, 10)
    
    # 초기 자세와 목표 자세 설정
    # theta_start = np.array([0, 0, 0, 0])
    # theta_end = np.array([-135, 45.7, 63, -124])
    theta_start = np.array([-135, -45.7, 63, -124])
    theta_end = np.array([172, 150, 30, -66])
    d_theta = theta_end - theta_start

    for i in range(2):
        if np.abs(d_theta[i]) > 180:
            d_theta[i] = -np.sign(d_theta[i])*(360-np.abs(d_theta[i]))
    if np.abs(d_theta[1]+theta_start[1]) > 180:
        d_theta[1] = -np.sign(d_theta[1])*(360-np.abs(d_theta[1]))

    # 시간 설정
    T = 1.0
    N = 50  # 프레임 수
    time = np.linspace(0, T, N)

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

    # 3D 애니메이션 시작
    scara.ax.view_init(elev=20, azim=-65)
    
    trajectory_points = []
    ee_path_line = scara.ax.plot([], [], [], 'r--', linewidth=1.0)[0]

    lines = [scara.ax.plot([0, 0], [0, 0], [0, 0], link.color + '-', linewidth=2)[0] for link in scara.links]
    joints = [scara.ax.scatter(0, 0, 0, color='red', s=10) for _ in scara.links]
    ee = scara.ax.scatter(0, 0, 0, color='green', s=10)

    def update(frame):
        # 현재 프레임의 각도로 로봇 업데이트
        scara.update_angles(trajectory[frame])
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

    # 애니메이션 생성 및 실행
    ani = FuncAnimation(scara.fig, update, frames=N, interval = T*1000/N, repeat=False)
    ani.save('scara_animation2.gif', writer = 'pillow', fps = 30)
    plt.show()
    # angles = [-135, 120, 30, 45]
    # scara.update_angles(angles)
    
    # # SCARA 로봇 시각화
    # scara.visualize() 