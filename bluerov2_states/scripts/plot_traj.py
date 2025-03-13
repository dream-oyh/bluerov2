#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import numpy as np
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # 导入3D工具包

class TrajPlotter:
    def __init__(self):
        # 加载参考轨迹
        self.ref_traj = self.load_reference_trajectory()
        
        # 初始化数据列表
        self.x_positions = []
        self.y_positions = []
        self.z_positions = []
        self.times = []
        self.start_time = None
        
        # 误差数据
        self.x_errors = []
        self.y_errors = []
        self.z_errors = []
        self.current_ref_idx = 0
        
        # 创建两个图形窗口
        self.fig1 = plt.figure(figsize=(15, 10))
        self.fig2 = plt.figure(figsize=(10, 8))
        
        # 设置第一个图形（位置和误差）
        self.gs = self.fig1.add_gridspec(3, 2)
        self.pos_axs = [
            self.fig1.add_subplot(self.gs[0, 0]),
            self.fig1.add_subplot(self.gs[1, 0]),
            self.fig1.add_subplot(self.gs[2, 0])
        ]
        self.err_axs = [
            self.fig1.add_subplot(self.gs[0, 1]),
            self.fig1.add_subplot(self.gs[1, 1]),
            self.fig1.add_subplot(self.gs[2, 1])
        ]
        self.fig1.suptitle('BlueROV2 Trajectory and Errors')
        
        # 设置第二个图形（3D轨迹）
        self.ax3d = self.fig2.add_subplot(111, projection='3d')
        self.ax3d.set_title('3D Trajectory')
        self.ax3d.set_xlabel('X (m)')
        self.ax3d.set_ylabel('Y (m)')
        self.ax3d.set_zlabel('Z (m)')
        
        # 绘制参考轨迹
        self.ax3d.plot(self.ref_traj[:, 0], self.ref_traj[:, 1], self.ref_traj[:, 2], 
                       'r-', label='Reference', alpha=0.5)
        
        # 初始化实际轨迹线和当前位置点
        self.traj_line, = self.ax3d.plot([], [], [], 'b-', label='Actual')
        self.current_point, = self.ax3d.plot([], [], [], 'bo', markersize=8)
        self.ax3d.legend()
        
        # 初始化2D图的线条
        self.pos_lines = []
        self.ref_lines = []  # 添加参考轨迹线
        self.err_lines = []
        
        # 为位置图添加实际轨迹和参考轨迹
        for ax in self.pos_axs:
            line, = ax.plot([], [], 'b-', label='Actual')
            ref_line, = ax.plot([], [], 'r--', label='Reference')  # 添加参考轨迹线
            self.pos_lines.append(line)
            self.ref_lines.append(ref_line)
            ax.grid(True)
            ax.legend()  # 添加图例
            
        for ax in self.err_axs:
            line, = ax.plot([], [], 'r-')
            self.err_lines.append(line)
            ax.grid(True)
        
        # 设置标签
        self.pos_axs[0].set_ylabel('X Position (m)')
        self.pos_axs[1].set_ylabel('Y Position (m)')
        self.pos_axs[2].set_ylabel('Z Position (m)')
        self.pos_axs[2].set_xlabel('Time (s)')
        
        self.err_axs[0].set_ylabel('X Error (m)')
        self.err_axs[1].set_ylabel('Y Error (m)')
        self.err_axs[2].set_ylabel('Z Error (m)')
        self.err_axs[2].set_xlabel('Time (s)')
        
        # 初始化ROS节点和订阅者
        rospy.init_node('plot_traj', anonymous=True)
        self.sub = rospy.Subscriber("/bluerov2/pose_gt", Odometry, self.callback)
        
        # 设置动画
        self.ani1 = FuncAnimation(self.fig1, self.update_2d, interval=300)
        self.ani2 = FuncAnimation(self.fig2, self.update_3d, interval=300)
        
        # 添加参考轨迹时间计算相关的变量
        self.ref_period = None  # 参考轨迹的周期
        self.calculate_ref_period()
        
        plt.show()
    
    def load_reference_trajectory(self):
        # 从文件加载参考轨迹
        traj = rospy.get_param('~traj', 'lemniscate')
        traj_file = f'/home/dream/catkin_ws/src/bluerov2/bluerov2_mpc/traj/{traj}.txt'
        try:
            data = np.loadtxt(traj_file)
            return data[:, :3]  # 只取前三列 (x, y, z)
        except Exception as e:
            rospy.logerr(f"Error loading trajectory file: {e}")
            return np.array([[0, 0, -20]])  # 默认位置
    
    def calculate_ref_period(self):
        """计算参考轨迹的周期"""
        if len(self.ref_traj) > 1:
            # 假设轨迹是周期性的，找到第一个和起点最近的点
            start_point = self.ref_traj[0]
            distances = np.linalg.norm(self.ref_traj[1:] - start_point, axis=1)
            period_idx = np.argmin(distances) + 1
            if period_idx < len(self.ref_traj):
                self.ref_period = period_idx
            else:
                self.ref_period = len(self.ref_traj)
    
    def get_reference_points(self, current_time):
        """根据当前时间获取对应的参考轨迹点"""
        if self.ref_period is None or len(self.ref_traj) == 0:
            return None, None
        
        # 计算每个轨迹点对应的时间
        points_per_second = 20  # 每秒的轨迹点数
        time_per_period = self.ref_period / points_per_second
        
        # 计算当前时间在周期内的位置
        time_in_period = current_time % time_per_period
        
        # 计算对应的轨迹点索引
        point_idx = int((time_in_period / time_per_period) * self.ref_period)
        point_idx = min(point_idx, len(self.ref_traj) - 1)
        
        # 返回参考点
        return self.ref_traj[point_idx], point_idx
    
    def callback(self, data):
        if self.start_time is None:
            self.start_time = rospy.get_time()
        
        # 记录当前位置
        current_time = rospy.get_time() - self.start_time
        current_x = data.pose.pose.position.x
        current_y = data.pose.pose.position.y
        current_z = data.pose.pose.position.z
        
        self.times.append(current_time)
        self.x_positions.append(current_x)
        self.y_positions.append(current_y)
        self.z_positions.append(current_z)
        
        # 获取当前时刻对应的参考位置
        ref_pos, _ = self.get_reference_points(current_time)
        if ref_pos is not None:
            self.x_errors.append(current_x - ref_pos[0])
            self.y_errors.append(current_y - ref_pos[1])
            self.z_errors.append(current_z - ref_pos[2])
        else:
            self.x_errors.append(0)
            self.y_errors.append(0)
            self.z_errors.append(0)
        
        # 限制数据点数量
        max_points = 300
        if len(self.times) > max_points:
            self.times = self.times[-max_points:]
            self.x_positions = self.x_positions[-max_points:]
            self.y_positions = self.y_positions[-max_points:]
            self.z_positions = self.z_positions[-max_points:]
            self.x_errors = self.x_errors[-max_points:]
            self.y_errors = self.y_errors[-max_points:]
            self.z_errors = self.z_errors[-max_points:]
    
    def update_2d(self, frame):
        # 更新2D图表
        positions = [self.x_positions, self.y_positions, self.z_positions]
        errors = [self.x_errors, self.y_errors, self.z_errors]
        
        # 计算要显示的参考轨迹段
        if len(self.times) > 0:
            current_time = self.times[-1]
            window_size = max(self.times) - min(self.times)
            
            # 生成参考轨迹的时间点
            num_points = 50  # 显示点数
            ref_times = np.linspace(current_time - window_size, current_time, num_points)
            
            # 计算每个时间点对应的参考位置
            ref_positions = [[], [], []]
            for t in ref_times:
                ref_pos, _ = self.get_reference_points(t)
                if ref_pos is not None:
                    for i in range(3):
                        ref_positions[i].append(ref_pos[i])
                
            # 更新位置图和参考轨迹
            for i, (pos_line, ref_line, pos) in enumerate(zip(
                self.pos_lines, self.ref_lines, positions)):
                
                # 更新实际位置
                pos_line.set_data(self.times, pos)
                
                # 更新参考轨迹
                if ref_positions[i]:
                    ref_line.set_data(ref_times, ref_positions[i])
                
                # 设置坐标轴范围
                if len(self.times) > 0:
                    self.pos_axs[i].set_xlim(min(self.times), max(self.times))
                    all_y_values = pos + ref_positions[i]
                    if len(all_y_values) > 0:
                        ymin, ymax = min(all_y_values), max(all_y_values)
                        margin = (ymax - ymin) * 0.1 if ymax != ymin else 0.1
                        self.pos_axs[i].set_ylim(ymin - margin, ymax + margin)
        
        # 更新误差图
        for i, (err_line, err) in enumerate(zip(self.err_lines, errors)):
            err_line.set_data(self.times, err)
            if len(self.times) > 0:
                self.err_axs[i].set_xlim(min(self.times), max(self.times))
                if len(err) > 0:
                    ymin, ymax = min(err), max(err)
                    margin = (ymax - ymin) * 0.1 if ymax != ymin else 0.1
                    self.err_axs[i].set_ylim(ymin - margin, ymax + margin)
        
        return self.pos_lines + self.ref_lines + self.err_lines
    
    def update_3d(self, frame):
        # 更新3D轨迹图
        if len(self.x_positions) > 0:
            # 更新轨迹线
            self.traj_line.set_data(self.x_positions, self.y_positions)
            self.traj_line.set_3d_properties(self.z_positions)
            
            # 更新当前位置点
            self.current_point.set_data([self.x_positions[-1]], [self.y_positions[-1]])
            self.current_point.set_3d_properties([self.z_positions[-1]])
            
            # 自动调整视角
            self.ax3d.relim()
            self.ax3d.autoscale_view()
        
        return self.traj_line, self.current_point

if __name__ == '__main__':
    try:
        plotter = TrajPlotter()
    except rospy.ROSInterruptException:
        pass