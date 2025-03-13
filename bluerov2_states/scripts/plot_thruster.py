#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
import numpy as np
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from matplotlib.animation import FuncAnimation

class ThrusterPlotter:
    def __init__(self):
        # 初始化数据列表
        self.times = []
        self.start_time = None
        self.thrusts = [[] for _ in range(6)]  # 6个推进器的数据
        
        # 创建图形
        self.fig, self.axs = plt.subplots(3, 2, figsize=(12, 8))
        self.fig.suptitle('BlueROV2 Thrusters')
        
        # 初始化线条
        self.lines = []
        for i, ax in enumerate(self.axs.flat):
            line, = ax.plot([], [], 'b-', label=f'Thruster {i}')
            self.lines.append(line)
            ax.grid(True)
            ax.set_ylabel('Thrust')
            ax.set_xlabel('Time (s)')
            ax.legend()
        
        # 调整子图布局
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        
        # 初始化ROS节点
        rospy.init_node('thruster_plotter', anonymous=True)
        
        # 订阅6个推进器的话题
        self.subs = []
        for i in range(6):
            sub = rospy.Subscriber(
                f'/bluerov2/thrusters/{i}/thrust',
                FloatStamped,
                self.callback,
                callback_args=i
            )
            self.subs.append(sub)
        
        # 设置动画
        self.ani = FuncAnimation(self.fig, self.update, interval=100)
        plt.show()
    
    def callback(self, data, thruster_id):
        if self.start_time is None:
            self.start_time = rospy.get_time()
            self.times.append(0)
        else:
            current_time = rospy.get_time() - self.start_time
            if not self.times or current_time > self.times[-1]:
                self.times.append(current_time)
                # 为所有推进器在这个时间点添加数据位置
                for thrust_list in self.thrusts:
                    if len(thrust_list) < len(self.times):
                        thrust_list.append(None)
        
        # 更新对应推进器的最新数据
        if len(self.thrusts[thruster_id]) > 0:
            self.thrusts[thruster_id][-1] = data.data
        
        # 限制数据点数量
        max_points = 200
        if len(self.times) > max_points:
            self.times = self.times[-max_points:]
            for i in range(6):
                self.thrusts[i] = self.thrusts[i][-max_points:]
    
    def update(self, frame):
        # 更新每个推进器的图表
        for i, (line, thrust_data) in enumerate(zip(self.lines, self.thrusts)):
            # 过滤掉None值
            valid_times = []
            valid_thrusts = []
            for t, thrust in zip(self.times, thrust_data):
                if thrust is not None:
                    valid_times.append(t)
                    valid_thrusts.append(thrust)
            
            # 更新数据
            line.set_data(valid_times, valid_thrusts)
            
            # 设置坐标轴范围
            ax = self.axs.flat[i]
            if valid_times:
                ax.set_xlim(min(self.times), max(self.times))
                if valid_thrusts:
                    ymin, ymax = min(valid_thrusts), max(valid_thrusts)
                    margin = (ymax - ymin) * 0.1 if ymax != ymin else 0.1
                    ax.set_ylim(ymin - margin, ymax + margin)
        
        return self.lines

if __name__ == '__main__':
    try:
        plotter = ThrusterPlotter()
    except rospy.ROSInterruptException:
        pass
