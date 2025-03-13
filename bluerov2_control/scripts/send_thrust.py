#!/usr/bin/env python3
import rospy
import numpy as np
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from std_msgs.msg import Header

class ThrustPublisher:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('thrust_publisher', anonymous=True)
        
        # 创建6个推进器的发布者
        self.pubs = []
        for i in range(6):
            pub = rospy.Publisher(
                f'/bluerov2/thrusters/{i}/input',  # 修改为正确的话题名称
                FloatStamped,
                queue_size=10
            )
            self.pubs.append(pub)
        
        # 设置发布频率
        self.rate = rospy.Rate(20)  # 20Hz
        
        # 初始化时间和相位
        self.start_time = rospy.get_time()
        self.phases = np.array([0, np.pi/3, 2*np.pi/3, np.pi, 4*np.pi/3, 5*np.pi/3])
        
        # 设置正弦波参数s
        self.amplitude = 5  # 振幅
        self.frequency = 0.2  # 频率 (Hz)
    
    def generate_thrust(self, t, thruster_id):
        """生成推进器的推力值"""
        # 为不同推进器生成不同的正弦波
        thrust = self.amplitude * np.sin(self.frequency * t + self.phases[thruster_id])
        
        # 添加一些随机噪声
        noise = np.random.normal(0, 0.05)  # 减小噪声
        thrust += noise
        
        return thrust
    
    def run(self):
        while not rospy.is_shutdown():
            current_time = rospy.get_time() - self.start_time
            
            # 为每个推进器发布数据
            for i in range(6):
                msg = FloatStamped()
                msg.header = Header()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = f'bluerov2/thruster_{i}'
                
                # 生成推力值
                msg.data = self.generate_thrust(current_time, i)
                
                # 发布消息
                self.pubs[i].publish(msg)
            
            # 按照设定的频率发布
            self.rate.sleep()

if __name__ == '__main__':
    try:
        thrust_publisher = ThrustPublisher()
        thrust_publisher.run()
    except rospy.ROSInterruptException:
        pass 