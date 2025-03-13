#!/usr/bin/env python3
import rospy
import os
import csv
from nav_msgs.msg import Odometry
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from datetime import datetime

class DataRecorder:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('data_recorder', anonymous=True)
        
        # 创建输出目录
        self.output_dir = os.path.expanduser('~/catkin_ws/output')
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            
        # 创建带时间戳的输出文件
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_file = os.path.join(self.output_dir, f'bluerov2_data_{timestamp}.csv')
        
        # 初始化数据字典
        self.data = {
            'timestamp': None,
            'pos_x': None, 'pos_y': None, 'pos_z': None,
            'orient_x': None, 'orient_y': None, 'orient_z': None, 'orient_w': None,
            'linear_x': None, 'linear_y': None, 'linear_z': None,
            'angular_x': None, 'angular_y': None, 'angular_z': None,
            'thrust_0': None, 'thrust_1': None, 'thrust_2': None,
            'thrust_3': None, 'thrust_4': None, 'thrust_5': None
        }
        
        # 创建CSV文件并写入表头
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.data.keys())
            writer.writeheader()
        
        # 订阅话题
        self.pose_sub = rospy.Subscriber('/bluerov2/pose_gt', Odometry, self.pose_callback)
        
        # 订阅6个推进器的话题
        self.thrust_subs = []
        for i in range(6):
            sub = rospy.Subscriber(
                f'/bluerov2/thrusters/{i}/thrust',
                FloatStamped,
                self.thrust_callback,
                callback_args=i
            )
            self.thrust_subs.append(sub)
        
        rospy.loginfo(f"Data recording started. Output file: {self.csv_file}")
    
    def pose_callback(self, msg):
        # 更新位置和姿态数据
        self.data['timestamp'] = msg.header.stamp.to_sec()
        
        # 位置
        self.data['pos_x'] = msg.pose.pose.position.x
        self.data['pos_y'] = msg.pose.pose.position.y
        self.data['pos_z'] = msg.pose.pose.position.z
        
        # 四元数
        self.data['orient_x'] = msg.pose.pose.orientation.x
        self.data['orient_y'] = msg.pose.pose.orientation.y
        self.data['orient_z'] = msg.pose.pose.orientation.z
        self.data['orient_w'] = msg.pose.pose.orientation.w
        
        # 线速度
        self.data['linear_x'] = msg.twist.twist.linear.x
        self.data['linear_y'] = msg.twist.twist.linear.y
        self.data['linear_z'] = msg.twist.twist.linear.z
        
        # 角速度
        self.data['angular_x'] = msg.twist.twist.angular.x
        self.data['angular_y'] = msg.twist.twist.angular.y
        self.data['angular_z'] = msg.twist.twist.angular.z
        
        # 写入数据
        self.write_data()
    
    def thrust_callback(self, msg, thruster_id):
        # 更新推进器数据
        self.data[f'thrust_{thruster_id}'] = msg.data
    
    def write_data(self):
        # 检查是否所有数据都已准备好
        if all(value is not None for value in self.data.values()):
            with open(self.csv_file, 'a', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.data.keys())
                writer.writerow(self.data)

if __name__ == '__main__':
    try:
        recorder = DataRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 