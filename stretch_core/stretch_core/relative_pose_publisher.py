#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose2D
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math


class RelativePosePublisher(Node):
    """
    發布從driver啟動點開始的相對位姿。
    
    此節點訂閱 /odom topic，計算相對於起始位置的位移，
    並發布兩個topics：
    - /relative_pose_2d: 簡單的2D位姿 (x, y, theta)
    - /relative_pose_stamped: 完整的位姿資訊（包含四元數）
    """
    
    def __init__(self):
        super().__init__('relative_pose_publisher')

        self.declare_parameter('tf_frame_prefix', '')
        tf_prefix = self.get_parameter('tf_frame_prefix').value
        tf_prefix = tf_prefix.strip('/') if tf_prefix else ''
        self.relative_frame_id = f'{tf_prefix}/odom_relative' if tf_prefix else 'odom_relative'
        
        # 訂閱里程計數據
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # 發布相對2D位姿 (簡單格式)
        self.pose_2d_publisher = self.create_publisher(
            Pose2D,
            '/relative_pose_2d',
            10
        )
        
        # 發布完整的相對位姿（包含四元數和時間戳）
        self.pose_stamped_publisher = self.create_publisher(
            PoseStamped,
            '/relative_pose_stamped',
            10
        )
        
        # 儲存起始位置和狀態
        self.initial_pose = None
        self.pose_initialized = False
        
        # 定時器用於發布狀態資訊
        self.status_timer = self.create_timer(5.0, self.status_callback)
        
        self.get_logger().info('Relative pose publisher initialized. Waiting for first odometry message...')
        
    def odom_callback(self, msg):
        """處理里程計數據並計算相對位姿"""
        
        if not self.pose_initialized:
            # 記錄起始位置
            self.initial_pose = msg.pose.pose
            self.pose_initialized = True
            self.get_logger().info(f'Initial pose recorded: x={self.initial_pose.position.x:.3f}, '
                                 f'y={self.initial_pose.position.y:.3f}')
            # 發布起始位置 (0, 0, 0)
            self._publish_zero_pose(msg.header.stamp)
            return
        
        # 計算相對位移
        current_pose = msg.pose.pose
        
        # 計算相對位置
        relative_x = current_pose.position.x - self.initial_pose.position.x
        relative_y = current_pose.position.y - self.initial_pose.position.y
        relative_z = current_pose.position.z - self.initial_pose.position.z
        
        # 計算相對角度
        try:
            _, _, initial_yaw = euler_from_quaternion([
                self.initial_pose.orientation.x,
                self.initial_pose.orientation.y,
                self.initial_pose.orientation.z,
                self.initial_pose.orientation.w
            ])
            
            _, _, current_yaw = euler_from_quaternion([
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w
            ])
            
            relative_yaw = current_yaw - initial_yaw
            
            # 正規化角度到 [-pi, pi]
            while relative_yaw > math.pi:
                relative_yaw -= 2 * math.pi
            while relative_yaw < -math.pi:
                relative_yaw += 2 * math.pi
                
        except Exception as e:
            self.get_logger().warn(f'Failed to calculate relative yaw: {e}')
            relative_yaw = 0.0
        
        # 發布2D位姿 (簡單格式)
        pose_2d_msg = Pose2D()
        pose_2d_msg.x = relative_x
        pose_2d_msg.y = relative_y
        pose_2d_msg.theta = relative_yaw
        self.pose_2d_publisher.publish(pose_2d_msg)
        
        # 發布完整位姿（包含四元數）
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = msg.header.stamp
        pose_stamped_msg.header.frame_id = self.relative_frame_id  # 使用自定義frame_id
        pose_stamped_msg.pose.position.x = relative_x
        pose_stamped_msg.pose.position.y = relative_y
        pose_stamped_msg.pose.position.z = relative_z
        
        # 計算相對四元數
        try:
            relative_quat = quaternion_from_euler(0, 0, relative_yaw)
            pose_stamped_msg.pose.orientation.x = relative_quat[0]
            pose_stamped_msg.pose.orientation.y = relative_quat[1]
            pose_stamped_msg.pose.orientation.z = relative_quat[2]
            pose_stamped_msg.pose.orientation.w = relative_quat[3]
        except Exception as e:
            self.get_logger().warn(f'Failed to calculate relative quaternion: {e}')
            pose_stamped_msg.pose.orientation.w = 1.0  # 默認無旋轉
            
        self.pose_stamped_publisher.publish(pose_stamped_msg)
        
    def _publish_zero_pose(self, timestamp):
        """發布起始位置 (0, 0, 0)"""
        
        # 發布2D位姿
        pose_2d_msg = Pose2D()
        pose_2d_msg.x = 0.0
        pose_2d_msg.y = 0.0
        pose_2d_msg.theta = 0.0
        self.pose_2d_publisher.publish(pose_2d_msg)
        
        # 發布完整位姿
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = timestamp
        pose_stamped_msg.header.frame_id = self.relative_frame_id
        pose_stamped_msg.pose.position.x = 0.0
        pose_stamped_msg.pose.position.y = 0.0
        pose_stamped_msg.pose.position.z = 0.0
        pose_stamped_msg.pose.orientation.w = 1.0  # 無旋轉
        self.pose_stamped_publisher.publish(pose_stamped_msg)
        
    def status_callback(self):
        """定期發布狀態資訊"""
        if self.pose_initialized:
            self.get_logger().info('Relative pose publisher is running and publishing topics: '
                                 '/relative_pose_2d, /relative_pose_stamped')
        else:
            self.get_logger().info('Waiting for odometry data to initialize relative pose...')


def main(args=None):
    rclpy.init(args=args)
    node = RelativePosePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()