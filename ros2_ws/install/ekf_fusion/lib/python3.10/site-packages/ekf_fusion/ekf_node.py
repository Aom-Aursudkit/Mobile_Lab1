import rclpy
from rclpy.node import Node

import math
import time

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        self.get_logger().info('EKF node started')
        
        self.wheel_odom_pub = self.create_publisher(
            Odometry,
            '/wheel_odom',
            10
        )

        self.ekf_odom_pub = self.create_publisher(
            Odometry,
            '/ekf_odom',
            10
        )
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Parameters
        self.r = 0.033   # wheel radius
        self.L = 0.160   # wheel base
        
        self.P_theta = 0.1        # heading covariance
        self.Q_theta = 0.01       # Wheel noise
        self.R_theta = 0.05       # IMU noise

        self.imu_yaw = None
        self.imu_yaw_rate = 0.0
        
        # IMU calibration variables
        self.first_imu = True
        self.first_imu_reading_count = 0
        self.first_imu_yaw = 0.0
        self.imu_yaw_offset = 0.0
        
        # Var
        self.prev_rad_l = None
        self.prev_rad_r = None
        
        self.x_w = 0.0
        self.y_w = 0.0
        self.theta_w = 0.0
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_time = None
        
        # Sub
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            50
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        
    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q
    
    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def joint_callback(self, msg: JointState):
        # left_vel = msg.velocity[0]
        # right_vel = msg.velocity[1]

        # self.get_logger().info(
        #     f'Wheel vel -> L: {left_vel:.3f}, R: {right_vel:.3f}'
        # )
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = current_time - self.prev_time
        
        if dt <= 0.0:
            return
        
        self.prev_time = current_time
        
        # Wheel angular velocity
        # omega_l = msg.velocity[0]
        # omega_r = msg.velocity[1]
        
        # Wheel angular position
        rad_l = msg.position[0]
        rad_r = msg.position[1]

        # Differential drive model
        # v_l = self.r * omega_l
        # v_r = self.r * omega_r
        # v = (v_l + v_r) / 2.0
        # omega = (v_r - v_l) / self.L
        

        # Wheel odometry
        # self.theta_w += omega * dt
        # self.theta_w = self.normalize_angle(self.theta_w)
        # self.x_w += v * math.cos(self.theta_w) * dt
        # self.y_w += v * math.sin(self.theta_w) * dt
        if self.prev_rad_l is None or self.prev_rad_r is None:
            self.prev_rad_l = rad_l
            self.prev_rad_r = rad_r
            return
        d_rad_l = rad_l - self.prev_rad_l
        d_rad_r = rad_r - self.prev_rad_r
        ds_l = self.r * d_rad_l
        ds_r = self.r * d_rad_r
        ds = (ds_r + ds_l) / 2.0
        d_theta = (ds_r - ds_l) / self.L
        self.theta_w += d_theta
        self.theta_w = self.normalize_angle(self.theta_w)
        self.x_w += ds * math.cos(self.theta_w)
        self.y_w += ds * math.sin(self.theta_w)
        self.prev_rad_l = rad_l
        self.prev_rad_r = rad_r
        
        # EKF prediction
        # theta_pred = self.theta + omega * dt
        theta_pred = self.theta + d_theta
        theta_pred = self.normalize_angle(theta_pred)
        P_pred = self.P_theta + self.Q_theta

        # EKF update
        if self.imu_yaw is None:
            theta_meas = theta_pred
        else:
            theta_meas = self.imu_yaw
        theta_error = self.normalize_angle(theta_meas - theta_pred)

        K = P_pred / (P_pred + self.R_theta)

        self.theta = theta_pred + K * theta_error
        self.theta = self.normalize_angle(self.theta)
        self.P_theta = (1.0 - K) * P_pred
        
        # self.x += v * math.cos(self.theta) * dt
        # self.y += v * math.sin(self.theta) * dt
        self.x += ds * math.cos(self.theta)
        self.y += ds * math.sin(self.theta)
        
        # if self.imu_yaw is not None:
        #     theta_meas = self.imu_yaw
        #     theta_error = self.normalize_angle(theta_meas - theta_pred)
        #     K = P_pred / (P_pred + self.R_theta)
        #     self.theta = self.normalize_angle(theta_pred + K * theta_error)
        #     self.P_theta = (1.0 - K) * P_pred
        # else:
        #     self.theta = theta_pred
        #     self.P_theta = P_pred
        
        # avg_theta_ekf = self.theta - (d_theta / 2.0)
        # self.x += ds * math.cos(avg_theta_ekf)
        # self.y += ds * math.sin(avg_theta_ekf)

        # Log
        # self.get_logger().info(
        #     f'Wheel Odom -> x={self.x_w:.3f}, y={self.y_w:.3f}, theta={math.degrees(self.theta_w):.2f} deg'
        #     f', EKF Odom -> x={self.x:.3f}, y={self.y:.3f}, theta={math.degrees(self.theta):.2f} deg'
        # )
        
        # Publish odometry
        wheel_odom = Odometry()
        wheel_odom.header.stamp = msg.header.stamp
        wheel_odom.header.frame_id = 'odom'
        wheel_odom.child_frame_id = 'base_link'
        wheel_odom.pose.pose.position.x = self.x_w
        wheel_odom.pose.pose.position.y = self.y_w
        wheel_odom.pose.pose.orientation = self.yaw_to_quaternion(self.theta_w)
        self.wheel_odom_pub.publish(wheel_odom)
        
        ekf_odom = Odometry()
        ekf_odom.header.stamp = wheel_odom.header.stamp
        ekf_odom.header.frame_id = 'odom'
        ekf_odom.child_frame_id = 'base_link'
        ekf_odom.pose.pose.position.x = self.x
        ekf_odom.pose.pose.position.y = self.y
        ekf_odom.pose.pose.orientation = self.yaw_to_quaternion(self.theta)
        self.ekf_odom_pub.publish(ekf_odom)
        
        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = ekf_odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.yaw_to_quaternion(self.theta)
        self.tf_broadcaster.sendTransform(t)

    def imu_callback(self, msg: Imu):
        current_yaw = self.quaternion_to_yaw(msg.orientation)
        
        if self.first_imu:
            self.imu_yaw_offset = math.radians(30.0)
            self.first_imu = False
        
        calibrated_yaw = self.normalize_angle(current_yaw - self.imu_yaw_offset)
        
        self.imu_yaw = calibrated_yaw
        self.imu_yaw_rate = msg.angular_velocity.z
    

def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
