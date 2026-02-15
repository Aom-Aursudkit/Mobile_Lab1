import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import numpy as np
import math

class ICPRefinementNode(Node):
    def __init__(self):
        super().__init__('icp_node')
        self.get_logger().info('ICP node started')
        
        self.create_subscription(Odometry, '/ekf_odom', self.ekf_callback, 10)
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(LaserScan, '/scan', self.scan_callback, scan_qos)       

        self.icp_pub = self.create_publisher(Odometry, '/icp_odom', 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # State Variables
        self.current_ekf_pose = None
        self.prev_ekf_pose = None
        self.prev_scan_points = None
        
        # Global ICP Pose
        self.icp_x = 0.0
        self.icp_y = 0.0
        self.icp_theta = 0.0

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def yaw_to_quaternion(self, yaw):
        return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def scan_to_np(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        valid = (ranges > scan_msg.range_min) & (ranges > 0.0) & (np.isfinite(ranges))
        r, a = ranges[valid], angles[valid]
        points = np.stack((r * np.cos(a), r * np.sin(a)), axis=1)
        return points[::5]
            
    def find_neighbors(self, source, target):
        indices = []
        valid_source_indices = []
        
        # Max distance to consider a match valid (30cm)
        max_dist_sq = 0.3**2 
        
        for i, s in enumerate(source):
            dist_sq = np.sum((target - s)**2, axis=1)
            best_idx = np.argmin(dist_sq)
            
            if dist_sq[best_idx] < max_dist_sq:
                indices.append(best_idx)
                valid_source_indices.append(i)
                
        return np.array(indices), np.array(valid_source_indices)

    def apply_transform(self, points, dx, dy, theta):
        R = np.array([[np.cos(theta), -np.sin(theta)], 
                      [np.sin(theta),  np.cos(theta)]])
        return (R @ points.T).T + np.array([dx, dy])
    
    def ekf_callback(self, msg):
        self.current_ekf_pose = msg

    # def scan_callback(self, msg):
    #     if self.current_ekf_pose is None:
    #         return

    #     current_points = self.scan_to_np(msg)

    #     if self.prev_ekf_pose is None:
    #         self.prev_ekf_pose = self.current_ekf_pose
    #         self.prev_scan_points = current_points
    #         return

    #     if len(current_points) < 20:
    #         return
            
    #     try:
    #         # Relative Delta from EKF (Initial Guess)
    #         curr_p = self.current_ekf_pose.pose.pose.position
    #         prev_p = self.prev_ekf_pose.pose.pose.position
            
    #         dx_ekf = curr_p.x - prev_p.x
    #         dy_ekf = curr_p.y - prev_p.y
            
    #         curr_yaw_ekf = self.quaternion_to_yaw(self.current_ekf_pose.pose.pose.orientation)
    #         prev_yaw_ekf = self.quaternion_to_yaw(self.prev_ekf_pose.pose.pose.orientation)
    #         dtheta_ekf = self.normalize_angle(curr_yaw_ekf - prev_yaw_ekf)
            
    #         # Map EKF world jump to local robot frame guess
    #         local_dx_guess = dx_ekf * math.cos(-prev_yaw_ekf) - dy_ekf * math.sin(-prev_yaw_ekf)
    #         local_dy_guess = dx_ekf * math.sin(-prev_yaw_ekf) + dy_ekf * math.cos(-prev_yaw_ekf)

    #         # ICP Refinement Loop 
    #         x_bar = self.apply_transform(current_points, local_dx_guess, local_dy_guess, dtheta_ekf)
    #         target = self.prev_scan_points
    #         e_prev = float('inf')
            
    #         R = np.eye(2)
    #         t = np.array([0.0, 0.0])
            
    #         for i in range(20):
    #             matched_indices, valid_src_indices = self.find_neighbors(x_bar, target)
                
    #             if len(matched_indices) < 15:
    #                 break

    #             y_matched = target[matched_indices]
    #             x_matched = x_bar[valid_src_indices]

    #             x0 = np.mean(x_matched, axis=0)
    #             y0 = np.mean(y_matched, axis=0)

    #             H = (x_matched - x0).T @ (y_matched - y0)
    #             U, S, Vt = np.linalg.svd(H)
    #             R = Vt.T @ U.T
                
    #             if np.linalg.det(R) < 0:
    #                 Vt[1, :] *= -1
    #                 R = Vt.T @ U.T
                
    #             t = y0 - R @ x0

    #             x_bar = (R @ x_bar.T).T + t

    #             e = np.mean(np.linalg.norm(x_matched - y_matched, axis=1))
    #             if abs(e_prev - e) < 1e-6:
    #                 break
    #             e_prev = e

    #         # Final Movement
    #         icp_correction_theta = math.atan2(R[1, 0], R[0, 0])
    #         step_total_dtheta = dtheta_ekf + icp_correction_theta

    #         final_centroid_start = np.mean(current_points, axis=0)
    #         final_centroid_end = np.mean(x_bar, axis=0)
    #         local_dx = final_centroid_end[0] - final_centroid_start[0]
    #         local_dy = final_centroid_end[1] - final_centroid_start[1]

    #         # World ICP Frame
    #         world_dx = local_dx * math.cos(self.icp_theta) - local_dy * math.sin(self.icp_theta)
    #         world_dy = local_dx * math.sin(self.icp_theta) + local_dy * math.cos(self.icp_theta)

    #         self.icp_x += world_dx
    #         self.icp_y += world_dy
    #         self.icp_theta = self.normalize_angle(self.icp_theta + step_total_dtheta)
            
    #         # Log
    #         # self.get_logger().info(
    #         #     f'ICP Refinement -> dx: {self.icp_x:.4f}, dy: {self.icp_y:.4f}, dtheta: {math.degrees(self.icp_theta):.2f} deg'
    #         # )

    #         # Publish Odometry
    #         odom_msg = Odometry()
    #         odom_msg.header.stamp = msg.header.stamp
    #         odom_msg.header.frame_id = 'odom'
    #         odom_msg.child_frame_id = 'base_link'
    #         odom_msg.pose.pose.position.x = self.icp_x
    #         odom_msg.pose.pose.position.y = self.icp_y
    #         odom_msg.pose.pose.orientation = self.yaw_to_quaternion(self.icp_theta)
    #         self.icp_pub.publish(odom_msg)
            
    #         # Broadcast TF
    #         t = TransformStamped()
    #         t.header.stamp = odom_msg.header.stamp
    #         t.header.frame_id = 'odom'
    #         t.child_frame_id = 'base_link'
    #         t.transform.translation.x = self.icp_x
    #         t.transform.translation.y = self.icp_y
    #         t.transform.translation.z = 0.0
    #         t.transform.rotation = self.yaw_to_quaternion(self.icp_theta)
    #         self.tf_broadcaster.sendTransform(t)

    #         # For next iteration
    #         self.prev_scan_points = current_points
    #         self.prev_ekf_pose = self.current_ekf_pose
            
    #     except Exception as e:
    #         self.get_logger().error(f"ICP Error: {e}")
            
    def scan_callback(self, msg):
        if self.current_ekf_pose is None:
            return

        current_points = self.scan_to_np(msg)

        if self.prev_ekf_pose is None:
            self.prev_ekf_pose = self.current_ekf_pose
            self.prev_scan_points = current_points
            return

        if len(current_points) < 20:
            return
            
        try:
            # Relative Delta from EKF (Initial Guess)
            curr_p = self.current_ekf_pose.pose.pose.position
            prev_p = self.prev_ekf_pose.pose.pose.position
            
            dx_ekf = curr_p.x - prev_p.x
            dy_ekf = curr_p.y - prev_p.y
            
            curr_yaw_ekf = self.quaternion_to_yaw(self.current_ekf_pose.pose.pose.orientation)
            prev_yaw_ekf = self.quaternion_to_yaw(self.prev_ekf_pose.pose.pose.orientation)
            dtheta_ekf = self.normalize_angle(curr_yaw_ekf - prev_yaw_ekf)
            
            # Map EKF world delta to local robot frame guess
            local_dx_guess = dx_ekf * math.cos(-prev_yaw_ekf) - dy_ekf * math.sin(-prev_yaw_ekf)
            local_dy_guess = dx_ekf * math.sin(-prev_yaw_ekf) + dy_ekf * math.cos(-prev_yaw_ekf)

            # For non-holonomic robot: forward distance and rotation only
            forward_guess = local_dx_guess  # Only forward motion matters
            dtheta_icp = dtheta_ekf
            
            target = self.prev_scan_points
            e_prev = float('inf')
            
            # ICP with non-holonomic constraints
            for iteration in range(20):
                # Apply current transform estimate (rotation + forward motion only)
                # For differential drive: first rotate by dtheta/2, move forward, rotate by dtheta/2
                # Simplified: rotate first, then translate along x-axis only
                x_transformed = self.apply_transform(current_points, forward_guess, 0.0, dtheta_icp)
                
                matched_indices, valid_src_indices = self.find_neighbors(x_transformed, target)
                
                if len(matched_indices) < 15:
                    break

                y_matched = target[matched_indices]
                x_matched = x_transformed[valid_src_indices]

                # Compute centroids
                x0 = np.mean(x_matched, axis=0)
                y0 = np.mean(y_matched, axis=0)

                # SVD for full transformation
                H = (x_matched - x0).T @ (y_matched - y0)
                U, S, Vt = np.linalg.svd(H)
                R = Vt.T @ U.T
                
                if np.linalg.det(R) < 0:
                    Vt[1, :] *= -1
                    R = Vt.T @ U.T
                
                t = y0 - R @ x0
                
                # Extract rotation
                correction_theta = math.atan2(R[1, 0], R[0, 0])
                
                # Project translation onto forward direction (constrain to non-holonomic)
                # The correction translation in the current rotated frame
                cos_curr = math.cos(dtheta_icp)
                sin_curr = math.sin(dtheta_icp)
                
                # Transform correction to local frame before rotation
                t_local_x = t[0] * cos_curr + t[1] * sin_curr
                t_local_y = -t[0] * sin_curr + t[1] * cos_curr
                
                # Only keep forward component (x-direction in robot frame)
                forward_correction = t_local_x
                
                # Update estimates
                forward_guess += forward_correction
                dtheta_icp = self.normalize_angle(dtheta_icp + correction_theta)

                # Check convergence
                e = np.mean(np.linalg.norm(x_matched - y_matched, axis=1))
                if abs(e_prev - e) < 1e-5:
                    break
                e_prev = e

            # Now transform the non-holonomic motion to world frame
            # In robot local frame: moved forward_guess along x-axis, rotated by dtheta_icp
            # Transform this to world coordinates
            world_dx = forward_guess * math.cos(self.icp_theta)
            world_dy = forward_guess * math.sin(self.icp_theta)

            # Update global ICP pose
            self.icp_x += world_dx
            self.icp_y += world_dy
            self.icp_theta = self.normalize_angle(self.icp_theta + dtheta_icp)

            # Publish Odometry
            odom_msg = Odometry()
            odom_msg.header.stamp = msg.header.stamp
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            odom_msg.pose.pose.position.x = self.icp_x
            odom_msg.pose.pose.position.y = self.icp_y
            odom_msg.pose.pose.orientation = self.yaw_to_quaternion(self.icp_theta)
            self.icp_pub.publish(odom_msg)
            
            # Broadcast TF
            t = TransformStamped()
            t.header.stamp = odom_msg.header.stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.icp_x
            t.transform.translation.y = self.icp_y
            t.transform.translation.z = 0.0
            t.transform.rotation = self.yaw_to_quaternion(self.icp_theta)
            self.tf_broadcaster.sendTransform(t)

            # For next iteration
            self.prev_scan_points = current_points
            self.prev_ekf_pose = self.current_ekf_pose
            
        except Exception as e:
            self.get_logger().error(f"ICP Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ICPRefinementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

#ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_scan --ros-args -p use_sim_time:=True
