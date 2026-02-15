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
        self.global_map = []
        self.debug_map = []
        
        # Current ICP transform (world frame)
        self.curr_tf = np.eye(3)  # 3x3 homogeneous transform
        
        # ICP pose
        self.icp_x = 0.0
        self.icp_y = 0.0
        self.icp_theta = 0.0
        
        # Flags
        self.first_run = True
        self.recovery_cooldown = 0
        
        # Parameters
        self.max_iterations = 20
        self.convergence_threshold = 1e-5
        self.max_correspondence_distance = 0.5
        self.max_map_size = 80000
        self.voxel_size = 0.05
        
        self.last_map_update_pose = np.eye(3)

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def yaw_to_quaternion(self, yaw):
        return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def odom_to_transform(self, x, y, theta):
        T = np.eye(3)
        T[0, 0] = np.cos(theta)
        T[0, 1] = -np.sin(theta)
        T[1, 0] = np.sin(theta)
        T[1, 1] = np.cos(theta)
        T[0, 2] = x
        T[1, 2] = y
        return T
    
    def transform_to_pose(self, T):
        x = T[0, 2]
        y = T[1, 2]
        theta = np.arctan2(T[1, 0], T[0, 0])
        return x, y, theta

    def scan_to_np(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        
        # Filter invalid ranges
        valid = (ranges > scan_msg.range_min) & (ranges < scan_msg.range_max) & (np.isfinite(ranges))
        r, a = ranges[valid], angles[valid]
        
        # Convert polar to Cartesian
        points = np.stack((r * np.cos(a), r * np.sin(a)), axis=1)
        return points

    def transform_points(self, points, T):
        if len(points) == 0:
            return points
        ones = np.ones((points.shape[0], 1))
        points_h = np.hstack([points, ones])
        transformed = (T @ points_h.T).T
        return transformed[:, :2]

    def update_map(self, local_scan):
        if len(local_scan) == 0:
            return
        
        # Transform points to global frame
        global_points = self.transform_points(local_scan, self.curr_tf)
        
        if len(self.global_map) == 0:
            self.global_map = global_points.tolist()
            self.debug_map = global_points.tolist()
        else:
            self.global_map.extend(global_points.tolist())
            self.debug_map.extend(global_points.tolist())
        
        self.last_map_update_pose = self.curr_tf.copy()
        
        # Voxel filter
        if len(self.global_map) > self.max_map_size:
            self.get_logger().info(f'Downsampling map from {len(self.global_map)} points')
            
            voxel_dict = {}
            for pt in self.global_map:
                vx = int(np.floor(pt[0] / self.voxel_size))
                vy = int(np.floor(pt[1] / self.voxel_size))
                key = (vx, vy)
                voxel_dict[key] = pt
            
            self.global_map = list(voxel_dict.values())
            self.get_logger().info(f'Map downsampled to {len(self.global_map)} points')

    class SpatialHash:
        def __init__(self, cell_size):
            self.grid = {}
            self.cell_size = cell_size
        
        def hash(self, x, y):
            ix = int(np.floor(x / self.cell_size))
            iy = int(np.floor(y / self.cell_size))
            return (ix, iy)
        
        def insert(self, pt):
            key = self.hash(pt[0], pt[1])
            if key not in self.grid:
                self.grid[key] = []
            self.grid[key].append(pt)
        
        def build(self, points):
            self.grid.clear()
            for pt in points:
                self.insert(pt)
        
        def find_nearest(self, query, max_dist):
            """Find nearest neighbor within max_dist"""
            cx, cy = self.hash(query[0], query[1])
            
            min_dist_sq = max_dist * max_dist
            nearest = None
            found = False
            
            # Search in 3x3 grid around query point
            for dx in range(-1, 2):
                for dy in range(-1, 2):
                    key = (cx + dx, cy + dy)
                    if key in self.grid:
                        for pt in self.grid[key]:
                            dist_sq = (query[0] - pt[0])**2 + (query[1] - pt[1])**2
                            if dist_sq < min_dist_sq:
                                min_dist_sq = dist_sq
                                nearest = pt
                                found = True
            
            return nearest, found

    def compute_match_quality(self, correspondences):
        """Compute quality metrics for ICP match"""
        if len(correspondences) == 0:
            return {
                'mean_error': 1000.0,
                'std_error': 1000.0,
                'inlier_ratio': 0.0,
                'num_matches': 0,
                'is_good': False,
                'is_reasonable': False
            }
        
        # Compute errors
        errors = []
        for src_pt, tgt_pt in correspondences:
            error = np.linalg.norm(np.array(src_pt) - np.array(tgt_pt))
            errors.append(error)
        
        mean_error = np.mean(errors)
        std_error = np.std(errors)
        
        inlier_threshold = mean_error + 2.0 * std_error
        inliers = sum(1 for e in errors if e < inlier_threshold)
        inlier_ratio = inliers / len(correspondences)
        
        is_good = inlier_ratio > 0.3 and len(correspondences) > 15 and mean_error < 0.5
        is_reasonable = inlier_ratio > 0.2 and len(correspondences) > 10
        
        return {
            'mean_error': mean_error,
            'std_error': std_error,
            'inlier_ratio': inlier_ratio,
            'num_matches': len(correspondences),
            'is_good': is_good,
            'is_reasonable': is_reasonable
        }
    
    def ekf_callback(self, msg):
        self.current_ekf_pose = msg

    def scan_callback(self, msg):
        if self.current_ekf_pose is None:
            return

        raw_local_scan = self.scan_to_np(msg)
        
        if len(raw_local_scan) < 20:
            return

        curr_x = self.current_ekf_pose.pose.pose.position.x
        curr_y = self.current_ekf_pose.pose.pose.position.y
        curr_theta = self.quaternion_to_yaw(self.current_ekf_pose.pose.pose.orientation)
        input_odom = [curr_x, curr_y, curr_theta]
        
        if self.first_run:
            self.curr_tf = self.odom_to_transform(curr_x, curr_y, curr_theta)
            self.update_map(raw_local_scan)
            self.prev_ekf_pose = self.current_ekf_pose
            self.icp_x, self.icp_y, self.icp_theta = curr_x, curr_y, curr_theta
            self.first_run = False
            self.get_logger().info(f'ICP: First run initialized at ({curr_x:.2f}, {curr_y:.2f})')
            self.publish_icp_result(msg.header.stamp)
            return
        
        try:
            prev_x = self.prev_ekf_pose.pose.pose.position.x
            prev_y = self.prev_ekf_pose.pose.pose.position.y
            prev_theta = self.quaternion_to_yaw(self.prev_ekf_pose.pose.pose.orientation)
            prev_odom = [prev_x, prev_y, prev_theta]
            
            prev_tf = self.odom_to_transform(prev_x, prev_y, prev_theta)
            curr_odom_tf = self.odom_to_transform(curr_x, curr_y, curr_theta)
            delta_odom = np.linalg.inv(prev_tf) @ curr_odom_tf
            
            delta_trans = delta_odom[:2, 2]
            delta_dist = np.linalg.norm(delta_trans)
            delta_yaw = np.arctan2(delta_odom[1, 0], delta_odom[0, 0])
            
            icp_initial_guess = self.curr_tf @ delta_odom
            
            if len(self.global_map) < 20:
                self.curr_tf = icp_initial_guess
                self.update_map(raw_local_scan)
                self.prev_ekf_pose = self.current_ekf_pose
                self.icp_x, self.icp_y, self.icp_theta = self.transform_to_pose(self.curr_tf)
                self.publish_icp_result(msg.header.stamp)
                return
            
            skip = max(1, len(raw_local_scan) // 250)
            source = raw_local_scan[::skip]
            
            cell_size = 0.5
            spatial_hash = self.SpatialHash(cell_size)
            spatial_hash.build(self.global_map)
            
            # Start with odometry estimate
            working_tf = icp_initial_guess.copy()
            
            # ICP main loop
            for iter in range(self.max_iterations):
                R = working_tf[:2, :2]
                t = working_tf[:2, 2]
                transformed_source = (R @ source.T).T + t
                
                correspondences = []
                for src_pt in transformed_source:
                    nearest_pt, found = spatial_hash.find_nearest(src_pt, self.max_correspondence_distance)
                    if found:
                        correspondences.append((src_pt.tolist(), nearest_pt))
                
                if len(correspondences) < 15:
                    break
                
                if iter > 0 and len(correspondences) > 30:
                    errors = []
                    for src_pt, tgt_pt in correspondences:
                        diff = np.array(src_pt) - np.array(tgt_pt)
                        errors.append(np.dot(diff, diff))
                    
                    errors.sort()
                    median_error = errors[len(errors) // 2]
                    threshold = 2.5 * np.sqrt(median_error)
                    
                    inliers = []
                    for src_pt, tgt_pt in correspondences:
                        if np.linalg.norm(np.array(src_pt) - np.array(tgt_pt)) < threshold:
                            inliers.append((src_pt, tgt_pt))
                    correspondences = inliers
                
                if len(correspondences) < 15:
                    break
                
                src_pts = np.array([c[0] for c in correspondences])
                tgt_pts = np.array([c[1] for c in correspondences])
                
                centroid_src = np.mean(src_pts, axis=0)
                centroid_tgt = np.mean(tgt_pts, axis=0)
                
                src_centered = src_pts - centroid_src
                tgt_centered = tgt_pts - centroid_tgt
                
                H = src_centered.T @ tgt_centered
                
                # SVD for rotation
                U, S, Vt = np.linalg.svd(H)
                R_corr = Vt.T @ U.T
                
                if np.linalg.det(R_corr) < 0:
                    Vt[-1, :] *= -1
                    R_corr = Vt.T @ U.T
                
                # Translation
                t_corr = centroid_tgt - R_corr @ centroid_src
                
                correction = np.eye(3)
                correction[:2, :2] = R_corr
                correction[:2, 2] = t_corr
                
                new_tf = correction @ working_tf
                
                delta_translation_sq = np.sum((new_tf[:2, 2] - working_tf[:2, 2])**2)
                
                R_delta = new_tf[:2, :2] @ working_tf[:2, :2].T
                delta_rotation = np.arctan2(R_delta[1, 0], R_delta[0, 0])
                
                working_tf = new_tf
                
                if delta_translation_sq < self.convergence_threshold**2 and \
                   abs(delta_rotation) < self.convergence_threshold:
                    break
            
            quality = self.compute_match_quality(correspondences)
            
            icp_delta = working_tf[:2, 2] - self.curr_tf[:2, 2]
            icp_delta_dist = np.linalg.norm(icp_delta)
            
            curr_yaw = np.arctan2(self.curr_tf[1, 0], self.curr_tf[0, 0])
            working_yaw = np.arctan2(working_tf[1, 0], working_tf[0, 0])
            icp_delta_yaw = self.normalize_angle(working_yaw - curr_yaw)
            
            max_allowed_dist_diff = max(0.8, delta_dist * 5.0)
            max_allowed_rot_diff = max(0.5, abs(delta_yaw) * 5.0)
            
            icp_reasonable = (abs(icp_delta_dist - delta_dist) < max_allowed_dist_diff) and \
                           (abs(icp_delta_yaw - delta_yaw) < max_allowed_rot_diff)
            
            use_icp = (quality['is_good'] and icp_reasonable) or \
                     (quality['is_reasonable'] and icp_reasonable and delta_dist < 0.2)
            
            if use_icp:
                # Trust ICP
                self.curr_tf = working_tf
                if self.recovery_cooldown > 0:
                    self.recovery_cooldown -= 1
            else:
                # Fall back to odometry
                self.curr_tf = icp_initial_guess
                self.recovery_cooldown = min(5, self.recovery_cooldown + 1)
                
                if self.recovery_cooldown >= 3:
                    self.get_logger().info(
                        f"ICP: Using odometry (quality={quality['inlier_ratio']:.2f}, "
                        f"matches={quality['num_matches']}, error={quality['mean_error']:.3f})",
                        throttle_duration_sec=2.0
                    )
            
            distance_from_last_update = np.linalg.norm(
                self.curr_tf[:2, 2] - self.last_map_update_pose[:2, 2]
            )
            
            curr_map_yaw = np.arctan2(self.curr_tf[1, 0], self.curr_tf[0, 0])
            last_map_yaw = np.arctan2(self.last_map_update_pose[1, 0], self.last_map_update_pose[0, 0])
            rotation_from_last_update = abs(self.normalize_angle(curr_map_yaw - last_map_yaw))
            
            update_dist_threshold = 0.5 if self.recovery_cooldown > 0 else 0.25
            update_rot_threshold = 0.4 if self.recovery_cooldown > 0 else 0.2
            
            if (distance_from_last_update > update_dist_threshold or \
                rotation_from_last_update > update_rot_threshold) and \
               quality['is_reasonable']:
                self.update_map(raw_local_scan)
            
            # Extract pose
            self.icp_x, self.icp_y, self.icp_theta = self.transform_to_pose(self.curr_tf)
            
            # Publish
            self.publish_icp_result(msg.header.stamp)
            
            # Update previous
            self.prev_ekf_pose = self.current_ekf_pose
            
        except Exception as e:
            self.get_logger().error(f"ICP Error: {e}")

    def publish_icp_result(self, timestamp):
        """Publish odometry and TF"""
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.icp_x
        odom_msg.pose.pose.position.y = self.icp_y
        odom_msg.pose.pose.orientation = self.yaw_to_quaternion(self.icp_theta)
        self.icp_pub.publish(odom_msg)
        
        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.icp_x
        t.transform.translation.y = self.icp_y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.yaw_to_quaternion(self.icp_theta)
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ICPRefinementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()