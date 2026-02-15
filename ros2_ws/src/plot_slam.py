import os
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

# ==========================================
# PATHS 
# ==========================================
SLAM_BAG = "ros2_ws/slam_output_seq01"

RAW_BAG = "ros2_ws/seq01"
# ==========================================

def read_bag(bag_path, topics):
    """Generic reader to extract x,y from specified topics."""
    results = {topic: {'x': [], 'y': []} for topic in topics}
    
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr'
    )
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening {bag_path}: {e}")
        return results

    print(f"Reading topics {topics} from {os.path.basename(bag_path)}...")
    
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic not in topics:
            continue
            
        # Handle different message types
        if topic in ['/wheel_odom', '/ekf_odom', '/icp_odom']:
            msg = deserialize_message(data, Odometry)
            results[topic]['x'].append(msg.pose.pose.position.x)
            results[topic]['y'].append(msg.pose.pose.position.y)
        
        elif topic == '/pose':
            msg = deserialize_message(data, PoseWithCovarianceStamped)
            results[topic]['x'].append(msg.pose.pose.position.x)
            results[topic]['y'].append(msg.pose.pose.position.y)

    return results

def main():
    raw_data = read_bag(RAW_BAG, ['/wheel_odom', '/ekf_odom', '/icp_odom'])
    slam_data = read_bag(SLAM_BAG, ['/pose'])

    plt.figure(figsize=(14, 10))

    # Plot Wheel Odom
    if raw_data['/wheel_odom']['x']:
        plt.plot(raw_data['/wheel_odom']['x'], raw_data['/wheel_odom']['y'], 
                 label='Wheel Odometry', color='gray', linestyle='--', alpha=0.5)

    # Plot EKF
    if raw_data['/ekf_odom']['x']:
        plt.plot(raw_data['/ekf_odom']['x'], raw_data['/ekf_odom']['y'], 
                 label='EKF', color='orange', alpha=0.8)

    # Plot ICP
    if raw_data['/icp_odom']['x']:
        plt.plot(raw_data['/icp_odom']['x'], raw_data['/icp_odom']['y'], 
                 label='ICP', color='blue', linewidth=2)

    # Plot SLAM Poses
    if slam_data['/pose']['x']:
        plt.plot(slam_data['/pose']['x'], slam_data['/pose']['y'], 
                 'go-', label='SLAM_toolbox', markersize=4, linewidth=1.5)

    plt.title("Comparison of Localization Methods")
    plt.xlabel("X Position (meters)")
    plt.ylabel("Y Position (meters)")
    plt.legend(loc='best')
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.axis('equal')

    print("Generating plot...")
    plt.savefig("comparison_plot.png", dpi=300)
    plt.show()

if __name__ == "__main__":
    main()