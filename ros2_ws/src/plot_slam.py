import os
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage

# ==========================================
# CONFIGURATION - PASTE YOUR PATH HERE
# ==========================================
BAG_PATH = "/home/aom/Documents/GitHub/Mobile_Lab1/ros2_ws/slam_output_seq00"
# ==========================================

def read_bag_data(bag_path):
    slam_poses = {'x': [], 'y': []}
    tf_path = {'x': [], 'y': []}
    
    # Initialize the reader for sqlite3 (.db3)
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error: Could not open bag at {bag_path}")
        print(f"Details: {e}")
        return None, None

    print(f"Reading bag data from: {bag_path}")
    count = 0

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        # 1. Extract SLAM Poses (The 65 correction points)
        if topic == '/pose':
            msg = deserialize_message(data, PoseWithCovarianceStamped)
            slam_poses['x'].append(msg.pose.pose.position.x)
            slam_poses['y'].append(msg.pose.pose.position.y)

        # 2. Extract High-Freq TF Trajectory (The 24k movement points)
        if topic == '/tf':
            msg = deserialize_message(data, TFMessage)
            for transform in msg.transforms:
                # We filter for the robot's pose relative to odom
                if transform.child_frame_id == 'base_link':
                    count += 1
                    # Downsample: Only plot every 5th TF message for speed
                    if count % 5 == 0:
                        tf_path['x'].append(transform.transform.translation.x)
                        tf_path['y'].append(transform.transform.translation.y)

    return slam_poses, tf_path

def main():
    if not os.path.exists(BAG_PATH):
        print(f"CRITICAL ERROR: The path '{BAG_PATH}' does not exist.")
        return

    slam_data, tf_data = read_bag_data(BAG_PATH)

    if slam_data is None:
        return

    print(f"Plotting {len(tf_data['x'])} TF points and {len(slam_data['x'])} SLAM points...")

    plt.figure(figsize=(12, 8))
    
    # 1. Plot the continuous path from TF
    if tf_data['x']:
        plt.plot(tf_data['x'], tf_data['y'], color='blue', label='ICP/Odom Path (TF)', alpha=0.5, linewidth=1)
    
    # 2. Plot the discrete SLAM points
    if slam_data['x']:
        plt.scatter(slam_data['x'], slam_data['y'], color='red', label='SLAM Poses (/pose)', s=15, zorder=5)
        # Optional: line connecting SLAM dots
        plt.plot(slam_data['x'], slam_data['y'], 'r--', alpha=0.3, linewidth=0.5)

    plt.title(f"Trajectory Analysis: {os.path.basename(BAG_PATH)}")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.legend()
    plt.grid(True, which='both', linestyle='--', alpha=0.5)
    plt.axis('equal')

    print("Opening plot window...")
    plt.show()

if __name__ == "__main__":
    main()