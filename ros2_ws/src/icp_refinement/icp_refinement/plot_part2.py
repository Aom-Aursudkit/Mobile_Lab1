import rosbag2_py
import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag_path = "ros2_ws/rosbag2_2026_02_12-04_05_19" 

storage_options = rosbag2_py.StorageOptions(
    uri=bag_path,
    storage_id='sqlite3'
)
converter_options = rosbag2_py.ConverterOptions('', '')

reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

topics = {
    '/icp_odom': []
}

while reader.has_next():
    topic, data, t = reader.read_next()
    if topic in topics:
        msg_type = get_message('nav_msgs/msg/Odometry')
        msg = deserialize_message(data, msg_type)
        topics[topic].append((msg.pose.pose.position.x,
                               msg.pose.pose.position.y))

# Convert to array
icp = np.array(topics['/icp_odom'])

if len(icp) == 0:
    print("Error: No data found in this bag.")
else:
    plt.figure(figsize=(8, 8))

    # Plot path
    plt.plot(icp[:, 0], icp[:, 1], '-g', linewidth=2, label='Trajectory')
    
    # Markers
    plt.scatter(icp[0, 0], icp[0, 1], c='green', s=100, label='Start', edgecolors='black')
    plt.scatter(icp[-1, 0], icp[-1, 1], c='red', s=100, label='End', edgecolors='black')

    plt.xlabel('X position (m)')
    plt.ylabel('Y position (m)')
    plt.title('Odometry')
    plt.axis('equal')
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()
    
    plt.show()