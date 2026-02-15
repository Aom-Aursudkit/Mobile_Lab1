import rosbag2_py
import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag_path = "ros2_ws/rosbag2_2026_02_13-01_26_45"

storage_options = rosbag2_py.StorageOptions(
    uri=bag_path,
    storage_id='sqlite3'
)
converter_options = rosbag2_py.ConverterOptions('', '')

reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

topics = {
    '/wheel_odom': [],
    '/ekf_odom': []
}

while reader.has_next():
    topic, data, t = reader.read_next()
    if topic in topics:
        msg_type = get_message('nav_msgs/msg/Odometry')
        msg = deserialize_message(data, msg_type)
        topics[topic].append((msg.pose.pose.position.x,
                               msg.pose.pose.position.y))

# Convert to arrays
wheel = np.array(topics['/wheel_odom'])
ekf = np.array(topics['/ekf_odom'])

plt.figure(figsize=(6, 10))

plt.plot(wheel[:, 0], wheel[:, 1], '-b', linewidth=2)
plt.scatter(wheel[0, 0], wheel[0, 1], c='green', s=60, label='Start')
plt.scatter(wheel[-1, 0], wheel[-1, 1], c='red', s=60, label='End')

plt.xlabel('X position (m)')
plt.ylabel('Y position (m)')
plt.title('Wheel Odometry Trajectory')
plt.axis('equal')
plt.grid(True)
plt.legend()

plt.figure(figsize=(6, 10))

plt.plot(ekf[:, 0], ekf[:, 1], '-r', linewidth=2)
plt.scatter(ekf[0, 0], ekf[0, 1], c='green', s=60, label='Start')
plt.scatter(ekf[-1, 0], ekf[-1, 1], c='red', s=60, label='End')

plt.xlabel('X position (m)')
plt.ylabel('Y position (m)')
plt.title('EKF Odometry Trajectory')
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.show()

