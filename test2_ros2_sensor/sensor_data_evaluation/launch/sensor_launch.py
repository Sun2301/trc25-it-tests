"""Launch file for the sensor data evaluation system.

This launch file starts both the publisher and subscriber nodes for the sensor
data evaluation system. It provides a convenient way to start the entire system
with a single command.

The following nodes are launched:
1. Publisher Node (sensor_publisher):
   - Generates simulated sensor data
   - Publishes to /sensor_data topic
   - Runs at 2 Hz

2. Subscriber Node (sensor_subscriber):
   - Evaluates sensor data against thresholds
   - Generates status alerts
   - Processes data in real-time
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Create and return a launch description with both nodes.
    
    Returns:
        LaunchDescription: Contains configurations for both the publisher
                         and subscriber nodes.
    """
    return LaunchDescription([
        # Publisher Node
        Node(
            package='sensor_data_evaluation',
            executable='publisher_node.py',
            name='sensor_publisher',
            output='screen'  # Display output in terminal
        ),
        # Subscriber Node
        Node(
            package='sensor_data_evaluation',
            executable='subscriber_node.py',
            name='sensor_subscriber',
            output='screen'  # Display output in terminal
        )
    ])
