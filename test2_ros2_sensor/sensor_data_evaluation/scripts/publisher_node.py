#!/usr/bin/env python3

"""ROS2 node that simulates sensor data publication.

This node publishes simulated sensor data (temperature, humidity, pressure) at a rate of 2 Hz.
The data is randomly generated within realistic ranges and published using a custom SensorData
message type. This simulates a real sensor system that would be used in the TekBot competition.

Message Type:
    sensor_data_evaluation/msg/SensorData:
        - temperature (float32): Temperature in Celsius
        - humidity (float32): Relative humidity percentage
        - pressure (float32): Atmospheric pressure in hPa

Publishing Rate: 2 Hz (every 0.5 seconds)
Topic: /sensor_data
"""

import rclpy
from rclpy.node import Node
from sensor_data_evaluation.msg import SensorData
import random

class SensorPublisher(Node):
    """Node for publishing simulated sensor data.
    
    This class creates a publisher that sends simulated sensor readings at regular
    intervals. The data is randomly generated within realistic ranges:
    - Temperature: 10-40°C
    - Humidity: 20-80%
    - Pressure: 900-1100 hPa
    """

    def __init__(self):
        """Initialize the publisher node.
        
        Sets up the publisher and creates a timer that triggers data publication
        every 0.5 seconds (2 Hz).
        """
        super().__init__('sensor_publisher')
        self.publisher_ = self.create_publisher(SensorData, 'sensor_data', 10)
        self.timer = self.create_timer(0.5, self.publish_data)
        self.get_logger().info('Publisher Node Started')

    def publish_data(self):
        """Generate and publish random sensor data.
        
        Creates a new SensorData message with random values within realistic ranges
        and publishes it to the 'sensor_data' topic.
        """
        msg = SensorData()
        # Generate random sensor values within realistic ranges
        msg.temperature = random.uniform(10.0, 40.0)  # Celsius
        msg.humidity = random.uniform(20.0, 80.0)     # Percent
        msg.pressure = random.uniform(900.0, 1100.0)  # hPa
        
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published: Temperature={msg.temperature:.2f}, '
            f'Humidity={msg.humidity:.2f}, Pressure={msg.pressure:.2f}')

def main(args=None):
    """Main function to initialize and run the publisher node."""
    rclpy.init(args=args)
    node = SensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
