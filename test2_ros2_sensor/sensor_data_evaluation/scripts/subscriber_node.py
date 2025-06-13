#!/usr/bin/env python3

"""ROS2 node that evaluates sensor data and generates status alerts.

This node subscribes to sensor data and evaluates it against predefined thresholds.
It processes incoming sensor readings (temperature, humidity, pressure) and determines
if they are within acceptable ranges. If any value is out of range, it generates
an alert.

Thresholds:
    Temperature: 15-35°C
    Humidity: 30-70%
    Pressure: 950-1050 hPa

Subscription Topic: /sensor_data
Message Type: sensor_data_evaluation/msg/SensorData
Output: Status messages (OK/ALERT) with sensor readings
"""

import rclpy
from rclpy.node import Node
from sensor_data_evaluation.msg import SensorData

class SensorSubscriber(Node):
    """Node for evaluating sensor data against thresholds.
    
    This class creates a subscriber that listens for sensor data messages and
    evaluates them against predefined thresholds. It outputs the status of
    the readings and generates alerts when values are out of range.
    """

    def __init__(self):
        """Initialize the subscriber node.
        
        Sets up the subscription to the 'sensor_data' topic with a queue size
        of 10 messages.
        """
        super().__init__('sensor_subscriber')
        self.subscription = self.create_subscription(
            SensorData,
            'sensor_data',
            self.listener_callback,
            10
        )
        self.get_logger().info('Subscriber Node Started')

    def listener_callback(self, msg):
        """Process incoming sensor data messages.
        
        Evaluates sensor readings against thresholds and determines the status.
        
        Args:
            msg (SensorData): The received sensor data message containing
                            temperature, humidity, and pressure readings.
                            
        Thresholds:
            Temperature: 15-35°C (comfortable room temperature range)
            Humidity: 30-70% (healthy indoor humidity range)
            Pressure: 950-1050 hPa (normal atmospheric pressure range)
        """
        # Check if each value is within its acceptable range
        temp_ok = 15 <= msg.temperature <= 35
        humid_ok = 30 <= msg.humidity <= 70
        pressure_ok = 950 <= msg.pressure <= 1050

        # Determine overall status
        status = "OK" if temp_ok and humid_ok and pressure_ok else "ALERT"
        
        # Log the results
        self.get_logger().info(
            f"Received: Temperature={msg.temperature:.2f}, "
            f"Humidity={msg.humidity:.2f}, Pressure={msg.pressure:.2f} → Status: {status}")

def main(args=None):
    """Main function to initialize and run the subscriber node."""
    rclpy.init(args=args)
    node = SensorSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
