# Test 2 – ROS2 Sensor Evaluation (TekBot Competition)

This package is our submission for Test 2 of the TekBot Robotics Competition. It implements a sensor data evaluation system using ROS2's publish/subscribe paradigm and custom messages to monitor and evaluate environmental sensor data.

## Competition Requirements Fulfilled

1. **Custom Message Implementation**
   - Created `SensorData.msg` for structured sensor data
   - Includes temperature, humidity, and pressure fields

2. **Publisher-Subscriber Architecture**
   - Publisher node simulates sensor data acquisition
   - Subscriber node performs real-time data evaluation

3. **Data Validation**
   - Temperature range: 15-35°C
   - Humidity range: 30-70%
   - Pressure range: 950-1050 hPa
   - Status alerts for out-of-range values

4. **Code Quality**
   - Comprehensive documentation
   - Clean code structure
   - Built-in tests
   - Launch file for easy demonstration

## Overview

The system consists of two main components:
1. A publisher node that simulates sensor readings (temperature, humidity, pressure)
2. A subscriber node that evaluates these readings against predefined thresholds

The system uses a custom message type for structured data transmission and provides immediate status alerts when sensor values fall outside acceptable ranges.

## Features

- Custom ROS2 message type (`SensorData.msg`) for structured sensor data
- Publisher node generating simulated sensor data
- Subscriber node evaluating sensor data against thresholds
- Launch file for easy deployment

## Project Structure
```
sensor_data_evaluation/
├── CMakeLists.txt          # Build system configuration
├── package.xml             # Package metadata and dependencies
├── setup.cfg              # Python package configuration
├── setup.py              # Python package setup
├── launch/
│   └── sensor_launch.py  # System launch configuration
├── msg/
│   └── SensorData.msg    # Custom message definition
├── scripts/
│   ├── publisher_node.py # Data publisher implementation
│   └── subscriber_node.py # Data evaluator implementation
└── test/                 # Quality assurance tests
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

## Dependencies

### Build Dependencies
- `ament_cmake` - ROS2 CMake build system
- `rosidl_default_generators` - Message generation tools

### Runtime Dependencies
- `rclpy` - ROS2 Python client library
- `std_msgs` - Standard ROS2 message types
- `rosidl_default_runtime` - Message runtime support

### Test Dependencies
- `ament_copyright` - Copyright checker
- `ament_flake8` - Python style checker
- `ament_pep257` - Docstring style checker
- `python3-pytest` - Python testing framework

## Technical Details

### Message Structure (`SensorData.msg`)
The custom message type includes three sensor readings:
- **Temperature** (float32)
  - Unit: Celsius (°C)
  - Valid range: 15-35°C
  - Purpose: Monitor ambient temperature
  - Alert: Generated if outside comfortable room temperature range

- **Humidity** (float32)
  - Unit: Percentage (%)
  - Valid range: 30-70%
  - Purpose: Monitor relative humidity
  - Alert: Generated if outside healthy indoor range

- **Pressure** (float32)
  - Unit: Hectopascals (hPa)
  - Valid range: 950-1050 hPa
  - Purpose: Monitor atmospheric pressure
  - Alert: Generated if outside normal atmospheric range

## Building and Running

The package can be built using standard ROS2 workspace tools:

```bash
colcon build --packages-select sensor_data_evaluation
source install/setup.bash
```

## Usage

Run the nodes using the launch file:
```bash
ros2 launch sensor_data_evaluation sensor_launch.py
```

This will start both the publisher and subscriber nodes. The publisher generates random sensor data every 0.5 seconds, and the subscriber evaluates it and outputs the status.

## System Architecture

### Publisher Node (`publisher_node.py`)
- **Purpose**: Simulates a sensor data acquisition system
- **Implementation**:
  - Uses ROS2's timer-based publishing mechanism
  - Generates random data within realistic ranges
  - Publishes at 2 Hz (every 0.5 seconds)
- **Topic**: `/sensor_data`
- **Message Type**: `sensor_data_evaluation/msg/SensorData`
- **Data Ranges**:
  - Temperature: 10-40°C (simulating varying room conditions)
  - Humidity: 20-80% (simulating indoor humidity variations)
  - Pressure: 900-1100 hPa (simulating atmospheric changes)

### Subscriber Node (`subscriber_node.py`)
- **Purpose**: Real-time sensor data evaluation system
- **Implementation**:
  - Uses ROS2's subscription callback mechanism
  - Evaluates data against predefined thresholds
  - Generates immediate status alerts
- **Topic**: `/sensor_data`
- **Evaluation Logic**:
  - "OK" status: All values within acceptable ranges
  - "ALERT" status: Any value outside acceptable range
- **Thresholds**:
  - Temperature: 15-35°C (comfortable range)
  - Humidity: 30-70% (healthy range)
  - Pressure: 950-1050 hPa (normal range)

## Testing

### Built-in Tests
The package includes three types of tests:
1. **Copyright Tests** (`test_copyright.py`)
   - Verifies proper copyright headers in source files
2. **Code Style Tests** (`test_flake8.py`)
   - Ensures PEP 8 compliance
   - Checks Python code style and quality
3. **Documentation Tests** (`test_pep257.py`)
   - Validates docstring format
   - Ensures documentation completeness

To run tests:
```bash
colcon test --packages-select sensor_data_evaluation
```

### Manual Testing
1. **Launch System**:
```bash
ros2 launch sensor_data_evaluation sensor_launch.py
```

2. **Monitor Topics**:
```bash
# In a new terminal
ros2 topic echo /sensor_data
```

3. **Check Node Info**:
```bash
ros2 node info /sensor_publisher
ros2 node info /sensor_subscriber
```

## Performance Considerations

- **Message Queue Size**: Set to 10 to prevent memory overflow
- **Publishing Rate**: 2 Hz provides good balance between responsiveness and system load
- **Resource Usage**: Minimal CPU and memory footprint
- **Scalability**: Can handle multiple instances for distributed sensing

## License

This project is licensed under the MIT License.
