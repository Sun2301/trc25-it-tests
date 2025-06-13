# SensorData Message
# This message type defines the structure for sensor readings in the TekBot competition.
# Each field represents a different sensor measurement with specific valid ranges.

# Temperature reading in Celsius (°C)
# Valid range: 15-35°C (comfortable room temperature)
float32 temperature

# Relative humidity percentage (%)
# Valid range: 30-70% (healthy indoor humidity)
float32 humidity

# Atmospheric pressure in hectopascals (hPa)
# Valid range: 950-1050 hPa (normal atmospheric pressure)
float32 pressure
