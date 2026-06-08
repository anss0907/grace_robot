# BME680 Environmental Sensor Data
# Consolidated message for temperature, humidity, pressure, and gas resistance

std_msgs/Header header

float32 temperature        # Temperature in °C
float32 humidity           # Relative humidity in %
float32 pressure           # Atmospheric pressure in hPa
float32 gas_resistance     # Gas resistance in kΩ (air quality indicator)
bool    sensor_available   # True if BME680 sensor is connected and responding
