# PMS5003 Particulate Matter Sensor Data
# Consolidated message for PM1.0, PM2.5, and PM10 concentrations

std_msgs/Header header

uint16 pm1_0              # PM1.0 concentration in µg/m³
uint16 pm2_5              # PM2.5 concentration in µg/m³
uint16 pm10_0             # PM10 concentration in µg/m³
bool   sensor_available   # True if PMS5003 sensor is connected and responding
