# Air quality readings from MQ and MH-MQ gas sensor pair
# Values are normalised ratios: 0.0 = clean air (minimum ADC), 1.0 = maximum ADC saturation
# Convert to ppm in the application layer using sensor calibration curves from datasheets

std_msgs/Header header

float32 mq_ratio      # MQ sensor normalised ratio 0.0-1.0 (raw_adc / 1023.0)
float32 mhmq_ratio    # MH-MQ sensor normalised ratio 0.0-1.0 (raw_adc / 1023.0)
