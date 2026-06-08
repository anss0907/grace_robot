# Directional ultrasonic distance scan for a differential-drive robot
# One sensor per cardinal direction of the robot chassis
# Distance = -1.0 cm means no echo was received (out of range or blocked)

std_msgs/Header header

float32 front_cm    # Front ultrasonic sensor distance in cm
float32 rear_cm     # Rear ultrasonic sensor distance in cm
float32 left_cm     # Left ultrasonic sensor distance in cm
float32 right_cm    # Right ultrasonic sensor distance in cm
