# Dual-battery power monitor for Grace robot
# VS1 (A6) = 24V battery, VS2 (A7) = 19V buck converter output (fed by 40V battery)
# CS1-CS4 = ACS712 ELC-20A hall-effect current sensors (100 mV/A sensitivity)
# Wiring: CS1=40V battery, CS2=24V battery, CS3=40V charger, CS4=24V charger

std_msgs/Header header

# Voltage channels (in Volts, scaled from ADC by firmware divider constants)
float32 battery_24v_v      # 24V battery bus voltage in Volts
float32 buck_19v_v         # 19V buck converter output voltage in Volts

# Current channels in Amperes (ACS712 ELC-20A, 100mV/A)
float32 battery_40v_a      # 40V battery discharge current in Amperes
float32 battery_24v_a      # 24V battery discharge current in Amperes
float32 charger_40v_a      # 40V charger adapter current in Amperes
float32 charger_24v_a      # 24V charger adapter current in Amperes
