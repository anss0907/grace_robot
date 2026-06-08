#!/usr/bin/env python3
import time
import os
import rclpy
from rclpy.node import Node
from supabase import create_client, Client
from threading import Lock
from datetime import datetime, timezone

# We will need the standard message types (assuming they match the ones from previous chats)
from sensor_msgs.msg import BatteryState, FluidPressure, RelativeHumidity
from grace_msgs.msg import EnvironmentData, AirQuality

class SupabasePusher(Node):
    def __init__(self):
        super().__init__('supabase_pusher')
        
        # Load Supabase URL and Key from env or hardcode fallback
        self.url = os.environ.get("SUPABASE_URL", "https://djinbjjvxpghlontswfj.supabase.co")
        self.key = os.environ.get("SUPABASE_KEY", "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImRqaW5iamp2eHBnaGxvbnRzd2ZqIiwicm9sZSI6ImFub24iLCJpYXQiOjE3ODA3MjI2MzUsImV4cCI6MjA5NjI5ODYzNX0.eFZxVDwcCwC1mIEqIXqR3QjfcUsfkdJcRJwgkfJoKlk")
        
        try:
            self.supabase: Client = create_client(self.url, self.key)
            self.get_logger().info("Connected to Supabase")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Supabase: {e}")
            raise e

        # Data store
        self.lock = Lock()
        self.latest_data = {
            "battery_v": None,
            "board_temp_c": None,
            "bme_temp_c": None,
            "bme_humidity_pct": None,
            "bme_pressure_hpa": None,
            "bme_gas_kohm": None,
            "pms_ok": False,
            "pm1_0": None,
            "pm2_5": None,
            "pm10": None,
        }

        # Subscribers
        self.sub_env = self.create_subscription(EnvironmentData, '/environment', self.env_cb, 10)
        self.sub_aq = self.create_subscription(AirQuality, '/air_quality', self.aq_cb, 10)
        self.sub_batt = self.create_subscription(BatteryState, '/battery_state', self.batt_cb, 10)
        
        # Timer for 2Hz (0.5s) pushing
        self.timer = self.create_timer(0.5, self.push_data)

    def env_cb(self, msg: EnvironmentData):
        with self.lock:
            self.latest_data["bme_temp_c"] = msg.temperature
            self.latest_data["bme_humidity_pct"] = msg.humidity
            self.latest_data["bme_pressure_hpa"] = msg.pressure
            self.latest_data["bme_gas_kohm"] = msg.gas_resistance

    def aq_cb(self, msg: AirQuality):
        with self.lock:
            self.latest_data["pms_ok"] = True
            self.latest_data["pm1_0"] = msg.pm1_0
            self.latest_data["pm2_5"] = msg.pm2_5
            self.latest_data["pm10"] = msg.pm10_0

    def batt_cb(self, msg: BatteryState):
        with self.lock:
            self.latest_data["battery_v"] = msg.voltage
            self.latest_data["board_temp_c"] = msg.temperature


    def push_data(self):
        with self.lock:
            data = self.latest_data.copy()
        
        # We must supply a timestamp
        data['ts'] = datetime.now(timezone.utc).isoformat()
        
        try:
            self.supabase.table("esp32_data").insert(data).execute()
        except Exception as e:
            self.get_logger().error(f"Failed to push to Supabase: {e}")

def main(args=None):
    rclpy.init(args=args)
    pusher = SupabasePusher()
    try:
        rclpy.spin(pusher)
    except KeyboardInterrupt:
        pass
    finally:
        pusher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
