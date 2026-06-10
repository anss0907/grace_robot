#!/usr/bin/env python3
import time
import os
import rclpy
from rclpy.node import Node
from supabase import create_client, Client
from threading import Lock
from datetime import datetime, timezone

from grace_msgs.msg import EnvironmentData

class STM32SupabasePusher(Node):
    def __init__(self):
        super().__init__('stm32_supabase_pusher')
        
        self.url = os.environ.get("SUPABASE_URL", "https://djinbjjvxpghlontswfj.supabase.co")
        self.key = os.environ.get("SUPABASE_KEY", "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImRqaW5iamp2eHBnaGxvbnRzd2ZqIiwicm9sZSI6ImFub24iLCJpYXQiOjE3ODA3MjI2MzUsImV4cCI6MjA5NjI5ODYzNX0.eFZxVDwcCwC1mIEqIXqR3QjfcUsfkdJcRJwgkfJoKlk")
        
        try:
            self.supabase: Client = create_client(self.url, self.key)
            self.get_logger().info("Connected to Supabase")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Supabase: {e}")
            raise e

        self.lock = Lock()
        self.latest_data = {
            "temp_c": None,
            "humidity_pct": None,
            "pressure_hpa": None
        }

        self.sub_env = self.create_subscription(EnvironmentData, '/sensors/environment', self.env_cb, 10)
        
        self.timer = self.create_timer(0.5, self.push_data)

    def env_cb(self, msg: EnvironmentData):
        with self.lock:
            self.latest_data["temp_c"] = msg.temperature
            self.latest_data["humidity_pct"] = msg.humidity
            self.latest_data["pressure_hpa"] = msg.pressure

    def push_data(self):
        with self.lock:
            if self.latest_data["temp_c"] is None:
                return
            data = self.latest_data.copy()
        
        data['ts'] = datetime.now(timezone.utc).isoformat()
        
        try:
            self.supabase.table("stm32_data").insert(data).execute()
        except Exception as e:
            self.get_logger().error(f"Failed to push to Supabase: {e}")

def main(args=None):
    rclpy.init(args=args)
    pusher = STM32SupabasePusher()
    try:
        rclpy.spin(pusher)
    except KeyboardInterrupt:
        pass
    finally:
        pusher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
