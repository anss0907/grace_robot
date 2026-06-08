#!/usr/bin/env python3
"""
nano_supabase_pusher.py
Pushes Arduino Nano sensor data to Supabase at 2 Hz.

Subscribes to:
  /nano/gas     (grace_msgs/GasReadings)
  /nano/power   (grace_msgs/PowerMonitor)

Publishes to Supabase table: arduino_nano_data
  Columns: ts, mq_ratio, mhmq_ratio,
           battery_24v_v, buck_19v_v,
           battery_40v_a, battery_24v_a,
           charger_40v_a, charger_24v_a

NOTE: ultrasonic and relay state are intentionally excluded.
"""

import os
import time
from threading import Lock
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from supabase import create_client, Client

from grace_msgs.msg import GasReadings, PowerMonitor


class NanoSupabasePusher(Node):

    SUPABASE_URL_DEFAULT = "https://djinbjjvxpghlontswfj.supabase.co"
    SUPABASE_KEY_DEFAULT = (
        "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9."
        "eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImRqaW5iamp2eHBnaGxvbnRzd2ZqIiwi"
        "cm9sZSI6ImFub24iLCJpYXQiOjE3ODA3MjI2MzUsImV4cCI6MjA5NjI5ODYzNX0."
        "eFZxVDwcCwC1mIEqIXqR3QjfcUsfkdJcRJwgkfJoKlk"
    )
    TABLE_NAME = "arduino_nano_data"
    PUSH_HZ    = 2.0   # push to Supabase at 2 Hz

    def __init__(self):
        super().__init__("nano_supabase_pusher")

        url = os.environ.get("SUPABASE_URL", self.SUPABASE_URL_DEFAULT)
        key = os.environ.get("SUPABASE_KEY", self.SUPABASE_KEY_DEFAULT)

        try:
            self.supabase: Client = create_client(url, key)
            self.get_logger().info("Connected to Supabase")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Supabase: {e}")
            raise

        self.lock = Lock()
        self._data = {
            "mq_ratio":      None,
            "mhmq_ratio":    None,
            "battery_24v_v": None,
            "buck_19v_v":    None,
            "battery_40v_a": None,
            "battery_24v_a": None,
            "charger_40v_a": None,
            "charger_24v_a": None,
        }

        # Subscribers
        self.create_subscription(GasReadings,   "/nano/gas",   self._gas_cb,   10)
        self.create_subscription(PowerMonitor,  "/nano/power", self._power_cb, 10)

        # Push timer
        self.create_timer(1.0 / self.PUSH_HZ, self._push)

    # ---- callbacks ----

    def _gas_cb(self, msg: GasReadings):
        with self.lock:
            self._data["mq_ratio"]   = round(float(msg.mq_ratio),   4)
            self._data["mhmq_ratio"] = round(float(msg.mhmq_ratio), 4)

    def _power_cb(self, msg: PowerMonitor):
        with self.lock:
            self._data["battery_24v_v"] = round(float(msg.battery_24v_v), 3)
            self._data["buck_19v_v"]    = round(float(msg.buck_19v_v), 3)
            self._data["battery_40v_a"] = round(float(msg.battery_40v_a), 3)
            self._data["battery_24v_a"] = round(float(msg.battery_24v_a), 3)
            self._data["charger_40v_a"] = round(float(msg.charger_40v_a), 3)
            self._data["charger_24v_a"] = round(float(msg.charger_24v_a), 3)

    # ---- push ----

    def _push(self):
        with self.lock:
            # Skip if we have no data yet
            if all(v is None for v in self._data.values()):
                return
            payload = dict(self._data)

        payload["ts"] = datetime.now(timezone.utc).isoformat()

        try:
            self.supabase.table(self.TABLE_NAME).insert(payload).execute()
        except Exception as e:
            self.get_logger().error(f"Supabase push failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = NanoSupabasePusher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
