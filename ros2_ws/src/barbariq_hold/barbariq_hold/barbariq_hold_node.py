#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pymavlink import mavutil


# ============================================================
# Configuration
# ============================================================

MAVLINK_DEVICE = "/dev/ttyAMA0"
BAUD_RATE = 115200

HOLD_TIME_S = 20.0
COMMAND_PERIOD_S = 0.5

ARRIVAL_RADIUS_M = 0.5


class BarbariqHold(Node):

    def __init__(self):
        super().__init__("barbariq_hold")

        # ----------------------------------------------------
        # ROS subscriptions
        # ----------------------------------------------------

        self.arrival_sub = self.create_subscription(
            String,
            "/barbariq/position/arrival",
            self.arrival_callback,
            10
        )

        self.telemetry_sub = self.create_subscription(
            String,
            "/barbariq/telemetry",
            self.telemetry_callback,
            10
        )

        # ----------------------------------------------------
        # ROS publishers
        # ----------------------------------------------------

        self.status_pub = self.create_publisher(
            String,
            "/barbariq/hold/status",
            10
        )

        # ----------------------------------------------------
        # MAVLink
        # ----------------------------------------------------

        self.master = None

        self.connect_mavlink()

        # ----------------------------------------------------
        # Current telemetry
        # ----------------------------------------------------

        self.current_lat = None
        self.current_lon = None
        self.current_alt = None

        self.mode = "UNKNOWN"
        self.armed = False

        # ----------------------------------------------------
        # Hold state
        # ----------------------------------------------------

        self.holding = False

        self.hold_lat = None
        self.hold_lon = None
        self.hold_alt = None

        self.hold_start_time = None

        # ----------------------------------------------------
        # Worker
        # ----------------------------------------------------

        self.worker = self.create_timer(
            COMMAND_PERIOD_S,
            self.hold_worker
        )

        self.get_logger().info(
            "Barbariq hold node started"
        )

    # ========================================================
    # MAVLink connection
    # ========================================================

    def connect_mavlink(self):

        try:

            self.master = mavutil.mavlink_connection(
                MAVLINK_DEVICE,
                baud=BAUD_RATE
            )

            self.get_logger().info(
                "Waiting for MAVLink heartbeat..."
            )

            self.master.wait_heartbeat(timeout=15)

            self.get_logger().info(
                "MAVLink connected"
            )

        except Exception as e:

            self.get_logger().error(
                f"MAVLink connection failed: {e}"
            )

            self.master = None

    # ========================================================
    # Telemetry
    # ========================================================

    def telemetry_callback(self, msg):

        try:

            values = self.parse_message(msg.data)

            if "lat" in values:
                self.current_lat = float(values["lat"])

            if "lon" in values:
                self.current_lon = float(values["lon"])

            if "rel_alt_m" in values:
                self.current_alt = float(
                    values["rel_alt_m"]
                )

            if "mode" in values:
                self.mode = values["mode"]

            if "armed" in values:
                self.armed = (
                    values["armed"] == "True"
                )

        except Exception as e:

            self.get_logger().warning(
                f"Telemetry parse error: {e}"
            )

    # ========================================================
    # Arrival callback
    # ========================================================

    def arrival_callback(self, msg):

        try:

            values = self.parse_message(msg.data)

            if values.get("ARRIVED") is not None:
                pass

            lat = float(
                values.get(
                    "lat",
                    self.current_lat
                )
            )

            lon = float(
                values.get(
                    "lon",
                    self.current_lon
                )
            )

            if self.current_alt is None:

                self.get_logger().warning(
                    "Cannot start hold: altitude unavailable"
                )

                return

            # ------------------------------------------------
            # Lock the position at the moment ARRIVED
            # ------------------------------------------------

            self.hold_lat = lat
            self.hold_lon = lon
            self.hold_alt = self.current_alt

            self.hold_start_time = time.monotonic()

            self.holding = True

            self.get_logger().info(
                f"POSITION LOCKED\n"
                f"  lat = {self.hold_lat:.7f}\n"
                f"  lon = {self.hold_lon:.7f}\n"
                f"  alt = {self.hold_alt:.2f} m\n"
                f"  hold = {HOLD_TIME_S:.1f} s"
            )

            self.publish_status(
                "HOLD_STARTED"
            )

        except Exception as e:

            self.get_logger().error(
                f"Arrival processing failed: {e}"
            )

    # ========================================================
    # Hold worker
    # ========================================================

    def hold_worker(self):

        if not self.holding:
            return

        # ----------------------------------------------------
        # Pilot override safety
        # ----------------------------------------------------

        if self.mode != "GUIDED":

            self.get_logger().warning(
                f"HOLD STOPPED: mode changed to {self.mode}"
            )

            self.holding = False

            self.publish_status(
                "HOLD_ABORTED_MODE_CHANGE"
            )

            return

        # ----------------------------------------------------
        # Calculate elapsed hold time
        # ----------------------------------------------------

        elapsed = (
            time.monotonic()
            - self.hold_start_time
        )

        remaining = max(
            0.0,
            HOLD_TIME_S - elapsed
        )

        # ----------------------------------------------------
        # Keep commanding locked position
        # ----------------------------------------------------

        if self.master is not None:

            self.send_goto(
                self.hold_lat,
                self.hold_lon,
                self.hold_alt
            )

        self.publish_status(
            f"HOLDING,remaining_s={remaining:.1f}"
        )

        self.get_logger().info(
            f"HOLDING position | "
            f"remaining={remaining:.1f}s"
        )

        # ----------------------------------------------------
        # Hold finished
        # ----------------------------------------------------

        if elapsed >= HOLD_TIME_S:

            self.holding = False

            self.get_logger().info(
                "20-second position hold complete"
            )

            self.publish_status(
                "HOLD_COMPLETE"
            )

    # ========================================================
    # Send position command
    # ========================================================

    def send_goto(
        self,
        lat,
        lon,
        alt_m
    ):

        if self.master is None:
            return False

        try:

            type_mask = (
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
                |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
                |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
                |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
                |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
                |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
                |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
                |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            )

            self.master.mav.set_position_target_global_int_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                type_mask,
                int(lat * 1e7),
                int(lon * 1e7),
                float(alt_m),
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0
            )

            return True

        except Exception as e:

            self.get_logger().error(
                f"Failed to send hold command: {e}"
            )

            return False

    # ========================================================
    # Message parser
    # ========================================================

    @staticmethod
    def parse_message(data):

        values = {}

        for item in data.split(","):

            if "=" not in item:
                continue

            key, value = item.split("=", 1)

            values[key.strip()] = value.strip()

        return values

    # ========================================================
    # Status
    # ========================================================

    def publish_status(self, status):

        msg = String()
        msg.data = status

        self.status_pub.publish(msg)

    # ========================================================
    # Shutdown
    # ========================================================

    def destroy_node(self):

        self.get_logger().info(
            "Stopping Barbariq hold node..."
        )

        if self.master is not None:

            try:
                self.master.close()
            except Exception:
                pass

            self.master = None

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = BarbariqHold()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
