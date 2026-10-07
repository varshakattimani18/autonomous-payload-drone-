#!/usr/bin/env python3

import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pymavlink import mavutil


# ============================================================
# Barbariq MAVLink configuration
# ============================================================

MAVLINK_DEVICE = "/dev/ttyAMA0"
BAUD_RATE = 115200

TELEMETRY_RATE_HZ = 10.0
HEARTBEAT_TIMEOUT_S = 15.0


class BarbariqTelemetry(Node):

    def __init__(self):
        super().__init__("barbariq_telemetry")

        self.publisher = self.create_publisher(
            String,
            "/barbariq/telemetry",
            10
        )

        self.master = None

        self.lock = threading.Lock()

        # ----------------------------------------------------
        # Latest telemetry state
        # ----------------------------------------------------

        self.lat = 0.0
        self.lon = 0.0

        self.rel_alt_m = 0.0
        self.amsl_m = 0.0

        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0

        self.vn = 0.0
        self.ve = 0.0

        self.hdg = 0.0

        self.fix_type = 0
        self.sats = 0

        self.armed = False
        self.mode = "UNKNOWN"

        self.last_gps_time = 0.0
        self.last_attitude_time = 0.0
        self.last_heartbeat_time = 0.0

        self.connected = False

        # ----------------------------------------------------
        # Connect to flight controller
        # ----------------------------------------------------

        self.connect_mavlink()

        # ----------------------------------------------------
        # Background MAVLink reader
        # ----------------------------------------------------

        self.reader_thread = threading.Thread(
            target=self.mavlink_reader,
            daemon=True
        )

        self.reader_thread.start()

        # ----------------------------------------------------
        # ROS publishing timer
        # ----------------------------------------------------

        period = 1.0 / TELEMETRY_RATE_HZ

        self.timer = self.create_timer(
            period,
            self.publish_telemetry
        )

        self.get_logger().info(
            "Barbariq telemetry node started"
        )

    # ========================================================
    # MAVLink connection
    # ========================================================

    def connect_mavlink(self):

        while rclpy.ok():

            try:
                self.get_logger().info(
                    f"Connecting to {MAVLINK_DEVICE} @ {BAUD_RATE}"
                )

                self.master = mavutil.mavlink_connection(
                    MAVLINK_DEVICE,
                    baud=BAUD_RATE
                )

                self.get_logger().info(
                    "Waiting for MAVLink heartbeat..."
                )

                self.master.wait_heartbeat(
                    timeout=HEARTBEAT_TIMEOUT_S
                )

                self.connected = True

                self.get_logger().info(
                    f"MAVLink connected "
                    f"(system={self.master.target_system}, "
                    f"component={self.master.target_component})"
                )

                return

            except Exception as e:

                self.connected = False

                self.get_logger().error(
                    f"MAVLink connection failed: {e}"
                )

                if self.master is not None:

                    try:
                        self.master.close()
                    except Exception:
                        pass

                    self.master = None

                time.sleep(2.0)

    # ========================================================
    # MAVLink telemetry reader
    # ========================================================

    def mavlink_reader(self):

        while rclpy.ok():

            if self.master is None:

                self.connected = False

                self.connect_mavlink()

                continue

            try:

                msg = self.master.recv_match(
                    blocking=True,
                    timeout=1.0
                )

                if msg is None:
                    continue

                msg_type = msg.get_type()

                now = time.time()

                # ------------------------------------------------
                # HEARTBEAT
                # ------------------------------------------------

                if msg_type == "HEARTBEAT":

                    with self.lock:

                        self.last_heartbeat_time = now

                        self.armed = bool(
                            msg.base_mode
                            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                        )

                        self.mode = (
                            mavutil.mode_string_v10(msg)
                        )

                        self.connected = True

                # ------------------------------------------------
                # GLOBAL_POSITION_INT
                # ------------------------------------------------

                elif msg_type == "GLOBAL_POSITION_INT":

                    with self.lock:

                        self.lat = msg.lat / 1e7
                        self.lon = msg.lon / 1e7

                        self.amsl_m = msg.alt / 1000.0

                        self.rel_alt_m = (
                            msg.relative_alt / 1000.0
                        )

                        self.vn = msg.vx / 100.0
                        self.ve = msg.vy / 100.0

                        self.hdg = (
                            msg.hdg / 100.0
                            if msg.hdg != 65535
                            else 0.0
                        )

                        self.last_gps_time = now

                # ------------------------------------------------
                # ATTITUDE
                # ------------------------------------------------

                elif msg_type == "ATTITUDE":

                    with self.lock:

                        self.roll = msg.roll * 180.0 / 3.141592653589793
                        self.pitch = msg.pitch * 180.0 / 3.141592653589793
                        self.yaw = msg.yaw * 180.0 / 3.141592653589793

                        if self.yaw < 0.0:
                            self.yaw += 360.0

                        self.last_attitude_time = now

                # ------------------------------------------------
                # GPS_RAW_INT
                # ------------------------------------------------

                elif msg_type == "GPS_RAW_INT":

                    with self.lock:

                        self.fix_type = msg.fix_type
                        self.sats = msg.satellites_visible

            except Exception as e:

                self.connected = False

                self.get_logger().error(
                    f"MAVLink reader error: {e}"
                )

                try:
                    self.master.close()
                except Exception:
                    pass

                self.master = None

    # ========================================================
    # Publish ROS telemetry
    # ========================================================

    def publish_telemetry(self):

        with self.lock:

            now = time.time()

            gps_age = (
                now - self.last_gps_time
                if self.last_gps_time > 0
                else 999.0
            )

            attitude_age = (
                now - self.last_attitude_time
                if self.last_attitude_time > 0
                else 999.0
            )

            heartbeat_age = (
                now - self.last_heartbeat_time
                if self.last_heartbeat_time > 0
                else 999.0
            )

            valid = (
                self.connected
                and gps_age <= 0.5
                and attitude_age <= 0.5
            )

            message = (
                f"lat={self.lat:.7f},"
                f"lon={self.lon:.7f},"
                f"rel_alt_m={self.rel_alt_m:.3f},"
                f"amsl_m={self.amsl_m:.3f},"
                f"yaw={self.yaw:.2f},"
                f"roll={self.roll:.2f},"
                f"pitch={self.pitch:.2f},"
                f"vn={self.vn:.3f},"
                f"ve={self.ve:.3f},"
                f"hdg={self.hdg:.2f},"
                f"fix_type={self.fix_type},"
                f"sats={self.sats},"
                f"armed={self.armed},"
                f"mode={self.mode},"
                f"gps_dt={gps_age:.3f},"
                f"att_dt={attitude_age:.3f},"
                f"heartbeat_dt={heartbeat_age:.3f},"
                f"valid={valid}"
            )

        msg = String()
        msg.data = message

        self.publisher.publish(msg)

    # ========================================================
    # Shutdown
    # ========================================================

    def destroy_node(self):

        self.get_logger().info(
            "Stopping Barbariq telemetry node..."
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

    node = BarbariqTelemetry()

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
