#!/usr/bin/env python3

import time
import threading
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pymavlink import mavutil


# ============================================================
# Configuration
# ============================================================

MAVLINK_DEVICE = "/dev/ttyAMA0"
BAUD_RATE = 115200

TARGET_ALT_M = 6.1

COMMAND_PERIOD_S = 0.5
TARGET_TIMEOUT_S = 120.0

MODE_GUIDED = 4
MODE_LOITER = 5


class BarbariqNavigation(Node):

    def __init__(self):
        super().__init__("barbariq_navigation")

        # ----------------------------------------------------
        # ROS interfaces
        # ----------------------------------------------------

        self.target_sub = self.create_subscription(
            String,
            "/barbariq/navigation/target",
            self.target_callback,
            10
        )

        self.telemetry_sub = self.create_subscription(
            String,
            "/barbariq/telemetry",
            self.telemetry_callback,
            10
        )

        self.status_pub = self.create_publisher(
            String,
            "/barbariq/navigation/status",
            10
        )

        # ----------------------------------------------------
        # MAVLink
        # ----------------------------------------------------

        self.master = None
        self.connect_lock = threading.Lock()

        # ----------------------------------------------------
        # Telemetry state
        # ----------------------------------------------------

        self.lat = 0.0
        self.lon = 0.0
        self.alt_m = 0.0

        self.armed = False
        self.mode = "UNKNOWN"

        self.telemetry_valid = False

        # ----------------------------------------------------
        # Navigation state
        # ----------------------------------------------------

        self.target_lat = None
        self.target_lon = None
        self.target_alt = TARGET_ALT_M

        self.navigation_active = False
        self.navigation_start_time = 0.0

        self.state_lock = threading.Lock()

        # ----------------------------------------------------
        # Connect
        # ----------------------------------------------------

        self.connect_mavlink()

        # ----------------------------------------------------
        # Navigation worker
        # ----------------------------------------------------

        self.worker = threading.Thread(
            target=self.navigation_worker,
            daemon=True
        )

        self.worker.start()

        self.get_logger().info(
            "Barbariq navigation node started"
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
    # Telemetry callback
    # ========================================================

    def telemetry_callback(self, msg):

        try:

            values = {}

            for item in msg.data.split(","):

                if "=" not in item:
                    continue

                key, value = item.split("=", 1)

                values[key] = value

            with self.state_lock:

                self.lat = float(values.get("lat", self.lat))
                self.lon = float(values.get("lon", self.lon))
                self.alt_m = float(
                    values.get("rel_alt_m", self.alt_m)
                )

                self.armed = (
                    values.get("armed", "False") == "True"
                )

                self.mode = values.get(
                    "mode",
                    self.mode
                )

                self.telemetry_valid = (
                    values.get("valid", "False") == "True"
                )

        except Exception as e:

            self.get_logger().warning(
                f"Telemetry parse error: {e}"
            )

    # ========================================================
    # Target callback
    # ========================================================

    def target_callback(self, msg):

        try:

            values = {}

            for item in msg.data.split(","):

                if "=" not in item:
                    continue

                key, value = item.split("=", 1)

                values[key.strip()] = value.strip()

            lat = float(values["lat"])
            lon = float(values["lon"])

            alt = float(
                values.get(
                    "alt_m",
                    TARGET_ALT_M
                )
            )

            with self.state_lock:

                self.target_lat = lat
                self.target_lon = lon
                self.target_alt = alt

                self.navigation_active = True
                self.navigation_start_time = time.time()

            self.get_logger().info(
                f"New target: "
                f"lat={lat:.7f}, "
                f"lon={lon:.7f}, "
                f"alt={alt:.2f} m"
            )

        except Exception as e:

            self.get_logger().error(
                f"Invalid navigation target: {e}"
            )

    # ========================================================
    # GUIDED mode
    # ========================================================

    def set_guided_mode(self):

        if self.master is None:
            return False

        try:

            self.master.set_mode("GUIDED")

            self.get_logger().info(
                "GUIDED mode command sent"
            )

            return True

        except Exception as e:

            self.get_logger().error(
                f"Failed to set GUIDED: {e}"
            )

            return False

    # ========================================================
    # Distance calculation
    # ========================================================

    @staticmethod
    def distance_m(
        lat1,
        lon1,
        lat2,
        lon2
    ):

        R = 6371000.0

        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)

        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)

        a = (
            math.sin(dlat / 2.0) ** 2
            +
            math.cos(lat1_rad)
            *
            math.cos(lat2_rad)
            *
            math.sin(dlon / 2.0) ** 2
        )

        c = 2.0 * math.atan2(
            math.sqrt(a),
            math.sqrt(1.0 - a)
        )

        return R * c

    # ========================================================
    # Send GUIDED position target
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

            # Position only.
            # Ignore velocity, acceleration and yaw fields.

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
                f"Failed to send position target: {e}"
            )

            return False

    # ========================================================
    # Navigation worker
    # ========================================================

    def navigation_worker(self):

        while rclpy.ok():

            with self.state_lock:

                active = self.navigation_active
                target_lat = self.target_lat
                target_lon = self.target_lon
                target_alt = self.target_alt
                current_mode = self.mode
                current_lat = self.lat
                current_lon = self.lon

            if not active:

                time.sleep(0.1)
                continue

            if target_lat is None or target_lon is None:

                time.sleep(0.1)
                continue

            # ------------------------------------------------
            # Pilot override / mode changed
            # ------------------------------------------------

            if current_mode != "GUIDED":

                self.get_logger().warning(
                    f"Navigation stopped: "
                    f"vehicle is in {current_mode}, "
                    f"not GUIDED"
                )

                with self.state_lock:
                    self.navigation_active = False

                self.publish_status(
                    "STOPPED_MODE_CHANGE"
                )

                time.sleep(0.5)
                continue

            # ------------------------------------------------
            # Timeout
            # ------------------------------------------------

            if (
                time.time()
                - self.navigation_start_time
                > TARGET_TIMEOUT_S
            ):

                self.get_logger().warning(
                    "Navigation target timeout"
                )

                with self.state_lock:
                    self.navigation_active = False

                self.publish_status(
                    "TIMEOUT"
                )

                continue

            # ------------------------------------------------
            # Distance
            # ------------------------------------------------

            distance = self.distance_m(
                current_lat,
                current_lon,
                target_lat,
                target_lon
            )

            self.publish_status(
                f"FLYING,distance_m={distance:.3f}"
            )

            # ------------------------------------------------
            # Target reached
            #
            # IMPORTANT:
            # Do not perform payload drop here.
            #
            # The position monitor / hold node will handle
            # the final 0.5 m lock.
            # ------------------------------------------------

            if distance <= 5.0:

                self.get_logger().info(
                    f"Navigation target reached "
                    f"within 5 m: {distance:.3f} m"
                )

                with self.state_lock:
                    self.navigation_active = False

                self.publish_status(
                    f"ARRIVED,distance_m={distance:.3f}"
                )

                continue

            # ------------------------------------------------
            # Continue sending target
            # ------------------------------------------------

            self.send_goto(
                target_lat,
                target_lon,
                target_alt
            )

            time.sleep(COMMAND_PERIOD_S)

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
            "Stopping Barbariq navigation node..."
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

    node = BarbariqNavigation()

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
