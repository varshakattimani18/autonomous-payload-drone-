#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


ARRIVAL_RADIUS_M = 0.5
PUBLISH_RATE_HZ = 1.0


class BarbariqPositionMonitor(Node):

    def __init__(self):
        super().__init__("barbariq_position_monitor")

        self.telemetry_sub = self.create_subscription(
            String,
            "/barbariq/telemetry",
            self.telemetry_callback,
            10
        )

        self.target_sub = self.create_subscription(
            String,
            "/barbariq/navigation/target",
            self.target_callback,
            10
        )

        self.distance_pub = self.create_publisher(
            String,
            "/barbariq/position/distance",
            10
        )

        self.arrival_pub = self.create_publisher(
            String,
            "/barbariq/position/arrival",
            10
        )

        self.current_lat = None
        self.current_lon = None

        self.target_lat = None
        self.target_lon = None

        self.target_active = False
        self.arrived = False

        self.timer = self.create_timer(
            1.0 / PUBLISH_RATE_HZ,
            self.monitor_position
        )

        self.get_logger().info(
            "Barbariq position monitor started"
        )

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

        except Exception as e:

            self.get_logger().warning(
                f"Telemetry parse error: {e}"
            )

    # ========================================================
    # Target
    # ========================================================

    def target_callback(self, msg):

        try:

            values = self.parse_message(msg.data)

            self.target_lat = float(values["lat"])
            self.target_lon = float(values["lon"])

            self.target_active = True
            self.arrived = False

            self.get_logger().info(
                f"Monitoring target "
                f"lat={self.target_lat:.7f}, "
                f"lon={self.target_lon:.7f}"
            )

        except Exception as e:

            self.get_logger().error(
                f"Invalid target: {e}"
            )

    # ========================================================
    # Monitor
    # ========================================================

    def monitor_position(self):

        if not self.target_active:
            return

        if self.current_lat is None:
            return

        if self.current_lon is None:
            return

        if self.target_lat is None:
            return

        if self.target_lon is None:
            return

        distance = self.haversine_m(
            self.current_lat,
            self.current_lon,
            self.target_lat,
            self.target_lon
        )

        # Always publish current distance.

        msg = String()

        msg.data = (
            f"distance_m={distance:.3f},"
            f"current_lat={self.current_lat:.7f},"
            f"current_lon={self.current_lon:.7f},"
            f"target_lat={self.target_lat:.7f},"
            f"target_lon={self.target_lon:.7f}"
        )

        self.distance_pub.publish(msg)

        self.get_logger().info(
            f"Distance to target: {distance:.3f} m"
        )

        # ----------------------------------------------------
        # Final arrival condition
        # ----------------------------------------------------

        if distance <= ARRIVAL_RADIUS_M:

            if not self.arrived:

                self.arrived = True

                self.get_logger().info(
                    f"TARGET LOCK RANGE REACHED: "
                    f"{distance:.3f} m <= "
                    f"{ARRIVAL_RADIUS_M:.3f} m"
                )

                arrival = String()

                arrival.data = (
                    f"ARRIVED,"
                    f"distance_m={distance:.3f},"
                    f"lat={self.current_lat:.7f},"
                    f"lon={self.current_lon:.7f}"
                )

                self.arrival_pub.publish(arrival)

    # ========================================================
    # Haversine
    # ========================================================

    @staticmethod
    def haversine_m(
        lat1,
        lon1,
        lat2,
        lon2
    ):

        earth_radius_m = 6371000.0

        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)

        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)

        a = (
            math.sin(dlat / 2.0) ** 2
            +
            math.cos(lat1_rad)
            * math.cos(lat2_rad)
            * math.sin(dlon / 2.0) ** 2
        )

        c = 2.0 * math.atan2(
            math.sqrt(a),
            math.sqrt(1.0 - a)
        )

        return earth_radius_m * c

    # ========================================================
    # Parse telemetry/target strings
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


def main(args=None):

    rclpy.init(args=args)

    node = BarbariqPositionMonitor()

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
