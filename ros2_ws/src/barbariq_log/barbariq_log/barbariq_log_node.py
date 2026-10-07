#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BarbariqLog(Node):

    def __init__(self):

        super().__init__("barbariq_log")

        # ====================================================
        # Subscribe to important Barbariq topics
        # ====================================================

        self.telemetry_sub = self.create_subscription(
            String,
            "/barbariq/telemetry",
            self.telemetry_callback,
            10
        )

        self.navigation_sub = self.create_subscription(
            String,
            "/barbariq/navigation/status",
            self.navigation_callback,
            10
        )

        self.position_sub = self.create_subscription(
            String,
            "/barbariq/position/distance",
            self.position_callback,
            10
        )

        self.arrival_sub = self.create_subscription(
            String,
            "/barbariq/position/arrival",
            self.arrival_callback,
            10
        )

        self.hold_sub = self.create_subscription(
            String,
            "/barbariq/hold/status",
            self.hold_callback,
            10
        )

        self.payload_sub = self.create_subscription(
            String,
            "/barbariq/payload/status",
            self.payload_callback,
            10
        )

        self.mission_sub = self.create_subscription(
            String,
            "/barbariq/mission/status",
            self.mission_callback,
            10
        )

        # ====================================================
        # Unified Barbariq log
        # ====================================================

        self.log_pub = self.create_publisher(
            String,
            "/barbariq/mission/log",
            10
        )

        self.get_logger().info(
            "Barbariq log node started"
        )

        self.get_logger().info(
            "Publishing unified log on /barbariq/mission/log"
        )

    # ========================================================
    # Generic log function
    # ========================================================

    def publish_log(self, source, text):

        log_message = String()

        log_message.data = (
            f"[BARBARIQ][{source}] {text}"
        )

        self.log_pub.publish(
            log_message
        )

        # Also show it in this terminal
        self.get_logger().info(
            f"[{source}] {text}"
        )

    # ========================================================
    # Telemetry
    # ========================================================

    def telemetry_callback(self, msg):

        self.publish_log(
            "TELEMETRY",
            msg.data
        )

    # ========================================================
    # Navigation
    # ========================================================

    def navigation_callback(self, msg):

        self.publish_log(
            "NAVIGATION",
            msg.data
        )

    # ========================================================
    # Distance
    # ========================================================

    def position_callback(self, msg):

        self.publish_log(
            "POSITION",
            msg.data
        )

    # ========================================================
    # Arrival
    # ========================================================

    def arrival_callback(self, msg):

        self.publish_log(
            "ARRIVAL",
            msg.data
        )

    # ========================================================
    # Hold
    # ========================================================

    def hold_callback(self, msg):

        self.publish_log(
            "HOLD",
            msg.data
        )

    # ========================================================
    # Payload
    # ========================================================

    def payload_callback(self, msg):

        self.publish_log(
            "PAYLOAD",
            msg.data
        )

    # ========================================================
    # Mission manager
    # ========================================================

    def mission_callback(self, msg):

        self.publish_log(
            "MISSION",
            msg.data
        )


def main(args=None):

    rclpy.init(args=args)

    node = BarbariqLog()

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
