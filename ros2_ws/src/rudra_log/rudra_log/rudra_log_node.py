#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RudraLog(Node):

    def __init__(self):
        super().__init__("rudra_log")

        # --------------------------------------------------
        # Rudra topics to monitor
        # --------------------------------------------------

        self.telemetry_sub = self.create_subscription(
            String,
            "/rudra/telemetry",
            self.telemetry_callback,
            10
        )

        self.detection_sub = self.create_subscription(
            String,
            "/rudra/detections",
            self.detection_callback,
            10
        )

        self.geotag_sub = self.create_subscription(
            String,
            "/rudra/geotagged_detections",
            self.geotag_callback,
            10
        )

        self.tracking_sub = self.create_subscription(
            String,
            "/rudra/tracked_detections",
            self.tracking_callback,
            10
        )

        self.survivor_sub = self.create_subscription(
            String,
            "/rudra/survivor_targets",
            self.survivor_callback,
            10
        )

        self.mission_sub = self.create_subscription(
            String,
            "/rudra/mission/status",
            self.mission_callback,
            10
        )

        self.payload_sub = self.create_subscription(
            String,
            "/rudra/payload/status",
            self.payload_callback,
            10
        )

        # --------------------------------------------------
        # Published combined log
        # --------------------------------------------------

        self.log_pub = self.create_publisher(
            String,
            "/rudra/mission/log",
            10
        )

        self.get_logger().info(
            "Rudra log node started"
        )

    # ======================================================
    # Logging helper
    # ======================================================

    def publish_log(self, source, data):

        text = f"[RUDRA][{source}] {data}"

        msg = String()
        msg.data = text

        self.log_pub.publish(msg)

        self.get_logger().info(text)

    # ======================================================
    # Callbacks
    # ======================================================

    def telemetry_callback(self, msg):

        self.publish_log(
            "TELEMETRY",
            msg.data
        )

    def detection_callback(self, msg):

        self.publish_log(
            "DETECTION",
            msg.data
        )

    def geotag_callback(self, msg):

        self.publish_log(
            "GEOTAG",
            msg.data
        )

    def tracking_callback(self, msg):

        self.publish_log(
            "TRACKING",
            msg.data
        )

    def survivor_callback(self, msg):

        self.publish_log(
            "SURVIVOR",
            msg.data
        )

    def mission_callback(self, msg):

        self.publish_log(
            "MISSION",
            msg.data
        )

    def payload_callback(self, msg):

        self.publish_log(
            "PAYLOAD",
            msg.data
        )


def main(args=None):

    rclpy.init(args=args)

    node = RudraLog()

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
