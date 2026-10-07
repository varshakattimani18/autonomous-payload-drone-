#!/usr/bin/env python3

import time
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None


class RudraPayloadNode(Node):

    # ============================================================
    # MAVLINK CONFIG
    # ============================================================

    MAVLINK_DEVICE = "/dev/ttyAMA0"
    MAVLINK_BAUD = 115200

    # ============================================================
    # PAYLOAD CONFIG
    #
    # IMPORTANT:
    # These are PLACEHOLDER values until Rudra's actual
    # servo wiring/channel mapping is confirmed.
    #
    # DO NOT connect the physical payload system using these
    # values until the actual Rudra servo mapping is verified.
    # ============================================================

    PAYLOADS = {
        1: {"servo": 6, "release_pwm": 800},
        2: {"servo": 6, "release_pwm": 1800},
        3: {"servo": 7, "release_pwm": 800},
        4: {"servo": 7, "release_pwm": 1800},

        # Placeholder slots for Rudra payloads 5-8.
        5: {"servo": 8, "release_pwm": 800},
        6: {"servo": 8, "release_pwm": 1800},
        7: {"servo": 9, "release_pwm": 800},
        8: {"servo": 9, "release_pwm": 1800},
    }

    PWM_MID = 1500

    RETURN_TO_MID = True
    MID_DELAY = 3.0

    # ============================================================
    # INIT
    # ============================================================

    def __init__(self):
        super().__init__("rudra_payload_node")

        self.get_logger().info("========================================")
        self.get_logger().info("      RUDRA PAYLOAD NODE STARTED")
        self.get_logger().info("========================================")

        self.master = None
        self.connected = False

        self.lock = threading.Lock()

        # Payload state
        self.released = set()
        self.busy = False

        # --------------------------------------------------------
        # ROS topics
        # --------------------------------------------------------

        self.drop_sub = self.create_subscription(
            String,
            "/rudra/payload/drop",
            self.drop_callback,
            10
        )

        self.status_pub = self.create_publisher(
            String,
            "/rudra/payload/status",
            10
        )

        self.get_logger().info(
            "Drop command topic: /rudra/payload/drop"
        )

        self.get_logger().info(
            "Status topic: /rudra/payload/status"
        )

        # --------------------------------------------------------
        # MAVLink connection
        # --------------------------------------------------------

        if mavutil is None:
            self.get_logger().error(
                "pymavlink is not installed."
            )
            return

        self.connect_mavlink()

    # ============================================================
    # MAVLINK CONNECTION
    # ============================================================

    def connect_mavlink(self):

        try:
            self.get_logger().info(
                f"Connecting to MAVLink: "
                f"{self.MAVLINK_DEVICE} @ {self.MAVLINK_BAUD}"
            )

            self.master = mavutil.mavlink_connection(
                self.MAVLINK_DEVICE,
                baud=self.MAVLINK_BAUD
            )

            self.get_logger().info(
                "Waiting for flight-controller heartbeat..."
            )

            self.master.wait_heartbeat(timeout=10)

            self.connected = True

            self.get_logger().info(
                f"MAVLink connected "
                f"(system={self.master.target_system}, "
                f"component={self.master.target_component})"
            )

            self.publish_status("READY")

        except Exception as e:

            self.connected = False

            self.get_logger().error(
                f"MAVLink connection failed: {e}"
            )

            self.publish_status(
                f"ERROR MAVLINK_CONNECTION_FAILED: {e}"
            )

    # ============================================================
    # ROS DROP COMMAND
    # ============================================================

    def drop_callback(self, msg):

        command = msg.data.strip()

        self.get_logger().info(
            f"Received payload command: '{command}'"
        )

        # --------------------------------------------------------
        # DROP ALL
        # --------------------------------------------------------

        if command.upper() == "ALL":

            self.get_logger().warn(
                "DROP ALL command received"
            )

            for payload_number in sorted(self.PAYLOADS.keys()):

                if payload_number in self.released:
                    continue

                self.drop_payload(payload_number)

                time.sleep(1.0)

            return

        # --------------------------------------------------------
        # NORMAL PAYLOAD NUMBER
        # --------------------------------------------------------

        try:
            payload_number = int(command)

        except ValueError:

            self.get_logger().error(
                f"Invalid payload command: {command}"
            )

            self.publish_status(
                f"ERROR INVALID_COMMAND {command}"
            )

            return

        self.drop_payload(payload_number)

    # ============================================================
    # DROP PAYLOAD
    # ============================================================

    def drop_payload(self, payload_number):

        with self.lock:

            # ----------------------------------------------------
            # Check payload number
            # ----------------------------------------------------

            if payload_number not in self.PAYLOADS:

                self.get_logger().error(
                    f"Payload #{payload_number} does not exist"
                )

                self.publish_status(
                    f"ERROR INVALID_PAYLOAD {payload_number}"
                )

                return False

            # ----------------------------------------------------
            # Prevent duplicate release
            # ----------------------------------------------------

            if payload_number in self.released:

                self.get_logger().warn(
                    f"Payload #{payload_number} already released"
                )

                self.publish_status(
                    f"ALREADY_RELEASED {payload_number}"
                )

                return False

            # ----------------------------------------------------
            # Prevent simultaneous operations
            # ----------------------------------------------------

            if self.busy:

                self.get_logger().warn(
                    "Payload mechanism is already busy"
                )

                self.publish_status(
                    "BUSY"
                )

                return False

            self.busy = True

        try:

            config = self.PAYLOADS[payload_number]

            servo = config["servo"]
            release_pwm = config["release_pwm"]

            self.get_logger().warn(
                f"DROP PAYLOAD #{payload_number} | "
                f"servo={servo} | PWM={release_pwm}"
            )

            self.publish_status(
                f"DROP_REQUESTED payload={payload_number}"
            )

            # ----------------------------------------------------
            # Send release command
            # ----------------------------------------------------

            success = self.set_servo(
                servo,
                release_pwm
            )

            if not success:

                self.get_logger().error(
                    f"Payload #{payload_number} release failed"
                )

                self.publish_status(
                    f"DROP_FAILED payload={payload_number}"
                )

                return False

            # ----------------------------------------------------
            # Mark released
            # ----------------------------------------------------

            self.released.add(payload_number)

            self.get_logger().info(
                f"Payload #{payload_number} RELEASED"
            )

            self.publish_status(
                f"RELEASED payload={payload_number}"
            )

            # ----------------------------------------------------
            # Return servo to MID
            # ----------------------------------------------------

            if self.RETURN_TO_MID:

                self.get_logger().info(
                    f"Waiting {self.MID_DELAY:.1f}s "
                    f"before returning servo {servo} to MID"
                )

                time.sleep(self.MID_DELAY)

                self.set_servo(
                    servo,
                    self.PWM_MID
                )

                self.get_logger().info(
                    f"Servo {servo} returned to "
                    f"PWM {self.PWM_MID}"
                )

                self.publish_status(
                    f"RETURNED_TO_MID payload={payload_number}"
                )

            return True

        finally:

            with self.lock:
                self.busy = False

    # ============================================================
    # MAVLINK SERVO COMMAND
    # ============================================================

    def set_servo(self, servo, pwm):

        if not self.connected or self.master is None:

            self.get_logger().error(
                "Cannot set servo: MAVLink not connected"
            )

            return False

        try:

            self.master.mav.command_long_send(

                self.master.target_system,
                self.master.target_component,

                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,

                0,

                servo,
                pwm,

                0,
                0,
                0,
                0,
                0
            )

            self.get_logger().info(
                f"MAV_CMD_DO_SET_SERVO sent: "
                f"servo={servo}, PWM={pwm}"
            )

            return True

        except Exception as e:

            self.get_logger().error(
                f"Servo command failed: {e}"
            )

            return False

    # ============================================================
    # STATUS
    # ============================================================

    def publish_status(self, text):

        msg = String()
        msg.data = text

        self.status_pub.publish(msg)

    # ============================================================
    # SHUTDOWN
    # ============================================================

    def destroy_node(self):

        self.get_logger().info(
            "Rudra payload node shutting down"
        )

        self.connected = False

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = RudraPayloadNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
