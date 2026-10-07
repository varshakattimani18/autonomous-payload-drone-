#!/usr/bin/env python3

import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pymavlink import mavutil


# ============================================================
# MAVLink configuration
# ============================================================

MAVLINK_DEVICE = "/dev/ttyAMA0"
BAUD_RATE = 115200


# ============================================================
# Payload configuration
# ============================================================

SERVO_A = 6
SERVO_B = 7

PWM_LOW = 800
PWM_MID = 1500
PWM_HIGH = 1800

RETURN_TO_MID = True
MID_DELAY = 3.0


# ============================================================
# Barbariq has ONLY 2 payloads
#
# Exact servo mapping can be edited later.
# ============================================================

PAYLOADS = {
    1: (SERVO_A, PWM_LOW),
    2: (SERVO_A, PWM_HIGH),
}


class BarbariqPayload(Node):

    def __init__(self):

        super().__init__("barbariq_payload")

        # ----------------------------------------------------
        # ROS subscribers
        # ----------------------------------------------------

        self.drop_sub = self.create_subscription(
            String,
            "/barbariq/payload/drop",
            self.drop_callback,
            10
        )

        # ----------------------------------------------------
        # ROS publisher
        # ----------------------------------------------------

        self.status_pub = self.create_publisher(
            String,
            "/barbariq/payload/status",
            10
        )

        # ----------------------------------------------------
        # State
        # ----------------------------------------------------

        self.master = None

        self.busy = False

        self.released_payloads = set()

        self.lock = threading.Lock()

        # ----------------------------------------------------
        # MAVLink connection
        # ----------------------------------------------------

        self.connect_mavlink()

        self.get_logger().info(
            "Barbariq payload node started"
        )

        self.get_logger().info(
            "Configured for 2 payloads"
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

            self.master.wait_heartbeat(
                timeout=15
            )

            self.get_logger().info(
                "MAVLink heartbeat received"
            )

            self.get_logger().info(
                f"Connected to system "
                f"{self.master.target_system}, "
                f"component "
                f"{self.master.target_component}"
            )

        except Exception as e:

            self.get_logger().error(
                f"MAVLink connection failed: {e}"
            )

            self.master = None

    # ========================================================
    # Drop command callback
    # ========================================================

    def drop_callback(self, msg):

        command = msg.data.strip()

        if not command:
            return

        try:

            payload_number = int(command)

        except ValueError:

            self.get_logger().warning(
                f"Invalid payload command: {command}"
            )

            self.publish_status(
                f"ERROR,invalid_command={command}"
            )

            return

        # ----------------------------------------------------
        # Only payloads 1 and 2 are valid
        # ----------------------------------------------------

        if payload_number not in PAYLOADS:

            self.get_logger().warning(
                f"Invalid payload number: {payload_number}"
            )

            self.publish_status(
                f"ERROR,invalid_payload={payload_number}"
            )

            return

        # ----------------------------------------------------
        # Run drop in worker thread
        # so ROS callback is not blocked
        # ----------------------------------------------------

        threading.Thread(
            target=self.drop_payload,
            args=(payload_number,),
            daemon=True
        ).start()

    # ========================================================
    # Drop payload
    # ========================================================

    def drop_payload(self, payload_number):

        # ----------------------------------------------------
        # Check state
        # ----------------------------------------------------

        with self.lock:

            if self.busy:

                self.get_logger().warning(
                    "Payload system is busy"
                )

                self.publish_status(
                    "BUSY"
                )

                return

            if payload_number in self.released_payloads:

                self.get_logger().warning(
                    f"Payload {payload_number} "
                    f"already released"
                )

                self.publish_status(
                    f"ALREADY_RELEASED,"
                    f"payload={payload_number}"
                )

                return

            self.busy = True

        try:

            servo, pwm = PAYLOADS[payload_number]

            # ------------------------------------------------
            # Log
            # ------------------------------------------------

            self.get_logger().info(
                f"DROP payload {payload_number}"
            )

            self.get_logger().info(
                f"Servo = {servo}, PWM = {pwm}"
            )

            self.publish_status(
                f"DROP_STARTED,"
                f"payload={payload_number},"
                f"servo={servo},"
                f"pwm={pwm}"
            )

            # ------------------------------------------------
            # Actual tested Barbariq mechanism
            # ------------------------------------------------

            success = self.set_servo(
                servo,
                pwm
            )

            if not success:

                self.publish_status(
                    f"DROP_FAILED,"
                    f"payload={payload_number}"
                )

                return

            # ------------------------------------------------
            # Mark payload as released
            # ------------------------------------------------

            with self.lock:

                self.released_payloads.add(
                    payload_number
                )

            self.publish_status(
                f"DROP_TRIGGERED,"
                f"payload={payload_number}"
            )

            self.get_logger().info(
                f"Payload {payload_number} "
                f"triggered successfully"
            )

            # ------------------------------------------------
            # Return servo to midpoint
            # ------------------------------------------------

            if RETURN_TO_MID:

                time.sleep(MID_DELAY)

                self.set_servo(
                    servo,
                    PWM_MID
                )

                self.get_logger().info(
                    f"Servo {servo} returned "
                    f"to PWM {PWM_MID}"
                )

                self.publish_status(
                    f"SERVO_MID,"
                    f"payload={payload_number}"
                )

            # ------------------------------------------------
            # Complete
            # ------------------------------------------------

            self.publish_status(
                f"DROP_COMPLETE,"
                f"payload={payload_number}"
            )

        except Exception as e:

            self.get_logger().error(
                f"Payload drop error: {e}"
            )

            self.publish_status(
                f"DROP_FAILED,"
                f"payload={payload_number},"
                f"error={e}"
            )

        finally:

            with self.lock:

                self.busy = False

    # ========================================================
    # MAVLink servo command
    # ========================================================

    def set_servo(self, servo, pwm):

        if self.master is None:

            self.get_logger().error(
                "MAVLink is not connected"
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
                f"SERVO {servo} -> PWM {pwm}"
            )

            return True

        except Exception as e:

            self.get_logger().error(
                f"Failed to send servo command: {e}"
            )

            return False

    # ========================================================
    # Publish status
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
            "Stopping Barbariq payload node"
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

    node = BarbariqPayload()

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
