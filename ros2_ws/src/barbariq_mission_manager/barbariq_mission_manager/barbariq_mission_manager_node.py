#!/usr/bin/env python3

import time
import threading
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pymavlink import mavutil


# ============================================================
# Mission configuration
# ============================================================

MAVLINK_DEVICE = "/dev/ttyAMA0"
BAUD_RATE = 115200

# Tested Barbariq start condition
MIN_START_ALT_M = 4.57       # 15 ft

# Final mission requirement
ARRIVAL_RADIUS_M = 0.5
HOLD_TIME_S = 20.0

# Navigation timeout
NAVIGATION_TIMEOUT_S = 120.0

# Final mission uses 2 payloads
TOTAL_PAYLOADS = 2


# ============================================================
# Flight modes from tested Barbariq code
# ============================================================

MODE_GUIDED = 4
MODE_LOITER = 5
MODE_AUTO = 3
MODE_RTL = 6
MODE_LAND = 9


class BarbariqMissionManager(Node):

    def __init__(self):

        super().__init__("barbariq_mission_manager")

        # ----------------------------------------------------
        # ROS interfaces
        # ----------------------------------------------------

        self.target_sub = self.create_subscription(
            String,
            "/barbariq/mission/target",
            self.target_callback,
            10
        )

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

        # ----------------------------------------------------
        # Publishers
        # ----------------------------------------------------

        self.navigation_pub = self.create_publisher(
            String,
            "/barbariq/navigation/target",
            10
        )

        self.payload_pub = self.create_publisher(
            String,
            "/barbariq/payload/drop",
            10
        )

        self.status_pub = self.create_publisher(
            String,
            "/barbariq/mission/status",
            10
        )

        # ----------------------------------------------------
        # MAVLink
        # ----------------------------------------------------

        self.master = None

        self.connect_mavlink()

        # ----------------------------------------------------
        # Mission state
        # ----------------------------------------------------

        self.lock = threading.Lock()

        self.mission_active = False
        self.mission_finished = False

        self.current_payload = 0

        self.targets = []

        self.current_target = None

        self.current_target_index = -1

        self.telemetry = {}

        self.navigation_arrived = False
        self.hold_complete = False
        self.payload_complete = False

        self.worker_running = False

        self.get_logger().info(
            "Barbariq mission manager started"
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
                "MAVLink connected"
            )

        except Exception as e:

            self.get_logger().error(
                f"MAVLink connection failed: {e}"
            )

            self.master = None

    # ========================================================
    # TARGET RECEIVED
    #
    # Expected:
    #
    # target_id=1,lat=13.0311178,lon=77.5653903,alt_m=6.1
    #
    # ========================================================

    def target_callback(self, msg):

        data = self.parse_key_values(msg.data)

        if "lat" not in data or "lon" not in data:

            self.get_logger().warning(
                f"Invalid target: {msg.data}"
            )

            return

        try:

            target = {
                "target_id": int(
                    float(data.get("target_id", 0))
                ),

                "lat": float(data["lat"]),

                "lon": float(data["lon"]),

                "alt_m": float(
                    data.get("alt_m", 6.1)
                )
            }

        except ValueError as e:

            self.get_logger().error(
                f"Could not parse target: {e}"
            )

            return

        with self.lock:

            self.targets.append(target)

        self.get_logger().info(
            f"Target received: "
            f"ID={target['target_id']} "
            f"lat={target['lat']} "
            f"lon={target['lon']} "
            f"alt={target['alt_m']}"
        )

        self.publish_status(
            f"TARGET_RECEIVED,"
            f"id={target['target_id']}"
        )

        # ----------------------------------------------------
        # Start mission worker only once
        # ----------------------------------------------------

        with self.lock:

            if not self.worker_running:

                self.worker_running = True

                threading.Thread(
                    target=self.mission_worker,
                    daemon=True
                ).start()

    # ========================================================
    # TELEMETRY
    # ========================================================

    def telemetry_callback(self, msg):

        self.telemetry = self.parse_key_values(
            msg.data
        )

    # ========================================================
    # NAVIGATION STATUS
    # ========================================================

    def navigation_callback(self, msg):

        text = msg.data

        self.get_logger().info(
            f"Navigation: {text}"
        )

        if "ARRIVED" in text:

            self.navigation_arrived = True

    # ========================================================
    # POSITION ARRIVAL
    # ========================================================

    def arrival_callback(self, msg):

        text = msg.data

        self.get_logger().info(
            f"Position monitor: {text}"
        )

        if "ARRIVED" in text:

            self.navigation_arrived = True

    # ========================================================
    # HOLD STATUS
    # ========================================================

    def hold_callback(self, msg):

        text = msg.data

        self.get_logger().info(
            f"Hold: {text}"
        )

        if "HOLD_COMPLETE" in text:

            self.hold_complete = True

    # ========================================================
    # PAYLOAD STATUS
    # ========================================================

    def payload_callback(self, msg):

        text = msg.data

        self.get_logger().info(
            f"Payload: {text}"
        )

        if "DROP_COMPLETE" in text:

            self.payload_complete = True

    # ========================================================
    # MAIN MISSION WORKER
    # ========================================================

    def mission_worker(self):

        try:

            self.publish_status(
                "MISSION_WAITING"
            )

            # ------------------------------------------------
            # Wait for start conditions
            # ------------------------------------------------

            if not self.wait_for_start():

                self.get_logger().error(
                    "Start conditions not satisfied"
                )

                self.publish_status(
                    "MISSION_ABORTED,START_CONDITION"
                )

                return

            # ------------------------------------------------
            # Mission begins
            # ------------------------------------------------

            self.mission_active = True

            self.publish_status(
                "MISSION_STARTED"
            )

            # ------------------------------------------------
            # Process maximum 2 targets
            # ------------------------------------------------

            while self.current_payload < TOTAL_PAYLOADS:

                with self.lock:

                    available_targets = [
                        t for t in self.targets
                        if t not in self.targets_processed()
                    ]

                if not available_targets:

                    self.get_logger().info(
                        "Waiting for next target..."
                    )

                    time.sleep(0.5)

                    continue

                target = available_targets[0]

                self.current_target = target

                self.current_target_index += 1

                self.current_payload += 1

                payload_number = self.current_payload

                self.publish_status(
                    f"TARGET_SELECTED,"
                    f"id={target['target_id']},"
                    f"payload={payload_number}"
                )

                # ------------------------------------------------
                # Make sure we are in GUIDED
                # ------------------------------------------------

                if not self.set_guided_mode():

                    self.publish_status(
                        "MISSION_ABORTED,"
                        "GUIDED_MODE_FAILED"
                    )

                    return

                # ------------------------------------------------
                # Reset state
                # ------------------------------------------------

                self.navigation_arrived = False
                self.hold_complete = False
                self.payload_complete = False

                # ------------------------------------------------
                # Send target to navigation node
                # ------------------------------------------------

                self.send_navigation_target(
                    target
                )

                self.publish_status(
                    f"NAVIGATION_STARTED,"
                    f"target={target['target_id']}"
                )

                # ------------------------------------------------
                # Wait for arrival
                # ------------------------------------------------

                if not self.wait_for_arrival():

                    self.publish_status(
                        f"MISSION_ABORTED,"
                        f"NAVIGATION_TIMEOUT,"
                        f"target={target['target_id']}"
                    )

                    return

                # ------------------------------------------------
                # Wait for 20-second position hold
                # ------------------------------------------------

                self.publish_status(
                    f"HOLD_STARTED,"
                    f"target={target['target_id']}"
                )

                if not self.wait_for_hold():

                    self.publish_status(
                        f"MISSION_ABORTED,"
                        f"HOLD_FAILED,"
                        f"target={target['target_id']}"
                    )

                    return

                # ------------------------------------------------
                # Trigger payload
                # ------------------------------------------------

                self.trigger_payload(
                    payload_number
                )

                if not self.wait_for_payload():

                    self.publish_status(
                        f"MISSION_ABORTED,"
                        f"PAYLOAD_FAILED,"
                        f"payload={payload_number}"
                    )

                    return

                self.publish_status(
                    f"PAYLOAD_COMPLETE,"
                    f"payload={payload_number}"
                )

                # ------------------------------------------------
                # Small separation before next target
                # ------------------------------------------------

                time.sleep(1.0)

            # ----------------------------------------------------
            # Both payloads completed
            # ----------------------------------------------------

            self.publish_status(
                "ALL_PAYLOADS_COMPLETE"
            )

            # ----------------------------------------------------
            # RTL
            # ----------------------------------------------------

            self.set_rtl()

            self.publish_status(
                "RTL_STARTED"
            )

            self.mission_finished = True

            self.publish_status(
                "MISSION_COMPLETE"
            )

        except Exception as e:

            self.get_logger().error(
                f"Mission worker error: {e}"
            )

            self.publish_status(
                f"MISSION_ABORTED,error={e}"
            )

        finally:

            self.mission_active = False

            with self.lock:
                self.worker_running = False

    # ========================================================
    # START CONDITION
    #
    # Based on tested Barbariq wait_for_start():
    #
    # FC alive
    # armed
    # LOITER
    # altitude >= 15 ft
    #
    # ========================================================

    def wait_for_start(self):

        self.publish_status(
            "WAITING_FOR_START"
        )

        start_time = None

        while rclpy.ok():

            telemetry = self.telemetry

            if not telemetry:

                time.sleep(0.2)

                continue

            armed = self.parse_bool(
                telemetry.get("armed", "false")
            )

            mode = telemetry.get(
                "mode",
                ""
            ).upper()

            try:

                altitude = float(
                    telemetry.get(
                        "rel_alt_m",
                        0
                    )
                )

            except ValueError:

                altitude = 0.0

            # ------------------------------------------------
            # Exact start conditions
            # ------------------------------------------------

            ready = (
                armed
                and mode == "LOITER"
                and altitude >= MIN_START_ALT_M
            )

            if ready:

                if start_time is None:

                    start_time = time.time()

                    self.get_logger().info(
                        "Start conditions detected"
                    )

                # ------------------------------------------------
                # Same 5-second continuous condition
                # used by tested code
                # ------------------------------------------------

                if time.time() - start_time >= 5.0:

                    self.get_logger().info(
                        "Start conditions confirmed"
                    )

                    self.publish_status(
                        "START_CONDITION_CONFIRMED"
                    )

                    return True

            else:

                start_time = None

            time.sleep(0.1)

        return False

    # ========================================================
    # GUIDED MODE
    # ========================================================

    def set_guided_mode(self):

        if self.master is None:

            self.get_logger().error(
                "No MAVLink connection"
            )

            return False

        try:

            self.master.set_mode(
                "GUIDED"
            )

            self.get_logger().info(
                "GUIDED mode command sent"
            )

            self.publish_status(
                "GUIDED_MODE_REQUESTED"
            )

            # ------------------------------------------------
            # Wait until telemetry reports GUIDED
            # ------------------------------------------------

            timeout = time.time() + 10.0

            while time.time() < timeout:

                mode = self.telemetry.get(
                    "mode",
                    ""
                ).upper()

                if mode == "GUIDED":

                    self.get_logger().info(
                        "GUIDED mode confirmed"
                    )

                    self.publish_status(
                        "GUIDED_MODE_CONFIRMED"
                    )

                    return True

                time.sleep(0.1)

            self.get_logger().error(
                "GUIDED mode not confirmed"
            )

            return False

        except Exception as e:

            self.get_logger().error(
                f"Failed to set GUIDED: {e}"
            )

            return False

    # ========================================================
    # SEND TARGET TO NAVIGATION NODE
    # ========================================================

    def send_navigation_target(self, target):

        msg = String()

        msg.data = (
            f"target_id={target['target_id']},"
            f"lat={target['lat']},"
            f"lon={target['lon']},"
            f"alt_m={target['alt_m']}"
        )

        self.navigation_pub.publish(
            msg
        )

        self.get_logger().info(
            f"Navigation target sent: {msg.data}"
        )

    # ========================================================
    # WAIT FOR ARRIVAL
    # ========================================================

    def wait_for_arrival(self):

        start = time.time()

        while rclpy.ok():

            if self.navigation_arrived:

                self.get_logger().info(
                    "Target arrival confirmed"
                )

                return True

            if time.time() - start > NAVIGATION_TIMEOUT_S:

                self.get_logger().error(
                    "Navigation timeout"
                )

                return False

            time.sleep(0.1)

        return False

    # ========================================================
    # WAIT FOR HOLD
    # ========================================================

    def wait_for_hold(self):

        start = time.time()

        # ----------------------------------------------------
        # Allow extra time for the 20-second hold
        # ----------------------------------------------------

        timeout = HOLD_TIME_S + 30.0

        while rclpy.ok():

            if self.hold_complete:

                self.get_logger().info(
                    "20-second hold completed"
                )

                return True

            if time.time() - start > timeout:

                self.get_logger().error(
                    "Hold timeout"
                )

                return False

            time.sleep(0.1)

        return False

    # ========================================================
    # TRIGGER PAYLOAD
    # ========================================================

    def trigger_payload(self, payload_number):

        msg = String()

        msg.data = str(
            payload_number
        )

        self.payload_pub.publish(
            msg
        )

        self.get_logger().info(
            f"Payload {payload_number} trigger sent"
        )

        self.publish_status(
            f"PAYLOAD_TRIGGERED,"
            f"payload={payload_number}"
        )

    # ========================================================
    # WAIT FOR PAYLOAD
    # ========================================================

    def wait_for_payload(self):

        start = time.time()

        timeout = 15.0

        while rclpy.ok():

            if self.payload_complete:

                return True

            if time.time() - start > timeout:

                self.get_logger().error(
                    "Payload timeout"
                )

                return False

            time.sleep(0.1)

        return False

    # ========================================================
    # RTL
    # ========================================================

    def set_rtl(self):

        if self.master is None:

            self.get_logger().error(
                "No MAVLink connection for RTL"
            )

            return False

        try:

            self.master.set_mode(
                "RTL"
            )

            self.get_logger().info(
                "RTL mode command sent"
            )

            return True

        except Exception as e:

            self.get_logger().error(
                f"RTL failed: {e}"
            )

            return False

    # ========================================================
    # Keep track of processed targets
    # ========================================================

    def targets_processed(self):

        processed = []

        for i in range(
            min(
                self.current_target_index + 1,
                len(self.targets)
            )
        ):

            processed.append(
                self.targets[i]
            )

        return processed

    # ========================================================
    # Parse key=value strings
    # ========================================================

    @staticmethod
    def parse_key_values(text):

        result = {}

        for item in text.split(","):

            if "=" not in item:
                continue

            key, value = item.split(
                "=",
                1
            )

            result[key.strip()] = value.strip()

        return result

    # ========================================================
    # Boolean parser
    # ========================================================

    @staticmethod
    def parse_bool(value):

        return str(value).lower() in (
            "true",
            "1",
            "yes",
            "armed"
        )

    # ========================================================
    # Status publisher
    # ========================================================

    def publish_status(self, status):

        msg = String()

        msg.data = status

        self.status_pub.publish(
            msg
        )

        self.get_logger().info(
            f"MISSION: {status}"
        )

    # ========================================================
    # Shutdown
    # ========================================================

    def destroy_node(self):

        self.get_logger().info(
            "Stopping Barbariq mission manager"
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

    node = BarbariqMissionManager()

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
