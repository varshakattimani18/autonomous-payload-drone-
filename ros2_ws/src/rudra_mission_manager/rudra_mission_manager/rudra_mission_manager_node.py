#!/usr/bin/env python3

import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ============================================================
# Rudra mission configuration
# ============================================================

DETECTION_ALTITUDE_FT = 85.0
DETECTION_ALTITUDE_M = DETECTION_ALTITUDE_FT * 0.3048

TOTAL_SURVIVORS = 10

BARBARIQ_TARGET_COUNT = 2
RUDRA_TARGET_START = 3

TOTAL_RUDRA_TARGETS = 8


class RudraMissionManager(Node):

    def __init__(self):

        super().__init__("rudra_mission_manager")

        # ====================================================
        # Subscribers
        # ====================================================

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

        # ====================================================
        # Publishers
        # ====================================================

        self.mission_status_pub = self.create_publisher(
            String,
            "/rudra/mission/status",
            10
        )

        self.mission_log_pub = self.create_publisher(
            String,
            "/rudra/mission/log",
            10
        )

        # ====================================================
        # Mission state
        # ====================================================

        self.state_lock = threading.Lock()

        self.mission_state = "WAITING"

        self.telemetry = {}

        self.latest_detections = []
        self.latest_geotags = []
        self.latest_tracks = []
        self.latest_targets = []

        self.detected_survivor_ids = set()

        self.barbariq_targets = set()
        self.rudra_targets = set()

        self.barbariq_complete = False

        self.detection_active = False

        self.mission_complete = False

        self.manager_running = True

        # ====================================================
        # Mission worker
        # ====================================================

        self.worker = threading.Thread(
            target=self.mission_worker,
            daemon=True
        )

        self.worker.start()

        self.get_logger().info(
            "Rudra mission manager started"
        )

    # ========================================================
    # Telemetry callback
    # ========================================================

    def telemetry_callback(self, msg):

        self.telemetry = self.parse_key_values(
            msg.data
        )

    # ========================================================
    # Detection callback
    # ========================================================

    def detection_callback(self, msg):

        self.latest_detections = self.parse_detection_message(
            msg.data
        )

    # ========================================================
    # Geotag callback
    # ========================================================

    def geotag_callback(self, msg):

        self.latest_geotags = self.parse_detection_message(
            msg.data
        )

    # ========================================================
    # Tracking callback
    # ========================================================

    def tracking_callback(self, msg):

        self.latest_tracks = self.parse_detection_message(
            msg.data
        )

        # ----------------------------------------------------
        # Extract survivor IDs where possible
        # ----------------------------------------------------

        for detection in self.latest_tracks:

            if "person_no" in detection:

                try:

                    person_id = int(
                        float(
                            detection["person_no"]
                        )
                    )

                    self.detected_survivor_ids.add(
                        person_id
                    )

                except ValueError:

                    pass

    # ========================================================
    # Survivor manager callback
    # ========================================================

    def survivor_callback(self, msg):

        text = msg.data

        self.get_logger().info(
            f"Survivor manager: {text}"
        )

        targets = self.parse_detection_message(
            text
        )

        self.latest_targets = targets

        # ----------------------------------------------------
        # Determine assignment if person ID is available
        # ----------------------------------------------------

        for target in targets:

            if "person_no" not in target:
                continue

            try:

                person_id = int(
                    float(
                        target["person_no"]
                    )
                )

            except ValueError:

                continue

            self.detected_survivor_ids.add(
                person_id
            )

            if person_id <= BARBARIQ_TARGET_COUNT:

                self.barbariq_targets.add(
                    person_id
                )

            else:

                self.rudra_targets.add(
                    person_id
                )

        self.publish_status(
            f"SURVIVOR_TARGET_UPDATE,"
            f"detected={len(self.detected_survivor_ids)},"
            f"barbariq={len(self.barbariq_targets)},"
            f"rudra={len(self.rudra_targets)}"
        )

    # ========================================================
    # Main mission worker
    # ========================================================

    def mission_worker(self):

        try:

            # ------------------------------------------------
            # State 1
            # ------------------------------------------------

            self.set_state(
                "WAITING"
            )

            self.publish_log(
                "Rudra mission manager waiting for telemetry"
            )

            # ------------------------------------------------
            # Wait until FC telemetry is available
            # ------------------------------------------------

            while self.manager_running:

                if self.telemetry_available():

                    break

                time.sleep(0.5)

            if not self.manager_running:
                return

            # ------------------------------------------------
            # State 2
            # ------------------------------------------------

            self.set_state(
                "TAKEOFF_AUTO"
            )

            self.publish_log(
                "Rudra telemetry available; "
                "mission expects autonomous AUTO takeoff"
            )

            # ------------------------------------------------
            # Wait for aircraft to climb
            # ------------------------------------------------

            self.set_state(
                "ASCENDING"
            )

            self.publish_log(
                f"Waiting for detection altitude "
                f"{DETECTION_ALTITUDE_FT:.1f} ft"
            )

            while self.manager_running:

                altitude = self.get_relative_altitude()

                if altitude >= DETECTION_ALTITUDE_M:

                    break

                time.sleep(0.2)

            if not self.manager_running:
                return

            # ------------------------------------------------
            # Detection becomes active
            #
            # The actual altitude gate remains in the tested
            # detection pipeline. This manager only observes
            # the mission altitude.
            # ------------------------------------------------

            self.detection_active = True

            self.set_state(
                "DETECTION_ACTIVE"
            )

            self.publish_log(
                f"Rudra crossed "
                f"{DETECTION_ALTITUDE_FT:.1f} ft; "
                "detection mission phase active"
            )

            # ------------------------------------------------
            # Wait until survivor assignments are produced
            # ------------------------------------------------

            self.set_state(
                "TARGET_ASSIGNMENT"
            )

            self.publish_log(
                "Waiting for survivor assignments"
            )

            while self.manager_running:

                if len(self.barbariq_targets) >= 2:

                    break

                time.sleep(0.5)

            if not self.manager_running:
                return

            # ------------------------------------------------
            # First two targets belong to Barbariq
            # ------------------------------------------------

            self.publish_status(
                f"BARBARIQ_TARGETS_READY,"
                f"count={len(self.barbariq_targets)}"
            )

            self.publish_log(
                "First two survivor assignments prepared "
                "for Barbariq"
            )

            # ------------------------------------------------
            # IMPORTANT:
            #
            # Actual transmission to Barbariq is intentionally
            # NOT done here.
            #
            # That belongs to rudra_inter_drone after custom
            # MAVLink messages are ready.
            # ------------------------------------------------

            self.set_state(
                "WAIT_FOR_BARBARIIQ"
            )

            self.publish_log(
                "Waiting for Barbariq mission completion"
            )

            # ------------------------------------------------
            # Wait for future mission-complete message
            # ------------------------------------------------

            while self.manager_running:

                if self.barbariq_complete:

                    break

                time.sleep(0.5)

            if not self.manager_running:
                return

            # ------------------------------------------------
            # Rudra resumes survivors 3-10
            # ------------------------------------------------

            self.set_state(
                "RESUME_RUDRA"
            )

            self.publish_log(
                "Barbariq complete; "
                "Rudra resumes survivors 3-10"
            )

            # ------------------------------------------------
            # Wait until all remaining Rudra targets are known
            # ------------------------------------------------

            while self.manager_running:

                if len(self.rudra_targets) >= TOTAL_RUDRA_TARGETS:

                    break

                time.sleep(0.5)

            if not self.manager_running:
                return

            self.publish_status(
                f"RUDRA_TARGETS_READY,"
                f"count={len(self.rudra_targets)}"
            )

            # ------------------------------------------------
            # Actual target execution remains with the
            # existing/tested flight pipeline and future
            # mission integration.
            # ------------------------------------------------

            self.publish_log(
                "Eight Rudra survivor assignments ready"
            )

            # ------------------------------------------------
            # Final mission state
            #
            # Completion will eventually be tied to the
            # actual payload/flight execution.
            # ------------------------------------------------

            self.set_state(
                "MISSION_COMPLETE"
            )

            self.mission_complete = True

            self.publish_log(
                "Rudra mission manager reached "
                "final mission state"
            )

        except Exception as e:

            self.get_logger().error(
                f"Mission manager error: {e}"
            )

            self.set_state(
                "ERROR"
            )

            self.publish_log(
                f"Mission manager error: {e}"
            )

    # ========================================================
    # Telemetry availability
    # ========================================================

    def telemetry_available(self):

        if not self.telemetry:

            return False

        valid = self.telemetry.get(
            "valid",
            "false"
        )

        return self.parse_bool(
            valid
        )

    # ========================================================
    # Get relative altitude
    # ========================================================

    def get_relative_altitude(self):

        try:

            return float(
                self.telemetry.get(
                    "rel_alt_m",
                    0.0
                )
            )

        except ValueError:

            return 0.0

    # ========================================================
    # Mission state
    # ========================================================

    def set_state(self, state):

        with self.state_lock:

            self.mission_state = state

        self.publish_status(
            f"STATE={state}"
        )

    # ========================================================
    # Publish mission status
    # ========================================================

    def publish_status(self, status):

        msg = String()

        msg.data = status

        self.mission_status_pub.publish(
            msg
        )

        self.get_logger().info(
            f"MISSION: {status}"
        )

    # ========================================================
    # Publish mission log
    # ========================================================

    def publish_log(self, text):

        msg = String()

        msg.data = (
            f"[RUDRA] {text}"
        )

        self.mission_log_pub.publish(
            msg
        )

        self.get_logger().info(
            text
        )

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
    # Parse detection-style strings
    # ========================================================

    @staticmethod
    def parse_detection_message(text):

        results = []

        # ----------------------------------------------------
        # Support multiple records separated by ';'
        # ----------------------------------------------------

        records = text.split(";")

        for record in records:

            record = record.strip()

            if not record:

                continue

            data = {}

            for item in record.split(","):

                if "=" not in item:

                    continue

                key, value = item.split(
                    "=",
                    1
                )

                data[key.strip()] = value.strip()

            if data:

                results.append(
                    data
                )

        return results

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
    # External completion hook
    #
    # Later the custom inter-drone node can call this through
    # a proper ROS interface/topic.
    # ========================================================

    def mark_barbariq_complete(self):

        self.barbariq_complete = True

        self.publish_status(
            "BARBARIQ_MISSION_COMPLETE"
        )

    # ========================================================
    # Shutdown
    # ========================================================

    def destroy_node(self):

        self.manager_running = False

        self.get_logger().info(
            "Stopping Rudra mission manager"
        )

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = RudraMissionManager()

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
