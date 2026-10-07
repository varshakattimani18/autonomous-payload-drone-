#!/usr/bin/env python3

import json

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


# ============================================================
# CONFIG
# ============================================================

# Same 5-frame decision window from det_geo.py
TX_WINDOW_FRAMES = 5

# Mission requirement
BARBARIQ_TARGET_COUNT = 2

TOTAL_SURVIVORS = 10


# ============================================================
# SURVIVOR MANAGER
# ============================================================

class SurvivorManager:

    def __init__(self):

        self.persons = {}

        self.sent_persons = set()

        self.frame_count = 0

        self.next_target_index = 1


    # ========================================================
    # UPDATE PERSON INFORMATION
    # ========================================================

    def update_persons(
        self,
        detections
    ):

        for detection in detections:

            person_no = detection.get(
                "person_no"
            )

            if person_no is None:
                continue

            geo = detection.get(
                "geo"
            )

            if geo is None:
                continue

            confidence = float(
                detection.get(
                    "confidence",
                    0.0
                )
            )

            person_no = int(
                person_no
            )


            # ------------------------------------------------
            # Create person record
            # ------------------------------------------------

            if person_no not in self.persons:

                self.persons[person_no] = {

                    "person": person_no,

                    "lat": geo["lat"],

                    "lon": geo["lon"],

                    "best_conf": confidence,

                    "sightings": 1,

                    "assigned_to": None,

                    "sent": False

                }

            else:

                person = self.persons[
                    person_no
                ]

                person["lat"] = (
                    geo["lat"]
                )

                person["lon"] = (
                    geo["lon"]
                )

                person["sightings"] += 1


                if confidence > person[
                    "best_conf"
                ]:

                    person["best_conf"] = (
                        confidence
                    )


    # ========================================================
    # SELECT NEXT UNSENT PERSON
    #
    # Same selection principle as det_geo.py:
    #
    # highest confidence among UNSENT targets
    # ========================================================

    def select_next_target(self):

        candidates = []


        for person_no, person in (
            self.persons.items()
        ):

            if person["sent"]:
                continue

            if person_no in self.sent_persons:
                continue

            candidates.append(
                person
            )


        if not candidates:

            return None


        return max(
            candidates,
            key=lambda p: p["best_conf"]
        )


    # ========================================================
    # ASSIGN TARGET
    # ========================================================

    def assign_target(
        self,
        person
    ):

        person_no = person[
            "person"
        ]


        # ----------------------------------------------------
        # FIRST TWO -> BARBARIQ
        # ----------------------------------------------------

        if (
            person_no <=
            BARBARIQ_TARGET_COUNT
        ):

            assigned_to = (
                "BARBARIQ"
            )

        # ----------------------------------------------------
        # REMAINING -> RUDRA
        # ----------------------------------------------------

        else:

            assigned_to = "RUDRA"


        person["assigned_to"] = (
            assigned_to
        )

        person["sent"] = True

        self.sent_persons.add(
            person_no
        )


        return {

            "person": person_no,

            "lat": person["lat"],

            "lon": person["lon"],

            "confidence": person[
                "best_conf"
            ],

            "assigned_to": assigned_to,

            "sent": True

        }


# ============================================================
# ROS NODE
# ============================================================

class RudraSurvivorManagerNode(
    Node
):

    def __init__(self):

        super().__init__(
            "rudra_survivor_manager_node"
        )


        # ----------------------------------------------------
        # MANAGER
        # ----------------------------------------------------

        self.manager = (
            SurvivorManager()
        )


        # ----------------------------------------------------
        # SUBSCRIBER
        # ----------------------------------------------------

        self.tracking_sub = (
            self.create_subscription(
                String,
                "/rudra/tracked_detections",
                self.tracking_callback,
                10
            )
        )


        # ----------------------------------------------------
        # PUBLISHER
        # ----------------------------------------------------

        self.target_pub = (
            self.create_publisher(
                String,
                "/rudra/survivor_targets",
                10
            )
        )


        self.get_logger().info(
            "rudra_survivor_manager_node started"
        )

        self.get_logger().info(
            "Subscribed: "
            "/rudra/tracked_detections"
        )

        self.get_logger().info(
            "Publishing: "
            "/rudra/survivor_targets"
        )

        self.get_logger().info(
            "Target allocation: "
            "Persons 1-2 -> BARBARIQ | "
            "Persons 3-10 -> RUDRA"
        )

        self.get_logger().info(
            f"Decision window: "
            f"{TX_WINDOW_FRAMES} processed frames"
        )


    # ========================================================
    # TRACKING CALLBACK
    # ========================================================

    def tracking_callback(
        self,
        msg
    ):

        try:

            data = json.loads(
                msg.data
            )

        except Exception as exc:

            self.get_logger().error(
                f"Tracking JSON error: "
                f"{exc}"
            )

            return


        detections = data.get(
            "detections",
            []
        )


        self.manager.frame_count += 1


        # ----------------------------------------------------
        # UPDATE PERSON REGISTRY
        # ----------------------------------------------------

        self.manager.update_persons(
            detections
        )


        # ----------------------------------------------------
        # SAME 5-FRAME WINDOW
        #
        # Only make a target decision every
        # fifth processed frame.
        # ----------------------------------------------------

        send_window_hit = (
            self.manager.frame_count
            % TX_WINDOW_FRAMES
            == 0
        )


        if not send_window_hit:

            return


        # ----------------------------------------------------
        # SELECT HIGHEST-CONFIDENCE
        # UNSENT SURVIVOR
        # ----------------------------------------------------

        target = (
            self.manager.select_next_target()
        )


        if target is None:

            self.get_logger().info(
                "5-frame window: "
                "no unsent survivor available"
            )

            return


        # ----------------------------------------------------
        # ASSIGN
        # ----------------------------------------------------

        assignment = (
            self.manager.assign_target(
                target
            )
        )


        # ----------------------------------------------------
        # PUBLISH TARGET
        # ----------------------------------------------------

        output = {

            "mission_type": (
                "SURVIVOR_TARGET"
            ),

            "target": assignment,

            "total_known_survivors": (
                len(
                    self.manager.persons
                )
            ),

            "sent_survivors": (
                len(
                    self.manager.sent_persons
                )
            )

        }


        ros_msg = String()

        ros_msg.data = json.dumps(
            output
        )


        self.target_pub.publish(
            ros_msg
        )


        # ----------------------------------------------------
        # LOG
        # ----------------------------------------------------

        self.get_logger().info(
            f"TARGET SELECTED | "
            f"Person #{assignment['person']} | "
            f"LAT={assignment['lat']:.7f} | "
            f"LON={assignment['lon']:.7f} | "
            f"CONF={assignment['confidence']:.3f} | "
            f"DESTINATION="
            f"{assignment['assigned_to']}"
        )


        # ----------------------------------------------------
        # MISSION STATUS
        # ----------------------------------------------------

        sent_count = len(
            self.manager.sent_persons
        )


        if sent_count >= TOTAL_SURVIVORS:

            self.get_logger().info(
                "ALL 10 SURVIVORS "
                "HAVE BEEN ASSIGNED"
            )


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = (
        RudraSurvivorManagerNode()
    )


    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
