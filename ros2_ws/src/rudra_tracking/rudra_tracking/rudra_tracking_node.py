#!/usr/bin/env python3

import json
import math

import numpy as np
from scipy.optimize import linear_sum_assignment

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


# ============================================================
# CONFIGURATION
# ============================================================

ASSOC_RADIUS_M = 6.0

EARTH_RADIUS_M = 6371000.0


# ============================================================
# HAVERSINE
# ============================================================

def haversine_m(lat1, lon1, lat2, lon2):

    p1 = math.radians(lat1)
    p2 = math.radians(lat2)

    dp = math.radians(
        lat2 - lat1
    )

    dl = math.radians(
        lon2 - lon1
    )

    a = (
        math.sin(dp / 2.0) ** 2
        +
        math.cos(p1)
        * math.cos(p2)
        * math.sin(dl / 2.0) ** 2
    )

    a = min(
        1.0,
        max(0.0, a)
    )

    return (
        2.0
        * EARTH_RADIUS_M
        * math.atan2(
            math.sqrt(a),
            math.sqrt(1.0 - a)
        )
    )


# ============================================================
# PERSON REGISTRY
# ============================================================

class PersonRegistry:

    def __init__(self, assoc_radius_m):

        self.persons = {}

        self.counter = 0

        self.assoc_radius_m = (
            assoc_radius_m
        )

    # --------------------------------------------------------
    # CREATE NEW PERSON
    # --------------------------------------------------------

    def _create(
        self,
        frame,
        lat,
        lon
    ):

        self.counter += 1

        no = self.counter

        self.persons[no] = {

            "samples": [],

            "first_frame": frame,

            "last_frame": frame,

            "last_lat": lat,

            "last_lon": lon,

            "sightings": 0,

            "best_conf": 0.0,

            "best_lat": None,

            "best_lon": None,

        }

        return no

    # --------------------------------------------------------
    # HUNGARIAN ASSIGNMENT
    # --------------------------------------------------------

    def assign(
        self,
        detections,
        frame
    ):

        if not detections:

            return

        # ----------------------------------------------------
        # FIRST FRAME / NO EXISTING PERSONS
        # ----------------------------------------------------

        if not self.persons:

            for detection in detections:

                geo = detection["geo"]

                person_no = self._create(
                    frame,
                    geo["lat"],
                    geo["lon"]
                )

                detection["person_no"] = (
                    person_no
                )

                detection["is_new"] = True

            return

        # ----------------------------------------------------
        # EXISTING PERSON IDs
        # ----------------------------------------------------

        person_ids = list(
            self.persons.keys()
        )

        # ----------------------------------------------------
        # COST MATRIX
        #
        # rows    = existing persons
        # columns = current detections
        #
        # cost = geographic distance
        # ----------------------------------------------------

        cost_matrix = np.zeros(
            (
                len(person_ids),
                len(detections)
            ),
            dtype=float
        )

        for r, person_id in enumerate(
            person_ids
        ):

            person = self.persons[
                person_id
            ]

            for c, detection in enumerate(
                detections
            ):

                geo = detection["geo"]

                cost_matrix[r, c] = (
                    haversine_m(
                        geo["lat"],
                        geo["lon"],
                        person["last_lat"],
                        person["last_lon"]
                    )
                )

        # ----------------------------------------------------
        # HUNGARIAN ALGORITHM
        # ----------------------------------------------------

        row_ind, col_ind = (
            linear_sum_assignment(
                cost_matrix
            )
        )

        assigned = set()

        # ----------------------------------------------------
        # ACCEPT ONLY MATCHES <= 6 METERS
        # ----------------------------------------------------

        for r, c in zip(
            row_ind,
            col_ind
        ):

            if (
                cost_matrix[r, c]
                <= self.assoc_radius_m
            ):

                detection = detections[c]

                detection["person_no"] = (
                    person_ids[r]
                )

                detection["is_new"] = False

                assigned.add(c)

        # ----------------------------------------------------
        # UNMATCHED DETECTIONS
        # -> NEW PERSON
        # ----------------------------------------------------

        for c, detection in enumerate(
            detections
        ):

            if c not in assigned:

                geo = detection["geo"]

                person_no = self._create(
                    frame,
                    geo["lat"],
                    geo["lon"]
                )

                detection["person_no"] = (
                    person_no
                )

                detection["is_new"] = True

    # --------------------------------------------------------
    # ADD CURRENT OBSERVATION
    # --------------------------------------------------------

    def add_sample(
        self,
        person_no,
        frame,
        confidence,
        geo
    ):

        person = self.persons[
            person_no
        ]

        person["samples"].append(
            (
                geo["lat"],
                geo["lon"]
            )
        )

        person["last_frame"] = frame

        person["last_lat"] = (
            geo["lat"]
        )

        person["last_lon"] = (
            geo["lon"]
        )

        person["sightings"] += 1

        if confidence > person[
            "best_conf"
        ]:

            person["best_conf"] = (
                confidence
            )

            person["best_lat"] = (
                geo["lat"]
            )

            person["best_lon"] = (
                geo["lon"]
            )


# ============================================================
# ROS NODE
# ============================================================

class RudraTrackingNode(Node):

    def __init__(self):

        super().__init__(
            "rudra_tracking_node"
        )

        self.get_logger().info(
            "Starting Rudra tracking node"
        )

        self.get_logger().info(
            f"Association radius: "
            f"{ASSOC_RADIUS_M} m"
        )

        # ----------------------------------------------------
        # PERSON REGISTRY
        # ----------------------------------------------------

        self.registry = PersonRegistry(
            ASSOC_RADIUS_M
        )

        self.frame_count = 0

        # ----------------------------------------------------
        # SUBSCRIBER
        #
        # Future geotag node publishes:
        #
        # /rudra/geotagged_detections
        # ----------------------------------------------------

        self.subscription = (
            self.create_subscription(
                String,
                "/rudra/geotagged_detections",
                self.detection_callback,
                10
            )
        )

        # ----------------------------------------------------
        # PUBLISHER
        # ----------------------------------------------------

        self.publisher = (
            self.create_publisher(
                String,
                "/rudra/tracked_detections",
                10
            )
        )

        self.get_logger().info(
            "Subscribed: "
            "/rudra/geotagged_detections"
        )

        self.get_logger().info(
            "Publishing: "
            "/rudra/tracked_detections"
        )

    # --------------------------------------------------------
    # CALLBACK
    # --------------------------------------------------------

    def detection_callback(
        self,
        msg
    ):

        try:

            data = json.loads(
                msg.data
            )

            detections = data.get(
                "detections",
                []
            )

            self.frame_count += 1

            # ------------------------------------------------
            # HUNGARIAN TRACKING
            # ------------------------------------------------

            self.registry.assign(
                detections,
                self.frame_count
            )

            # ------------------------------------------------
            # UPDATE PERSON REGISTRY
            # ------------------------------------------------

            for detection in detections:

                person_no = detection.get(
                    "person_no"
                )

                geo = detection.get(
                    "geo"
                )

                if (
                    person_no is None
                    or geo is None
                ):
                    continue

                self.registry.add_sample(
                    person_no,
                    self.frame_count,
                    float(
                        detection.get(
                            "confidence",
                            0.0
                        )
                    ),
                    geo
                )

            # ------------------------------------------------
            # OUTPUT
            # ------------------------------------------------

            result = {

                "frame_id": data.get(
                    "frame_id",
                    ""
                ),

                "frame": self.frame_count,

                "count": len(
                    detections
                ),

                "persons_seen": len(
                    self.registry.persons
                ),

                "detections": detections,

            }

            output = String()

            output.data = json.dumps(
                result
            )

            self.publisher.publish(
                output
            )

            # ------------------------------------------------
            # LOG
            # ------------------------------------------------

            for detection in detections:

                person_no = detection.get(
                    "person_no"
                )

                confidence = detection.get(
                    "confidence",
                    0.0
                )

                is_new = detection.get(
                    "is_new",
                    False
                )

                geo = detection.get(
                    "geo"
                )

                if geo:

                    self.get_logger().info(
                        f"PERSON #{person_no} | "
                        f"LAT={geo['lat']:.7f} | "
                        f"LON={geo['lon']:.7f} | "
                        f"CONF={confidence:.3f} | "
                        f"NEW={is_new}"
                    )

        except Exception as e:

            self.get_logger().error(
                f"Tracking error: {e}"
            )

    # --------------------------------------------------------
    # CLEANUP
    # --------------------------------------------------------

    def destroy_node(self):

        self.registry.persons.clear()

        super().destroy_node()


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = None

    try:

        node = RudraTrackingNode()

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    except Exception as e:

        print(
            f"[FATAL] {e}"
        )

    finally:

        if node is not None:

            node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
