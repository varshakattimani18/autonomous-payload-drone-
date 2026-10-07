#!/usr/bin/env python3

import json
import math
import os

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ============================================================
# CAMERA MODEL + GSD GEOLOCATION
#
# Extracted from tested Rudra det_geo.py
# ============================================================

IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720

FX = 724.967
FY = 725.028

CX = 626.364
CY = 352.733


DIST_COEFFS = np.array(
    [
        -0.12134,
         0.12149,
         0.00351,
        -0.00423,
        -0.00894
    ],
    dtype=np.float64
)


K = np.array(
    [
        [FX, 0.0, CX],
        [0.0, FY, CY],
        [0.0, 0.0, 1.0]
    ],
    dtype=np.float64
)


EARTH_RADIUS_M = 6_371_000.0


# ============================================================
# CALIBRATION
#
# Same parameter order/defaults as det_geo.py
# ============================================================

PARAM_NAMES = [
    "offset_forward_m",
    "offset_right_m",
    "yaw_offset_deg",
    "alt_scale",
    "roll_offset_deg",
    "pitch_offset_deg",
    "north_bias_m",
    "east_bias_m",
    "north_scale",
    "east_scale",
    "north_x_m",
    "north_y_m",
    "east_x_m",
    "east_y_m",
]


PARAM_DEFAULTS = np.array(
    [
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ],
    dtype=float
)


MAX_TILT_DEG = 30.0
MAX_TELEM_AGE_S = 0.5
MIN_GPS_FIX = 3


# ============================================================
# PIXEL UNDISTORTION
# ============================================================

def undistort_pixel(u, v):

    pixel = np.array(
        [
            [
                [
                    float(u),
                    float(v)
                ]
            ]
        ],
        dtype=np.float64
    )

    normalized = cv2.undistortPoints(
        pixel,
        K,
        DIST_COEFFS
    )

    return (
        float(normalized[0, 0][0]),
        float(normalized[0, 0][1])
    )


# ============================================================
# BODY -> NED ROTATION
#
# Same implementation as det_geo.py
# ============================================================

def rotation_ned_from_body(
    yaw_deg,
    pitch_deg,
    roll_deg
):

    y = math.radians(yaw_deg)
    p = math.radians(pitch_deg)
    r = math.radians(roll_deg)

    cy = math.cos(y)
    sy = math.sin(y)

    cp = math.cos(p)
    sp = math.sin(p)

    cr = math.cos(r)
    sr = math.sin(r)

    rz = np.array(
        [
            [cy, -sy, 0],
            [sy,  cy, 0],
            [0,    0, 1]
        ]
    )

    ry = np.array(
        [
            [ cp, 0, sp],
            [  0, 1,  0],
            [-sp, 0, cp]
        ]
    )

    rx = np.array(
        [
            [1,  0,   0],
            [0, cr, -sr],
            [0, sr,  cr]
        ]
    )

    return rz @ ry @ rx


# ============================================================
# PREDICT GROUND OFFSET
#
# Same 3D ray-cast logic as det_geo.py
# ============================================================

def predict_offset(
    x_norm,
    y_norm,
    alt_m,
    yaw_deg,
    roll_deg,
    pitch_deg,
    params,
    use_attitude=True
):

    (
        fwd,
        right,
        yaw_off,
        alt_scale,
        roll_off,
        pitch_off,
        n_bias,
        e_bias,
        n_scale,
        e_scale,
        nx,
        ny,
        ex,
        ey
    ) = params


    roll = (
        roll_deg
        if (
            use_attitude
            and roll_deg is not None
        )
        else 0.0
    ) + roll_off


    pitch = (
        pitch_deg
        if (
            use_attitude
            and pitch_deg is not None
        )
        else 0.0
    ) + pitch_off


    yaw = yaw_deg + yaw_off


    # Camera ray in body coordinates
    d_body = np.array(
        [
            -y_norm,
             x_norm,
             1.0
        ],
        dtype=float
    )


    # Convert body ray to NED
    d_ned = (
        rotation_ned_from_body(
            yaw,
            pitch,
            roll
        )
        @ d_body
    )


    # Ray must intersect ground
    if d_ned[2] <= 1e-6:

        raise ValueError(
            "Ray does not hit the ground "
            "(tilt too large)."
        )


    # Ground intersection
    t = (
        alt_m
        * alt_scale
        / d_ned[2]
    )


    north = (
        t * d_ned[0]
    )

    east = (
        t * d_ned[1]
    )


    # Camera mounting offsets
    yr = math.radians(yaw)

    north += (
        fwd * math.cos(yr)
        -
        right * math.sin(yr)
    )

    east += (
        fwd * math.sin(yr)
        +
        right * math.cos(yr)
    )


    # Calibration correction
    north = (
        north * n_scale
        + n_bias
        + nx * x_norm
        + ny * y_norm
    )

    east = (
        east * e_scale
        + e_bias
        + ex * x_norm
        + ey * y_norm
    )


    return (
        float(north),
        float(east)
    )


# ============================================================
# NORTH/EAST -> LAT/LON
# ============================================================

def offset_to_latlon(
    lat_deg,
    lon_deg,
    north_m,
    east_m
):

    cos_lat = math.cos(
        math.radians(lat_deg)
    )


    if abs(cos_lat) < 1e-12:

        raise ValueError(
            "Longitude conversion is unstable "
            "near the poles."
        )


    return (
        lat_deg
        +
        math.degrees(
            north_m
            / EARTH_RADIUS_M
        ),

        lon_deg
        +
        math.degrees(
            east_m
            /
            (
                EARTH_RADIUS_M
                * cos_lat
            )
        )
    )


# ============================================================
# CALIBRATION LOADER
# ============================================================

def load_calibration(path):

    if not path:

        return (
            PARAM_DEFAULTS.copy(),
            True,
            0
        )


    if not os.path.isfile(path):

        return (
            PARAM_DEFAULTS.copy(),
            True,
            0
        )


    with open(
        path,
        "r",
        encoding="utf-8"
    ) as f:

        calibration = json.load(f)


    stored = calibration.get(
        "params",
        {}
    )


    params = np.array(
        [
            float(
                stored.get(
                    name,
                    default
                )
            )

            for name, default
            in zip(
                PARAM_NAMES,
                PARAM_DEFAULTS
            )
        ],
        dtype=float
    )


    use_attitude = bool(
        calibration.get(
            "use_attitude",
            True
        )
    )


    samples = len(
        calibration.get(
            "samples",
            []
        )
    )


    return (
        params,
        use_attitude,
        samples
    )


# ============================================================
# GEOLOCATOR
# ============================================================

class Geolocator:

    def __init__(
        self,
        params,
        use_attitude
    ):

        self.params = params

        self.use_attitude = (
            use_attitude
        )


    def locate(
        self,
        u,
        v,
        frame_w,
        frame_h,
        telemetry
    ):

        # ----------------------------------------------------
        # ATTITUDE VALIDATION
        # ----------------------------------------------------

        if self.use_attitude:

            for name in (
                "roll",
                "pitch"
            ):

                value = telemetry.get(
                    name
                )

                if (
                    value is not None
                    and abs(value)
                    > MAX_TILT_DEG
                ):

                    raise ValueError(
                        f"|{name}|="
                        f"{abs(value):.1f} deg "
                        f"> MAX_TILT_DEG="
                        f"{MAX_TILT_DEG}"
                    )


        # ----------------------------------------------------
        # SCALE DETECTION PIXEL TO CAMERA RESOLUTION
        # ----------------------------------------------------

        su = (
            u
            * IMAGE_WIDTH
            / frame_w
        )

        sv = (
            v
            * IMAGE_HEIGHT
            / frame_h
        )


        # ----------------------------------------------------
        # REMOVE LENS DISTORTION
        # ----------------------------------------------------

        x_norm, y_norm = (
            undistort_pixel(
                su,
                sv
            )
        )


        # ----------------------------------------------------
        # 3D RAY CAST
        # ----------------------------------------------------

        north, east = predict_offset(
            x_norm,
            y_norm,
            telemetry["alt"],
            telemetry["yaw"],
            telemetry["roll"],
            telemetry["pitch"],
            self.params,
            self.use_attitude
        )


        # ----------------------------------------------------
        # GROUND POSITION
        # ----------------------------------------------------

        lat, lon = (
            offset_to_latlon(
                telemetry["lat"],
                telemetry["lon"],
                north,
                east
            )
        )


        return {

            "north": north,

            "east": east,

            "lat": lat,

            "lon": lon,

            "x_norm": x_norm,

            "y_norm": y_norm,

            "gsd": (
                telemetry["alt"]
                / FX
            ),

            "ground_range_m": math.hypot(
                north,
                east
            )

        }


# ============================================================
# TELEMETRY PARSER
#
# Matches the String format of your existing
# rudra_telemetry_node exactly.
# ============================================================

def parse_telemetry(text):

    result = {}

    for item in text.split(","):

        if "=" not in item:

            continue

        key, value = item.split(
            "=",
            1
        )

        key = key.strip()
        value = value.strip()

        if key == "hdg":

            if value == "None":

                result[key] = None

            else:

                result[key] = float(value)

        elif key == "valid":

            result[key] = (
                value.lower()
                == "true"
            )

        elif key in (
            "fix_type",
            "sats"
        ):

            result[key] = int(
                value
            )

        else:

            result[key] = float(
                value
            )


    return result


# ============================================================
# VALIDATE TELEMETRY
#
# Same validation logic as det_geo.py
# ============================================================

def validate_telemetry(
    state,
    video_latency
):

    problems = []
    warnings = []


    # --------------------------------------------------------
    # GPS AGE
    # --------------------------------------------------------

    if not state.get(
        "valid",
        False
    ):

        problems.append(
            "GPS sample is stale"
        )


    # --------------------------------------------------------
    # GPS POSITION
    # --------------------------------------------------------

    if (
        state.get("lat", 0.0) == 0.0
        and
        state.get("lon", 0.0) == 0.0
    ):

        problems.append(
            "lat/lon == (0,0): no GPS fix"
        )


    # --------------------------------------------------------
    # GPS FIX
    # --------------------------------------------------------

    if (
        state.get(
            "fix_type",
            0
        )
        < MIN_GPS_FIX
    ):

        problems.append(
            f"GPS fix_type="
            f"{state.get('fix_type', 0)} "
            f"< {MIN_GPS_FIX}"
        )


    # --------------------------------------------------------
    # ALTITUDE
    # --------------------------------------------------------

    if (
        state.get(
            "rel_alt_m",
            0.0
        )
        <= 0.5
    ):

        problems.append(
            "relative altitude "
            "too low"
        )


    # --------------------------------------------------------
    # ATTITUDE AGE
    # --------------------------------------------------------

    yaw = None
    roll = None
    pitch = None

    yaw_src = "none"


    if (
        state.get(
            "att_dt",
            999.0
        )
        <= MAX_TELEM_AGE_S
    ):

        yaw = state["yaw"]

        roll = state["roll"]

        pitch = state["pitch"]

        yaw_src = "ATTITUDE"


    else:

        warnings.append(
            "ATTITUDE sample is stale; "
            "falling back to GPS heading"
        )

        if state.get(
            "hdg"
        ) is not None:

            yaw = state["hdg"]

            yaw_src = (
                "GPS heading"
            )


    # --------------------------------------------------------
    # NO YAW
    # --------------------------------------------------------

    if yaw is None:

        problems.append(
            "no yaw available"
        )


    if problems:

        return (
            None,
            problems,
            warnings
        )


    # --------------------------------------------------------
    # TELEMETRY TIME SHIFT
    #
    # Same logic as det_geo.py
    # --------------------------------------------------------

    lat = state["lat"]

    lon = state["lon"]

    back = max(
        -1.0,
        min(
            1.0,
            state["gps_dt"]
            + video_latency
        )
    )


    # --------------------------------------------------------
    # COMPENSATE FOR DRONE MOTION
    # --------------------------------------------------------

    if (
        state["vn"] != 0.0
        or
        state["ve"] != 0.0
    ):

        lat, lon = (
            offset_to_latlon(
                lat,
                lon,
                -state["vn"] * back,
                -state["ve"] * back
            )
        )


    telemetry = {

        "lat": lat,

        "lon": lon,

        "alt": state[
            "rel_alt_m"
        ],

        "yaw": yaw,

        "roll": roll,

        "pitch": pitch,

        "yaw_src": yaw_src,

        "gps_dt": state[
            "gps_dt"
        ],

        "att_dt": state[
            "att_dt"
        ],

        "hdg": state.get(
            "hdg"
        ),

        "back_s": back

    }


    return (
        telemetry,
        problems,
        warnings
    )


# ============================================================
# ROS NODE
# ============================================================

class RudraGeotagNode(Node):

    def __init__(self):

        super().__init__(
            "rudra_geotag_node"
        )


        # ----------------------------------------------------
        # PARAMETERS
        # ----------------------------------------------------

        self.declare_parameter(
            "calibration_path",
            ""
        )

        self.declare_parameter(
            "use_attitude",
            True
        )

        self.declare_parameter(
            "video_latency",
            0.0
        )


        calibration_path = (
            self.get_parameter(
                "calibration_path"
            ).value
        )

        use_attitude = (
            self.get_parameter(
                "use_attitude"
            ).value
        )

        video_latency = (
            self.get_parameter(
                "video_latency"
            ).value
        )


        self.video_latency = float(
            video_latency
        )


        # ----------------------------------------------------
        # CALIBRATION
        # ----------------------------------------------------

        params, calibration_attitude, samples = (
            load_calibration(
                calibration_path
            )
        )


        # If calibration file exists and defines
        # use_attitude, preserve it.
        if calibration_path:

            use_attitude = (
                calibration_attitude
            )


        self.geolocator = Geolocator(
            params,
            bool(use_attitude)
        )


        self.get_logger().info(
            "Rudra geotag node started"
        )

        self.get_logger().info(
            f"Camera: "
            f"{IMAGE_WIDTH}x{IMAGE_HEIGHT}"
        )

        self.get_logger().info(
            f"FX={FX:.3f}, FY={FY:.3f}"
        )

        self.get_logger().info(
            f"MAX_TILT_DEG="
            f"{MAX_TILT_DEG}"
        )

        self.get_logger().info(
            f"MAX_TELEM_AGE_S="
            f"{MAX_TELEM_AGE_S}"
        )

        self.get_logger().info(
            f"Calibration samples="
            f"{samples}"
        )


        # ----------------------------------------------------
        # LATEST TELEMETRY
        # ----------------------------------------------------

        self.latest_telemetry = None


        # ----------------------------------------------------
        # SUBSCRIBER: TELEMETRY
        # ----------------------------------------------------

        self.telemetry_sub = (
            self.create_subscription(
                String,
                "/rudra/telemetry",
                self.telemetry_callback,
                10
            )
        )


        # ----------------------------------------------------
        # SUBSCRIBER: DETECTIONS
        # ----------------------------------------------------

        self.detection_sub = (
            self.create_subscription(
                String,
                "/rudra/detections",
                self.detection_callback,
                10
            )
        )


        # ----------------------------------------------------
        # PUBLISHER
        # ----------------------------------------------------

        self.geotag_pub = (
            self.create_publisher(
                String,
                "/rudra/geotagged_detections",
                10
            )
        )


        self.get_logger().info(
            "Subscribed: /rudra/telemetry"
        )

        self.get_logger().info(
            "Subscribed: /rudra/detections"
        )

        self.get_logger().info(
            "Publishing: "
            "/rudra/geotagged_detections"
        )


    # ========================================================
    # TELEMETRY CALLBACK
    # ========================================================

    def telemetry_callback(
        self,
        msg
    ):

        try:

            self.latest_telemetry = (
                parse_telemetry(
                    msg.data
                )
            )

        except Exception as exc:

            self.get_logger().error(
                f"Telemetry parse error: "
                f"{exc}"
            )


    # ========================================================
    # DETECTION CALLBACK
    # ========================================================

    def detection_callback(
        self,
        msg
    ):

        # ----------------------------------------------------
        # Need telemetry first
        # ----------------------------------------------------

        if self.latest_telemetry is None:

            self.get_logger().warn(
                "No telemetry received yet"
            )

            return


        try:

            data = json.loads(
                msg.data
            )

        except Exception as exc:

            self.get_logger().error(
                f"Detection JSON error: "
                f"{exc}"
            )

            return


        detections = data.get(
            "detections",
            []
        )


        # ----------------------------------------------------
        # VALIDATE TELEMETRY
        # ----------------------------------------------------

        telemetry, problems, warnings = (
            validate_telemetry(
                self.latest_telemetry,
                self.video_latency
            )
        )


        if telemetry is None:

            self.get_logger().warn(
                "Geotag skipped: "
                + "; ".join(problems)
            )

            return


        for warning in warnings:

            self.get_logger().warn(
                warning
            )


        # ----------------------------------------------------
        # FRAME DIMENSIONS
        # ----------------------------------------------------

        frame_w = int(
            data.get(
                "frame_width",
                IMAGE_WIDTH
            )
        )

        frame_h = int(
            data.get(
                "frame_height",
                IMAGE_HEIGHT
            )
        )


        # ----------------------------------------------------
        # GEOTAG EACH DETECTION
        # ----------------------------------------------------

        valid_detections = []


        for detection in detections:

            try:

                u = float(
                    detection["u"]
                )

                v = float(
                    detection["v"]
                )


                geo = (
                    self.geolocator.locate(
                        u,
                        v,
                        frame_w,
                        frame_h,
                        telemetry
                    )
                )


                detection["geo"] = geo

                valid_detections.append(
                    detection
                )


                self.get_logger().info(
                    f"GEOTAG | "
                    f"u={u:.1f} "
                    f"v={v:.1f} | "
                    f"LAT={geo['lat']:.7f} | "
                    f"LON={geo['lon']:.7f} | "
                    f"range="
                    f"{geo['ground_range_m']:.2f} m"
                )


            except Exception as exc:

                self.get_logger().warn(
                    f"Geotag failed for "
                    f"detection: {exc}"
                )


        # ----------------------------------------------------
        # OUTPUT
        # ----------------------------------------------------

        result = {

            "frame_id": data.get(
                "frame_id",
                ""
            ),

            "frame_width": frame_w,

            "frame_height": frame_h,

            "telemetry": telemetry,

            "detections": (
                valid_detections
            )

        }


        output = String()

        output.data = json.dumps(
            result
        )


        self.geotag_pub.publish(
            output
        )


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = RudraGeotagNode()


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
