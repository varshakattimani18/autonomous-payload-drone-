#!/usr/bin/env python3

import threading
import time

import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# ============================================================
# CAMERA SETTINGS
# Taken from the tested Rudra det_geo.py pipeline
# ============================================================

DEFAULT_RTSP_URL = "rtsp://192.168.144.25:8554/main.264"

CAMERA_FPS = 1.0
FRAME_PERIOD_S = 1.0

CAM_OPEN_TIMEOUT_MS = 5000
CAM_READ_TIMEOUT_MS = 5000

CAM_MAX_READ_FAILURES = 20
CAM_STALL_S = 10.0


# ============================================================
# LOGGING
# ============================================================

def log(tag, message):
    print(f"[{tag}] {message}", flush=True)


# ============================================================
# CAMERA HELPERS
# ============================================================

def cv2_has_ffmpeg():
    """
    Same basic FFmpeg availability check as the original pipeline.
    """
    try:
        for line in cv2.getBuildInformation().splitlines():
            if line.strip().startswith("FFMPEG:"):
                return "YES" in line.upper()
    except Exception:
        pass

    return False


def fourcc_str(value):
    value = int(value)

    if value <= 0:
        return "?"

    return "".join(
        chr((value >> (8 * i)) & 0xFF)
        for i in range(4)
    )


# ============================================================
# FRAME GRABBER
#
# Extracted from the tested det_geo.py FrameGrabber.
#
# Important:
# - grab() continuously decodes the RTSP stream
# - retrieve() only happens when a 1 FPS frame is due
# - only newest paced frame is stored
# - automatic reconnect
# ============================================================

class FrameGrabber(threading.Thread):

    def __init__(self, url, period_s):

        super().__init__(
            daemon=True,
            name="cam-grabber"
        )

        self.url = url
        self.period_s = period_s

        self.cond = threading.Condition()

        self.frame = None
        self.frame_t = 0.0
        self.frame_seq = 0

        self.connected = False

        self.decoded = 0
        self.reconnects = 0

        self.stop_event = threading.Event()

        self.cap = None

    # --------------------------------------------------------
    # STOP
    # --------------------------------------------------------

    def stop(self):

        self.stop_event.set()

        with self.cond:
            self.cond.notify_all()

    # --------------------------------------------------------
    # OPEN RTSP
    # --------------------------------------------------------

    def _open(self):

        params = []

        if (
            hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC")
            and hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC")
        ):

            params = [
                cv2.CAP_PROP_OPEN_TIMEOUT_MSEC,
                CAM_OPEN_TIMEOUT_MS,

                cv2.CAP_PROP_READ_TIMEOUT_MSEC,
                CAM_READ_TIMEOUT_MS
            ]

        try:

            if params:

                cap = cv2.VideoCapture(
                    self.url,
                    cv2.CAP_FFMPEG,
                    params
                )

            else:

                cap = cv2.VideoCapture(
                    self.url,
                    cv2.CAP_FFMPEG
                )

        except TypeError:

            cap = cv2.VideoCapture(
                self.url,
                cv2.CAP_FFMPEG
            )

        if not cap.isOpened():

            cap.release()

            raise RuntimeError(
                "cv2.VideoCapture(CAP_FFMPEG) "
                "could not open RTSP stream"
            )

        width = int(
            cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        )

        height = int(
            cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        )

        fps = cap.get(
            cv2.CAP_PROP_FPS
        )

        codec = fourcc_str(
            cap.get(cv2.CAP_PROP_FOURCC)
        )

        log(
            "CAM",
            f"RTSP connected | "
            f"FFmpeg | "
            f"codec={codec} | "
            f"resolution={width}x{height} | "
            f"reported fps={fps}"
        )

        self.cap = cap

    # --------------------------------------------------------
    # CLOSE
    # --------------------------------------------------------

    def _close(self):

        self.connected = False

        if self.cap is not None:

            try:
                self.cap.release()

            except Exception:
                pass

            self.cap = None

    # --------------------------------------------------------
    # CAMERA THREAD
    # --------------------------------------------------------

    def run(self):

        while not self.stop_event.is_set():

            # ----------------------------------------------
            # CONNECT
            # ----------------------------------------------

            try:

                self._open()

            except Exception as exc:

                log(
                    "CAM",
                    f"open failed: {exc!r} - "
                    f"retrying every 3 seconds"
                )

                self.stop_event.wait(3.0)

                continue

            self.connected = True

            # ----------------------------------------------
            # GRAB LOOP
            # ----------------------------------------------

            try:

                self._grab_loop()

            except Exception as exc:

                log(
                    "CAM",
                    f"grabber error: {exc!r}"
                )

            # ----------------------------------------------
            # CLOSE
            # ----------------------------------------------

            self._close()

            self.reconnects += 1

            if not self.stop_event.is_set():

                log(
                    "CAM",
                    f"stream lost -> reconnecting "
                    f"(reconnects={self.reconnects})"
                )

                self.stop_event.wait(1.0)

    # --------------------------------------------------------
    # GRAB LOOP
    #
    # IMPORTANT:
    # This follows the original tested pipeline.
    # --------------------------------------------------------

    def _grab_loop(self):

        next_due = 0.0

        fails = 0

        last_ok = time.monotonic()

        while not self.stop_event.is_set():

            # ----------------------------------------------
            # Decode next frame
            # ----------------------------------------------

            ok = self.cap.grab()

            now = time.monotonic()

            if not ok:

                fails += 1

                log(
                    "CAM",
                    f"RTSP frame grab failed "
                    f"({fails} in a row)"
                )

                if (
                    fails >= CAM_MAX_READ_FAILURES
                    or
                    now - last_ok > CAM_STALL_S
                ):

                    log(
                        "CAM",
                        f"too many failures / stall "
                        f"({now - last_ok:.1f} s)"
                    )

                    return

                time.sleep(0.02)

                continue

            fails = 0

            last_ok = now

            self.decoded += 1

            # ----------------------------------------------
            # Not time for the ROS frame yet
            #
            # We still decoded the frame but do NOT perform
            # retrieve / colour conversion.
            # ----------------------------------------------

            if now < next_due:

                continue

            # ----------------------------------------------
            # Retrieve only the paced frame
            # ----------------------------------------------

            ok, frame = self.cap.retrieve()

            if not ok or frame is None:

                continue

            # ----------------------------------------------
            # Keep a steady period.
            #
            # If we fell behind, resynchronize instead of
            # producing a burst of frames.
            # ----------------------------------------------

            if (now - next_due) < self.period_s:

                next_due = (
                    next_due + self.period_s
                )

            else:

                next_due = (
                    now + self.period_s
                )

            # ----------------------------------------------
            # Store newest frame
            # ----------------------------------------------

            with self.cond:

                self.frame = frame

                self.frame_t = now

                self.frame_seq += 1

                self.cond.notify_all()

    # --------------------------------------------------------
    # WAIT FOR NEW FRAME
    # --------------------------------------------------------

    def wait_frame(
        self,
        last_seq,
        timeout
    ):

        with self.cond:

            self.cond.wait_for(
                lambda:
                    self.frame_seq != last_seq
                    or self.stop_event.is_set(),
                timeout
            )

            if self.frame_seq == last_seq:

                return None

            return (
                self.frame,
                self.frame_t,
                self.frame_seq
            )


# ============================================================
# ROS 2 CAMERA NODE
# ============================================================

class RudraCameraNode(Node):

    def __init__(self):

        super().__init__(
            "rudra_camera_node"
        )

        # ----------------------------------------------------
        # ROS PARAMETERS
        # ----------------------------------------------------

        self.declare_parameter(
            "rtsp_url",
            DEFAULT_RTSP_URL
        )

        self.declare_parameter(
            "camera_fps",
            CAMERA_FPS
        )

        self.declare_parameter(
            "frame_id",
            "rudra_camera"
        )

        # ----------------------------------------------------
        # PARAMETERS
        # ----------------------------------------------------

        self.rtsp_url = self.get_parameter(
            "rtsp_url"
        ).value

        self.camera_fps = float(
            self.get_parameter(
                "camera_fps"
            ).value
        )

        self.frame_id = self.get_parameter(
            "frame_id"
        ).value

        self.period_s = 1.0 / self.camera_fps

        # ----------------------------------------------------
        # CV BRIDGE
        # ----------------------------------------------------

        self.bridge = CvBridge()

        # ----------------------------------------------------
        # CAMERA
        # ----------------------------------------------------

        self.camera = FrameGrabber(
            self.rtsp_url,
            self.period_s
        )

        # ----------------------------------------------------
        # ROS IMAGE PUBLISHER
        # ----------------------------------------------------

        self.image_pub = self.create_publisher(
            Image,
            "/rudra/camera/image_raw",
            10
        )

        # ----------------------------------------------------
        # STATUS
        # ----------------------------------------------------

        self.status_timer = self.create_timer(
            5.0,
            self.publish_status
        )

        self.last_seq = 0

        # ----------------------------------------------------
        # START CAMERA
        # ----------------------------------------------------

        self.camera.start()

        log(
            "ROS",
            "rudra_camera_node started"
        )

        log(
            "ROS",
            f"RTSP URL: {self.rtsp_url}"
        )

        log(
            "ROS",
            f"Publishing: /rudra/camera/image_raw"
        )

        log(
            "ROS",
            f"Camera output rate: {self.camera_fps} FPS"
        )

        log(
            "ROS",
            f"OpenCV FFmpeg available: "
            f"{cv2_has_ffmpeg()}"
        )

        # ----------------------------------------------------
        # PUBLISH LOOP
        #
        # This timer does NOT read frames from a queue.
        #
        # It waits for the newest frame from the same
        # FrameGrabber architecture used by det_geo.py.
        # ----------------------------------------------------

        self.publish_timer = self.create_timer(
            0.01,
            self.process_camera
        )

    # --------------------------------------------------------
    # PROCESS CAMERA
    # --------------------------------------------------------

    def process_camera(self):

        result = self.camera.wait_frame(
            self.last_seq,
            timeout=0.0
        )

        if result is None:

            return

        frame, frame_t, seq = result

        self.last_seq = seq

        # ----------------------------------------------------
        # Convert OpenCV BGR -> ROS Image
        #
        # We keep BGR internally exactly like the original
        # pipeline. Conversion here is only for ROS transport.
        # ----------------------------------------------------

        try:

            msg = self.bridge.cv2_to_imgmsg(
                frame,
                encoding="bgr8"
            )

        except Exception as exc:

            self.get_logger().error(
                f"Failed to convert camera frame: {exc}"
            )

            return

        # ----------------------------------------------------
        # ROS TIMESTAMP
        # ----------------------------------------------------

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.header.frame_id = self.frame_id

        # ----------------------------------------------------
        # PUBLISH
        # ----------------------------------------------------

        self.image_pub.publish(msg)

    # --------------------------------------------------------
    # STATUS
    # --------------------------------------------------------

    def publish_status(self):

        connected = self.camera.connected

        decoded = self.camera.decoded

        reconnects = self.camera.reconnects

        log(
            "CAM",
            f"connected={connected} | "
            f"decoded={decoded} | "
            f"published_seq={self.last_seq} | "
            f"reconnects={reconnects}"
        )

    # --------------------------------------------------------
    # SHUTDOWN
    # --------------------------------------------------------

    def destroy_node(self):

        log(
            "ROS",
            "Stopping Rudra camera..."
        )

        self.camera.stop()

        self.camera.join(
            timeout=2.0
        )

        super().destroy_node()


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = RudraCameraNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()
