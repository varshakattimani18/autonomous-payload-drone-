#!/usr/bin/env python3

import time
import math
import threading
import collections

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pymavlink import mavutil


# ============================================================
# CONFIG
# Extracted from Rudra det_geo.py
# ============================================================

MAV_CONNECTION = "/dev/ttyUSB0"
MAV_BAUD = 115200
TELEM_RATE_HZ = 10.0

LINK_TIMEOUT_S = 10.0
MAX_TELEM_AGE_S = 0.5

MAV_SRC_SYSTEM = 1
MAV_SRC_COMPONENT = 191


# ============================================================
# MAVLINK TELEMETRY
# Based directly on det_geo.py MavTelemetry
# ============================================================

class MavTelemetry(threading.Thread):

    def __init__(
        self,
        conn,
        baud,
        rate_hz,
        src_sys=MAV_SRC_SYSTEM,
        src_comp=MAV_SRC_COMPONENT
    ):
        super().__init__(
            daemon=True,
            name="mav-telemetry"
        )

        self.conn = conn
        self.baud = baud
        self.rate_hz = rate_hz

        self.src_sys = src_sys
        self.src_comp = src_comp

        self.lock = threading.Lock()

        self.state = {
            'lat': 0.0,
            'lon': 0.0,
            'rel_alt_m': 0.0,
            'amsl_m': 0.0,

            'yaw': 0.0,
            'roll': 0.0,
            'pitch': 0.0,

            'vn': 0.0,
            've': 0.0,

            'hdg': None,

            'fix_type': 0,
            'sats': 0,

            'last_gps_t': 0.0,
            'last_att_t': 0.0,

            'valid': False
        }

        self.connected = False

        self.counts = collections.Counter()

        self.stop_event = threading.Event()

        self.master = None


    # ========================================================
    # MAIN THREAD LOOP
    # ========================================================

    def run(self):

        while not self.stop_event.is_set():

            try:

                self._connect()

                self._read_loop()

            except Exception as exc:

                print(
                    f"[MAV] ERROR: {exc!r}",
                    flush=True
                )

            self.connected = False

            if not self.stop_event.is_set():

                print(
                    "[MAV] Reconnecting in 2 s ...",
                    flush=True
                )

                self.stop_event.wait(2.0)


    # ========================================================
    # STOP
    # ========================================================

    def stop(self):

        self.stop_event.set()


    # ========================================================
    # MAVLINK CONNECTION
    # ========================================================

    def _connect(self):

        print(
            f"[MAV] Connecting to {self.conn} "
            f"(baud={self.baud}) ...",
            flush=True
        )

        if self.master is not None:

            try:
                self.master.close()
            except Exception:
                pass


        self.master = mavutil.mavlink_connection(
            self.conn,
            baud=self.baud,
            source_system=self.src_sys,
            source_component=self.src_comp
        )


        # ----------------------------------------------------
        # WAIT FOR AUTOPILOT HEARTBEAT
        # Same 15 second timeout as det_geo.py
        # ----------------------------------------------------

        t0 = time.monotonic()

        hb = None

        while (
            time.monotonic() - t0 < 15
            and not self.stop_event.is_set()
        ):

            msg = self.master.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=1.0
            )

            if (
                msg is None
                or msg.autopilot
                == mavutil.mavlink.MAV_AUTOPILOT_INVALID
                or msg.type
                == mavutil.mavlink.MAV_TYPE_GCS
            ):
                continue

            hb = msg

            self.master.target_system = msg.get_srcSystem()
            self.master.target_component = msg.get_srcComponent()

            break


        if hb is None:

            raise RuntimeError(
                "No autopilot HEARTBEAT within 15 s "
                "(check port/baud/wiring)"
            )


        print(
            "[MAV] Heartbeat OK: "
            f"sys={self.master.target_system} "
            f"comp={self.master.target_component}",
            flush=True
        )


        self._request_streams()

        self.connected = True


    # ========================================================
    # REQUEST MAVLINK STREAMS
    # Same streams and 10 Hz rate as det_geo.py
    # ========================================================

    def _request_streams(self):

        m = self.master
        ml = mavutil.mavlink

        for msg_id in (
            ml.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            ml.MAVLINK_MSG_ID_ATTITUDE,
            ml.MAVLINK_MSG_ID_GPS_RAW_INT
        ):

            m.mav.command_long_send(
                m.target_system,
                m.target_component,
                ml.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                msg_id,
                int(1e6 / self.rate_hz),
                0,
                0,
                0,
                0,
                0
            )


        print(
            f"[MAV] Requested background async streams "
            f"@ {self.rate_hz} Hz",
            flush=True
        )


    # ========================================================
    # MAVLINK READ LOOP
    # ========================================================

    def _read_loop(self):

        last_rx = time.monotonic()

        types = [
            "HEARTBEAT",
            "GLOBAL_POSITION_INT",
            "ATTITUDE",
            "GPS_RAW_INT",
        ]


        while not self.stop_event.is_set():

            msg = self.master.recv_match(
                type=types,
                blocking=True,
                timeout=0.02
            )

            now = time.monotonic()


            if msg is None:

                if now - last_rx > LINK_TIMEOUT_S:

                    raise RuntimeError(
                        f"No MAVLink data for "
                        f"{LINK_TIMEOUT_S:.0f} s"
                    )

                continue


            last_rx = now

            mtype = msg.get_type()

            self.counts[mtype] += 1


            with self.lock:

                # ==========================================
                # GLOBAL POSITION
                # ==========================================

                if mtype == "GLOBAL_POSITION_INT":

                    self.state['lat'] = msg.lat / 1e7

                    self.state['lon'] = msg.lon / 1e7

                    self.state['rel_alt_m'] = (
                        msg.relative_alt / 1000.0
                    )

                    self.state['amsl_m'] = (
                        msg.alt / 1000.0
                    )

                    self.state['hdg'] = (
                        None
                        if msg.hdg == 65535
                        else msg.hdg / 100.0
                    )

                    self.state['vn'] = (
                        msg.vx / 100.0
                    )

                    self.state['ve'] = (
                        msg.vy / 100.0
                    )

                    self.state['last_gps_t'] = now


                # ==========================================
                # ATTITUDE
                # ==========================================

                elif mtype == "ATTITUDE":

                    self.state['roll'] = math.degrees(
                        msg.roll
                    )

                    self.state['pitch'] = math.degrees(
                        msg.pitch
                    )

                    self.state['yaw'] = math.degrees(
                        msg.yaw
                    )

                    self.state['last_att_t'] = now


                # ==========================================
                # GPS RAW
                # ==========================================

                elif mtype == "GPS_RAW_INT":

                    self.state['fix_type'] = (
                        msg.fix_type
                    )

                    self.state['sats'] = (
                        msg.satellites_visible
                    )


    # ========================================================
    # GET CURRENT TELEMETRY STATE
    # Same state-age calculation as det_geo.py
    # ========================================================

    def get_state(self):

        with self.lock:

            s = dict(self.state)

            now = time.monotonic()

            s['gps_dt'] = (
                now - s['last_gps_t']
            )

            s['att_dt'] = (
                now - s['last_att_t']
            )

            s['valid'] = (
                s['gps_dt'] <= MAX_TELEM_AGE_S
            )

            return s


# ============================================================
# ROS 2 NODE
# ============================================================

class RudraTelemetryNode(Node):

    def __init__(self):

        super().__init__(
            'rudra_telemetry_node'
        )


        # ----------------------------------------------------
        # ROS PARAMETERS
        # ----------------------------------------------------

        self.declare_parameter(
            'mav_connection',
            MAV_CONNECTION
        )

        self.declare_parameter(
            'mav_baud',
            MAV_BAUD
        )

        self.declare_parameter(
            'telemetry_rate_hz',
            TELEM_RATE_HZ
        )


        conn = self.get_parameter(
            'mav_connection'
        ).value

        baud = self.get_parameter(
            'mav_baud'
        ).value

        rate_hz = self.get_parameter(
            'telemetry_rate_hz'
        ).value


        # ----------------------------------------------------
        # TELEMETRY THREAD
        # ----------------------------------------------------

        self.telemetry = MavTelemetry(
            conn=conn,
            baud=baud,
            rate_hz=rate_hz
        )


        self.telemetry.start()


        # ----------------------------------------------------
        # ROS PUBLISHER
        # ----------------------------------------------------

        self.telemetry_pub = self.create_publisher(
            String,
            '/rudra/telemetry',
            10
        )


        # ----------------------------------------------------
        # 10 Hz ROS PUBLISH TIMER
        # ----------------------------------------------------

        self.timer = self.create_timer(
            0.1,
            self.publish_telemetry
        )


        # ----------------------------------------------------
        # STATUS TIMER
        # ----------------------------------------------------

        self.status_timer = self.create_timer(
            5.0,
            self.print_status
        )


        self.get_logger().info(
            "rudra_telemetry_node started"
        )

        self.get_logger().info(
            f"MAVLink: {conn} @ {baud}"
        )

        self.get_logger().info(
            f"Telemetry rate: {rate_hz} Hz"
        )

        self.get_logger().info(
            "Publishing: /rudra/telemetry"
        )


    # ========================================================
    # PUBLISH TELEMETRY
    # ========================================================

    def publish_telemetry(self):

        state = self.telemetry.get_state()


        msg = String()

        msg.data = (
            f"lat={state['lat']:.7f},"
            f"lon={state['lon']:.7f},"
            f"rel_alt_m={state['rel_alt_m']:.3f},"
            f"amsl_m={state['amsl_m']:.3f},"
            f"yaw={state['yaw']:.3f},"
            f"roll={state['roll']:.3f},"
            f"pitch={state['pitch']:.3f},"
            f"vn={state['vn']:.3f},"
            f"ve={state['ve']:.3f},"
            f"hdg={state['hdg']},"
            f"fix_type={state['fix_type']},"
            f"sats={state['sats']},"
            f"gps_dt={state['gps_dt']:.3f},"
            f"att_dt={state['att_dt']:.3f},"
            f"valid={state['valid']}"
        )


        self.telemetry_pub.publish(msg)


    # ========================================================
    # STATUS
    # ========================================================

    def print_status(self):

        state = self.telemetry.get_state()

        self.get_logger().info(
            "MAV: "
            f"connected={self.telemetry.connected} | "
            f"GPS=({state['lat']:.7f}, "
            f"{state['lon']:.7f}) | "
            f"alt={state['rel_alt_m']:.2f} m | "
            f"yaw={state['yaw']:.2f}° | "
            f"roll={state['roll']:.2f}° | "
            f"pitch={state['pitch']:.2f}° | "
            f"fix={state['fix_type']} | "
            f"sats={state['sats']} | "
            f"valid={state['valid']}"
        )


    # ========================================================
    # CLEAN SHUTDOWN
    # ========================================================

    def shutdown(self):

        self.get_logger().info(
            "Stopping Rudra telemetry..."
        )

        self.telemetry.stop()

        self.telemetry.join(
            timeout=2.0
        )

        if self.telemetry.master is not None:

            try:
                self.telemetry.master.close()
            except Exception:
                pass


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = RudraTelemetryNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.shutdown()

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':
    main()
