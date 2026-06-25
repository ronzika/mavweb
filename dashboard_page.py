import os
import time
import math
import io
import struct
import threading
import datetime
import logging
from pathlib import Path

import uuid
import pandas as pd
import pydeck as pdk
import streamlit as st
from dotenv import load_dotenv
from pymavlink import mavutil
from pymavlink import mavftp

from shared_state import get_shared_state
from mavlink_utils import upload_mission
import mavsdk_mission
import streamlit.components.v1 as components
from telegram_bot import start_telegram_bridge

# SVG = """
# <svg xmlns="http://www.w3.org/2000/svg"
#      viewBox="0 0 225 225"
#      style="width:100%; height:auto;"
#      aria-label="Zero turn mower silhouette"
#      role="img">

#   <style>
#     .mower {
#       fill: #2ECC71;   /* <-- CHANGE COLOR HERE */
#     }
#   </style>

#   <path class="mower" fill-rule="evenodd" d="
# M 224.0 133.5 L 203.0 149.5 L 197.5 149.0 L 193.5 137.0 L 186.0 133.5 L 180.5 126.0 L 180.5 117.0 L 183.5 112.0 L 197.0 103.5 L 203.0 103.5 L 216.5 112.0 L 224.0 121.5 L 224.0 133.5 Z
# M 174.0 178.5 L 162.0 178.5 L 156.0 176.5 L 145.5 167.0 L 141.5 155.0 L 143.5 146.0 L 151.0 137.5 L 160.0 133.5 L 170.0 133.5 L 180.5 138.0 L 187.5 146.0 L 189.5 155.0 L 187.5 165.0 L 181.0 174.5 L 174.0 178.5 Z
# M 142.0 98.5 L 107.0 98.5 L 102.5 95.0 L 101.5 90.0 L 105.0 84.5 L 110.0 82.5 L 142.0 82.5 L 150.0 84.5 L 155.5 90.0 L 154.5 95.0 L 149.5 98.5 L 142.0 98.5 Z
# M 104.0 165.5 L 53.0 165.5 L 48.5 160.0 L 48.5 150.0 L 52.0 145.5 L 104.0 145.5 L 108.5 150.0 L 108.5 160.0 L 104.0 165.5 Z
# M 0.0 156.5 L 3.5 150.0 L 8.0 146.5 L 13.0 144.5 L 21.0 144.5 L 24.0 146.5 L 24.0 177.5 L 12.0 178.5 L 2.5 172.0 L 0.0 166.5 L 0.0 156.5 Z
# M 98.0 183.5 L 60.0 183.5 L 55.5 180.0 L 54.5 175.0 L 58.0 170.5 L 64.0 168.5 L 98.0 168.5 L 105.0 170.5 L 110.5 175.0 L 109.5 180.0 L 104.5 183.5 L 98.0 183.5 Z
# M 77.0 121.5 L 64.0 121.5 L 60.5 119.0 L 60.5 110.0 L 64.0 107.5 L 77.0 107.5 L 80.5 110.0 L 80.5 119.0 L 77.0 121.5 Z
# M 33.0 112.5 L 23.0 112.5 L 20.5 110.0 L 20.5 101.0 L 23.0 98.5 L 33.0 98.5 L 35.5 101.0 L 35.5 110.0 L 33.0 112.5 Z
# M 41.0 98.5 L 35.5 95.0 L 35.5 86.0 L 41.0 82.5 L 53.0 82.5 L 58.5 86.0 L 58.5 95.0 L 53.0 98.5 L 41.0 98.5 Z
# " />
# </svg>
# """

# with st.sidebar:
#     # optional spacing control
#     st.markdown("<div style='margin-top:-8px'></div>", unsafe_allow_html=True)
#     components.html(SVG, height=220)

# --- Configuration ---
env_path = Path(__file__).parent / '.env'
print(
    f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [SYSTEM] Loading .env from: {env_path} (Exists: {env_path.exists()})"
)
load_dotenv(dotenv_path=env_path, override=True)

MAVLINK_ENDPOINT = os.getenv("MAVLINK_ENDPOINT", "udpin:0.0.0.0:14550")
MAPBOX_API_KEY = os.getenv("MAPBOX_API_KEY")
DEBUG = os.getenv("DEBUG", "0") != "0"
#MAVSDK_SYSTEM_ADDRESS = (os.getenv("MAVSDK_SYSTEM_ADDRESS", "") or "udp://:14540").strip()

# Optional relay controls (set RELAY1..RELAY6 in .env to enable)
RELAY_LABELS: dict[int, str] = {
    i: (os.getenv(f"RELAY{i}", "") or "").strip()
    for i in range(1, 7)
    if (os.getenv(f"RELAY{i}", "") or "").strip()
}

# Mission transfer tuning (speed)
MISSION_FETCH_METHOD = (os.getenv("MISSION_FETCH_METHOD", "mavlink") or "mavlink").strip().lower()
MISSION_UPLOAD_METHOD = (os.getenv("MISSION_UPLOAD_METHOD", "mavlink") or "mavlink").strip().lower()
MISSION_RETRY_INTERVAL_S = float(os.getenv("MISSION_RETRY_INTERVAL_S", "0.2") or 0.2)
WORKER_RECV_TIMEOUT_S = float(os.getenv("WORKER_RECV_TIMEOUT_S", "0.1") or 0.1)
MISSION_REQUEST_WINDOW = max(1, int(os.getenv("MISSION_REQUEST_WINDOW", "3") or 3))
MISSION_PROGRESS_GRACE_S = float(os.getenv("MISSION_PROGRESS_GRACE_S", "3") or 3)

def material_icon(name, size=24):
    return f'<i class="material-icons" style="font-size:{size}px;">{name}</i>'

def _is_valid_mapbox_key(val) -> bool:
    try:
        v = str(val or "").strip()
    except Exception:
        return False
    if not v:
        return False
    if v.lower() in {"none", "null", "undefined"}:
        return False
    # Most Mapbox public tokens start with pk. (or sk. for secret).
    # Keep validation light: avoid rejecting tokens that work, but ignore obvious junk.
    if not (v.startswith("pk.") or v.startswith("sk.")):
        return False
    return len(v) >= 20


MAPBOX_API_KEY_VALID = _is_valid_mapbox_key(MAPBOX_API_KEY)
MAP_BASE_PROVIDER = "mapbox" if MAPBOX_API_KEY_VALID else "carto"

if MAPBOX_API_KEY_VALID:
    print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [SYSTEM] Mapbox Key Loaded: {str(MAPBOX_API_KEY)[:10]}...")
else:
    print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [SYSTEM] WARNING: Mapbox Key missing/invalid; using Carto basemap.")

# --- MAVLink Utilities ---
def get_mode_name(custom_mode):
    mapping = {0: 'MANUAL', 1: 'ACRO', 3: 'STEERING', 4: 'HOLD', 5: 'LOITER', 10: 'AUTO', 11: 'RTL', 15: 'GUIDED'}
    return mapping.get(custom_mode, f"MODE({custom_mode})")


def get_gps_fix_string(fix_type):
    mapping = {0: 'No GPS', 1: 'No Fix', 2: '2D Fix', 3: '3D Fix', 4: 'DGPS', 5: 'RTK Float', 6: 'RTK Fixed'}
    return mapping.get(fix_type, f"Fix({fix_type})")


def format_gps_accuracy(acc_m):
    try:
        value_m = float(acc_m)
    except Exception:
        return 'N/A'

    if not math.isfinite(value_m) or value_m < 0:
        return 'N/A'

    if value_m < 0.05:
        return f"{int(round(value_m * 1000.0))} mm"
    if value_m < 1.0:
        return f"{value_m * 100.0:.1f} cm"
    return f"{value_m:.2f} m"


def format_gps_yaw(yaw_deg):
    try:
        value_deg = float(yaw_deg)
    except Exception:
        return 'N/A'

    if not math.isfinite(value_deg) or value_deg >= 655.35:
        return 'N/A'
    return f"{value_deg:.2f} deg"


def request_message_interval(conn, msg_id, hz):
    try:
        interval_us = int(1_000_000 / hz)
        set_message_interval_us(conn, msg_id, interval_us)
    except Exception:
        pass


def set_message_interval_us(conn, msg_id, interval_us: int):
    """Set a MAVLink message interval in microseconds.

    interval_us=0 disables the stream (vehicle dependent, but common behavior).
    """
    try:
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            int(msg_id),
            int(interval_us),
            0,
            0,
            0,
            0,
            0,
        )
    except Exception:
        pass


# Telemetry message intervals the worker requests on connect.
TELEMETRY_MESSAGE_INTERVALS: list[tuple[int, float]] = [
    (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 2),
    (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2),
    (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1),
    (mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT, 2),
    (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 2),
    (mavutil.mavlink.MAVLINK_MSG_ID_GPS2_RAW, 1),
]


def _parse_qgc_wpl_text_to_lonlat(text: str) -> list[list[float]]:
    """Parse QGroundControl WPL text into [[lon, lat], ...] points.

    Returns an empty list if parsing fails.
    """
    try:
        lines = [ln.strip() for ln in text.splitlines() if ln.strip() and not ln.strip().startswith('#')]
        if not lines or not lines[0].startswith('QGC WPL'):
            return []
        points: list[list[float]] = []
        # Format: seq, current, frame, command, param1..param4, x(lat), y(lon), z(alt), autocontinue
        for ln in lines[1:]:
            cols = ln.split('\t')
            if len(cols) < 12:
                continue
            try:
                lat = float(cols[8])
                lon = float(cols[9])
            except Exception:
                continue
            if abs(lat) < 1e-9 and abs(lon) < 1e-9:
                continue
            points.append([lon, lat])
        return points
    except Exception:
        return []


def _parse_ardupilot_mission_dat_to_lonlat(blob: bytes) -> list[list[float]]:
    """Parse ArduPilot MAVFTP @MISSION/mission.dat into [[lon, lat], ...]."""
    try:
        if not blob or len(blob) < 10:
            return []

        # AP_Filesystem_Mission::header
        magic, data_type, _options, _start, num_items = struct.unpack_from("<HHHHH", blob, 0)
        if magic != 0x763D:
            return []
        if data_type != 0:
            return []

        item_fmt = "<ffffiifHHBBBBB"
        item_size = struct.calcsize(item_fmt)
        points: list[list[float]] = []
        offset = 10

        for _ in range(int(num_items)):
            if offset + item_size > len(blob):
                break
            item = struct.unpack_from(item_fmt, blob, offset)
            offset += item_size

            lat_e7 = int(item[4])
            lon_e7 = int(item[5])
            frame = int(item[11])

            if lat_e7 == 0 and lon_e7 == 0:
                continue

            # Global frames are encoded as degE7 in MISSION_ITEM_INT payload.
            if frame in {0, 3, 5, 6, 10, 11}:
                lat = lat_e7 / 1e7
                lon = lon_e7 / 1e7
            else:
                continue

            if abs(lat) < 1e-9 and abs(lon) < 1e-9:
                continue
            if abs(lat) > 90.0 or abs(lon) > 180.0:
                continue
            points.append([lon, lat])

        return points
    except Exception:
        return []


def _mission_items_to_qgc_wpl_text(mission_items: list[dict]) -> str:
    """Serialize mission items to QGC WPL 110 text.

    Preserves command/params/frame where possible so MAVFTP file uploads can
    reuse the same mission representation used by MAVLink upload flow.
    """
    lines = ["QGC WPL 110"]
    for idx, item in enumerate(mission_items or []):
        try:
            seq = int(item.get('seq', idx) or idx)
        except Exception:
            seq = idx
        try:
            current = int(item.get('current', 1 if seq == 0 else 0) or 0)
        except Exception:
            current = 1 if seq == 0 else 0
        try:
            frame = int(item.get('frame', 3) or 3)
        except Exception:
            frame = 3
        try:
            command = int(item.get('command', 16) or 16)
        except Exception:
            command = 16
        try:
            p1 = float(item.get('param1', 0.0) or 0.0)
            p2 = float(item.get('param2', 0.0) or 0.0)
            p3 = float(item.get('param3', 0.0) or 0.0)
            p4 = float(item.get('param4', 0.0) or 0.0)
            x = float(item.get('x', 0.0) or 0.0)
            y = float(item.get('y', 0.0) or 0.0)
            z = float(item.get('z', 0.0) or 0.0)
        except Exception:
            continue
        try:
            autocontinue = int(item.get('autocontinue', 1) or 1)
        except Exception:
            autocontinue = 1

        lines.append(
            f"{seq}\t{current}\t{frame}\t{command}\t{p1:.6f}\t{p2:.6f}\t{p3:.6f}\t{p4:.6f}\t{x:.8f}\t{y:.8f}\t{z:.6f}\t{autocontinue}"
        )
    return "\n".join(lines) + "\n"


def _try_upload_mission_via_mavftp(conn, mission_items: list[dict], progress_cb=None) -> bool:
    """Best-effort mission upload via MAVFTP by writing a QGC WPL mission file.

    Returns True when at least one remote path accepted the upload.
    """
    tsys = getattr(conn, 'target_system', None)
    tcomp = getattr(conn, 'target_component', None)
    if tsys is None or tcomp is None:
        return False

    text = _mission_items_to_qgc_wpl_text(mission_items)
    if text.strip() == "QGC WPL 110":
        return False

    ftp = mavftp.MAVFTP(conn, tsys, tcomp)
    override_path = (os.getenv("MAVFTP_UPLOAD_PATH", "") or os.getenv("MAVFTP_MISSION_PATH", "")).strip()
    paths = [
        p for p in [
            override_path,
            "@MISSION/mission.waypoints",
            "/APM/mission.waypoints",
            "/mission.waypoints",
            "/fs/microsd/APM/mission.waypoints",
        ] if p
    ]

    # Keep logs quieter unless DEBUG is enabled; MAVFTP can be chatty on failures.
    root_logger = logging.getLogger()
    prev_level = root_logger.level
    if not DEBUG:
        root_logger.setLevel(logging.ERROR)

    try:
        payload = text.encode("ascii", errors="ignore")
        success_code = getattr(mavftp.FtpError, "Success", 0)

        for remote_path in paths:
            try:
                try:
                    ftp.cmd_rm([remote_path])
                except Exception:
                    pass

                def _ftp_progress(frac):
                    if progress_cb is None:
                        return
                    try:
                        pct = max(0.0, min(1.0, float(frac))) if frac is not None else 0.0
                        total = max(1, int(len(mission_items) or 1))
                        progress_cb(int(round(total * pct)), total)
                    except Exception:
                        pass

                ret0 = ftp.cmd_put([
                    "mission.waypoints",
                    remote_path,
                ], fh=io.BytesIO(payload), progress_callback=_ftp_progress)
                if getattr(ret0, "error_code", None) != success_code:
                    continue
                ret = ftp.process_ftp_reply("CreateFile", timeout=12)
                if getattr(ret, "error_code", None) == success_code:
                    return True
            except Exception:
                continue
        return False
    finally:
        if not DEBUG:
            root_logger.setLevel(prev_level)


# Map style options (used by sidebar fragment + map fragment)
# If Mapbox key is not available/valid, fall back to Carto basemaps so the map still renders.
if MAPBOX_API_KEY_VALID:
    map_styles = {
        "Satellite": "mapbox://styles/mapbox/satellite-v9",
        "Satellite Streets": "mapbox://styles/mapbox/satellite-streets-v12",
        "Streets": "mapbox://styles/mapbox/streets-v11",
        "Dark": "mapbox://styles/mapbox/dark-v10",
        "Light": "mapbox://styles/mapbox/light-v10",
        "Outdoors": "mapbox://styles/mapbox/outdoors-v11",
    }
else:
    # These identifiers are understood by pydeck for the Carto provider.
    map_styles = {
        "Dark": "dark",
        "Light": "light",
        "Road": "road",
        "Satellite": "satellite",
        "Dark (No Labels)": "dark_no_labels",
        "Light (No Labels)": "light_no_labels",
    }

map_options = list(map_styles.keys())

default_map_style_name = "Dark" if "Dark" in map_styles else (map_options[0] if map_options else "Dark")

# Initialize / repair session state if options changed (e.g., Mapbox key removed).
sel = st.session_state.get("map_style_select_fixed")
if (not sel) or (sel not in map_options):
    st.session_state["map_style_select_fixed"] = default_map_style_name

try:
    st.session_state["map_style_ind"] = map_options.index(st.session_state["map_style_select_fixed"])
except Exception:
    st.session_state["map_style_ind"] = 0


def update_map_style():
    try:
        st.session_state["map_style_ind"] = map_options.index(st.session_state["map_style_select_fixed"])
    except Exception:
        st.session_state["map_style_ind"] = 0


@st.fragment(run_every=0.5)
def _render_sidebar_controls():
    """Render command buttons and map settings.

    IMPORTANT: Call inside `with st.sidebar:`. Do not call `st.sidebar.*` here.
    """
    cmd_conn = get_shared_state().get_connection()
    is_disabled = not cmd_conn

    # Snapshot rover state for command enable/disable decisions.
    state_snapshot = get_shared_state().get()
    is_armed_now = bool(state_snapshot.get('armed', False))

    st.subheader("Commands")

    if st.button("MANUAL", use_container_width=True, disabled=is_disabled, key="cmd_manual"):
        if cmd_conn:
            try:
                with get_shared_state().acquire_mav_lock():
                    cmd_conn.mav.set_mode_send(cmd_conn.target_system, 1, 0)
                st.toast("Set Mode: MANUAL")
            except Exception as e:
                st.error(f"Failed to set mode: {e}")

    if st.button("AUTO", use_container_width=True, disabled=is_disabled, key="cmd_auto"):
        if cmd_conn:
            try:
                with get_shared_state().acquire_mav_lock():
                    cmd_conn.mav.set_mode_send(cmd_conn.target_system, 1, 10)
                st.toast("Set Mode: AUTO")
            except Exception as e:
                st.error(f"Failed to set mode: {e}")

    if st.button("HOLD", use_container_width=True, disabled=is_disabled, key="cmd_hold"):
        if cmd_conn:
            try:
                with get_shared_state().acquire_mav_lock():
                    cmd_conn.mav.set_mode_send(cmd_conn.target_system, 1, 4)
                st.toast("Set Mode: HOLD")
            except Exception as e:
                st.error(f"Failed to set mode: {e}")

    if st.button("ARM", use_container_width=True, disabled=(is_disabled or is_armed_now), key="cmd_arm"):
        if cmd_conn:
            try:
                with get_shared_state().acquire_mav_lock():
                    cmd_conn.mav.command_long_send(
                        cmd_conn.target_system,
                        cmd_conn.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        0,
                        1,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                    )
                st.toast("Sent ARM Command")
            except Exception as e:
                st.error(f"Failed to arm: {e}")

    if st.button("Disarm", use_container_width=True, disabled=(is_disabled or (not is_armed_now)), key="cmd_disarm"):
        if cmd_conn:
            try:
                with get_shared_state().acquire_mav_lock():
                    cmd_conn.mav.command_long_send(
                        cmd_conn.target_system,
                        cmd_conn.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                    )
                st.toast("Sent DISARM Command")
            except Exception as e:
                st.error(f"Failed to disarm: {e}")

    # Reboot Flight Controller (only allowed when disarmed)
    reboot_disabled = is_disabled or is_armed_now

    if "confirm_reboot_fc" not in st.session_state:
        st.session_state["confirm_reboot_fc"] = False

    # If the rover becomes armed (or command becomes invalid), drop any pending confirmation.
    if reboot_disabled and st.session_state.get("confirm_reboot_fc", False):
        st.session_state["confirm_reboot_fc"] = False

    if st.button("Reboot FC", use_container_width=True, disabled=reboot_disabled, key="cmd_reboot_fc"):
        st.session_state["confirm_reboot_fc"] = True

    if st.session_state.get("confirm_reboot_fc", False):
        st.warning("Are you sure?")

        col_confirm, col_cancel = st.columns(2)
        with col_confirm:
            if st.button(
                "Yes",
                use_container_width=True,
                disabled=reboot_disabled,
                key="cmd_reboot_fc_confirm",
            ):
                if cmd_conn and not is_armed:
                    try:
                        # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
                        # param1=1 -> reboot autopilot
                        with get_shared_state().acquire_mav_lock():
                            cmd_conn.mav.command_long_send(
                                cmd_conn.target_system,
                                cmd_conn.target_component,
                                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                0,
                                1,
                                0,
                                0,
                                0,
                                0,
                                0,
                                0,
                            )
                        get_shared_state().append_message("[UI] Sent Reboot FC command")
                        st.toast("Sent Reboot FC")
                    except Exception as e:
                        get_shared_state().append_message(f"[UI] Failed to reboot FC: {e}")

                st.session_state["confirm_reboot_fc"] = False

        with col_cancel:
            if st.button(
                "Cancel",
                use_container_width=True,
                key="cmd_reboot_fc_cancel",
            ):
                st.session_state["confirm_reboot_fc"] = False

    # Reset Mission: set current mission item to 0 (disabled while in AUTO)
    state_snapshot = get_shared_state().get()
    current_mode = str(state_snapshot.get('mode') or '')
    reset_disabled = is_disabled or (current_mode == 'AUTO')

    if "confirm_reset_mission" not in st.session_state:
        st.session_state["confirm_reset_mission"] = False

    if st.button("Reset Mission", use_container_width=True, disabled=reset_disabled, key="cmd_reset_mission"):
        st.session_state["confirm_reset_mission"] = True

    if st.session_state.get("confirm_reset_mission", False):
        st.warning("Are you sure?")

        col_confirm, col_cancel = st.columns(2)
        with col_confirm:
            if st.button(
                "Yes",
                use_container_width=True,
                disabled=reset_disabled,
                key="cmd_reset_mission_confirm",
            ):
                if cmd_conn:
                    try:
                        with get_shared_state().acquire_mav_lock():
                            cmd_conn.mav.mission_set_current_send(
                                cmd_conn.target_system,
                                cmd_conn.target_component,
                                0,
                            )
                        get_shared_state().append_message("[UI] Reset mission to WP 0")
                    except Exception as e:
                        get_shared_state().append_message(f"[UI] Failed to reset mission: {e}")
                st.session_state["confirm_reset_mission"] = False

        with col_cancel:
            if st.button(
                "Cancel",
                use_container_width=True,
                key="cmd_reset_mission_cancel",
            ):
                st.session_state["confirm_reset_mission"] = False

    state_snapshot = get_shared_state().get()
    wp_known = bool(state_snapshot.get('wp_current_known', False))
    wp_current = int(state_snapshot.get('wp_current') or 0)
    mission_total = int(state_snapshot.get('mission_dl_total') or 0)

    skip_disabled = is_disabled or (not wp_known)
    if st.button("Skip WP", use_container_width=True, disabled=skip_disabled, key="cmd_skip_wp"):
        if cmd_conn and wp_known:
            try:
                target_wp = wp_current + 1
                if mission_total > 0:
                    target_wp = min(target_wp, mission_total - 1)

                if mission_total > 0 and wp_current >= mission_total - 1:
                    get_shared_state().append_message(f"[UI] Skip WP ignored: already at last WP ({wp_current}/{mission_total - 1}).")
                else:
                    with get_shared_state().acquire_mav_lock():
                        cmd_conn.mav.mission_set_current_send(
                            cmd_conn.target_system,
                            cmd_conn.target_component,
                            int(target_wp),
                        )
                    get_shared_state().append_message(f"[UI] Skipped from WP {wp_current} to WP {target_wp}.")
            except Exception as e:
                get_shared_state().append_message(f"[UI] Failed to skip waypoint: {e}")

    if st.button("Fetch Mission Map", use_container_width=True, disabled=is_disabled, key="cmd_fetch_mission"):
        if cmd_conn:
            try:
                get_shared_state().set_loading(True)
                get_shared_state().mission_fetch_queue.put({'ts': time.time()})
                get_shared_state().append_message("[UI] Requested mission list from vehicle.")
            except Exception as e:
                get_shared_state().set_loading(False)
                get_shared_state().append_message(f"[UI] Failed to request mission: {e}")

    # Save Waypoint: append current position to mission in the FC (disabled in AUTO)
    state_snapshot = get_shared_state().get()
    current_mode = str(state_snapshot.get('mode') or '')
    cur_lat = state_snapshot.get('lat')
    cur_lon = state_snapshot.get('lon')
    cur_alt_m = state_snapshot.get('alt_m')
    save_wp_disabled = is_disabled or (current_mode == 'AUTO') or (cur_lat is None) or (cur_lon is None)

    if st.button("Save Waypoint", use_container_width=True, disabled=save_wp_disabled, key="cmd_save_waypoint"):
        try:
            get_shared_state().save_wp_queue.put({
                'ts': time.time(),
                'lat': float(cur_lat),
                'lon': float(cur_lon),
                'alt_m': (float(cur_alt_m) if cur_alt_m is not None else None),
            })
            get_shared_state().append_message("[UI] Save Waypoint requested")
            st.toast("Save Waypoint requested")
        except Exception as e:
            get_shared_state().append_message(f"[UI] Failed to request Save Waypoint: {e}")

    # Breadcrumbs
    if st.button("Clear Breadcrumbs", use_container_width=True, key="cmd_clear_breadcrumbs"):
        try:
            get_shared_state().update({'history': []})
            get_shared_state().append_message("[UI] Cleared breadcrumb history")
            st.toast("Breadcrumbs cleared")
        except Exception as e:
            get_shared_state().append_message(f"[UI] Failed to clear breadcrumbs: {e}")

    # Relay toggles (only shown when RELAY1..RELAY6 are set in .env)
    if RELAY_LABELS:
        st.divider()
        st.subheader("Relays")

        for relay_num in sorted(RELAY_LABELS.keys()):
            label = RELAY_LABELS[relay_num]
            toggle_key = f"relay_toggle_{relay_num}"
            last_sent_key = f"relay_last_sent_{relay_num}"
            current_relays = state_snapshot.get("relays", {})

            if toggle_key not in st.session_state:
                st.session_state[toggle_key] = current_relays.get(int(relay_num), False)
            if last_sent_key not in st.session_state:
                st.session_state[last_sent_key] = bool(st.session_state[toggle_key])
            
            # Sync remote changes
            if st.session_state[last_sent_key] != current_relays.get(int(relay_num), False):
                st.session_state[toggle_key] = current_relays.get(int(relay_num), False)
                st.session_state[last_sent_key] = current_relays.get(int(relay_num), False)

            val = st.toggle(label, key=toggle_key, disabled=is_disabled)

            # Debounce: only send when the user changes the toggle.
            if (not is_disabled) and (bool(st.session_state[last_sent_key]) != bool(val)) and cmd_conn:
                try:
                    desired = 1 if val else 0
                    # ArduPilot uses 0-based relay numbering for MAV_CMD_DO_SET_RELAY:
                    # Mission Planner "Relay 1" corresponds to relay index 0.
                    ap_relay_index = int(relay_num) - 1
                    with get_shared_state().acquire_mav_lock():
                        cmd_conn.mav.command_long_send(
                            cmd_conn.target_system,
                            cmd_conn.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
                            0,
                            float(ap_relay_index),
                            float(desired),
                            0,
                            0,
                            0,
                            0,
                            0,
                        )
                    st.session_state[last_sent_key] = bool(val)
                    
                    # Update background state structure
                    current_relays = get_shared_state().get().get("relays", {})
                    current_relays[int(relay_num)] = bool(val)
                    get_shared_state().update({"relays": current_relays})
                    
                    get_shared_state().append_message(f"[UI] Relay {relay_num} set to {desired} ({label})")
                except Exception as e:
                    get_shared_state().append_message(f"[UI] Relay {relay_num} command failed: {e}")

    st.divider()
    st.subheader("Map Settings")
    st.selectbox(
        "Map Style",
        map_options,
        index=st.session_state["map_style_ind"],
        disabled=False,
        key="map_style_select_fixed",
        on_change=update_map_style,
    )

    if is_disabled:
        st.warning("Commands disabled: MAVLink connection not available.")
    elif not wp_known:
        st.info("Skip WP disabled: waiting for current waypoint (MISSION_CURRENT).")


def _try_fetch_mission_via_mavftp(conn) -> list[list[float]]:
    """Best-effort mission download via MAVFTP.

    ArduPilot exposes some configuration objects through MAVFTP depending on version/build.
    We try a few common/special paths and parse QGC WPL if returned.

    Returns [[lon, lat], ...] or [] on failure.
    """
    # MAVFTP needs explicit target ids
    tsys = getattr(conn, 'target_system', None)
    tcomp = getattr(conn, 'target_component', None)
    if tsys is None or tcomp is None:
        return []

    ftp = mavftp.MAVFTP(conn, tsys, tcomp)

    # Optional override if you know the exact remote path.
    override_path = (os.getenv("MAVFTP_MISSION_PATH", "") or os.getenv("MAVFTP_MISSION_FILE", "")).strip()

    # Reduce spammy OpenFileRO warnings unless DEBUG is enabled.
    root_logger = logging.getLogger()
    prev_level = root_logger.level
    if not DEBUG:
        root_logger.setLevel(logging.CRITICAL)

    try:
        # Try in this order:
        #  1) explicit override (if provided)
        #  2) discovered files from directory listings (most likely to exist)
        #  3) a small set of common guessed paths
        candidates: list[str] = []

        # Common/special paths and filesystem-like locations used by some firmwares.
        common_guesses = [
            "@MISSION",
            "@MISSION/mission.dat",
            "@MISSION/mission.waypoints",
            "@MISSION/mission.txt",
            "/@MISSION/mission.dat",
            "/APM/mission.waypoints",
            "/APM/mission.txt",
            "/APM/mission.dat",
            "/mission.dat",
            "/mission.waypoints",
            "/mission.txt",
            "/fs/microsd/APM/mission.dat",
            "/fs/microsd/APM/mission.waypoints",
            "/fs/microsd/APM/mission.txt",
        ]

        def _list_dir(d: str):
            try:
                ftp.cmd_list([d])
                entries = getattr(ftp, 'list_result', []) or []
                if DEBUG:
                    names = []
                    for ent in entries:
                        try:
                            prefix = "D" if ent.is_dir else "F"
                            names.append(f"{prefix}:{ent.name}")
                        except Exception:
                            continue
                    max_show = 40
                    shown = names[:max_show]
                    suffix = "" if len(names) <= max_show else f" ... (+{len(names) - max_show} more)"
                    print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [DEBUG {threading.get_ident()}] MAVFTP list {d}: {shown}{suffix}")
                return entries
            except Exception:
                return []

        def _join(d: str, name: str) -> str:
            if d == '/':
                return f"/{name}"
            return f"{d.rstrip('/')}/{name}"

        # Discover mission-like files by listing visible directories (bounded).
        discovered: list[tuple[str, int]] = []  # (full_path, size)

        # Start at root and only recurse into dirs we can actually list.
        base_dirs = ["/"]
        root_entries = _list_dir('/')
        for ent in root_entries:
            try:
                if ent.is_dir and ent.name not in ('.', '..'):
                    # Avoid very large/slow listings that can time out (e.g. /logs)
                    nm_l = (ent.name or '').lower()
                    if nm_l in ('logs',):
                        continue
                    # ArduPilot often exposes virtual dirs like @SYS and @ROMFS
                    base_dirs.append(_join('/', ent.name))
            except Exception:
                continue

        # Prefer known mission-related virtual/firmware dirs when present in root.
        for known in ("/APM", "/fs", "/mission", "@MISSION"):
            if known not in base_dirs:
                base_dirs.append(known)

        # De-dup base dirs
        seen_dirs: set[str] = set()
        dirs_to_probe: list[str] = []
        for d in base_dirs:
            if d not in seen_dirs:
                seen_dirs.add(d)
                dirs_to_probe.append(d)

        # Probe up to 2 levels deep in selected dirs
        def _looks_like_mission_file(name_l: str) -> bool:
            if not (name_l.endswith('.txt') or name_l.endswith('.waypoints') or name_l.endswith('.dat')):
                return False
            return ('mission' in name_l) or ('waypoint' in name_l) or name_l.endswith('.waypoints')

        def _parent_dir_and_name(p: str) -> tuple[str, str]:
            p = (p or '').strip()
            if not p or p.endswith('/'):
                return ("", "")
            if '/' not in p:
                # e.g. "@MISSION" (treated as name in root-like namespace)
                return ("/", p)
            parent, name = p.rsplit('/', 1)
            if parent == '':
                parent = '/'
            return (parent, name)

        def _dir_has_file(d: str, name: str) -> bool:
            if not d or not name:
                return False
            for ent in _list_dir(d):
                try:
                    if not ent.is_dir and ent.name == name:
                        return True
                except Exception:
                    continue
            return False

        # First level scan
        next_level_dirs: list[str] = []
        for d in dirs_to_probe:
            for ent in _list_dir(d):
                try:
                    if ent.is_dir:
                        nm = ent.name
                        if nm in ('.', '..'):
                            continue
                        # Only recurse into a small set to avoid slow walk
                        nm_l = (nm or '').lower()
                        if nm_l in ('apm', 'missions', 'mission', 'wp', 'waypoints', 'microsd') or nm_l.startswith('@'):
                            next_level_dirs.append(_join(d, nm))
                        continue
                    name = ent.name or ""
                    name_l = name.lower()
                    if _looks_like_mission_file(name_l):
                        full = _join(d, name)
                        discovered.append((full, int(getattr(ent, 'size_b', 0) or 0)))
                except Exception:
                    continue

        # Second level scan (bounded)
        seen_lvl2: set[str] = set()
        for d in next_level_dirs:
            if d in seen_lvl2:
                continue
            seen_lvl2.add(d)
            for ent in _list_dir(d):
                try:
                    if ent.is_dir:
                        continue
                    name = ent.name or ""
                    name_l = name.lower()
                    if _looks_like_mission_file(name_l):
                        full = _join(d, name)
                        discovered.append((full, int(getattr(ent, 'size_b', 0) or 0)))
                except Exception:
                    continue

        # Prefer larger mission-like files (often the actual mission).
        discovered_paths: list[str] = []
        if discovered:
            discovered.sort(key=lambda t: t[1], reverse=True)
            discovered_paths = [p for p, _sz in discovered]

        if override_path:
            candidates.append(override_path)
        candidates.extend(discovered_paths)
        candidates.extend(common_guesses)

        # Deduplicate while preserving order, and cap attempts to avoid long blocking.
        seen: set[str] = set()
        uniq_candidates: list[str] = []
        for c in candidates:
            if not c or c in seen:
                continue
            seen.add(c)
            uniq_candidates.append(c)
            if len(uniq_candidates) >= 12:
                break

        if DEBUG:
            print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [DEBUG {threading.get_ident()}] MAVFTP mission candidates: {uniq_candidates}")

        for path in uniq_candidates:
            try:
                # Avoid OpenFileRO spam by pre-checking normal file paths.
                # Do not pre-filter virtual endpoints (e.g. @MISSION) because they
                # may not appear as regular files in directory listings.
                parent, name = _parent_dir_and_name(path)
                is_virtual = path.startswith('@') or '/@' in path
                if (not is_virtual) and parent and name:
                    if not _dir_has_file(parent, name):
                        continue
                blob = ftp.read(path, size=256 * 1024, offset=0)
                if not blob:
                    continue
                pts = _parse_ardupilot_mission_dat_to_lonlat(blob)
                if pts:
                    if DEBUG:
                        print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [DEBUG {threading.get_ident()}] MAVFTP mission.dat parsed from {path}: {len(pts)} points")
                    return pts
                text = blob.decode('utf-8', errors='ignore')
                pts = _parse_qgc_wpl_text_to_lonlat(text)
                if pts:
                    if DEBUG:
                        print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [DEBUG {threading.get_ident()}] MAVFTP mission parsed from {path}: {len(pts)} points")
                    return pts
            except Exception:
                continue
        return []
    finally:
        if not DEBUG:
            root_logger.setLevel(prev_level)

# --- MAVLink Worker (Background Thread) ---
def mavlink_worker(endpoint, state):
    """
    Runs in background. Uses 'state' object directly to avoid Streamlit Context errors.
    """
    # Worker Identity for Cleanup
    worker_id = str(uuid.uuid4())
    state.current_worker_id = worker_id
    print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [SYSTEM] Starting MAVLink Worker {worker_id}")

    _mission_cache = {}
    _mission_items_cache = {}
    _mission_total = 0
    
    # Link Quality Tracking (per-source PRR + EWMA + stale-link decay)
    per_source_seq: dict[tuple[int, int], int] = {}
    per_source_history: dict[tuple[int, int], list[int]] = {}
    WINDOW_SIZE = 100
    EWMA_ALPHA = 0.2
    STALE_DECAY_START_S = 2.0
    STALE_DECAY_PER_S = 20.0
    ewma_lq = 100.0
    last_packet_ts = time.time()
    last_decay_eval_ts = time.time()
    last_sparkline_update = 0.0

    # Radio-health signal from RADIO_STATUS deltas
    radio_prev_rxerrors = None
    radio_prev_fixed = None
    radio_health_ewma = 100.0
    
    # Mission Download State
    mission_download_active = False
    mission_count_pending = False
    mission_next_seq = 0
    mission_last_req_time = 0
    _mission_requested = set()
    last_auto_mission_list_req_ts = 0.0

    # Breadcrumb gating: only record breadcrumbs when at least one GPS has RTK Fixed
    # When RTK is not present, we pause updates but keep the last trail visible.
    gps1_rtk_ok = False
    gps2_rtk_ok = False
    last_breadcrumb_ts = 0.0

    # Breadcrumb noise controls for real GPS streams.
    BREADCRUMB_MIN_STEP_M = 0.8
    BREADCRUMB_MIN_DT_S = 0.35

    def _distance_m(lon1: float, lat1: float, lon2: float, lat2: float) -> float:
        # Haversine distance in meters for small-step breadcrumb filtering.
        r = 6378137.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
        return 2.0 * r * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

    while True:
        # Check if a new worker has taken over
        if state.current_worker_id != worker_id:
            print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [SYSTEM] Stopping MAVLink Worker {worker_id} (Superseded)")
            return

        conn = None
        state.set_connection(None)
        per_source_seq = {}
        per_source_history = {}
        ewma_lq = 100.0
        last_packet_ts = time.time()
        last_decay_eval_ts = time.time()
        last_sparkline_update = 0.0
        radio_prev_rxerrors = None
        radio_prev_fixed = None
        radio_health_ewma = 100.0
        mission_download_active = False
        last_auto_mission_list_req_ts = 0.0

        gps1_rtk_ok = False
        gps2_rtk_ok = False
        
        try:
            state.append_message(f"Attempting MAVLink: {endpoint}")
            conn = mavutil.mavlink_connection(endpoint)
            
            # Wait for Autopilot Heartbeat (Ignore GCS)
            # Prefer Component 1 (Autopilot)
            found_sys = None
            found_comp = None
            start_wait = time.time()
            
            state.append_message("Scanning for vehicle...")
            while time.time() - start_wait < 10:
                msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
                if not msg:
                    continue
                
                if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                    continue
                
                # If we found an autopilot, lock on and break
                if msg.get_srcComponent() == 1:
                    found_sys = msg.get_srcSystem()
                    found_comp = msg.get_srcComponent()
                    break
                
                # Otherwise, keep the first candidate
                if found_sys is None:
                    found_sys = msg.get_srcSystem()
                    found_comp = msg.get_srcComponent()
            
            if found_sys is None:
                 raise Exception("No Vehicle Heartbeat found")
            
            conn.target_system = found_sys
            conn.target_component = found_comp
            
            state.set_connection(conn)
            state.update({'link_active': True})
            state.append_message(f"Link Established (Sys: {conn.target_system}, Comp: {conn.target_component})")
            
            # Request All Data Streams (Fallback)
            conn.mav.request_data_stream_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 2, 1
            )

            # Request Specific Data Streams
            for msg_id, hz in TELEMETRY_MESSAGE_INTERVALS:
                request_message_interval(conn, msg_id, hz)

            def _set_mission_transfer_telemetry_throttle(enabled: bool):
                """Reduce telemetry during mission transfers to prioritize MISSION_* traffic."""
                try:
                    if not enabled:
                        # Best-effort: stop legacy stream requests.
                        conn.mav.request_data_stream_send(
                            conn.target_system,
                            conn.target_component,
                            mavutil.mavlink.MAV_DATA_STREAM_ALL,
                            0,
                            0,
                        )
                        # Disable the specific message intervals we request.
                        for mid, _hz in TELEMETRY_MESSAGE_INTERVALS:
                            set_message_interval_us(conn, mid, 0)
                    else:
                        # Restore defaults.
                        conn.mav.request_data_stream_send(
                            conn.target_system,
                            conn.target_component,
                            mavutil.mavlink.MAV_DATA_STREAM_ALL,
                            2,
                            1,

                        )
                        for mid, hz in TELEMETRY_MESSAGE_INTERVALS:
                            request_message_interval(conn, mid, hz)
                except Exception:
                    pass

            def _start_mission_download():
                nonlocal mission_download_active, mission_count_pending, mission_next_seq
                nonlocal mission_last_req_time, _mission_total, _mission_cache
                nonlocal _mission_items_cache, _mission_requested

                _mission_cache = {}
                _mission_items_cache = {}
                _mission_total = 0
                _mission_requested = set()
                mission_download_active = True
                mission_count_pending = True
                mission_next_seq = 0
                mission_last_req_time = time.time()
                state.update({'mission_dl_active': True, 'mission_dl_total': 0, 'mission_dl_received': 0, 'mission_dl_done_ts': 0.0})
                _set_mission_transfer_telemetry_throttle(False)
                with state.acquire_mav_lock():
                    conn.mav.mission_request_list_send(conn.target_system, conn.target_component)

            def _request_mission_window(force: bool = False):
                nonlocal mission_last_req_time, _mission_requested

                if not mission_download_active or mission_count_pending or _mission_total <= 0:
                    return 0

                missing_window = []
                for seq in range(mission_next_seq, int(_mission_total)):
                    if seq in _mission_cache:
                        continue
                    missing_window.append(seq)
                    if len(missing_window) >= MISSION_REQUEST_WINDOW:
                        break

                if not missing_window:
                    return 0

                request_window = missing_window if force else [seq for seq in missing_window if seq not in _mission_requested]
                if not request_window:
                    return 0

                with state.acquire_mav_lock():
                    for seq in request_window:
                        conn.mav.mission_request_int_send(conn.target_system, conn.target_component, seq)
                        _mission_requested.add(seq)

                mission_last_req_time = time.time()
                if DEBUG:
                    print(
                        f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [DEBUG {threading.get_ident()}] "
                        f"Worker: Requested mission window {request_window}."
                    )
                return len(request_window)
            last_heartbeat = 0
            pending_save_wp = None
            while True:
                # Check if superseded
                if state.current_worker_id != worker_id:
                    print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [SYSTEM] Stopping MAVLink Worker {worker_id} (Superseded)")
                    conn.close()
                    return

                # Handle pending "Save Waypoint" requests.
                # We preserve the current mission by downloading full mission items (if needed),
                # then appending a NAV_WAYPOINT at the rover's current position and re-uploading.
                try:
                    while not state.save_wp_queue.empty():
                        pending_save_wp = state.save_wp_queue.get_nowait()
                except Exception:
                    pass

                if pending_save_wp is not None:
                    try:
                        snap = state.get()
                    except Exception:
                        snap = {}

                    ul_active = bool(snap.get('mission_ul_active', False))
                    dl_active = bool(snap.get('mission_dl_active', False))
                    have_items = bool(snap.get('mission_items'))

                    if ul_active or dl_active:
                        # Wait until transfers complete.
                        pass
                    elif not have_items:
                        # Need mission items to preserve the existing mission.
                        try:
                            state.append_message("[Worker] Save WP: requesting mission list (need full mission items)")
                            _start_mission_download()
                        except Exception as e:
                            state.append_message(f"[Worker] Save WP: mission list request failed: {e}")
                            pending_save_wp = None
                    else:
                        try:
                            base_items = list(snap.get('mission_items') or [])
                        except Exception:
                            base_items = []

                        try:
                            lat = float(pending_save_wp.get('lat'))
                            lon = float(pending_save_wp.get('lon'))
                        except Exception:
                            lat = None
                            lon = None

                        try:
                            alt_m = float(pending_save_wp.get('alt_m')) if pending_save_wp.get('alt_m') is not None else 0.0
                        except Exception:
                            alt_m = 0.0

                        if lat is None or lon is None:
                            state.append_message("[Worker] Save WP: missing lat/lon; ignoring")
                            pending_save_wp = None
                        else:
                            # Normalize mission items to sequential seq values.
                            new_items = []
                            for it in base_items:
                                try:
                                    new_items.append(dict(it))
                                except Exception:
                                    continue

                            for i, it in enumerate(new_items):
                                it['seq'] = int(i)
                                it['current'] = 0
                                it['autocontinue'] = int(it.get('autocontinue', 1) or 1)

                            # Append a simple NAV_WAYPOINT at current position.
                            new_items.append({
                                'seq': int(len(new_items)),
                                'current': 0,
                                'frame': int(mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT),
                                'command': int(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT),
                                'param1': 0.0,
                                'param2': 0.0,
                                'param3': 0.0,
                                'param4': 0.0,
                                'x': float(lat),
                                'y': float(lon),
                                'z': float(alt_m),
                                'autocontinue': 1,
                            })

                            # Optimistically update mission overlay in UI.
                            try:
                                pts = list(snap.get('mission_points') or [])
                                pts.append([float(lon), float(lat)])
                                state.update({'mission_points': pts, 'mission_items': new_items})
                            except Exception:
                                state.update({'mission_items': new_items})

                            state.upload_queue.put(new_items)
                            state.append_message(f"[Worker] Save WP: appended waypoint at {lat:.7f}, {lon:.7f}")
                            pending_save_wp = None

                # Handle mission fetch requests
                try:
                    if not state.mission_fetch_queue.empty():
                        # Drain queue; one fetch satisfies all pending clicks
                        while not state.mission_fetch_queue.empty():
                            _ = state.mission_fetch_queue.get_nowait()

                        state.append_message("Fetching mission...")
                        if DEBUG:

                            print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [DEBUG {threading.get_ident()}] Worker: Mission fetch requested")

                        method = (MISSION_FETCH_METHOD or "mavlink").strip().lower()
                        if method not in ("mavlink", "mavftp", "mavsdk", "auto"):
                            method = "mavlink"

                        # Default safe path: MAVLink mission protocol over the already-open connection.
                        if method == "mavlink":
                            _start_mission_download()
                        elif method in ("mavftp", "auto"):
                            pts: list[list[float]] = []
                            try:
                                with state.acquire_mav_lock():
                                    pts = _try_fetch_mission_via_mavftp(conn)
                            except Exception as e:
                                if DEBUG:
                                    print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [DEBUG {threading.get_ident()}] Worker: MAVFTP fetch error: {e}")
                            if pts:
                                state.update({'mission_points': pts})
                                state.append_message(f"Mission loaded via MAVFTP: {len(pts)} points")
                            else:
                                if method == "mavftp":
                                    state.append_message("MAVFTP mission fetch failed.")
                                else:
                                    state.append_message("MAVFTP mission fetch failed; using MAVLink mission protocol...")
                                    _start_mission_download()
                        else:
                            # Opt-in: MAVSDK (requires its own connection/port)
                            pts: list[list[float]] = []
                            try:
                                mavsdk_addr = mavsdk_mission.get_mavsdk_system_address()
                                if mavsdk_addr:
                                    if DEBUG:
                                        print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [DEBUG {threading.get_ident()}] Worker: MAVSDK download_mission via {mavsdk_addr}")
                                    pts = mavsdk_mission.download_mission_points_sync(mavsdk_addr, timeout_s=10.0)
                            except Exception as e:
                                if DEBUG:
                                    print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [DEBUG {threading.get_ident()}] Worker: MAVSDK mission download failed: {e}")
                            if pts:
                                state.update({'mission_points': pts})
                                state.append_message(f"Mission loaded via MAVSDK: {len(pts)} points")
                            else:
                                state.append_message("MAVSDK mission download failed; using MAVLink mission protocol...")
                                _start_mission_download()
                except Exception as e:
                    state.append_message(f"Mission fetch handler error: {e}")

                # Check for pending mission upload
                if not state.upload_queue.empty():
                    try:
                        mission_items = state.upload_queue.get_nowait()
                        state.set_upload_status('uploading', 'Starting upload...')
                        state.update({'mission_ul_active': True, 'mission_ul_total': len(mission_items), 'mission_ul_sent': 0, 'mission_ul_done_ts': 0.0})

                        def _upload_progress(sent: int, total: int):
                            # Keep updates cheap; UI reads from shared state.
                            state.update({'mission_ul_active': True, 'mission_ul_total': int(total), 'mission_ul_sent': int(sent)})
                            state.set_upload_status('uploading', f'Uploading {sent}/{total}...')

                        success = False
                        upload_method = MISSION_UPLOAD_METHOD

                        # Optional fast path for ArduPilot builds that expose mission files via MAVFTP.
                        if upload_method in ("mavftp", "auto"):
                            try:
                                with state.acquire_mav_lock():
                                    success = _try_upload_mission_via_mavftp(conn, mission_items, progress_cb=_upload_progress)
                            except Exception:
                                success = False

                        if not success and upload_method in ("mavlink", "auto"):
                            # Use the existing connection to upload.
                            # Throttle telemetry to speed up MISSION_REQUEST/ITEM exchanges.
                            _set_mission_transfer_telemetry_throttle(False)
                            try:
                                success = upload_mission(conn, mission_items, progress_cb=_upload_progress)
                            finally:
                                _set_mission_transfer_telemetry_throttle(True)

                        if success:
                            state.set_upload_status('success', 'Upload complete!')
                            state.update({'mission_ul_active': False, 'mission_ul_done_ts': time.time()})
                        else:
                            state.set_upload_status('error', f'Upload failed ({upload_method}).')
                            state.update({'mission_ul_active': False, 'mission_ul_done_ts': time.time()})
                    except Exception as e:
                        state.set_upload_status('error', f'Upload error: {e}')
                        state.update({'mission_ul_active': False, 'mission_ul_done_ts': time.time()})

                # Send Heartbeat to GCS every 1s
                if time.time() - last_heartbeat > 1.0:
                    conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                    last_heartbeat = time.time()
                
                # Mission Download Retry Logic
                if mission_download_active and time.time() - mission_last_req_time > MISSION_RETRY_INTERVAL_S:
                    if mission_count_pending:
                        print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [MAVLINK] Retry requesting mission list")
                        with state.acquire_mav_lock():
                            conn.mav.mission_request_list_send(conn.target_system, conn.target_component)
                        mission_last_req_time = time.time()
                    elif mission_next_seq < _mission_total:
                        print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [MAVLINK] Retry requesting mission window from {mission_next_seq}")
                        _request_mission_window(force=True)
                    else:
                        mission_download_active = False
                        mission_count_pending = False
                        _mission_requested = set()
                        _set_mission_transfer_telemetry_throttle(True)

                msg = conn.recv_match(blocking=True, timeout=WORKER_RECV_TIMEOUT_S)
                if not msg:
                    now_ts = time.time()
                    # Decay link quality when packets stop arriving.
                    idle_s = now_ts - last_packet_ts
                    if idle_s > STALE_DECAY_START_S:
                        dt = max(0.0, now_ts - last_decay_eval_ts)
                        if dt > 0:
                            ewma_lq = max(0.0, ewma_lq - (STALE_DECAY_PER_S * dt))
                            decay_data = {'link_quality': int(round(ewma_lq))}
                            if now_ts - last_sparkline_update > 1.0:
                                current_hist = state.get().get('link_quality_history', [])
                                current_hist.append(int(round(ewma_lq)))
                                if len(current_hist) > 50:
                                    current_hist = current_hist[-50:]
                                decay_data['link_quality_history'] = current_hist
                                last_sparkline_update = now_ts
                            state.update(decay_data)
                    last_decay_eval_ts = now_ts
                    # Check for link timeout
                    if time.time() - state.get()['last_update'] > 5:
                        state.update({'link_active': False})
                    continue

                mtype = msg.get_type()
                if mtype == 'BAD_DATA':
                    continue

                now_ts = time.time()
                last_packet_ts = now_ts
                last_decay_eval_ts = now_ts

                data = {'link_active': True}

                # Link Quality Calculation
                try:
                    seq = msg.get_seq()
                    src_key = (int(msg.get_srcSystem()), int(msg.get_srcComponent()))
                    src_last_seq = per_source_seq.get(src_key)
                    src_hist = per_source_history.get(src_key)
                    if src_hist is None:
                        src_hist = []

                    accepted_for_accounting = False
                    if src_last_seq is None:
                        src_hist.append(1)
                        per_source_seq[src_key] = seq
                        accepted_for_accounting = True
                    else:
                        diff = (seq - src_last_seq) % 256
                        # diff values:
                        # - 1: normal next packet
                        # - 0: duplicate packet (ignore for LQ)
                        # - >128: likely out-of-order/backwards jump (ignore for LQ)
                        if diff != 0 and diff <= 128:
                            lost = max(0, diff - 1)
                            if lost > 0:
                                src_hist.extend([0] * lost)
                            src_hist.append(1)
                            per_source_seq[src_key] = seq
                            accepted_for_accounting = True

                    if len(src_hist) > WINDOW_SIZE:
                        src_hist = src_hist[-WINDOW_SIZE:]
                    per_source_history[src_key] = src_hist

                    if accepted_for_accounting:
                        total_seen = 0
                        total_received = 0
                        for hist in per_source_history.values():
                            total_seen += len(hist)
                            total_received += sum(hist)

                        if total_seen > 0:
                            prr = (total_received / total_seen) * 100.0
                            ewma_lq = (EWMA_ALPHA * prr) + ((1.0 - EWMA_ALPHA) * ewma_lq)
                            est_lost = max(0, total_seen - total_received)

                            data['link_quality'] = int(round(ewma_lq))
                            data['packets_received'] = int(total_received)
                            data['packets_lost'] = int(est_lost)

                            if now_ts - last_sparkline_update > 1.0:
                                current_hist = state.get().get('link_quality_history', [])
                                current_hist.append(int(round(ewma_lq)))
                                if len(current_hist) > 50:
                                    current_hist = current_hist[-50:]
                                data['link_quality_history'] = current_hist
                                last_sparkline_update = now_ts
                except Exception:
                    pass

                if mtype == 'HEARTBEAT':
                    data['mode'] = get_mode_name(msg.custom_mode)
                    data['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

                    # If we're in AUTO and we don't yet know total waypoints, request mission list.
                    # This prompts the vehicle to send MISSION_COUNT (and we can then compute mission progress).
                    try:
                        mode_name = str(data.get('mode') or '')
                        known_total = int(state.get().get('mission_dl_total') or 0)
                    except Exception:
                        mode_name = ''
                        known_total = 0

                    now_ts = time.time()
                    if (
                        mode_name == 'AUTO'
                        and known_total <= 0
                        and (not mission_download_active)
                        and (now_ts - last_auto_mission_list_req_ts) >= 5.0
                    ):
                        try:
                            _start_mission_download()
                            last_auto_mission_list_req_ts = now_ts
                            state.append_message("[Worker] AUTO mode: requesting mission list to compute progress")
                        except Exception:
                            last_auto_mission_list_req_ts = now_ts

                elif mtype == 'GLOBAL_POSITION_INT':
                    if msg.lat != 0 and msg.lon != 0:
                        data['lat'] = msg.lat / 1e7
                        data['lon'] = msg.lon / 1e7
                        data['heading_deg'] = msg.hdg / 100.0
                        try:
                            # relative_alt is in millimeters
                            data['alt_m'] = float(getattr(msg, 'relative_alt', 0) or 0) / 1000.0
                        except Exception:
                            pass

                        # Breadcrumbs (only when RTK Fixed is available on at least one GPS)
                        breadcrumbs_enabled = bool(gps1_rtk_ok or gps2_rtk_ok)
                        if breadcrumbs_enabled:
                            try:
                                snap_hist = state.get().get('history') or []
                                history = list(snap_hist)
                            except Exception:
                                history = []
                            pt = [float(data['lon']), float(data['lat'])]
                            now_breadcrumb_ts = time.time()
                            if not history:
                                history.append(pt)
                                last_breadcrumb_ts = now_breadcrumb_ts
                            else:
                                last = history[-1]
                                moved_m = _distance_m(float(last[0]), float(last[1]), pt[0], pt[1])
                                dt_ok = (now_breadcrumb_ts - last_breadcrumb_ts) >= BREADCRUMB_MIN_DT_S
                                if moved_m >= BREADCRUMB_MIN_STEP_M and dt_ok:
                                    history.append(pt)
                                    last_breadcrumb_ts = now_breadcrumb_ts
                            if len(history) > 5000:
                                history = history[-5000:]
                            data['history'] = history
                
                elif mtype == 'GPS_RAW_INT':
                    if msg.lat != 0 and msg.lon != 0:
                        data['lat'] = msg.lat / 1e7
                        data['lon'] = msg.lon / 1e7

                    # Update RTK gating state for GPS1 (RTK Fixed only)
                    try:
                        gps1_rtk_ok = int(getattr(msg, 'fix_type', 0) or 0) == 6
                    except Exception:
                        gps1_rtk_ok = False
                    # Intentionally do not append breadcrumbs from GPS_RAW_INT.
                    # Breadcrumb path should use only GLOBAL_POSITION_INT to avoid
                    # alternating between fused and raw GPS positions (zig-zag trail).

                    data['gps1_fix'] = get_gps_fix_string(msg.fix_type)
                    data['satellites_visible'] = msg.satellites_visible
                    try:
                        data['gps1_h_acc_m'] = round(float(getattr(msg, 'h_acc', 0) or 0) / 1000.0, 2)
                    except Exception:
                        data['gps1_h_acc_m'] = None

                elif mtype == 'MISSION_CURRENT':
                    seq = int(getattr(msg, 'seq', 0) or 0)
                    data['wp_current_raw'] = seq

                    # Keep UI WP stable across disarm/stop: some vehicles report seq=0 when not running.
                    # Only let the UI reset to 0 when we believe the mission has restarted.
                    try:
                        snap = state.get()
                    except Exception:
                        snap = {}
                    prev_known = bool(snap.get('wp_current_known', False))
                    prev_mode = str(snap.get('mode') or '')
                    prev_armed = bool(snap.get('armed', False))

                    if seq > 0:
                        data['wp_current'] = seq
                    elif not prev_known:
                        # First ever report can legitimately be 0.
                        data['wp_current'] = 0
                    elif prev_armed and prev_mode == 'AUTO':
                        # Treat as mission restart.
                        data['wp_current'] = 0

                elif mtype == 'GPS2_RAW':
                    if msg.lat != 0 and msg.lon != 0:
                        data['gps2_lat'], data['gps2_lon'] = msg.lat / 1e7, msg.lon / 1e7

                    # Update RTK gating state for GPS2 (RTK Fixed only)
                    try:
                        gps2_rtk_ok = int(getattr(msg, 'fix_type', 0) or 0) == 6
                    except Exception:
                        gps2_rtk_ok = False

                    data['gps2_fix'] = get_gps_fix_string(msg.fix_type)
                    data['gps2_satellites_visible'] = msg.satellites_visible
                    try:
                        data['gps2_h_acc_m'] = round(float(getattr(msg, 'h_acc', 0) or 0) / 1000.0, 2)
                    except Exception:
                        data['gps2_h_acc_m'] = None
                    try:
                        gps2_yaw_cdeg = getattr(msg, 'yaw', None)
                        if gps2_yaw_cdeg is None:
                            data['gps2_yaw_deg'] = None
                        else:
                            gps2_yaw_cdeg = int(gps2_yaw_cdeg)
                            data['gps2_yaw_deg'] = None if gps2_yaw_cdeg == 65535 else round(gps2_yaw_cdeg / 100.0, 2)
                    except Exception:
                        data['gps2_yaw_deg'] = None

                elif mtype == 'COMMAND_LONG':
                    cmd_id = int(getattr(msg, 'command', -1))
                    if cmd_id in {181, 182, 184}: # DO_SET_RELAY
                        relay_idx = int(float(getattr(msg, 'param1', 0.0)))
                        state_val = int(float(getattr(msg, 'param2', 0.0)))
                        # Convert ap_relay_index back to RELAY_NUM (1-based)
                        relay_num = relay_idx + 1
                        current_relays = state.get().get("relays", {})
                        current_relays[relay_num] = state_val != 0
                        data["relays"] = current_relays

                elif mtype == 'SYS_STATUS':
                    data['battery_v'] = msg.voltage_battery / 1000.0
                    data['battery_pct'] = msg.battery_remaining

                elif mtype == 'VFR_HUD':
                    data['speed_ms'] = round(msg.groundspeed, 2)
                    data['heading_deg'] = msg.heading

                elif mtype == 'RADIO_STATUS':
                    try:
                        data['radio_rssi'] = int(getattr(msg, 'rssi', 0) or 0)
                        data['radio_remrssi'] = int(getattr(msg, 'remrssi', 0) or 0)
                        data['radio_rxerrors'] = int(getattr(msg, 'rxerrors', 0) or 0)
                        data['radio_fixed'] = int(getattr(msg, 'fixed', 0) or 0)

                        curr_rxerrors = data['radio_rxerrors']
                        curr_fixed = data['radio_fixed']
                        if radio_prev_rxerrors is not None and radio_prev_fixed is not None:
                            d_rx = curr_rxerrors - radio_prev_rxerrors
                            d_fix = curr_fixed - radio_prev_fixed
                            if d_rx >= 0 and d_fix >= 0:
                                denom = d_rx + d_fix
                                if denom > 0:
                                    inst_health = (d_fix / denom) * 100.0
                                    radio_health_ewma = (0.3 * inst_health) + (0.7 * radio_health_ewma)
                                    data['radio_health'] = int(round(max(0.0, min(100.0, radio_health_ewma))))

                        radio_prev_rxerrors = curr_rxerrors
                        radio_prev_fixed = curr_fixed
                    except Exception:
                        pass

                elif mtype == 'STATUSTEXT':
                    txt = str(msg.text)
                    state.append_message(txt)
                    print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [STATUSTEXT] setting {txt}")
                    
                    lower_txt = txt.lower()
                    if 'failsafe' in lower_txt:
                        clearing = any(kw in lower_txt for kw in ['end', 'ended', 'recovered', 'cleared'])
                        if clearing:
                            data['failsafe_active'] = False
                            data['failsafe_desc'] = ""
                        else:
                            data['failsafe_active'] = True
                            data['failsafe_desc'] = txt

                    # # Auto-reset mission when complete
                    # if "Mission Complete" in txt:
                    #     print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [SYSTEM] Mission Complete detected. Resetting WP to 0.")
                    #     try:
                    #         with state.acquire_mav_lock():
                    #             cmd_conn.mav.mission_set_current_send(conn.target_system, conn.target_component, 0)
                    #         state.append_message("Mission Reset to WP 0")
                    #     except Exception as e:
                    #         print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [SYSTEM] Failed to reset mission: {e}")

                    # Parse "Mission: #" messages (ArduPilot)
                    # Also handle "Reached command #"
                    wp_num = -1
                    if "Mission:" in txt:
                        try:
                            parts = txt.split(':')
                            if len(parts) > 1:
                                wp_num = int(parts[1].strip())
                        except:
                            pass
                    elif "Reached command #" in txt:
                        try:
                            parts = txt.split('#')
                            if len(parts) > 1:
                                wp_num = int(parts[1].strip())
                        except:
                            pass
                            
                    if wp_num >= 0:
                        data['wp_current_raw'] = int(wp_num)
                        # Apply the same "stable display" rule as MISSION_CURRENT.
                        try:
                            snap = state.get()
                        except Exception:
                            snap = {}
                        prev_known = bool(snap.get('wp_current_known', False))
                        prev_mode = str(snap.get('mode') or '')
                        prev_armed = bool(snap.get('armed', False))

                        if wp_num > 0:
                            data['wp_current'] = int(wp_num)
                        elif not prev_known:
                            data['wp_current'] = 0
                        elif prev_armed and prev_mode == 'AUTO':
                            data['wp_current'] = 0

                elif mtype == 'MISSION_ACK':
                    print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [MAVLINK {threading.get_ident()}] Mission Ack: {msg.type} from {msg.get_srcSystem()}:{msg.get_srcComponent()}")

                elif mtype == 'MISSION_REQUEST_INT' or mtype == 'MISSION_REQUEST':
                     print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [MAVLINK {threading.get_ident()}] Mission Request {msg.seq} from {msg.get_srcSystem()}:{msg.get_srcComponent()}")

                elif mtype == 'MISSION_COUNT':
                    src_sys = msg.get_srcSystem()
                    src_comp = msg.get_srcComponent()
                    
                    # Check mission_type if available (MAVLink 2)
                    m_type = getattr(msg, 'mission_type', 0)
                    
                    print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [MAVLINK] Mission Count: {msg.count} from {src_sys}:{src_comp} (Type: {m_type})")
                    
                    # Filter out messages from non-target components (e.g. cameras)
                    if src_sys != conn.target_system or src_comp != conn.target_component:
                        print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [MAVLINK] Ignored MISSION_COUNT from {src_sys}:{src_comp} (Target: {conn.target_system}:{conn.target_component})")
                        continue
                        
                    # Filter out non-main mission types (e.g. Fence=1, Rally=2)
                    if m_type != 0:
                        print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [MAVLINK] Ignored MISSION_COUNT for type {m_type}")
                        continue

                    # Prevent restarting download if already active for same count
                    if mission_download_active and _mission_total == msg.count:
                        print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [MAVLINK] Ignoring duplicate MISSION_COUNT (Already downloading)")
                        continue

                    _mission_cache = {}
                    _mission_items_cache = {}
                    _mission_requested = set()
                    _mission_total = msg.count
                    mission_count_pending = False
                    state.update({'mission_dl_active': True, 'mission_dl_total': int(_mission_total), 'mission_dl_received': 0, 'mission_dl_done_ts': 0.0})
                    if DEBUG:
                        print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [DEBUG {threading.get_ident()}] Worker: Received MISSION_COUNT {_mission_total}. Starting download.")
                    if _mission_total > 0:
                        mission_download_active = True
                        mission_next_seq = 0
                        _request_mission_window()
                    else:
                        # Handle empty mission
                        mission_download_active = False
                        _set_mission_transfer_telemetry_throttle(True)
                        state.update({'mission_points': []})
                        state.update({'mission_items': []})
                        state.append_message("Mission loaded: 0 points")
                        with state.acquire_mav_lock():
                            conn.mav.mission_ack_send(conn.target_system, conn.target_component, 0)
                        if DEBUG:
                            print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [DEBUG {threading.get_ident()}] Worker: Empty mission. Sent ACK.")

                elif mtype == 'MISSION_ITEM_INT' or mtype == 'MISSION_ITEM':
                    # Ignore stray mission items if we aren't actively downloading
                    if not mission_download_active:
                        continue

                    src_sys = msg.get_srcSystem()
                    src_comp = msg.get_srcComponent()
                    
                    if src_sys != conn.target_system or src_comp != conn.target_component:
                        continue

                    # Check mission_type if available
                    m_type = getattr(msg, 'mission_type', 0)
                    if m_type != 0:
                        continue

                    # Handle both INT and float versions
                    if mtype == 'MISSION_ITEM_INT':
                        lat = msg.x / 1e7
                        lon = msg.y / 1e7
                    else:
                        lat = msg.x
                        lon = msg.y
                    if DEBUG:
                        print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [DEBUG {threading.get_ident()}] Worker: Received item {msg.seq}/{_mission_total}: {lat}, {lon}")

                    # Store as [lon, lat] for PyDeck
                    _mission_cache[msg.seq] = [lon, lat]

                    # Also cache the full mission item (so we can later append a waypoint without losing commands).
                    try:
                        _mission_items_cache[msg.seq] = {
                            'seq': int(getattr(msg, 'seq', 0) or 0),
                            'current': int(getattr(msg, 'current', 0) or 0),
                            'frame': int(getattr(msg, 'frame', 0) or 0),
                            'command': int(getattr(msg, 'command', 0) or 0),
                            'param1': float(getattr(msg, 'param1', 0.0) or 0.0),
                            'param2': float(getattr(msg, 'param2', 0.0) or 0.0),
                            'param3': float(getattr(msg, 'param3', 0.0) or 0.0),
                            'param4': float(getattr(msg, 'param4', 0.0) or 0.0),
                            # Store x/y as degrees when global; upload_mission() will convert to INT as needed.
                            'x': float(lat),
                            'y': float(lon),
                            'z': float(getattr(msg, 'z', 0.0) or 0.0),
                            'autocontinue': int(getattr(msg, 'autocontinue', 1) or 1),
                        }
                    except Exception:
                        pass
                    # Progress
                    try:
                        state.update({'mission_dl_received': int(len(_mission_cache))})
                    except Exception:
                        pass
                    _mission_requested.discard(int(msg.seq))
                    
                    if msg.seq == mission_next_seq:
                        mission_next_seq += 1
                    while mission_next_seq in _mission_cache:
                        mission_next_seq += 1

                    if mission_next_seq < _mission_total:
                        _request_mission_window()
                    elif mission_download_active:
                        mission_download_active = False
                        mission_count_pending = False
                        _mission_requested = set()
                        _set_mission_transfer_telemetry_throttle(True)
                        sorted_pts = [v for k, v in sorted(_mission_cache.items())]
                        state.update({'mission_points': sorted_pts})
                        try:
                            sorted_items = [v for k, v in sorted(_mission_items_cache.items())]
                        except Exception:
                            sorted_items = []
                        state.update({'mission_items': sorted_items})
                        state.update({'mission_dl_active': False, 'mission_dl_total': int(_mission_total), 'mission_dl_received': int(len(sorted_pts)), 'mission_dl_done_ts': time.time()})
                        state.append_message(f"Mission loaded: {len(sorted_pts)} points")
                        with state.acquire_mav_lock():
                            conn.mav.mission_ack_send(conn.target_system, conn.target_component, 0)
                        if DEBUG:
                            print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [DEBUG {threading.get_ident()}] Worker: Download complete. Updated state with {len(sorted_pts)} points. Sent ACK.")

                if data:
                    state.update(data)

        except Exception as e:
            state.set_connection(None)
            state.update({'link_active': False})
            state.append_message(f"Worker Exception: {str(e)}")
            time.sleep(5) # Cooldown before reconnect

@st.cache_resource
def start_background_threads():
    print("Starting background MAVLink worker thread...")
    shared_state = get_shared_state()
    t = threading.Thread(
        target=mavlink_worker, 
        args=(MAVLINK_ENDPOINT, shared_state), 
        daemon=True
    )
    t.start()
    return True

# --- Helper Functions ---
def get_offset_point(lat, lon, dist, bearing_deg):
    r_earth = 6378137
    bearing_rad = math.radians(bearing_deg)
    dy = dist * math.cos(bearing_rad)
    dx = dist * math.sin(bearing_rad)
    d_lat = (dy / r_earth) * (180 / math.pi)
    d_lon = (dx / r_earth) * (180 / math.pi) / math.cos(math.radians(lat))
    return [lon + d_lon, lat + d_lat]

def generate_sparkline(min_val, max_val, data, width=280, height=40, color="#4caf50"):
    if not data or len(data) < 2:
        return ""
    # min_val = 0
    # max_val = 100
    points = []

    # Fixed max points to ensure scrolling effect (matches history buffer size)
    max_points = 50
    step = width / (max_points - 1)

    for i, val in enumerate(data):
        x = i * step
        y = height - ((val - min_val) / (max_val - min_val)) * height
        points.append(f"{x},{y}")
    
    polyline = " ".join(points)
    # return f"""
    # <svg width="100%" height="{height}" viewBox="0 0 {width} {height}" preserveAspectRatio="none" style="background-color: rgba(255,255,255,0.05); border-radius: 3px; margin-bottom: 5px;">
    #     <polyline points="{polyline}" fill="none" stroke="{color}" stroke-width="2" />
    # </svg>
    # """
    return f"""
    <svg width="100%" height="{height}" viewBox="0 0 {width} {height}" preserveAspectRatio="none" style="background-color: rgba(255,255,255,0.05); border-radius: 3px; margin-bottom: 5px;">
    <polygon 
        points="{polyline} {width},{height} 0,{height}" 
        fill="{color}" 
        fill-opacity="0.3" 
        stroke="none" 
    />
    <polyline points="{polyline}" fill="none" stroke="{color}" stroke-width="2" />
    </svg>
    """

# --- UI Functions ---
def create_map_deck(data, map_style=None):
    # if DEBUG:
    #     print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [DEBUG] Map: Creating map deck.")
    lat, lon = data.get('lat'), data.get('lon')
    if not lat or lat == 0:
        if DEBUG:
            print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [DEBUG] Map: No valid lat/lon. Skipping.")
        return None

    # Guard against lat=90/-90 for math.cos
    if abs(lat) > 89.9:
        lat = 89.9 if lat > 0 else -89.9

    view_state = pdk.ViewState(latitude=lat, longitude=lon, zoom=19, pitch=0)
    
    # Rover visual marker (Triangle)
    heading = data.get('heading_deg', 0)
    # Size in meters
    size = 0.4 
    
    nose = get_offset_point(lat, lon, size, heading)
    rl = get_offset_point(lat, lon, size, heading - 145)
    rr = get_offset_point(lat, lon, size, heading + 145)
    
    layers = []

    # Breadcrumbs Path
    history = data.get('history', [])
    # Limit history rendering to prevent browser crash
    if len(history) > 2000:
        # Downsample
        step = len(history) // 2000 + 1
        history = history[::step]

    if len(history) > 1:
        layers.append(pdk.Layer(
            "PathLayer",
            data=[{"path": history}],
            get_path="path",
            get_color=[255, 255, 0, 255], # Bright yellow trail (fully opaque)
            width_min_pixels=1,
            get_width=0.05, # meters
            width_units='"meters"'
        ))

    # Mission Path
    mission_pts = data.get('mission_points', [])
    wp_current = data.get('wp_current', 0)

    if mission_pts:
        # mission_pts from worker are already [lon, lat]
        # Also limit points
        if len(mission_pts) > 2000:
             mission_pts = mission_pts[:2000] # Truncate for safety
        
        mission_pts_lonlat = mission_pts

        if len(mission_pts_lonlat) > 1:
            # Split into Completed (Green) and Pending (White)
            split_idx = max(1, wp_current)
            # Start from index 1 to exclude Home
            completed_path = mission_pts_lonlat[1:split_idx]
            # Ensure pending starts at least at index 1
            pending_path = mission_pts_lonlat[max(1, split_idx-1):]

            if len(completed_path) > 1:
                layers.append(pdk.Layer(
                    "PathLayer",
                    data=[{"path": completed_path}],
                    get_path="path",
                    get_color=[0, 255, 0, 200], # Green line
                    width_min_pixels=1,
                    get_width=0.05,
                    width_units='"meters"',
                ))

            if len(pending_path) > 1:
                layers.append(pdk.Layer(
                    "PathLayer",
                    data=[{"path": pending_path}],
                    get_path="path",
                    get_color=[255, 255, 255, 200], # White line
                    width_min_pixels=1,
                    get_width=0.05,
                    width_units='"meters"',
                    dash_justified=True,
                ))

        # Mission Waypoints (Dots)
        waypoint_data = []
        for i, p in enumerate(mission_pts_lonlat):
            # Skip Home (Index 0)
            if i == 0:
                continue
            # Green for completed (index < current), White for pending
            color = [0, 255, 0, 255] if i < wp_current else [255, 255, 255, 255]
            waypoint_data.append({"pos": p, "color": color})

        layers.append(pdk.Layer(
            "ScatterplotLayer",
            data=waypoint_data,
            get_position="pos",
            get_color="color",
            get_radius=0.15,
            radius_units='"meters"',
        ))

    # Rover Triangle
    layers.append(pdk.Layer(
        "PolygonLayer",
        data=[{"poly": [nose, rr, rl]}],
        get_polygon="poly",
        get_fill_color=[255, 0, 0, 255],
        get_line_color=[200, 0, 0, 255],
        filled=True,
        stroked=True,
        line_width_min_pixels=1,
    ))

    if not map_style:
        map_style = "mapbox://styles/mapbox/satellite-v9" if MAPBOX_API_KEY_VALID else "dark"

    return pdk.Deck(
        initial_view_state=view_state, 
        layers=layers, 
        tooltip=True, 
        map_style=map_style,
        map_provider=MAP_BASE_PROVIDER,
        api_keys={"mapbox": MAPBOX_API_KEY} if MAPBOX_API_KEY_VALID else None
    )


def _map_signature(data: dict, map_style_name: str) -> tuple:
    """Return a stable signature for when the *map visualization* should update.

    Streamlit reruns re-execute the script; to avoid unnecessary PyDeck churn, we only
    rebuild the Deck when map-relevant inputs change.
    """
    lat = data.get('lat')
    lon = data.get('lon')
    heading = data.get('heading_deg', 0)
    wp_current = int(data.get('wp_current') or 0)
    mission_pts = data.get('mission_points') or []
    # Compress mission identity without hashing full arrays.
    if mission_pts:
        first = mission_pts[0]
        last = mission_pts[-1]
        try:
            first_sig = (round(float(first[0]), 6), round(float(first[1]), 6))
        except Exception:
            first_sig = None
        try:
            last_sig = (round(float(last[0]), 6), round(float(last[1]), 6))
        except Exception:
            last_sig = None
        mission_sig = (len(mission_pts), first_sig, last_sig)
    else:
        mission_sig = (0, None, None)

    history = data.get('history') or []
    if history:
        try:
            h_last = history[-1]
            history_sig = (len(history), round(float(h_last[0]), 6), round(float(h_last[1]), 6))
        except Exception:
            history_sig = (len(history), None, None)
    else:
        history_sig = (0, None, None)

    try:
        lat_sig = None if lat is None else round(float(lat), 6)
    except Exception:
        lat_sig = None
    try:
        lon_sig = None if lon is None else round(float(lon), 6)
    except Exception:
        lon_sig = None

    try:
        heading_sig = round(float(heading or 0), 1)
    except Exception:
        heading_sig = 0

    return (
        map_style_name,
        lat_sig,
        lon_sig,
        heading_sig,
        wp_current,
        mission_sig,
        history_sig,
    )

# --- MAIN APP ---
start_background_threads()
start_telegram_bridge()
# st_autorefresh removed in favor of while loop

# NOTE: Sidebar UI is rendered via fragments later (see bottom of file).

# Custom CSS for Metrics
st.markdown("""
    <style>
    [data-testid="stMetric"] {
        display: flex;
        flex-direction: column;
        align-items: center;
        text-align: center;
    }
    [data-testid="stMetricLabel"] {
        font-size: 1rem !important;
        width: 100%;
        justify-content: center;
        display: flex;
    }
    [data-testid="stMetricValue"] {
        font-size: 1.5rem !important;
        width: 100%;
        justify-content: center;
        display: flex;
    }
    </style>
    """, unsafe_allow_html=True)

# white line separators
st.markdown('<hr style="border:0;border-top:2px solid #fff;margin:8px 0 8px 0;">', unsafe_allow_html=True)

# Horizontal Telemetry Header Placeholders
c1, c2, c3, c4, c5, c6 = st.columns(6)
metric_mode = c1.empty()
metric_armed = c2.empty()
metric_speed = c3.empty()
metric_gps = c4.empty()
metric_battery = c5.empty()
metric_wp = c6.empty()

st.markdown('<hr style="border:0;border-top:2px solid #fff;margin:8px 0 8px 0;">', unsafe_allow_html=True)

# Mission Progress Placeholder (above map)
mission_progress_placeholder = st.empty()

# Map Placeholder
map_placeholder = st.empty()

st.markdown('<hr style="border:0;border-top:2px solid #fff;margin:8px 0 8px 0;">', unsafe_allow_html=True)

# System Console Placeholder
st.markdown("**System Messages**")
console_placeholder = st.empty()

if 'history' not in st.session_state:
    st.session_state.history = []

print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [App] Starting main update loop...")
# --- Live UI (no full-page rerun loop) ---
#
# Using st.fragment lets Streamlit re-run ONLY this section on a timer.
# That avoids full-page flicker and reduces map remount/redraw.
@st.fragment(run_every=0.5)
def _render_live_metrics():
    current_data = get_shared_state().get()
    metric_mode.metric("Mode", current_data.get('mode'))
    metric_armed.metric("Armed", "ARMED" if current_data.get('armed') else "DISARMED")
    metric_speed.metric("Speed", f"{current_data.get('speed_ms')} m/s")
    metric_gps.metric("GPS", current_data.get('gps1_fix'))
    metric_battery.metric("Battery", f"{current_data.get('battery_v')}V")
    metric_wp.metric("WP", f"{current_data.get('wp_current', 0)}")


@st.fragment(run_every=0.5)
def _render_live_sidebar():
    current_data = get_shared_state().get()

    now_ts = time.time()
    last_ts = float(current_data.get('last_update') or 0)
    age_s = (now_ts - last_ts) if last_ts else 999.0

    # IMPORTANT: This fragment must be invoked inside `with st.sidebar:`.
    # Do NOT use `st.sidebar.*` or sidebar placeholders here.

    # Status
    if current_data.get('link_active'):
        dbg_line = ""
        if DEBUG:
            dbg_line = f"<br><span style='font-size:0.75em; color:gray'>age={age_s:.1f}s</span>"
        st.markdown(
            f"✅ **ROVER ONLINE**{dbg_line}<br><span style='font-size:0.8em; color:gray'>Last Update: {datetime.datetime.now().strftime('%H:%M:%S')}</span>",
            unsafe_allow_html=True,
        )
    else:
        if DEBUG:
            st.markdown(
                f"⚠️ **OFFLINE**<br><span style='font-size:0.75em; color:gray'>age={age_s:.1f}s</span>",
                unsafe_allow_html=True,
            )
        else:
            st.error("OFFLINE: Searching...")

    # Mission transfer progress
    dl_active = bool(current_data.get('mission_dl_active'))
    dl_total = int(current_data.get('mission_dl_total') or 0)
    dl_recv = int(current_data.get('mission_dl_received') or 0)
    dl_done_ts = float(current_data.get('mission_dl_done_ts') or 0)
    if dl_active:
        if dl_total > 0:
            st.progress(min(1.0, dl_recv / dl_total), text=f"Mission download {dl_recv}/{dl_total}")
        else:
            st.progress(0.0, text="Mission download: waiting for count…")
    else:
        if dl_total > 0 and (time.time() - dl_done_ts) <= MISSION_PROGRESS_GRACE_S:
            st.progress(1.0, text=f"Mission download done {dl_recv}/{dl_total}")

    ul_active  = bool(current_data.get('mission_ul_active'))
    ul_total   = int(current_data.get('mission_ul_total') or 0)
    ul_sent    = int(current_data.get('mission_ul_sent') or 0)
    ul_done_ts = float(current_data.get('mission_ul_done_ts') or 0)

    if ul_active:
        if ul_total > 0:
            st.progress(min(1.0, ul_sent / ul_total), text=f"Mission upload {ul_sent}/{ul_total}")
        else:
            st.markdown("**Mission upload:** starting…")
    else:
        if ul_total > 0 and (time.time() - ul_done_ts) <= MISSION_PROGRESS_GRACE_S:
            st.progress(1.0, text=f"Mission upload done {ul_sent}/{ul_total}")

    # Link quality + GPS blocks
    lq = current_data.get('link_quality', 100)
    lq_color = "#4caf50" if lq >= 90 else "#ff9800" if lq >= 70 else "#f44336"
    lq_hist = current_data.get('link_quality_history', [])
    radio_health = current_data.get('radio_health', None)
    sparkline_svg = generate_sparkline(0, 100,lq_hist, color=lq_color)
    st.markdown(sparkline_svg, unsafe_allow_html=True)
    st.markdown(f":material/signal_cellular_alt: **Link Quality:** {lq}%")
    if radio_health is not None:
        st.markdown(f":material/network_cell: **Radio Health:** {int(radio_health)}%")

    # Box Temp sparkline (from MQTT_VAR1_TOPIC -> SharedState.mqtt_var1)
    try:
        box_temp_val = float(current_data.get('mqtt_var1', 0) or 0)
    except Exception:
        box_temp_val = 0.0
    box_temp_i = int(round(box_temp_val))

    if "mqtt_var1_history" not in st.session_state:
        st.session_state["mqtt_var1_history"] = []
    if "_last_mqtt_var1_hist_ts" not in st.session_state:
        st.session_state["_last_mqtt_var1_hist_ts"] = 0.0

    now_ts = time.time()
    if (now_ts - float(st.session_state.get("_last_mqtt_var1_hist_ts") or 0.0)) >= 1.0:
        hist = list(st.session_state.get("mqtt_var1_history") or [])
        hist.append(box_temp_i)
        if len(hist) > 50:
            hist = hist[-50:]
        st.session_state["mqtt_var1_history"] = hist
        st.session_state["_last_mqtt_var1_hist_ts"] = now_ts

    bt_hist = st.session_state.get("mqtt_var1_history") or []
    lq_color =  "#f44336" if box_temp_i > 85 else "#ff9800" if box_temp_i > 0 else "#4caf50"
    st.markdown(generate_sparkline(32, 130,bt_hist, color=lq_color), unsafe_allow_html=True)
    st.markdown(f":material/device_thermostat: **Box Temperature:** {box_temp_i} °F")

    gps_html = f"""
<div style="line-height: 1.2; font-size: 0.9rem;">
    <hr style="margin: 5px 0; border-color: #333;">
    <b><u>GPS 1:</b>&nbsp;&nbsp;&nbsp;{current_data.get('gps1_fix')}</u><br>
    -<b>Lat:</b>&nbsp;&nbsp;&nbsp;{current_data.get('lat') or 'N/A'}<br>
    -<b>Lon:</b>&nbsp;{current_data.get('lon') or 'N/A'}<br>
    -<b>Acc:</b>&nbsp;{format_gps_accuracy(current_data.get('gps1_h_acc_m'))}<br>
    -<b>Sats:</b>&nbsp;{current_data.get('satellites_visible')}
</div>
"""
    if current_data.get('gps2_fix'):
        gps_html += f"""
<br>
<div style="line-height: 1.2; font-size: 0.9rem;">
    <b><u>GPS 2:</b>&nbsp;&nbsp;&nbsp;{current_data.get('gps2_fix')}</u><br>
    -<b>Lat:</b>&nbsp;&nbsp;&nbsp;{current_data.get('gps2_lat') or 'N/A'}<br>
    -<b>Lon:</b>&nbsp;{current_data.get('gps2_lon') or 'N/A'}<br>
    -<b>Acc:</b>&nbsp;{format_gps_accuracy(current_data.get('gps2_h_acc_m'))}<br>
    -<b>Yaw:</b>&nbsp;{format_gps_yaw(current_data.get('gps2_yaw_deg'))}<br>
    -<b>Sats:</b>&nbsp;{current_data.get('gps2_satellites_visible')}
</div>
"""
    st.markdown(gps_html, unsafe_allow_html=True)

    # st.markdown("""
    # ### <u>GPS 1: &nbsp;RTK Fix</u>
    # - **Lat:** &nbsp;38.4045464
    # - **Lon:** -90.233347
    # - **Sats:** 10
    # """, unsafe_allow_html=True)    

@st.fragment(run_every=1.5)
def _render_live_map():

    current_data = get_shared_state().get()

    # Mission progress (only show while in AUTO)
    current_mode = str(current_data.get('mode') or '')
    if current_mode != 'AUTO':
        mission_progress_placeholder.empty()
    else:
        try:
            wp_current = int(current_data.get('wp_current') or 0)
        except Exception:
            wp_current = 0

        total_wps = int(current_data.get('mission_dl_total') or 0)
        if total_wps <= 0:
            pts = current_data.get('mission_points') or []
            if isinstance(pts, list):
                total_wps = len(pts)

        with mission_progress_placeholder.container():
            if total_wps > 0:
                denom = max(total_wps - 1, 1)
                frac = max(0.0, min(1.0, wp_current / denom))
                shown_cur = min(max(wp_current + 1, 1), total_wps)
                st.progress(frac, text=f"Mission progress {shown_cur}/{total_wps}")
            else:
                st.progress(0.0, text="Mission progress: awaiting mission…")

    # Map style comes from session_state (set by the sidebar controls fragment)
    try:
        map_style_name = st.session_state.get("map_style_select_fixed") or map_options[st.session_state.get("map_style_ind", 0)]
    except Exception:
        map_style_name = map_options[0]
    selected_map_style = map_styles.get(map_style_name, list(map_styles.values())[0])

    # Map (cache deck unless signature changes)
    map_sig = _map_signature(current_data, map_style_name)
    last_map_sig = st.session_state.get('_last_map_sig')
    if (last_map_sig != map_sig) or ('_last_map_deck' not in st.session_state):
        deck = create_map_deck(current_data, selected_map_style)
        st.session_state['_last_map_deck'] = deck
        st.session_state['_last_map_sig'] = map_sig
    else:
        deck = st.session_state.get('_last_map_deck')

    if deck:
        map_placeholder.pydeck_chart(deck, key="map_chart", width="stretch")
    else:
        map_placeholder.info("🛰️ Waiting for GPS Position...")

@st.fragment(run_every=2.0)
def _mqtt_publish_tick():
    mqtt_enabled = (os.getenv("MQTT_ENABLED", "") or "").strip().lower()
    if mqtt_enabled in ("", "0", "false", "no", "off"):
        return
    try:
        from mavweb_mqtt import publish_stats
        publish_stats()
    except Exception:
        # best-effort; never break UI
        pass

@st.fragment(run_every=0.8)
def _render_live_console():
    current_data = get_shared_state().get()
    msg_log = "<br>".join(current_data.get('messages', []))
    console_placeholder.markdown(
        f"""
        <div style="height:200px; overflow-y:auto; background-color:#0e1117; border:1px solid #30363d; padding:10px; color:#58a6ff; font-family:monospace; font-size:0.8rem;">
            {msg_log}
        </div>
        """,
        unsafe_allow_html=True,
    )

try:
    _render_live_metrics()
    _render_live_map()
    _render_live_console()
    _mqtt_publish_tick()

    # MQTT publish (best-effort; never crash UI)
    mqtt_enabled = (os.getenv("MQTT_ENABLED", "") or "").strip().lower()
    print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [App] MQTT_ENABLED={mqtt_enabled}")
    if mqtt_enabled not in ("", "0", "false", "no", "off"):
        try:
            from mavweb_mqtt import publish_stats
            print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [App] Publishing MQTT stats...")
            publish_stats()
        except Exception:
            pass

    # Sidebar: invoke fragments inside a sidebar context.
    with st.sidebar:
#        st.title(":material/agriculture: MavWeb")
#        st.divider()
        _render_live_sidebar()
        st.divider()
        _render_sidebar_controls()
except Exception as e:
    # Avoid killing the whole app if the live fragment hits a transient error.
    if DEBUG:
        print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [App] Live UI error: {e}")