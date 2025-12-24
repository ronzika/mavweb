import base64
import socket
import re
import os
import json
import time
import math
import threading
import datetime
from pathlib import Path

import uuid
import pandas as pd
import pydeck as pdk
import streamlit as st
from dotenv import load_dotenv
from pymavlink import mavutil
from shared_state import get_shared_state
from mavlink_utils import upload_mission

# --- Configuration ---
env_path = Path(__file__).parent / '.env'
print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [SYSTEM] Loading .env from: {env_path} (Exists: {env_path.exists()})")
load_dotenv(dotenv_path=env_path, override=True)

MAVLINK_ENDPOINT = os.getenv("MAVLINK_ENDPOINT", "udpin:0.0.0.0:14550")
MAPBOX_API_KEY = os.getenv("MAPBOX_API_KEY")
DEBUG = os.getenv("DEBUG", "0") != "0"

# Debug Mapbox Key
if MAPBOX_API_KEY:
    print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [SYSTEM] Mapbox Key Loaded: {MAPBOX_API_KEY[:10]}...")
else:
    print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [SYSTEM] WARNING: Mapbox Key NOT Found!")

st.set_page_config(page_title="Rover GCS", layout="wide", initial_sidebar_state="expanded")

# --- Settings Page ---
def load_env_example():
    env_path = Path(__file__).parent / '.env'
    env_vars = {}
    if env_path.exists():
        with open(env_path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                match = re.match(r'([A-Za-z0-9_]+)=(.*)', line)
                if match:
                    key, value = match.groups()
                    env_vars[key] = value
    return env_vars

def save_env_vars(new_vars):
    env_path = Path(__file__).parent / '.env'
    with open(env_path, 'w') as f:
        for k, v in new_vars.items():
            f.write(f'{k}={v}\n')

def settings_page():
    st.title('Settings')
    st.info('Edit and save environment variables. Changes will be written to .env.')
    env_vars = load_env_example()
    current = {k: st.session_state.get(f'env_{k}', v) for k, v in env_vars.items()}
    with st.form('env_form'):
        new_vars = {}
        for k, v in env_vars.items():
            new_vars[k] = st.text_input(k, value=current[k], key=f'env_{k}')
        submitted = st.form_submit_button('Save')
        if submitted:
            save_env_vars(new_vars)
            st.session_state['Pages'] = 'Dashboard'
            st.success('.env updated! Reloading app...')
            st.rerun()




# --- MAVLink Utilities ---
def get_mode_name(custom_mode):
    mapping = {0: 'MANUAL', 1: 'ACRO', 3: 'STEERING', 4: 'HOLD', 5: 'LOITER',
               10: 'AUTO', 11: 'RTL', 15: 'GUIDED'}
    return mapping.get(custom_mode, f"MODE({custom_mode})")

def get_gps_fix_string(fix_type):
    mapping = {0: 'No GPS', 1: 'No Fix', 2: '2D Fix', 3: '3D Fix', 4: 'DGPS', 5: 'RTK Float', 6: 'RTK Fixed'}
    return mapping.get(fix_type, f"Fix({fix_type})")

def request_message_interval(conn, msg_id, hz):
    try:
        interval_us = int(1_000_000 / hz)
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, interval_us, 0, 0, 0, 0, 0
        )
    except Exception:
        pass

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
    _mission_total = 0
    
    # Link Quality Tracking
    last_seq = None
    packet_history = []
    WINDOW_SIZE = 100
    raw_lq_history = [] # For smoothing
    last_sparkline_update = 0
    
    # Mission Download State
    mission_download_active = False
    mission_next_seq = 0
    mission_last_req_time = 0

    while True:
        # Check if a new worker has taken over
        if state.current_worker_id != worker_id:
            print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [SYSTEM] Stopping MAVLink Worker {worker_id} (Superseded)")
            return

        conn = None
        state.set_connection(None)
        last_seq = None
        packet_history = []
        raw_lq_history = []
        last_sparkline_update = 0
        mission_download_active = False
        
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
            for msg_id, hz in [
                (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 2),
                (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2),
                (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1),
                (mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT, 2),
                (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 2),
                (mavutil.mavlink.MAVLINK_MSG_ID_GPS2_RAW, 1)
            ]:
                request_message_interval(conn, msg_id, hz)

            last_heartbeat = 0
            while True:
                # Check if superseded
                if state.current_worker_id != worker_id:
                    print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [SYSTEM] Stopping MAVLink Worker {worker_id} (Superseded)")
                    conn.close()
                    return

                # Check for pending mission upload
                if not state.upload_queue.empty():
                    try:
                        mission_items = state.upload_queue.get_nowait()
                        state.set_upload_status('uploading', 'Starting upload...')
                        # Use the existing connection to upload
                        success = upload_mission(conn, mission_items)
                        if success:
                            state.set_upload_status('success', 'Upload complete!')
                        else:
                            state.set_upload_status('error', 'Upload failed.')
                    except Exception as e:
                        state.set_upload_status('error', f'Upload error: {e}')

                # Send Heartbeat to GCS every 1s
                if time.time() - last_heartbeat > 1.0:
                    conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                    last_heartbeat = time.time()
                
                # Mission Download Retry Logic
                if mission_download_active and time.time() - mission_last_req_time > 0.5:
                    if mission_next_seq < _mission_total:
                        print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [MAVLINK] Retry requesting mission item {mission_next_seq}")
                        with state.acquire_mav_lock():
                            conn.mav.mission_request_int_send(conn.target_system, conn.target_component, mission_next_seq)
                        mission_last_req_time = time.time()
                    else:
                        mission_download_active = False

                msg = conn.recv_match(blocking=True, timeout=0.5)
                if not msg:
                    # Check for link timeout
                    if time.time() - state.get()['last_update'] > 5:
                        state.update({'link_active': False})
                    continue

                mtype = msg.get_type()
                if mtype == 'BAD_DATA':
                    continue

                data = {'link_active': True}

                # Link Quality Calculation
                try:
                    seq = msg.get_seq()
                    if last_seq is not None:
                        diff = (seq - last_seq) % 256
                        lost = diff - 1
                        if lost < 0: lost = 0
                        
                        if lost > 0:
                            packet_history.extend([0] * lost)
                        packet_history.append(1)
                        
                        if len(packet_history) > WINDOW_SIZE:
                            packet_history = packet_history[-WINDOW_SIZE:]
                        
                        lq = (sum(packet_history) / len(packet_history)) * 100
                        data['link_quality'] = int(lq)
                        
                        # Update history for sparkline (Smoothed)
                        raw_lq_history.append(lq)
                        if len(raw_lq_history) > 20:
                            raw_lq_history.pop(0)
                        
                        if time.time() - last_sparkline_update > 1.0:
                            smoothed_lq = sum(raw_lq_history) / len(raw_lq_history)
                            current_hist = state.get().get('link_quality_history', [])
                            current_hist.append(int(smoothed_lq))
                            if len(current_hist) > 50:
                                current_hist = current_hist[-50:]
                            data['link_quality_history'] = current_hist
                            last_sparkline_update = time.time()
                        
                    else:
                        packet_history.append(1)
                    last_seq = seq
                except Exception:
                    pass

                if mtype == 'HEARTBEAT':
                    data['mode'] = get_mode_name(msg.custom_mode)
                    data['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

                elif mtype == 'GLOBAL_POSITION_INT':
                    if msg.lat != 0 and msg.lon != 0:
                        data['lat'] = msg.lat / 1e7
                        data['lon'] = msg.lon / 1e7
                        data['heading_deg'] = msg.hdg / 100.0
                
                elif mtype == 'GPS_RAW_INT':
                    if msg.lat != 0 and msg.lon != 0:
                        data['lat'] = msg.lat / 1e7
                        data['lon'] = msg.lon / 1e7
                    data['gps1_fix'] = get_gps_fix_string(msg.fix_type)
                    data['satellites_visible'] = msg.satellites_visible

                elif mtype == 'MISSION_CURRENT':
                    data['wp_current'] = msg.seq

                elif mtype == 'GPS2_RAW':
                    if msg.lat != 0 and msg.lon != 0:
                        data['gps2_lat'], data['gps2_lon'] = msg.lat / 1e7, msg.lon / 1e7
                    data['gps2_fix'] = get_gps_fix_string(msg.fix_type)
                    data['gps2_satellites_visible'] = msg.satellites_visible

                elif mtype == 'SYS_STATUS':
                    data['battery_v'] = msg.voltage_battery / 1000.0
                    data['battery_pct'] = msg.battery_remaining

                elif mtype == 'VFR_HUD':
                    data['speed_ms'] = round(msg.groundspeed, 2)
                    data['heading_deg'] = msg.heading

                elif mtype == 'STATUSTEXT':
                    txt = str(msg.text)
                    state.append_message(txt)
                    print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [STATUSTEXT] setting {txt}")
                    
                    # Auto-reset mission when complete
                    if "Mission Complete" in txt:
                        print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [SYSTEM] Mission Complete detected. Resetting WP to 0.")
                        try:
                            with state.acquire_mav_lock():
                                conn.mav.mission_set_current_send(conn.target_system, conn.target_component, 0)
                            state.append_message("Mission Reset to WP 0")
                        except Exception as e:
                            print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [SYSTEM] Failed to reset mission: {e}")

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
                        data['wp_current'] = wp_num
                        state.update({'wp_current': wp_num})

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
                    _mission_total = msg.count
                    if DEBUG:
                        print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [DEBUG {threading.get_ident()}] Worker: Received MISSION_COUNT {_mission_total}. Starting download.")
                    if _mission_total > 0:
                        mission_download_active = True
                        mission_next_seq = 0
                        with state.acquire_mav_lock():
                            conn.mav.mission_request_int_send(conn.target_system, conn.target_component, 0)
                        mission_last_req_time = time.time()
                        if DEBUG:
                            print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [DEBUG {threading.get_ident()}] Worker: Requested item 0.")
                    else:
                        # Handle empty mission
                        state.update({'mission_points': []})
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
                    
                    if msg.seq == mission_next_seq:
                        mission_next_seq += 1
                        
                    if mission_next_seq < _mission_total:
                        with state.acquire_mav_lock():
                            conn.mav.mission_request_int_send(conn.target_system, conn.target_component, mission_next_seq)
                        mission_last_req_time = time.time()
                        if DEBUG:
                            print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [DEBUG {threading.get_ident()}] Worker: Requested item {mission_next_seq}.")
                    elif mission_download_active:
                        mission_download_active = False
                        sorted_pts = [v for k, v in sorted(_mission_cache.items())]
                        state.update({'mission_points': sorted_pts})
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

def generate_sparkline(data, width=280, height=40, color="#4caf50"):
    if not data or len(data) < 2:
        return ""
    min_val = 0
    max_val = 100
    points = []
    
    # Fixed max points to ensure scrolling effect (matches history buffer size)
    max_points = 50
    step = width / (max_points - 1)

    for i, val in enumerate(data):
        x = i * step
        y = height - ((val - min_val) / (max_val - min_val)) * height
        points.append(f"{x},{y}")
    
    polyline = " ".join(points)
    return f"""
    <svg width="100%" height="{height}" viewBox="0 0 {width} {height}" preserveAspectRatio="none" style="background-color: rgba(255,255,255,0.05); border-radius: 3px; margin-bottom: 5px;">
        <polyline points="{polyline}" fill="none" stroke="{color}" stroke-width="2" />
    </svg>
    """

# --- UI Functions ---
def create_map_deck(data, map_style='mapbox://styles/mapbox/satellite-v9'):
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

    return pdk.Deck(
        initial_view_state=view_state, 
        layers=layers, 
        tooltip=True, 
        map_style=map_style,
        map_provider="mapbox",
        api_keys={"mapbox": MAPBOX_API_KEY} if MAPBOX_API_KEY else None
    )

# --- MAIN APP ---
start_background_threads()
# st_autorefresh removed in favor of while loop

# Sidebar Placeholders
st.sidebar.title("🎮 Rover Control")
sidebar_status_ph = st.sidebar.empty()
st.sidebar.divider()
sidebar_sparkline_ph = st.sidebar.empty()
sidebar_gps_ph = st.sidebar.empty()
st.sidebar.divider()
st.sidebar.subheader("Commands")

# Get connection from shared state
cmd_conn = get_shared_state().get_connection()

# Disable buttons if not connected
is_disabled = not cmd_conn

if st.sidebar.button("MANUAL", use_container_width=True, disabled=is_disabled):
    if cmd_conn:
        try:
            with get_shared_state().acquire_mav_lock():
                cmd_conn.mav.set_mode_send(cmd_conn.target_system, 1, 0)
            st.toast("Set Mode: MANUAL")
        except Exception as e:
            st.error(f"Failed to set mode: {e}")

if st.sidebar.button("AUTO", use_container_width=True, disabled=is_disabled):
    if cmd_conn:
        try:
            with get_shared_state().acquire_mav_lock():
                cmd_conn.mav.set_mode_send(cmd_conn.target_system, 1, 10)
            st.toast("Set Mode: AUTO")
        except Exception as e:
            st.error(f"Failed to set mode: {e}")

if st.sidebar.button("HOLD", use_container_width=True, disabled=is_disabled):
    if cmd_conn:
        try:
            with get_shared_state().acquire_mav_lock():
                cmd_conn.mav.set_mode_send(cmd_conn.target_system, 1, 4)
            st.toast("Set Mode: HOLD")
        except Exception as e:
            st.error(f"Failed to set mode: {e}")

if st.sidebar.button("ARM", use_container_width=True, disabled=is_disabled):
    if cmd_conn:
        try:
            with get_shared_state().acquire_mav_lock():
                cmd_conn.mav.command_long_send(cmd_conn.target_system, cmd_conn.target_component, 
                                          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
            st.toast("Sent ARM Command")
        except Exception as e:
            st.error(f"Failed to arm: {e}")

if st.sidebar.button("Disarm", use_container_width=True, disabled=is_disabled):
    if cmd_conn:
        try:
            with get_shared_state().acquire_mav_lock():
                cmd_conn.mav.command_long_send(cmd_conn.target_system, cmd_conn.target_component, 
                                          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
            st.toast("Sent DISARM Command")
        except Exception as e:
            st.error(f"Failed to disarm: {e}")

if st.sidebar.button("Skip WP", use_container_width=True, disabled=is_disabled):
    if cmd_conn:
        try:
            data = get_shared_state().get()
            wp_current = data.get('wp_current', 0)
            mission_pts = data.get('mission_points', [])
            total_wp = len(mission_pts)
            
            if total_wp > 0:
                next_wp = min(wp_current + 1, max(0, total_wp - 1))
                with get_shared_state().acquire_mav_lock():
                    cmd_conn.mav.mission_set_current_send(
                        cmd_conn.target_system,
                        cmd_conn.target_component,
                        int(next_wp)
                    )
                st.toast(f"Skipped to WP {next_wp}")
                get_shared_state().append_message(f"[UI] Skipped to WP {next_wp}")
            else:
                st.warning("No mission loaded.")
        except Exception as e:
            st.error(f"Failed to skip waypoint: {e}")

if st.sidebar.button("Fetch Mission", use_container_width=True, disabled=is_disabled):
    if cmd_conn:
        try:
            with get_shared_state().acquire_mav_lock():
                cmd_conn.mav.mission_request_list_send(cmd_conn.target_system, cmd_conn.target_component)
            st.toast("Requesting Mission List...")
            st.rerun()
        except Exception as e:
            st.error(f"Failed to load map: {e}")

st.sidebar.divider()
st.sidebar.subheader("Map Settings")

map_styles = {
    "Satellite": "mapbox://styles/mapbox/satellite-v9",
    "Satellite Streets": "mapbox://styles/mapbox/satellite-streets-v12",
    "Streets": "mapbox://styles/mapbox/streets-v11",
    "Dark": "mapbox://styles/mapbox/dark-v10",
    "Light": "mapbox://styles/mapbox/light-v10",
    "Outdoors": "mapbox://styles/mapbox/outdoors-v11"
}
map_options = list(map_styles.keys())

# Initialize session state for map style index if not present
if "map_style_ind" not in st.session_state:
    st.session_state["map_style_ind"] = 3

def update_map_style():
    st.session_state["map_style_ind"] = map_options.index(st.session_state["map_style_select_fixed"])

map_style_name = st.sidebar.selectbox(
    "Map Style", 
    map_options, 
    index=st.session_state["map_style_ind"], 
    disabled=False, 
    key="map_style_select_fixed",
    on_change=update_map_style
)
selected_map_style = map_styles[map_style_name]

if is_disabled:
    st.sidebar.warning("Commands disabled: MAVLink connection not available.")

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

# Map Placeholder
map_placeholder = st.empty()

st.markdown('<hr style="border:0;border-top:2px solid #fff;margin:8px 0 8px 0;">', unsafe_allow_html=True)

# System Console Placeholder
st.markdown("**System Messages**")
console_placeholder = st.empty()

if 'history' not in st.session_state:
    st.session_state.history = []

print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [App] Starting main update loop...")
# --- Update Loop ---
try:
    while True:
        current_data = get_shared_state().get()

        # Auto-refresh if connection state changes to update UI buttons
        if is_disabled and current_data.get('link_active'):
            st.rerun()
        elif not is_disabled and not current_data.get('link_active'):
            st.rerun()
        
        # Update History (Breadcrumbs)
        lat = current_data.get('lat')
        lon = current_data.get('lon')
        if lat and lon and lat != 0 and lon != 0:
            history = st.session_state.history
            if not history or (abs(history[-1][0] - lon) > 1e-7 or abs(history[-1][1] - lat) > 1e-7):
                history.append([lon, lat])
                if len(history) > 5000:
                    history.pop(0)
        
        # Inject history into data for map creation
        current_data['history'] = st.session_state.history
    
        # Sidebar Status
        if current_data.get('link_active'):
            sidebar_status_ph.markdown(f"✅ **ONLINE**<br><span style='font-size:0.8em; color:gray'>Last Update: {datetime.datetime.now().strftime('%H:%M:%S')}</span>", unsafe_allow_html=True)
        else:
            sidebar_status_ph.error("OFFLINE: Searching...")
    
        # Sidebar GPS
        lq = current_data.get('link_quality', 100)
        lq_color = "#4caf50" if lq >= 90 else "#ff9800" if lq >= 70 else "#f44336"
        
        # Render Sparkline
        lq_hist = current_data.get('link_quality_history', [])
        sparkline_svg = generate_sparkline(lq_hist, color=lq_color)
        sidebar_sparkline_ph.markdown(sparkline_svg, unsafe_allow_html=True)
    
        gps_html = f"""
    <div style="line-height: 1.2; font-size: 0.9rem;">
        <b>Link Quality:</b> <span style="color:{lq_color}; font-weight:bold;">{lq}%</span><br>
        <hr style="margin: 5px 0; border-color: #333;">
        <b>GPS 1:</b> {current_data.get('gps1_fix')}<br>
        Lat: {current_data.get('lat') or 'N/A'}<br>
        Lon: {current_data.get('lon') or 'N/A'}<br>
        Sats: {current_data.get('satellites_visible')}
    </div>
    """
        if current_data.get('gps2_fix'):
            gps_html += f"""
    <hr>
    <div style="line-height: 1.2; font-size: 0.9rem;">
        <b>GPS 2:</b> {current_data.get('gps2_fix')}<br>
        Lat: {current_data.get('gps2_lat') or 'N/A'}<br>
        Lon: {current_data.get('gps2_lon') or 'N/A'}<br>
        Sats: {current_data.get('gps2_satellites_visible')}
    </div>
    """
        sidebar_gps_ph.markdown(gps_html, unsafe_allow_html=True)
    
        # Metrics
        metric_mode.metric("Mode", current_data.get('mode'))
        metric_armed.metric("Armed", "ARMED" if current_data.get('armed') else "DISARMED")
        metric_speed.metric("Speed", f"{current_data.get('speed_ms')} m/s")
        metric_gps.metric("GPS", current_data.get('gps1_fix'))
        metric_battery.metric("Battery", f"{current_data.get('battery_v')}V")
        metric_wp.metric("WP", f"{current_data.get('wp_current', 0)}")
    
        # Map
        deck = create_map_deck(current_data, selected_map_style)
        if deck:
            # Use a stable key that only changes when the style changes
            # This prevents the map from reloading tiles every second
            map_key = f"map_{map_style_name}"
            map_placeholder.pydeck_chart(deck, key=map_key, width="stretch")
        else:
            map_placeholder.info("🛰️ Waiting for GPS Position...")
    
        # Console
        msg_log = "<br>".join(current_data.get('messages', []))
        console_placeholder.markdown(f"""
            <div style="height:200px; overflow-y:auto; background-color:#0e1117; border:1px solid #30363d; padding:10px; color:#58a6ff; font-family:monospace; font-size:0.8rem;">
                {msg_log}
            </div>
        """, unsafe_allow_html=True)
    
        time.sleep(1)
    
except Exception as e:
    print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [App] Main loop stopped: {e}")
except KeyboardInterrupt:
    pass