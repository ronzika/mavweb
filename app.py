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
# from streamlit_autorefresh import st_autorefresh

# --- Configuration ---
load_dotenv()
MAVLINK_ENDPOINT = os.getenv("MAVLINK_ENDPOINT", "udpin:0.0.0.0:14550")

st.set_page_config(page_title="Rover GCS", layout="wide", initial_sidebar_state="expanded")

# --- Thread-Safe State Class ---
class SharedState:
    def __init__(self):
        self.rover_data = {
            'link_active': False,
            'mode': 'DISCONNECTED',
            'armed': False,
            'lat': None,
            'lon': None,
            'heading_deg': 0,
            'speed_ms': 0,
            'gps1_fix': 'No Fix',
            'satellites_visible': 0,
            'gps2_fix': None,
            'gps2_satellites_visible': 0,
            'gps2_lat': None,
            'gps2_lon': None,
            'battery_v': 0,
            'battery_pct': 0,
            'messages': [],
            'mission_points': [],
            'wp_current': 0,
            'last_update': 0
        }
        self.lock = threading.Lock()
        self.mav_lock = threading.Lock()
        self.connection = None

    def acquire_mav_lock(self):
        return self.mav_lock

    def set_connection(self, conn):
        with self.lock:
            self.connection = conn

    def get_connection(self):
        with self.lock:
            return self.connection

    def update(self, data):
        with self.lock:
            self.rover_data.update(data)
            self.rover_data['last_update'] = time.time()

    def append_message(self, msg_text):
        print(f"[Rover Message] {msg_text}")
        with self.lock:
            msg_text = str(msg_text).strip()

            if not msg_text:
                return

            msgs = self.rover_data.get('messages', [])
            
            # Simple Deduplication: Only check the very last message
            # This allows sequential updates (A -> B -> A) but stops immediate bursts (A -> A -> A)
            if msgs and msgs[0] == msg_text:
                return
            
            msgs.insert(0, msg_text)
            self.rover_data['messages'] = msgs[:15] # Keep last 15
            self.rover_data['last_update'] = time.time()
#            self.rover_data['last_update'] = time.time()

    def get(self):
        with self.lock:
            return self.rover_data.copy()

@st.cache_resource
def get_shared_state():
    return SharedState()

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
    _mission_cache = {}
    _mission_total = 0
    
    while True:
        conn = None
        state.set_connection(None)
        try:
            state.append_message(f"Attempting MAVLink: {endpoint}")
            conn = mavutil.mavlink_connection(endpoint)
            
            # Wait for Autopilot Heartbeat (Ignore GCS)
            while True:
                msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=10)
                if not msg:
                    raise Exception("No Heartbeat found")
                if msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                    conn.target_system = msg.get_srcSystem()
                    conn.target_component = msg.get_srcComponent()
                    break
            
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
                # Send Heartbeat to GCS every 1s
                if time.time() - last_heartbeat > 1.0:
                    conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                    last_heartbeat = time.time()

                msg = conn.recv_match(blocking=True, timeout=0.5)
                if not msg:
                    # Check for link timeout
                    if time.time() - state.get()['last_update'] > 5:
                        state.update({'link_active': False})
                    continue

                mtype = msg.get_type()
                data = {'link_active': True}

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
                    print(f"[STATUSTEXT] setting {txt}")
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

                elif mtype == 'MISSION_COUNT':
                    _mission_cache = {}
                    _mission_total = msg.count
                    if _mission_total > 0:
                        with state.acquire_mav_lock():
                            conn.mav.mission_request_int_send(conn.target_system, conn.target_component, 0)

                elif mtype == 'MISSION_ITEM_INT':
                    # Store as [lon, lat] for PyDeck
                    _mission_cache[msg.seq] = [msg.y / 1e7, msg.x / 1e7]
                    if msg.seq < _mission_total - 1:
                        with state.acquire_mav_lock():
                            conn.mav.mission_request_int_send(conn.target_system, conn.target_component, msg.seq + 1)
                    else:
                        sorted_pts = [v for k, v in sorted(_mission_cache.items())]
                        state.update({'mission_points': sorted_pts})
                        state.append_message(f"Mission loaded: {len(sorted_pts)} points")
                        with state.acquire_mav_lock():
                            conn.mav.mission_ack_send(conn.target_system, conn.target_component, 0)

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

# --- UI Functions ---
def create_map_deck(data):
    lat, lon = data.get('lat'), data.get('lon')
    if not lat or lat == 0:
        return None

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

    if mission_pts and len(mission_pts) > 1:
        # Split into Completed (Green) and Pending (White)
        split_idx = max(1, wp_current)
        completed_path = mission_pts[:split_idx]
        pending_path = mission_pts[split_idx-1:]

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
        for i, p in enumerate(mission_pts):
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

    return pdk.Deck(initial_view_state=view_state, layers=layers, tooltip=True)

# --- MAIN APP ---
start_background_threads()
# st_autorefresh removed in favor of while loop

# Sidebar Placeholders
st.sidebar.title("🎮 Rover Control")
sidebar_status_ph = st.sidebar.empty()
st.sidebar.divider()
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

if st.sidebar.button("ARM", type="primary", use_container_width=True, disabled=is_disabled):
    if cmd_conn:
        try:
            with get_shared_state().acquire_mav_lock():
                cmd_conn.mav.command_long_send(cmd_conn.target_system, cmd_conn.target_component, 
                                          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
            st.toast("Sent ARM Command")
        except Exception as e:
            st.error(f"Failed to arm: {e}")

if st.sidebar.button("DISARM", use_container_width=True, disabled=is_disabled):
    if cmd_conn:
        try:
            with get_shared_state().acquire_mav_lock():
                cmd_conn.mav.command_long_send(cmd_conn.target_system, cmd_conn.target_component, 
                                          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
            st.toast("Sent DISARM Command")
        except Exception as e:
            st.error(f"Failed to disarm: {e}")

if st.sidebar.button("Load Map", use_container_width=True, disabled=is_disabled):
    if cmd_conn:
        try:
            with get_shared_state().acquire_mav_lock():
                cmd_conn.mav.mission_request_list_send(cmd_conn.target_system, cmd_conn.target_component)
            st.toast("Requesting Mission List...")
        except Exception as e:
            st.error(f"Failed to load map: {e}")

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

print("[App] Starting main update loop...")
# --- Update Loop ---
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
    gps_html = f"""
    <div style="line-height: 1.2; font-size: 0.9rem;">
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
    deck = create_map_deck(current_data)
    if deck:
        map_placeholder.pydeck_chart(deck, key=str(uuid.uuid4()))
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