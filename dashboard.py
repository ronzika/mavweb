import base64
import socket
import re
from pathlib import Path
import streamlit as st
import os
import streamlit as st
import json
import time
import pydeck as pdk
import math
from dotenv import load_dotenv
import os
import threading
from pymavlink import mavutil

load_dotenv()
MAVLINK_ENDPOINT = os.getenv("MAVLINK_ENDPOINT", "udpin:0.0.0.0:14550")

st.set_page_config(page_title="Rover Dashboard", layout="wide")
st.sidebar.title("Rover Dashboard")
st.sidebar.markdown('---')

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

# --- Streamlit Page Routing ---
page = st.sidebar.radio('Pages', ['Dashboard', 'Missions', 'Settings'], key='Pages')

def missions_page():
    st.title('Missions')
    missions_dir = os.path.join(os.path.dirname(__file__), 'missions')
    st.subheader('Upload Missions')
    uploaded_files = st.file_uploader(
        'Upload .waypoints files',
        type=['waypoints'],
        accept_multiple_files=True,
        key='missions_upload_dialog'
    )
    if uploaded_files:
        for uploaded_file in uploaded_files:
            save_path = os.path.join(missions_dir, uploaded_file.name)
            with open(save_path, 'wb') as f:
                f.write(uploaded_file.read())
        st.success(f"Uploaded {len(uploaded_files)} mission(s). Reload the page to see them in the table.")

    try:
        files = os.listdir(missions_dir)
        missions = []
        for f in files:
            if not f.startswith('.'):
                base = os.path.splitext(f)[0].replace('.', '')
                path = os.path.join(missions_dir, f)
                try:
                    ts = os.path.getmtime(path)
                    import datetime
                    ts_str = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
                except Exception:
                    ts_str = '?'
                missions.append({'Mission': base, 'Timestamp': ts_str, 'Filename': f, 'Path': path})
        if missions:
            import pandas as pd
            df = pd.DataFrame(missions)
            selected = st.radio('Select a mission to preview:', df['Mission'] + ' (' + df['Filename'] + ')')
            sel_row = df[df['Mission'] + ' (' + df['Filename'] + ')' == selected].iloc[0]
            st.dataframe(df[['Mission', 'Timestamp', 'Filename']], hide_index=True)
            # Preview selected mission
            st.subheader(f"Preview: {sel_row['Mission']} ({sel_row['Filename']})")
            # Try to parse the .waypoints file as lat/lon pairs per line
            try:
                with open(sel_row['Path']) as f:
                    all_lines = [line.strip() for line in f if line.strip() and not line.startswith('#')]
                # Skip the first 2 lines (headers/metadata)
                lines = all_lines[2:]
                points = []
                for line in lines:
                    cols = line.split('\t')
                    if len(cols) >= 10:
                        try:
                            lat = float(cols[8])
                            lon = float(cols[9])
                            points.append({'lat': lat, 'lon': lon})
                        except Exception:
                            continue
                if points:
                    import pydeck as pdk
                    # Use same settings as main map
                    # Convert to [[lon, lat], ...] for path, and for waypoints
                    path_points = [[p['lon'], p['lat']] for p in points]
                    waypoint_data = [{"pos": [p['lon'], p['lat']], "color": [255, 255, 255, 255]} for p in points]
                    view_state = pdk.ViewState(
                        latitude=points[0]['lat'],
                        longitude=points[0]['lon'],
                        zoom=20,
                        pitch=0,
                    )
                    layers = []
                    # Path (white, like pending)
                    if len(path_points) > 1:
                        layers.append(pdk.Layer(
                            "PathLayer",
                            data=[{"path": path_points}],
                            get_path="path",
                            get_color=[255, 255, 255, 200],
                            width_min_pixels=1,
                            get_width=0.05,
                            width_units='"meters"',
                            dash_justified=True,
                        ))
                    # Waypoints (white dots)
                    layers.append(pdk.Layer(
                        "ScatterplotLayer",
                        data=waypoint_data,
                        get_position="pos",
                        get_color="color",
                        get_radius=0.15,
                        radius_units='"meters"',
                    ))
                    deck = pdk.Deck(
                        initial_view_state=view_state,
                        layers=layers,
                        tooltip=True
                    )
                    st.pydeck_chart(deck)
                else:
                    st.info('No valid points found in this mission file.')
            except Exception as e:
                st.error(f'Could not parse or render mission: {e}')
        else:
            st.info('No missions found.')
    except Exception as e:
        st.error(f'Error reading missions: {e}')

if page == 'Settings':
    settings_page()
    st.stop()
elif page == 'Missions':
    missions_page()
    st.stop()
    

st.markdown("""
    <style>
    div[data-testid="stMetric"] > label, div[data-testid="stMetric"] > div > label {
        display: block;
        text-align: center !important;
        width: 100%;
        margin-left: auto;
        margin-right: auto;
    }
    /* For Streamlit >=1.30, metric label is in a div with role=heading */
    div[data-testid="stMetric"] [role="heading"] {
        text-align: center !important;
        width: 100%;
        margin-left: auto;
        margin-right: auto;
    }
    /* Reduce font size and center metric value/statistic only */
    div[data-testid="stMetric"] > div > div {
        text-align: center !important;
        width: 100%;
        margin-left: auto;
        margin-right: auto;
        font-size: 1.05rem !important;
        font-weight: 500;
    }
    </style>
""", unsafe_allow_html=True)

# NTRIP connection state (global)
NTRIP_CASTER = os.getenv("NTRIP_CASTER", "")
NTRIP_PORT = int(os.getenv("NTRIP_PORT", 2101))
NTRIP_MOUNTPOINT = os.getenv("NTRIP_MOUNTPOINT", "")
NTRIP_USER = os.getenv("NTRIP_USER", "")
NTRIP_PASS = os.getenv("NTRIP_PASS", "")
NTRIP_CONNECTED = {'status': False, 'rtcm_count': 0}

def start_ntrip_injection(conn):
    """
    Starts a background thread to fetch RTCM3 data from an NTRIP caster and inject it into MAVLink.
    """
    def ntrip_thread():
        if not (NTRIP_CASTER and NTRIP_MOUNTPOINT and NTRIP_USER and NTRIP_PASS):
            print("[NTRIP] Missing NTRIP configuration, skipping NTRIP injection.")
            NTRIP_CONNECTED['status'] = False
            return
        try:
            print(f"[NTRIP] Connecting to {NTRIP_CASTER}:{NTRIP_PORT}/{NTRIP_MOUNTPOINT}")
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(10)
            s.connect((NTRIP_CASTER, NTRIP_PORT))
            # NTRIP request
            print(f"[NTRIP] Sending NTRIP request...")
            auth = base64.b64encode(f"{NTRIP_USER}:{NTRIP_PASS}".encode()).decode()
            # ...existing NTRIP logic continues here...
        except Exception as e:
            print(f"[NTRIP] Error: {e}")
            NTRIP_CONNECTED['status'] = False

    t = threading.Thread(target=ntrip_thread, daemon=True)
    t.start()


# Log client IP
if 'ip_logged' not in st.session_state:
    try:
        headers = st.context.headers
        client_ip = None
        if headers:
            client_ip = headers.get("X-Forwarded-For")
            if client_ip:
                client_ip = client_ip.split(",")[0]
            else:
                client_ip = headers.get("X-Real-Ip")
        
        if client_ip:
            print(f"New session started from IP: {client_ip}")
        else:
            print("New session started (IP not found)")
            
        st.session_state.ip_logged = True
    except Exception as e:
        print(f"Error logging IP: {e}")
        st.session_state.ip_logged = True

if 'breadcrumbs' not in st.session_state:
    st.session_state.breadcrumbs = []

# Thread-safe shared state
class SharedState:
    def __init__(self):
        self.rover_data = {}
        self.last_update = 0
        self.lock = threading.Lock()
        self.mission_loading = False

    def update(self, data):
        with self.lock:
            self.rover_data.update(data)
            self.last_update = time.time()
#            print(f"[DEBUG] last_update set to {self.last_update} ({time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.last_update))})")

    def set_mission(self, points):
        with self.lock:
            self.rover_data['mission_points'] = points
            self.last_update = time.time()
            self.mission_loading = False

    def set_loading(self, loading):
        with self.lock:
            self.mission_loading = loading

    def is_loading(self):
        with self.lock:
            return self.mission_loading

    def append_message(self, msg_text):
        with self.lock:
            msgs = self.rover_data.get('messages', [])
            msgs.insert(0, msg_text)
            self.rover_data['messages'] = msgs[:10] # Keep last 10
            self.last_update = time.time()

    def get(self):
        with self.lock:
            data = self.rover_data.copy()
            data['last_update'] = self.last_update
            return data

@st.cache_resource
def get_shared_state():
    return SharedState()

# MQTT Callback
def on_message(client, userdata, message):
    try:
        payload = json.loads(message.payload.decode())
        get_shared_state().update(payload)
    except Exception as e:
        print(f"Error parsing message: {e}")

if 'mission_points' not in st.session_state:
    st.session_state.mission_points = []

# MAVLink Client Setup
def get_mode_name(custom_mode):
    mapping = {
        0: 'MANUAL', 1: 'ACRO', 3: 'STEERING', 4: 'HOLD', 5: 'LOITER',
        6: 'FOLLOW', 7: 'SIMPLE', 10: 'AUTO', 11: 'RTL', 12: 'SMART_RTL',
        15: 'GUIDED', 16: 'INITIALISING'
    }
    return mapping.get(custom_mode, f"Unknown({custom_mode})")

def get_gps_fix_string(fix_type):
    mapping = {
        0: 'No GPS', 1: 'No Fix', 2: '2D Fix', 3: '3D Fix', 
        4: 'DGPS', 5: 'RTK Float', 6: 'RTK Fixed', 7: 'Static', 8: 'PPP'
    }
    return mapping.get(fix_type, f"Unknown({fix_type})")

# Global temporary storage for mission download
_mission_cache = {}
_mission_total = 0

def mavlink_loop(conn):
    global _mission_total
    while True:
        try:
            msg = conn.recv_match(blocking=True, timeout=1.0)
            if not msg: continue
            
            data = {}
            mtype = msg.get_type()
            
            if mtype == 'HEARTBEAT':
                if msg.type == mavutil.mavlink.MAV_TYPE_GCS: continue
                data['mode'] = get_mode_name(msg.custom_mode)
                data['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                
            elif mtype == 'GPS_RAW_INT':
                data['lat'] = msg.lat / 1e7
                data['lon'] = msg.lon / 1e7
                data['gps1_fix'] = get_gps_fix_string(msg.fix_type)
                data['satellites_visible'] = msg.satellites_visible

            elif mtype == 'ATTITUDE':
                data['roll'] = msg.roll
                data['pitch'] = msg.pitch

            elif mtype == 'GPS2_RAW':
                data['gps2_fix'] = get_gps_fix_string(msg.fix_type)
                data['gps2_satellites_visible'] = msg.satellites_visible
                
            elif mtype == 'VFR_HUD':
                data['speed_ms'] = round(msg.groundspeed, 2)
                data['heading_deg'] = msg.heading
                
            elif mtype == 'STATUSTEXT':
                get_shared_state().append_message(msg.text)

            elif mtype == 'MISSION_CURRENT':
                data['wp_current'] = msg.seq

            elif mtype == 'MISSION_COUNT':
                _mission_cache.clear()
                _mission_total = msg.count
                print(f"Downloading mission: {_mission_total} items")
                if _mission_total > 0:
                    # Request first item
                    conn.mav.mission_request_int_send(conn.target_system, conn.target_component, 0)
                else:
                    get_shared_state().set_mission([])
                    conn.mav.mission_ack_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_MISSION_ACCEPTED)

            elif mtype == 'MISSION_ITEM_INT':
                if msg.x != 0 and msg.y != 0:
                    _mission_cache[msg.seq] = [msg.y / 1e7, msg.x / 1e7] # lon, lat
                
                # Request next item if we haven't reached the total
                if msg.seq < _mission_total - 1:
                    conn.mav.mission_request_int_send(conn.target_system, conn.target_component, msg.seq + 1)
                else:
                    # Finished downloading
                    sorted_pts = []
                    if _mission_cache:
                        # Sort by sequence
                        sorted_pts = [v for k, v in sorted(_mission_cache.items())]
                    
                    get_shared_state().set_mission(sorted_pts)
                    print(f"Mission loaded: {len(sorted_pts)} points")
                    
                    # Send ACK to complete the transaction
                    conn.mav.mission_ack_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_MISSION_ACCEPTED)

            if data:
                get_shared_state().update(data)
        except Exception as e:
            print(f"MAVLink error: {e}")
            time.sleep(1)

@st.cache_resource
def start_mavlink_client():
    try:
        print(f"Connecting to MAVLink: {MAVLINK_ENDPOINT}")
        conn = mavutil.mavlink_connection(MAVLINK_ENDPOINT)
        t = threading.Thread(target=mavlink_loop, args=(conn,), daemon=True)
        t.start()
        return conn
    except Exception as e:
        st.error(f"Failed to connect to MAVLink: {e}")
        return None

mavlink_conn = start_mavlink_client()


# Sidebar: NTRIP enable checkbox and status
if 'ntrip_enabled' not in st.session_state:
    st.session_state['ntrip_enabled'] = False
st.sidebar.markdown('---')
st.session_state['ntrip_enabled'] = st.sidebar.checkbox('Enable NTRIP injection', value=st.session_state['ntrip_enabled'])
if mavlink_conn and st.session_state['ntrip_enabled']:
    if not NTRIP_CONNECTED['status']:
        start_ntrip_injection(mavlink_conn)
    if NTRIP_CONNECTED['status']:
        st.sidebar.success(f'NTRIP: Connected ({NTRIP_CONNECTED["rtcm_count"]} RTCM packets)')
    else:
        st.sidebar.warning('NTRIP: Not connected')
else:
    st.sidebar.info('NTRIP: Disabled')


client = mavlink_conn
mavlink_data_seen = False
if client:
    # Check for any recent MAVLink data in shared state
    data = get_shared_state().get()
    import time
    now = time.time()
    last_update = data.get('last_update', 0) if data else 0
#    print(f"[DEBUG] Checking last_update: now={now}, last_update={last_update}, delta={now - last_update if last_update else 'N/A'}")
    if last_update and (now - last_update) < 3:
        mavlink_data_seen = True
    mavlink_endpoint = os.getenv("MAVLINK_ENDPOINT", "N/A")
    if mavlink_data_seen:
        st.sidebar.success(f"Connected to MAVLink:\n: {mavlink_endpoint}")
    else:
        st.sidebar.error(f"No MAVLink data!\n: {mavlink_endpoint}")
else:
    st.stop()

# Layout

st.markdown('<hr style="border:0;border-top:2px solid #fff;margin:8px 0 8px 0;">', unsafe_allow_html=True)
# Stats row (now with white line above)

# Add GCS Link metric (8 columns)
col1, col2, col3, col4, col5, col6, col7, col8 = st.columns(8)
metric_mode = col1.empty()
metric_armed = col2.empty()
metric_speed = col3.empty()
metric_gps = col4.empty()
metric_heading = col5.empty()
metric_sats = col6.empty()
metric_wp = col7.empty()
metric_gcs_link = col8.empty()


# White line directly above the map
st.markdown('<hr style="border:0;border-top:2px solid #fff;margin:8px 0 8px 0;">', unsafe_allow_html=True)
progress_placeholder = st.empty()
map_placeholder = st.empty()

# White line directly under the map
st.markdown('<hr style="border:0;border-top:2px solid #fff;margin:8px 0 8px 0;">', unsafe_allow_html=True)

col_btn1, col_btn2, col_btn3, col_btn4, col_btn5, col_btn6, col_btn7, col_btn8, col_btn9 = st.columns(9)
with col_btn1:
    if st.button("Auto"):
        try:
            if client:
                # Set mode to AUTO (mode number 10 for Rover)
                client.mav.set_mode_send(
                    client.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    10
                )
                st.toast("Set mode: AUTO")
            else:
                st.warning("MAVLink connection not available.")
        except Exception as e:
            st.error(f"Failed to set AUTO mode: {e}")
with col_btn2:
    if st.button("Hold"):
        try:
            if client:
                # Set mode to HOLD (mode number 4 for Rover)
                client.mav.set_mode_send(
                    client.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    4
                )
                st.toast("Set mode: HOLD")
            else:
                st.warning("MAVLink connection not available.")
        except Exception as e:
            st.error(f"Failed to set HOLD mode: {e}")
with col_btn3:
    if st.button("Manual"):
        try:
            if client:
                # Set mode to MANUAL (mode number 0 for Rover)
                client.mav.set_mode_send(
                    client.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    0
                )
                st.toast("Set mode: MANUAL")
            else:
                st.warning("MAVLink connection not available.")
        except Exception as e:
            st.error(f"Failed to set MANUAL mode: {e}")
with col_btn4:
    if st.button("Save WP"):
        st.session_state['action'] = 'save_wp'
with col_btn5:
    if st.button("Skip WP"):
        try:
            if client:
                # Advance to next waypoint
                # Use current WP from shared state, increment, and send mission_set_current
                data = get_shared_state().get()
                wp_current = data.get('wp_current', 0)
                mission_pts = data.get('mission_points', [])
                total_wp = len(mission_pts)
                next_wp = min(wp_current + 1, max(0, total_wp - 1))
                client.mav.mission_set_current_send(
                    client.target_system,
                    client.target_component,
                    int(next_wp)
                )
                st.toast(f"Skipped to WP {next_wp}")
                # Add message to STATUSTEXT window
                get_shared_state().append_message(f"[UI] Skipped to WP {next_wp}")
            else:
                st.warning("MAVLink connection not available.")
                get_shared_state().append_message("[UI] Failed to skip waypoint: MAVLink connection not available.")
        except Exception as e:
            st.error(f"Failed to skip waypoint: {e}")
            get_shared_state().append_message(f"[UI] Failed to skip waypoint: {e}")
with col_btn6:
    if st.button("Arm", disabled=False):
        try:
            if client:
                client.mav.command_long_send(
                    client.target_system,
                    client.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    1, 0, 0, 0, 0, 0, 0
                )
                st.toast("Sent ARM command")
            else:
                st.warning("MAVLink connection not available.")
        except Exception as e:
            st.error(f"Failed to send ARM command: {e}")
with col_btn7:
    if st.button("Disarm", disabled=False):
        try:
            if client:
                client.mav.command_long_send(
                    client.target_system,
                    client.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    0, 0, 0, 0, 0, 0, 0
                )
                st.toast("Sent DISARM command")
            else:
                st.warning("MAVLink connection not available.")
        except Exception as e:
            st.error(f"Failed to send DISARM command: {e}")

with col_btn8:
    is_loading = get_shared_state().is_loading()
    if is_loading:
        st.button("Loading...", disabled=True)
    else:
        if st.button("Load Map"):
            if client:
                try:
                    get_shared_state().set_loading(True)
                    # Request mission list
                    client.mav.mission_request_list_send(client.target_system, client.target_component)
                    st.toast("Requesting mission...")
                    get_shared_state().append_message("[UI] Requested mission list from vehicle.")
                    st.rerun()
                except Exception as e:
                    st.error(f"Error: {e}")
                    get_shared_state().set_loading(False)
                    get_shared_state().append_message(f"[UI] Failed to request mission: {e}")
            else:
                st.warning("Mission loading only supported in MAVLink mode for now.")
                get_shared_state().append_message("[UI] Failed to request mission: MAVLink connection not available.")

# Reboot button (far right), only enabled when not armed
with col_btn9:
    data = get_shared_state().get()
    armed = data.get('armed', False)
    if st.button("Reboot", disabled=armed):
        try:
            if client:
                client.mav.command_long_send(
                    client.target_system,
                    client.target_component,
                    mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                    0,
                    1, 0, 0, 0, 0, 0, 0
                )
                st.toast("Sent REBOOT command")
            else:
                st.warning("MAVLink connection not available.")
        except Exception as e:
            st.error(f"Failed to send REBOOT command: {e}")


details_placeholder = st.empty()


st.markdown('<hr style="border:0;border-top:2px solid #fff;margin:8px 0 16px 0;">', unsafe_allow_html=True)
# Scrollable status messages box directly below the buttons
messages_placeholder = st.empty()

# White line directly under the buttons
st.markdown('<hr style="border:0;border-top:2px solid #fff;margin:8px 0 16px 0;">', unsafe_allow_html=True)

# Update Loop
while True:
    data = get_shared_state().get()
    if data:

        # Metrics
        metric_mode.metric("Mode", data.get('mode', 'N/A'))
        armed = data.get('armed', False)
        metric_armed.metric("Armed", "YES" if armed else "NO", delta_color="normal" if armed else "off")
        speed = data.get('speed_ms')
        metric_speed.metric("Speed", f"{speed} m/s" if speed is not None else "N/A")
        gps1_fix = data.get('gps1_fix', 'No Fix')
        gps2_fix = data.get('gps2_fix')
        if gps2_fix:
            metric_gps.metric("GPS Fix", f"1: {gps1_fix}\n2: {gps2_fix}")
        else:
            metric_gps.metric("GPS Fix", gps1_fix)
        heading = data.get('heading_deg')
        metric_heading.metric("Heading", f"{heading}°" if heading is not None else "N/A")
        sats1 = data.get('satellites_visible', 0)
        sats2 = data.get('gps2_satellites_visible')
        if sats2 is not None:
            metric_sats.metric("Sats", f"1: {sats1} / 2: {sats2}")
        else:
            metric_sats.metric("Sats", f"{sats1}")
        wp_current = data.get('wp_current', 0)
        metric_wp.metric("WP", f"{wp_current}")
        # GCS Link metric
        gcs_link = data.get('linkqualitygcs')
        metric_gcs_link.metric("GCS Link", f"{gcs_link}%" if gcs_link is not None else "N/A")

        # Mission Progress Bar
        mission_pts = data.get('mission_points', [])
        if data.get('mode') == 'AUTO' and mission_pts:
            total_wp = len(mission_pts)
            if total_wp > 0:
                # Calculate progress (clamp between 0.0 and 1.0)
                prog_val = min(1.0, max(0.0, wp_current / total_wp))
                progress_placeholder.progress(prog_val, text=f"Mission Progress: {int(prog_val*100)}% (WP {wp_current}/{total_wp})")
            else:
                progress_placeholder.empty()
        else:
            progress_placeholder.empty()

        # Map
        lat = data.get('lat')
        lon = data.get('lon')
        heading = data.get('heading_deg', 0)
        if heading is None: heading = 0
        if lat is not None and lon is not None and lat != 0 and lon != 0:
            # Update breadcrumbs
            current_pos = [lon, lat]
            history = st.session_state.breadcrumbs
            # Append if history is empty or if position changed significantly (approx > 10cm)
            if not history or (abs(history[-1][0] - lon) > 1e-7 or abs(history[-1][1] - lat) > 1e-7):
                history.append(current_pos)
                # Limit history to last 5000 points
                if len(history) > 5000:
                    history.pop(0)

            # Calculate Triangle Vertices (Rover Shape)
            # Size in meters (distance from center to vertices)
            size = 0.4 
            r_earth = 6378137
            def get_offset_point(lat, lon, dist, bearing_deg):
                bearing_rad = math.radians(bearing_deg)
                dy = dist * math.cos(bearing_rad)
                dx = dist * math.sin(bearing_rad)
                d_lat = (dy / r_earth) * (180 / math.pi)
                d_lon = (dx / r_earth) * (180 / math.pi) / math.cos(math.radians(lat))
                return [lon + d_lon, lat + d_lat]

            nose = get_offset_point(lat, lon, size, heading)
            rear_left = get_offset_point(lat, lon, size, heading - 140)
            rear_right = get_offset_point(lat, lon, size, heading + 140)
            triangle_data = [{"polygon": [nose, rear_right, rear_left]}]

            view_state = pdk.ViewState(
                latitude=lat,
                longitude=lon,
                zoom=20,
                pitch=0,
            )

            layers = []

            # Breadcrumbs Path
            if len(history) > 1:
                layers.append(pdk.Layer(
                    "PathLayer",
                    data=[{"path": history}],
                    get_path="path",
                    get_color=[255, 255, 0, 255], # Bright yellow trail (fully opaque)
                    width_min_pixels=1,
                    get_width=0.05, # meters (match mission path)
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
                data=triangle_data,
                get_polygon="polygon",
                get_fill_color=[255, 0, 0, 255],
                get_line_color=[200, 0, 0, 255],
                filled=True,
                stroked=True,
                line_width_min_pixels=1,
            ))
            deck = pdk.Deck(
                initial_view_state=view_state,
                layers=layers,
                tooltip=True
            )
            map_placeholder.pydeck_chart(deck)
        # else:
        #     map_placeholder.info("Waiting for GPS location...")


        # Messages
        msgs = data.get('messages', [])
        if msgs:
            # Render as scrollable text box
            msg_html = "<br>".join([str(m) for m in msgs])
            messages_placeholder.markdown(f"<div style='height: 120px; overflow-y: auto; border: 1px solid #ccc; border-radius: 6px; background: #181c24; color: #fff; padding: 8px;'>{msg_html}</div>", unsafe_allow_html=True)
        else:
            messages_placeholder.markdown("<div style='height: 120px; overflow-y: auto; border: 1px solid #ccc; border-radius: 6px; background: #181c24; color: #fff; padding: 8px;'>No messages</div>", unsafe_allow_html=True)
    else:
        map_placeholder.info("Waiting for data...")
    time.sleep(1)
    # Rerun is not strictly necessary if we use placeholders in a loop, 
    # but Streamlit might timeout the script execution if it runs too long without interaction.
    # However, for a dashboard, a loop with sleep is a common pattern if st.rerun() is not used.
    # If we want to be more 'streamlit-native', we would rely on st.rerun() triggered by something,
    # but MQTT callbacks happen in a thread.
    # Using st.rerun() inside the loop ensures the UI stays fresh and responsive to other interactions if any.
    # But calling st.rerun() constantly can be heavy.
    # Let's just rely on the loop updating placeholders.
