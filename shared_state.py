import threading
import time
import streamlit as st
import queue

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
            # Latest mission index reported by the vehicle (may jump to 0 on disarm/stop).
            'wp_current_raw': 0,
            # Waypoint index shown in UI (can intentionally ignore raw resets to 0).
            'wp_current': 0,
            # True once we have received a real waypoint index from the vehicle.
            # Used to disable UI actions that depend on a known current waypoint.
            'wp_current_known': False,
            'last_update': 0,
            'link_quality': 100,
            'link_quality_history': [],
            'packets_received': 0,
            'packets_lost': 0,

            # Mission transfer progress (batch operation UX)
            'mission_dl_active': False,
            'mission_dl_total': 0,
            'mission_dl_received': 0,
            'mission_dl_done_ts': 0.0,
            'mission_ul_active': False,
            'mission_ul_total': 0,
            'mission_ul_sent': 0,
            'mission_ul_done_ts': 0.0,

            # Backwards-compat UI flag used by some pages/buttons.
            'mission_loading': False,
        }
        self.lock = threading.Lock()
        self.mav_lock = threading.Lock()
        self.connection = None
        self.upload_queue = queue.Queue()
        self.mission_fetch_queue = queue.Queue()
        self.upload_status = {'status': 'idle', 'message': ''}
        self.current_worker_id = None

    def set_upload_status(self, status, message):
        with self.lock:
            self.upload_status = {'status': status, 'message': message}

    def get_upload_status(self):
        with self.lock:
            return self.upload_status

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
            if (
                ('wp_current' in data and data.get('wp_current') is not None)
                or ('wp_current_raw' in data and data.get('wp_current_raw') is not None)
            ):
                self.rover_data['wp_current_known'] = True
            self.rover_data['last_update'] = time.time()

    def append_message(self, msg_text):
        print(f"[Rover Message {threading.get_ident()}] {msg_text}")
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

    def get(self):
        with self.lock:
            return self.rover_data.copy()

    # --- Backwards-compatible helpers ---
    def set_loading(self, loading: bool):
        """Compatibility shim for older UI code.

        This does NOT drive mission download/upload logic; it's only a UI hint.
        """
        with self.lock:
            self.rover_data['mission_loading'] = bool(loading)
            self.rover_data['last_update'] = time.time()

    def is_loading(self) -> bool:
        with self.lock:
            return bool(self.rover_data.get('mission_loading', False))

@st.cache_resource
def get_shared_state():
    return SharedState()
