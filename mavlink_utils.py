import os
import time
import datetime
import traceback
import threading
import socket
import base64
from typing import Any
from pymavlink import mavutil
from dotenv import load_dotenv

load_dotenv()

def parse_mission_file(filepath: str):
    try:
        with open(filepath) as f:
            lines = [line.strip() for line in f if line.strip() and not line.startswith('#')]
        if not lines or not lines[0].startswith('QGC WPL'):
            print("[MAVLINK] Not a QGC WPL mission file.")
            return None
        mission_items = []
        for line in lines[1:]:
            cols = line.split('\t')
            if len(cols) < 12:
                continue
            mission_items.append({
                'seq': int(cols[0]),
                'current': int(cols[1]),
                'frame': int(cols[2]),
                'command': int(float(cols[3])),
                'param1': float(cols[4]),
                'param2': float(cols[5]),
                'param3': float(cols[6]),
                'param4': float(cols[7]),
                'x': float(cols[8]),
                'y': float(cols[9]),
                'z': float(cols[10]),
                'autocontinue': int(cols[11])
            })
        return mission_items
    except Exception as e:
        print(f"[MAVLINK] Error parsing mission file: {e}")
        return None

def upload_mission(master, mission_items):
    try:
        print("[MAVLINK] Clearing existing mission...")
        master.mav.mission_clear_all_send(master.target_system, master.target_component)
        
        # Wait for ACK for clear
        ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=3)
        if ack:
            print(f"[MAVLINK] Clear ACK: {ack.type}")
        
        print(f"[MAVLINK] Sending {len(mission_items)} mission items...")
        master.mav.mission_count_send(master.target_system, master.target_component, len(mission_items))
        
        last_req_seq = -1
        
        while True:
            # Wait for REQUEST or ACK
            # We use a shorter timeout to allow for potential retries or UI updates if needed, 
            # but here we just block.
            msg = master.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], blocking=True, timeout=5)
            
            if msg is None:
                print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [MAVLINK] Timeout waiting for mission request/ack")
                return False
                
            if msg.get_type() == 'MISSION_ACK':
                if msg.type == 0: # MAV_MISSION_ACCEPTED
                    print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [MAVLINK] Mission upload success!")
                    return True
                else:
                    print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [MAVLINK] Mission upload failed with ACK error: {msg.type}")
                    return False
            
            # Handle Request
            seq = msg.seq
            if seq >= len(mission_items):
                print(f"[MAVLINK] Requested seq {seq} out of bounds")
                continue
                
            item = mission_items[seq]
            
            # Use INT for precision and robustness
            master.mav.mission_item_int_send(
                master.target_system,
                master.target_component,
                item['seq'],
                item['frame'],
                item['command'],
                item['current'],
                item['autocontinue'],
                item['param1'],
                item['param2'],
                item['param3'],
                item['param4'],
                int(item['x'] * 1e7), # Lat
                int(item['y'] * 1e7), # Lon
                float(item['z'])      # Alt
            )
            
            if seq != last_req_seq:
                print(f"[MAVLINK] Sent mission item {seq}")
                last_req_seq = seq

    except Exception as e:
        print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [MAVLINK] Mission upload failed: {e}\n{traceback.format_exc()}")
        return False

def upload_mission_to_fc(filepath: str) -> bool:
    endpoint = os.getenv("MAVLINK_ENDPOINT", "udpin:0.0.0.0:14550")
    mission_items = parse_mission_file(filepath)
    if not mission_items:
        return False
        
    try:
        print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] [MAVLINK] Connecting to {endpoint}")
        master = mavutil.mavlink_connection(endpoint)
        master.wait_heartbeat(timeout=10)
        return upload_mission(master, mission_items)
    except Exception as e:
        print(f"[MAVLINK] Connection failed: {e}")
        return False

"""
MAVLink and NTRIP logic
"""

NTRIP_CONNECTED = {'status': False, 'rtcm_count': 0}

class MavlinkClient:
    def __init__(self, endpoint: str):
        self.endpoint = endpoint
        self.conn = None
        self.thread = None

    def start(self, loop_fn):
        self.conn = mavutil.mavlink_connection(self.endpoint)
        self.thread = threading.Thread(target=loop_fn, args=(self.conn,), daemon=True)
        self.thread.start()
        return self.conn

def start_ntrip_injection(conn: Any, config: dict):
    def ntrip_thread():
        caster = config.get('NTRIP_CASTER', '')
        port = int(config.get('NTRIP_PORT', 2101))
        mountpoint = config.get('NTRIP_MOUNTPOINT', '')
        user = config.get('NTRIP_USER', '')
        passwd = config.get('NTRIP_PASS', '')
        if not (caster and mountpoint and user and passwd):
            print("[NTRIP] Missing NTRIP configuration, skipping NTRIP injection.")
            NTRIP_CONNECTED['status'] = False
            return
        try:
            print(f"[NTRIP] Connecting to {caster}:{port}/{mountpoint}")
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(10)
            s.connect((caster, port))
            print(f"[NTRIP] Sending NTRIP request...")
            auth = base64.b64encode(f"{user}:{passwd}".encode()).decode()
            # ...existing NTRIP logic continues here...
        except Exception as e:
            print(f"[NTRIP] Error: {e}")
            NTRIP_CONNECTED['status'] = False
    t = threading.Thread(target=ntrip_thread, daemon=True)
    t.start()
