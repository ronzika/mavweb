from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
import requests
import time
import threading
import os

TELEGRAM_BOT_TOKEN = os.getenv('TELEGRAM_BOT_TOKEN', '7925780241:AAEXW4ct59A3VKtL3Ib0vrMHrhAim1qKgM8')
TELEGRAM_CHAT_ID = os.getenv('TELEGRAM_CHAT_ID', '778781384')
TELEGRAM_API_URL = f"https://api.telegram.org/bot{TELEGRAM_BOT_TOKEN}"

class MavMonitor(mp_module.MPModule):
    def __init__(self, mpstate):
        super().__init__(mpstate, "MavMonitor")
        self.last_rover_request_time = 0
        self.rover_cooldown = 5
        self.LAST_UPDATE_ID = None
        self.telegram_update_available = False
        self.lock = threading.Lock()

        # MAVLink data for GPS1 (rover)
        self.current_mode = 'UNKNOWN'
        self.gps_lat = 0.0
        self.gps_lon = 0.0
        self.battery_voltage = 0.0
        self.ground_speed = 0.0
        self.is_armed = False
        self.last_waypoint = 0
        self.gps_fix_type = 0
        self.gps_satellites_visible = 0
        self.gps_hdop = 0.0
        self.gps_vdop = 0.0
        self.total_distance_traveled = 0.0
        self.last_time = time.time()

        # GPS2 (moving base) data
        self.gps2_fix_type = None
        self.gps2_lat = 0.0
        self.gps2_lon = 0.0
        self.gps2_satellites_visible = 0
        self.gps2_hdop = 0.0
        self.gps2_vdop = 0.0

        # Mission and system timing
        self.mission_start_time = None
        self.system_boot_time = None
        self.last_boot_time_ms = 0  # Track last seen time_boot_ms for reboot detection
        self.reboot_detected = False  # Flag to prevent repeated messages on startup

        # Debug flags
        self.gps_received = False
        self.heartbeat_received = False

        self.console.write("[MavMonitor] Module loaded.\n")
        self.telegram_thread = threading.Thread(target=self.fetch_telegram_updates_loop, daemon=True)
        self.telegram_thread.start()

    def mavlink_packet(self, m):
        """Handle incoming MAVLink packets."""
        msg_type = m.get_type()
        if msg_type == 'HEARTBEAT':
            mode_num = m.custom_mode
            self.current_mode = self.master.mode_mapping().get(mode_num, f"UNKNOWN ({mode_num})")
            self.is_armed = m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if self.current_mode == 'AUTO' and self.mission_start_time is None and self.is_armed:
                self.mission_start_time = time.time()
            elif self.current_mode != 'AUTO':
                self.mission_start_time = None
            if not self.heartbeat_received:
                self.console.write(f"[MavMonitor] First HEARTBEAT received: Mode={self.current_mode}, Armed={self.is_armed}\n")
            self.heartbeat_received = True
        elif msg_type == 'GPS_RAW_INT':
            self.gps_lat = m.lat / 1e7
            self.gps_lon = m.lon / 1e7
            self.gps_fix_type = m.fix_type
            self.gps_satellites_visible = m.satellites_visible
            self.gps_hdop = m.eph / 100.0
            self.gps_vdop = m.epv / 100.0
            if not self.gps_received:
        elif msg_type == 'GPS2_RAW':
            self.gps2_lat = m.lat / 1e7
            self.gps2_lon = m.lon / 1e7
            self.gps2_fix_type = m.fix_type
            self.gps2_satellites_visible = m.satellites_visible
            self.gps2_hdop = m.eph / 100.0
            self.gps2_vdop = m.epv / 100.0
        elif msg_type == 'VFR_HUD':
            current_time = time.time()
            time_delta = current_time - self.last_time
            self.total_distance_traveled += m.groundspeed * time_delta
            self.last_time = current_time
            self.ground_speed = m.groundspeed
        elif msg_type == 'SYS_STATUS':
            self.battery_voltage = m.voltage_battery / 1000.0
        elif msg_type == 'SYSTEM_TIME':
            current_boot_time_ms = m.time_boot_ms
            # Detect reboot if time_boot_ms decreases significantly or resets
            if self.last_boot_time_ms > 10000 and current_boot_time_ms < self.last_boot_time_ms - 5000 and not self.reboot_detected:
                self.console.write(f"[MavMonitor] Flight controller reboot detected: {self.last_boot_time_ms}ms -> {current_boot_time_ms}ms\n")
                self.send_telegram_message("🔄 Flight controller rebooted!")
                self.reboot_detected = True
            elif current_boot_time_ms > self.last_boot_time_ms:
                self.reboot_detected = False  # Reset flag after normal increment
            self.last_boot_time_ms = current_boot_time_ms
            self.system_boot_time = time.time() - (current_boot_time_ms / 1000.0)
        elif msg_type == 'MISSION_CURRENT':
            self.last_waypoint = m.seq

    def get_gps_status(self, gps_type=1):
        fix_types = {3: '3D Fix', 4: 'DGPS', 5: 'RTK Float', 6: 'RTK Fixed'}
        if gps_type == 1:
            return fix_types.get(self.gps_fix_type, f"Unknown ({self.gps_fix_type})")
        elif gps_type == 2 and self.gps2_fix_type is not None:
            return fix_types.get(self.gps2_fix_type, f"Unknown ({self.gps2_fix_type})")
        return 'No Data'

    def format_time(self, seconds):
        """Convert seconds to a human-readable string."""
        if seconds is None or seconds < 0:
            return "N/A"
        minutes, secs = divmod(int(seconds), 60)
        hours, minutes = divmod(minutes, 60)
        return f"{hours:02d}:{minutes:02d}:{secs:02d}"

    def fetch_telegram_updates_loop(self):
        while True:
            if self.fetch_telegram_updates():
                with self.lock:
                    self.telegram_update_available = True
            time.sleep(1)

    def fetch_telegram_updates(self, retries=3):
        url = f"{TELEGRAM_API_URL}/getUpdates?offset={self.LAST_UPDATE_ID + 1 if self.LAST_UPDATE_ID else ''}"
        for attempt in range(retries):
            try:
                response = requests.get(url, timeout=5)
                response.raise_for_status()
                data = response.json()
                for update in data["result"]:
                    update_id = update["update_id"]
                    if update_id > (self.LAST_UPDATE_ID or 0) and update.get("message", {}).get("text", "").strip() == "/rover":
                        self.LAST_UPDATE_ID = update_id
                        return True
            except requests.exceptions.RequestException as e:
                self.console.write(f"[MavMonitor] Telegram fetch error (Attempt {attempt+1}): {e}\n")
                time.sleep(2)
        return False

    def idle_task(self):
        """Handle Telegram requests and RTK Fixed monitoring."""
        current_time = time.time()

        # Debug: Log if no data received yet
        if not self.gps_received:
            self.console.write("[MavMonitor] Waiting for GPS_RAW_INT message...\n")
        if not self.heartbeat_received:
            self.console.write("[MavMonitor] Waiting for HEARTBEAT message...\n")

        # Handle Telegram /rover command
        with self.lock:
            if self.telegram_update_available and (current_time - self.last_rover_request_time >= self.rover_cooldown):
                self.telegram_update_available = False
                self.last_rover_request_time = current_time

                mission_duration = current_time - self.mission_start_time if self.mission_start_time else 0
                uptime = current_time - self.system_boot_time if self.system_boot_time else 0

                message = (
                    f"Rover Status: \n"
                    f"📌 Mode: {self.current_mode}\n"
                    f"📍 GPS: {self.gps_lat:.6f}, {self.gps_lon:.6f}\n"
                    f"⚡ Battery: {self.battery_voltage:.2f}V\n"
                    f"🚗 Speed: {self.ground_speed:.2f} m/s\n"
                    f"🔒 Armed: {'Yes' if self.is_armed else 'No'}\n"
                    f"📍 Waypoint: {self.last_waypoint}\n"
                    f"📡 GPS1 Status: {self.get_gps_status(1)}\n"
                    f"🛰️ GPS1 Satellites: {self.gps_satellites_visible}\n"
                    f"📏 GPS1 HDOP/VDOP: {self.gps_hdop:.2f}/{self.gps_vdop:.2f}\n"
                )
                if self.gps2_fix_type is not None:
                    message += (
                        f"📡 GPS2 Status: {self.get_gps_status(2)}\n"
                        f"🛰️ GPS2 Satellites: {self.gps2_satellites_visible}\n"
                        f"📏 GPS2 HDOP/VDOP: {self.gps2_hdop:.2f}/{self.gps2_vdop:.2f}\n"
                    )
                message += (
                    f"🛤️ Distance Traveled: {self.total_distance_traveled:.2f} meters\n"
                    f"⏱️ Mission Duration: {self.format_time(mission_duration)}\n"
                    f"🔌 Uptime: {self.format_time(uptime)}"
                )

                self.console.write("[MavMonitor] Inbound Telegram Command Received\n")
                self.send_telegram_message(message)
                self.send_telegram_location(self.gps_lat, self.gps_lon)

    def send_telegram_message(self, message, silent=False):
        url = f"{TELEGRAM_API_URL}/sendMessage"
        payload = {"chat_id": TELEGRAM_CHAT_ID, "text": message, "disable_notification": silent}
        self._send_with_retries(url, payload)

    def send_telegram_location(self, lat, lon, silent=False):
        url = f"{TELEGRAM_API_URL}/sendLocation"
        payload = {"chat_id": TELEGRAM_CHAT_ID, "latitude": lat, "longitude": lon, "disable_notification": silent}
        self._send_with_retries(url, payload)

    def _send_with_retries(self, url, payload, retries=3):
        for attempt in range(retries):
            try:
                response = requests.post(url, json=payload, timeout=5)
                response.raise_for_status()
                return
            except requests.exceptions.RequestException as e:
                self.console.write(f"[MavMonitor] Telegram send error (Attempt {attempt+1}): {e}\n")
                time.sleep(2)

    def clear_telegram_queue(self):
        url = f"{TELEGRAM_API_URL}/getUpdates"
        try:
            response = requests.get(url, timeout=5).json()
            if "result" in response:
                for update in response["result"][-5:]:
                    update_id = update["update_id"]
                    message_id = update.get("message", {}).get("message_id", None)
                    if message_id:
                        self.delete_telegram_message(message_id)
                    self.LAST_UPDATE_ID = update_id
        except requests.exceptions.RequestException as e:
            self.console.write(f"[MavMonitor] Failed to clear Telegram queue: {e}\n")

    def delete_telegram_message(self, message_id):
        url = f"{TELEGRAM_API_URL}/deleteMessage"
        payload = {"chat_id": TELEGRAM_CHAT_ID, "message_id": message_id}
        for attempt in range(3):
            try:
                response = requests.post(url, json=payload, timeout=3)
                response.raise_for_status()
                self.console.write(f"[MavMonitor] Deleted queued Telegram message: {message_id}\n")
                return
            except requests.exceptions.RequestException as e:
                if "Bad Request" in str(e):
                    self.console.write(f"[MavMonitor] Message already deleted, skipping: {message_id}\n")
                    return
                self.console.write(f"[MavMonitor] Failed to delete message {message_id} (Attempt {attempt+1}): {e}\n")
                time.sleep(1)

def init(mpstate):
    return MavMonitor(mpstate)