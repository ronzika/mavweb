# MavInject

from pymavlink import mavutil
from dotenv import load_dotenv
from telegram import Update, InlineKeyboardButton, InlineKeyboardMarkup
from telegram.ext import Application, CommandHandler, ContextTypes, CallbackQueryHandler
import urllib.request
import urllib.parse
import threading
import os
import time
import re

load_dotenv()

MAVLINK_TX_ENDPOINT = os.getenv("MAVLINK_TX_ENDPOINT", "udpout:127.0.0.1:14550")
MAVLINK_RX_ENDPOINT = os.getenv("MAVLINK_RX_ENDPOINT", "udpin:0.0.0.0:14551")
MAVLINK_NAME = os.getenv("MAVLINK_NAME", "DS18F     ")
TELEGRAM_BOT_TOKEN = os.getenv("TELEGRAM_BOT_TOKEN")
TELEGRAM_CHAT_ID = os.getenv("TELEGRAM_CHAT_ID")
MAVLINK_THROTTLE_SECONDS = float(os.getenv("MAVLINK_THROTTLE_SECONDS", "10"))
EXTENDED_STATUS = os.getenv("EXTENDED_STATUS", "0") == "1"
DEBUG = os.getenv("DEBUG", "0") == "1"
TEMP_SENDER_ENABLED = os.getenv("TEMP_SENDER_ENABLED", "1") == "1"

def debug_print(msg: str):
    if DEBUG:
        print(msg)

from typing import Any
master_tx: Any = mavutil.mavlink_connection(
    MAVLINK_TX_ENDPOINT,
    dialect="ardupilotmega",
    source_system=1,
    # Use a non-autopilot component id to avoid our own heartbeats being
    # mistaken for vehicle mode changes (200 = MAV_COMP_ID_PERIPHERAL / custom)
    source_component=200,
    force_mavlink_version=2
)
try:
    master_rx = mavutil.mavlink_connection(
        MAVLINK_RX_ENDPOINT,
        dialect="ardupilotmega",
        source_system=1,
        source_component=1,
        force_mavlink_version=2
    )
except Exception as e:
    debug_print(f"Failed to open RX endpoint '{MAVLINK_RX_ENDPOINT}': {e}")
    master_rx = None

def read_cpu_temp_c():
    path = "/sys/class/thermal/thermal_zone0/temp"
    try:
        with open(path, "r") as f:
            millis = f.read().strip()
        return float(millis) / 1000.0
    except Exception:
        return None

sensor = None
if TEMP_SENDER_ENABLED:
    try:
        from w1thermsensor import W1ThermSensor
        from w1thermsensor.errors import NoSensorFoundError
        sensor = W1ThermSensor()
    except Exception as e:
        debug_print(f"Temperature sensor disabled or unavailable: {e}. Using CPU/dummy temperature.")


def read_temp_c() -> float:
    if sensor is not None:
        return float(sensor.get_temperature())
    temp_c = read_cpu_temp_c()
    if temp_c is None:
        dummy = os.environ.get("MAVINJECT_DUMMY_TEMP_C", "99.0")
        try:
            temp_c = float(dummy)
        except ValueError:
            temp_c = 99.0
    return float(temp_c)


def mavlink_loop():
    while True:
        temp_c = read_temp_c()
        temp_f = temp_c * 9.0 / 5.0 + 32.0
        time_ms = int(time.time() * 1000) & 0xFFFFFFFF
        name_bytes = MAVLINK_NAME.encode("ascii")[:10].ljust(10, b" ")
        master_tx.mav.named_value_float_send(
            time_ms,
            name_bytes,
            float(temp_f)
        )
        time.sleep(1)


app: Application | None = None
chat_id_lock = threading.Lock()
chat_id: int | None = None
last_mode: str | None = None
last_named_value_sent_ts: float = 0.0
last_gps1_fix: int | None = None
last_gps2_fix: int | None = None
monitor_job_started: bool = False
last_heartbeat_notify_ts: float = 0.0
last_custom_mode: int | None = None
last_mode_sent_ts: float = 0.0
last_custom_mode_sent: int | None = None
pending_custom_mode: int | None = None
pending_mode_name: str | None = None
pending_mode_first_seen: float | None = None
autopilot_sysid: int | None = None
autopilot_compid: int | None = None
rover_lat: float | None = None
rover_lon: float | None = None
rover_alt_m: float | None = None
rover_hdop: float | None = None
rover_vdop: float | None = None
rover_ground_speed: float | None = None
rover_batt_voltage: float | None = None
rover_armed: bool | None = None
rover_wp_seq: int | None = None
saved_wp_seq: int | None = None
rover_start_time: float | None = None
rover_last_gps_time: float | None = None
rover2_lat: float | None = None
rover2_lon: float | None = None
rover2_alt_m: float | None = None
rover2_hdop: float | None = None
rover2_vdop: float | None = None
rover2_last_gps_time: float | None = None
rover_failsafe_active: bool = False
rover_failsafe_desc: str | None = None
rover_failsafe_reason: str | None = None
gcs_link_quality_pct: float | None = None
gcs_link_rssi: int | None = None
gcs_link_remrssi: int | None = None
gcs_link_noise: int | None = None
gcs_link_remnoise: int | None = None
gcs_link_txbuf: int | None = None
gcs_link_rxerrors: int | None = None
gcs_link_fixed: int | None = None
last_status_texts: list[str] = []

MAV_MODE_FLAG_SAFETY_ARMED = getattr(mavutil.mavlink, 'MAV_MODE_FLAG_SAFETY_ARMED', 128)


def set_chat_id(new_id: int):
    global chat_id
    with chat_id_lock:
        chat_id = new_id


def get_chat_id() -> int | None:
    with chat_id_lock:
        return chat_id


async def bot_send_message_async(text: str, silent: bool = True):
    global app
    target = get_chat_id()
    if app is None or target is None:
        return
    try:
        await app.bot.send_message(chat_id=target, text=text, disable_notification=silent)
    except Exception:
        pass

def bot_send_message_http(text: str, silent: bool = True):
    target = get_chat_id()
    if not TELEGRAM_BOT_TOKEN or target is None:
        return
    url = f"https://api.telegram.org/bot{TELEGRAM_BOT_TOKEN}/sendMessage"
    data = urllib.parse.urlencode({
        "chat_id": str(target),
        "text": text,
        "disable_notification": "true" if silent else "false"
    }).encode("utf-8")
    req = urllib.request.Request(url, data=data)
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            # Basic confirmation log to help debug delivery
            try:
                status = getattr(resp, 'status', None)
            except Exception:
                status = None
            debug_print(f"[mavinject] Bot send OK (status={status}): {text[:60]}")
    except Exception:
        debug_print(f"[mavinject] Bot send FAILED: {text[:60]}")

def bot_send_location_http(lat: float, lon: float, silent: bool = True, accuracy: float | None = None):
    target = get_chat_id()
    if not TELEGRAM_BOT_TOKEN or target is None:
        return
    url = f"https://api.telegram.org/bot{TELEGRAM_BOT_TOKEN}/sendLocation"
    payload = {
        "chat_id": str(target),
        "latitude": f"{lat:.7f}",
        "longitude": f"{lon:.7f}",
        "disable_notification": "true" if silent else "false",
    }
    if accuracy is not None:
        # Telegram expects meters in [0,1500]
        try:
            acc_m = max(0.0, min(1500.0, float(accuracy)))
            payload["horizontal_accuracy"] = f"{acc_m:.1f}"
        except Exception:
            pass
    data = urllib.parse.urlencode(payload).encode("utf-8")
    req = urllib.request.Request(url, data=data)
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            try:
                status = getattr(resp, 'status', None)
            except Exception:
                status = None
            debug_print(f"[mavinject] Location send OK (status={status}) lat={lat:.6f} lon={lon:.6f}")
    except Exception:
        debug_print(f"[mavinject] Location send FAILED lat={lat:.6f} lon={lon:.6f}")


async def cmd_start(update: Update, context: ContextTypes.DEFAULT_TYPE):
    if update.effective_chat and getattr(update.effective_chat, 'id', None) is not None:
        set_chat_id(update.effective_chat.id)
    if update.message is None:
        return
    await update.message.reply_text("Bot is ready. Send /temp to get the temperature.")
    # Monitor runs in its own thread; no JobQueue scheduling required.


async def cmd_temp(update: Update, context: ContextTypes.DEFAULT_TYPE):
    if update.effective_chat and getattr(update.effective_chat, 'id', None) is not None:
        set_chat_id(update.effective_chat.id)
    if update.message is None:
        return
    temp_c = read_temp_c()
    temp_f = temp_c * 9.0 / 5.0 + 32.0
    await update.message.reply_text(f"Temperature: {temp_c:.2f} °C / {temp_f:.2f} °F")

async def cmd_ping(update: Update, context: ContextTypes.DEFAULT_TYPE):
    if update.effective_chat and getattr(update.effective_chat, 'id', None) is not None:
        set_chat_id(update.effective_chat.id)
    if update.message is None:
        return
    await update.message.reply_text("Ping received. Attempting to send test notification.")
    bot_send_message_http("Test notification from mavinject", silent=True)


def mavlink_monitor_loop():
    global last_mode, last_named_value_sent_ts, last_gps1_fix, last_gps2_fix, last_heartbeat_notify_ts, last_custom_mode, last_mode_sent_ts, \
        last_custom_mode_sent, pending_custom_mode, pending_mode_name, pending_mode_first_seen, autopilot_sysid, autopilot_compid, rover_lat, rover_lon, rover_alt_m, rover_hdop, rover_vdop, rover_ground_speed, rover_batt_voltage, rover_armed, rover_wp_seq, rover_start_time, rover_last_gps_time, rover2_lat, rover2_lon, rover2_alt_m, rover2_hdop, rover2_vdop, rover2_last_gps_time, rover_failsafe_active, rover_failsafe_desc, rover_failsafe_reason, gcs_link_quality_pct, gcs_link_rssi, gcs_link_remrssi, gcs_link_noise, gcs_link_remnoise, gcs_link_txbuf, gcs_link_rxerrors, gcs_link_fixed
    if TELEGRAM_CHAT_ID and get_chat_id() is None:
        try:
            set_chat_id(int(TELEGRAM_CHAT_ID))
        except ValueError:
            pass
    name_str = MAVLINK_NAME[:10].ljust(10)
    while True:
        drained = 0
        while True:
            msg = None
            if master_rx is not None:
                msg = master_rx.recv_match(blocking=False)
            if not msg:
                break
            drained += 1
            debug_print(f"[mavinject] Detected MAVLink message: {msg.get_type()}")
            mtype = msg.get_type()
            if mtype == 'HEARTBEAT':
                src_sys = getattr(msg, 'get_srcSystem', lambda: None)()
                src_comp = getattr(msg, 'get_srcComponent', lambda: None)()
                # Establish autopilot identity if not fixed yet
                if autopilot_sysid is None:
                    env_sys = os.getenv('AUTOPILOT_SYSID')
                    try:
                        autopilot_sysid = int(env_sys) if env_sys else src_sys
                    except Exception:
                        autopilot_sysid = src_sys
                if autopilot_compid is None:
                    env_comp = os.getenv('AUTOPILOT_COMPID')
                    try:
                        autopilot_compid = int(env_comp) if env_comp else src_comp
                    except Exception:
                        autopilot_compid = src_comp
                # Ignore heartbeats not from selected autopilot system/component
                if (autopilot_sysid is not None and src_sys != autopilot_sysid) or \
                   (autopilot_compid is not None and src_comp != autopilot_compid):
                    debug_print(f"[mavinject] Ignoring heartbeat from sys={src_sys} comp={src_comp}; tracking sys={autopilot_sysid} comp={autopilot_compid}")
                    continue
                # Extract custom_mode first for stable comparisons
                cm_raw = getattr(msg, 'custom_mode', None)
                try:
                    cm = int(cm_raw) if cm_raw is not None else None
                except Exception:
                    cm = None
                # Decode Rover mode from custom_mode when possible
                mode_name = None
                if cm is not None:
                    decoded = rover_mode_from_custom_mode(cm)
                    if decoded != 'UNKNOWN':
                        mode_name = decoded
                # Fallback to mavutil mode string if still unknown
                if mode_name is None:
                    try:
                        ms = mavutil.mode_string_v10(msg)
                        # Filter generic placeholder like Mode(0x00000000)
                        if ms and not ms.startswith('Mode('):
                            mode_name = ms
                    except Exception:
                        pass
                if mode_name is None:
                    # Final fallback with raw custom_mode
                    mode_name = f"UNKNOWN(cm={cm if cm is not None else 'NA'})"
                now = time.time()
                if rover_start_time is None:
                    rover_start_time = now
                try:
                    base_mode = int(getattr(msg, 'base_mode', 0))
                    rover_armed = bool(base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
                except Exception:
                    pass
                periodic_enabled = os.getenv('HEARTBEAT_PERIODIC', '0') == '1'
                # Debounce flapping: require stability window before sending
                debounce_seconds = float(os.getenv('MODE_DEBOUNCE_SECONDS', '2.0'))
                # First mode after startup: send immediately
                if last_custom_mode_sent is None and cm is not None:
                    last_custom_mode_sent = cm
                    last_custom_mode = cm
                    last_mode = mode_name
                    last_mode_sent_ts = now
                    debug_print(f"[mavinject] Initial mode: {mode_name} (custom_mode={cm})")
                    bot_send_message_http(f"[Mode Change]: {mode_name}", silent=False)
                elif cm is not None and cm != last_custom_mode_sent:
                    # Track pending candidate
                    if pending_custom_mode != cm:
                        pending_custom_mode = cm
                        pending_mode_name = mode_name
                        pending_mode_first_seen = now
                        debug_print(f"[mavinject] Mode candidate detected: {mode_name} (cm={cm}) - waiting debounce")
                    else:
                        # Same candidate persists; check elapsed time
                        if pending_mode_first_seen and (now - pending_mode_first_seen) >= debounce_seconds:
                            last_custom_mode_sent = cm
                            last_custom_mode = cm
                            last_mode = pending_mode_name
                            last_mode_sent_ts = now
                            debug_print(f"[mavinject] Mode change confirmed: {pending_mode_name} (custom_mode={cm})")
                            bot_send_message_http(f"[Mode changed]: {pending_mode_name}", silent=False)
                            pending_custom_mode = None
                            pending_mode_name = None
                            pending_mode_first_seen = None
                elif periodic_enabled and now - last_heartbeat_notify_ts >= 15:
                    last_heartbeat_notify_ts = now
                    debug_print(f"[mavinject] Heartbeat periodic: {mode_name} (cm={cm})")
                    bot_send_message_http(f"Heartbeat alive. Mode={mode_name}", silent=True)
            elif mtype == 'NAMED_VALUE_FLOAT':
                try:
                    n = msg.name.decode('ascii') if isinstance(msg.name, (bytes, bytearray)) else str(msg.name)
                except Exception:
                    n = str(msg.name)
                # Relax match: compare trimmed names
                if n.strip() == MAVLINK_NAME.strip() or n[:10].ljust(10) == name_str:
                    now = time.time()
                    if now - last_named_value_sent_ts >= MAVLINK_THROTTLE_SECONDS:
                        last_named_value_sent_ts = now
                        debug_print(f"[mavinject] Forwarding named value {n.strip()}: {msg.value:.2f}")
                        bot_send_message_http(f"{n.strip()}: {msg.value:.2f}")
            elif mtype in ('[MSG]: ', 'STATUSTEXT_LONG'):
                # Forward status text messages like Mission Planner's Messages tab
                try:
                    sev = int(getattr(msg, 'severity', 0))
                except Exception:
                    sev = 0
                # Extract text robustly
                txt = None
                try:
                    raw = getattr(msg, 'text', None)
                    if raw is not None:
                        txt = raw.decode('utf-8', errors='ignore') if isinstance(raw, (bytes, bytearray)) else str(raw)
                except Exception:
                    txt = None
                if not txt:
                    txt = f"{mtype} (sev={sev})"
                # Make critical messages louder by disabling silence; otherwise silent
                is_critical = sev >= 6  # MAV_SEVERITY: 0-7 (6=CRITICAL, 7=ALERT)
                debug_print(f"[mavinject] {mtype}(sev={sev}): {txt}")
                # Append to ring buffer of last status texts
                try:
                    last_status_texts.append(f"[{mtype}:{sev}] {txt}")
                    if len(last_status_texts) > 50:
                        # Keep buffer bounded; retain last 50
                        del last_status_texts[:len(last_status_texts)-50]
                except Exception:
                    pass
                # Failsafe detection
                failsafe_loud = os.getenv('FAILSAFE_NOTIFY_LOUD', '1') == '1'
                lower_txt = txt.lower()
                if 'failsafe' in lower_txt:
                    # Determine if clearing phrase present
                    clearing = any(kw in lower_txt for kw in ['end', 'ended', 'recovered', 'cleared']) and 'failsafe' in lower_txt
                    if clearing:
                        rover_failsafe_active = False
                        rover_failsafe_desc = None
                        rover_failsafe_reason = None
                        bot_send_message_http(f"Failsafe cleared: {txt}", silent=not failsafe_loud)
                    else:
                        rover_failsafe_active = True
                        rover_failsafe_desc = txt
                        rover_failsafe_reason = extract_failsafe_reason(lower_txt)
                        display_txt = txt if rover_failsafe_reason is None else f"{txt} (reason={rover_failsafe_reason})"
                        bot_send_message_http(f"[FAILSAFE]: {display_txt}", silent=not failsafe_loud)
                else:
                    # Only mode or GPS fix changes should be loud; force silent for other status text
                    bot_send_message_http(f"Status: {txt}", silent=True)
                # Optional: parse linkqualitygcs from STATUSTEXT if present
                try:
                    if 'linkqualitygcs' in lower_txt:
                        m = re.search(r'linkqualitygcs\s*[:=]?\s*(\d+)', lower_txt)
                        if m:
                            gcs_link_quality_pct = float(m.group(1))
                            debug_print(f"[mavinject] Parsed linkqualitygcs={gcs_link_quality_pct}% from status text")
                except Exception:
                    pass
            # Fallback: some builds may embed human text in other packets; optionally forward
            elif os.getenv("FORWARD_ALL_TEXT", "0") == "1" and hasattr(msg, 'text'):
                # Attempt to extract generic text
                try:
                    raw = getattr(msg, 'text')
                    txt = raw.decode('utf-8', errors='ignore') if isinstance(raw, (bytes, bytearray)) else str(raw)
                except Exception:
                    txt = None
                if txt:
                    debug_print(f"[mavinject] GenericText {mtype}: {txt}")
                    bot_send_message_http(f"Status?: {txt}", silent=True)
            elif mtype == 'GPS_RAW_INT':
                try:
                    fix = int(getattr(msg, 'fix_type', 0))
                except Exception:
                    fix = 0
                try:
                    lat_i = int(getattr(msg, 'lat', 0))
                    lon_i = int(getattr(msg, 'lon', 0))
                    alt_i = int(getattr(msg, 'alt', 0))
                    eph = int(getattr(msg, 'eph', 0))
                    epv = int(getattr(msg, 'epv', 0))
                    if lat_i != 0:
                        rover_lat = lat_i / 1e7
                    if lon_i != 0:
                        rover_lon = lon_i / 1e7
                    if alt_i != 0:
                        rover_alt_m = alt_i / 1000.0
                    if eph != 0:
                        rover_hdop = eph / 100.0
                    if epv != 0:
                        rover_vdop = epv / 100.0
                    rover_last_gps_time = time.time()
                except Exception:
                    pass
                # Compare explicitly; do not treat 0 as falsy (was causing repeated messages)
                if last_gps1_fix is None or fix != last_gps1_fix:
                    prev = last_gps1_fix
                    last_gps1_fix = fix
                    debug_print(f"[mavinject] GPS1 fix change: {fix}")
                    # Loud only when entering RTK Fixed (6) or leaving it
                    entering_rtk_fixed = (prev != 6 and fix == 6)
                    leaving_rtk_fixed = (prev == 6 and fix != 6)
                    bot_send_message_http(
                        f"[GPS1 fix changed]: {gps_fix_label(fix)}",
                        silent=not (entering_rtk_fixed or leaving_rtk_fixed)
                    )
            elif mtype in ('GPS2_RAW', 'GPS2_RAW_INT'):
                try:
                    fix = int(getattr(msg, 'fix_type', 0))
                except Exception:
                    fix = 0
                # Capture GPS2 fields if present
                try:
                    lat_i = int(getattr(msg, 'lat', 0))
                    lon_i = int(getattr(msg, 'lon', 0))
                    alt_i = int(getattr(msg, 'alt', 0))
                    eph = int(getattr(msg, 'eph', 0))
                    epv = int(getattr(msg, 'epv', 0))
                    if lat_i != 0:
                        rover2_lat = lat_i / 1e7
                    if lon_i != 0:
                        rover2_lon = lon_i / 1e7
                    if alt_i != 0:
                        rover2_alt_m = alt_i / 1000.0
                    if eph != 0:
                        rover2_hdop = eph / 100.0
                    if epv != 0:
                        rover2_vdop = epv / 100.0
                    rover2_last_gps_time = time.time()
                except Exception:
                    pass
                if last_gps2_fix is None or fix != last_gps2_fix:
                    prev = last_gps2_fix
                    last_gps2_fix = fix
                    debug_print(f"[mavinject] GPS2 fix change: {fix}")
                    entering_rtk_fixed = (prev != 6 and fix == 6)
                    leaving_rtk_fixed = (prev == 6 and fix != 6)
                    bot_send_message_http(
                        f"[GPS2 fix changed]: {gps_fix_label(fix)}",
                        silent=not (entering_rtk_fixed or leaving_rtk_fixed)
                    )
            elif mtype == 'COMMAND_LONG' and EXTENDED_STATUS:
                # Forward relay-related command invocations (before ACK), decode ON/OFF
                try:
                    cmd_id = int(getattr(msg, 'command', -1))
                except Exception:
                    cmd_id = -1
                relay_related = {181, 182, 184}
                if cmd_id in relay_related:
                    p1 = float(getattr(msg, 'param1', 0.0))
                    p2 = float(getattr(msg, 'param2', 0.0))
                    relay_idx = int(p1)
                    state = int(p2)
                    state_str = 'ON' if state != 0 else 'OFF'
                    debug_print(f"[mavinject] COMMAND_LONG Relay: idx={relay_idx} state={state_str}")
                    # Relay notifications now always silent (only mode/GPS loud)
                    bot_send_message_http(f"[Relay {relay_idx}]: {state_str}", silent=True)
            elif mtype == 'COMMAND_ACK' and EXTENDED_STATUS:
                # Forward command acknowledgements (e.g., relay toggles)
                try:
                    cmd_id = int(getattr(msg, 'command', -1))
                except Exception:
                    cmd_id = -1
                try:
                    result = int(getattr(msg, 'result', -1))
                except Exception:
                    result = -1
                cmd_name = None
                try:
                    enum_cmd = mavutil.mavlink.enums.get('MAV_CMD', {})
                    entry = enum_cmd.get(cmd_id)
                    if entry and getattr(entry, 'name', None):
                        cmd_name = entry.name
                except Exception:
                    cmd_name = None
                if cmd_name is None:
                    cmd_name = f"MAV_CMD_{cmd_id}"
                # Relay-related commands to highlight
                relay_related = {181, 182, 184}  # DO_SET_RELAY, DO_REPEAT_RELAY, DO_SET_DIGITAL_PIN
                debug_print(f"[mavinject] COMMAND_ACK: command={cmd_name} result={result}")
                # Always silent unless it's a mode or GPS change (ACKs forced silent)
                bot_send_message_http(f"[Command ACK]: {cmd_name} result={result}", silent=True)
            elif mtype == 'VFR_HUD':
                try:
                    rover_ground_speed = float(getattr(msg, 'groundspeed', rover_ground_speed or 0.0))
                except Exception:
                    pass
            elif mtype == 'SYS_STATUS':
                try:
                    vb_mv = int(getattr(msg, 'voltage_battery', 0))
                    if vb_mv > 0:
                        rover_batt_voltage = vb_mv / 1000.0
                except Exception:
                    pass
            elif mtype == 'RADIO_STATUS':
                # SiK telemetry radio link statistics
                try:
                    gcs_link_rssi = int(getattr(msg, 'rssi', gcs_link_rssi or 0))
                except Exception:
                    pass
                try:
                    gcs_link_remrssi = int(getattr(msg, 'remrssi', gcs_link_remrssi or 0))
                except Exception:
                    pass
                try:
                    gcs_link_noise = int(getattr(msg, 'noise', gcs_link_noise or 0))
                except Exception:
                    pass
                try:
                    gcs_link_remnoise = int(getattr(msg, 'remnoise', gcs_link_remnoise or 0))
                except Exception:
                    pass
                try:
                    gcs_link_txbuf = int(getattr(msg, 'txbuf', gcs_link_txbuf or 0))
                except Exception:
                    pass
                try:
                    gcs_link_rxerrors = int(getattr(msg, 'rxerrors', gcs_link_rxerrors or 0))
                except Exception:
                    pass
                try:
                    gcs_link_fixed = int(getattr(msg, 'fixed', gcs_link_fixed or 0))
                except Exception:
                    pass
                try:
                    total = (gcs_link_rxerrors or 0) + (gcs_link_fixed or 0)
                    if total > 0:
                        gcs_link_quality_pct = max(0.0, min(100.0, 100.0 * (gcs_link_fixed or 0) / total))
                except Exception:
                    pass
            elif mtype == 'MISSION_CURRENT':
                try:
                    rover_wp_seq = int(getattr(msg, 'seq', rover_wp_seq or 0))
                except Exception:
                    pass
            elif mtype == 'SYSTEM_TIME':
                try:
                    if rover_start_time is None:
                        rover_start_time = time.time()
                except Exception:
                    pass
        if drained == 0:
            debug_print("[mavinject] No MAVLink messages this tick")
        time.sleep(0.5)

def gps_fix_label(fix_type: int) -> str:
    mapping = {
        0: 'No GPS',
        1: 'No Fix',
        2: '2D Fix',
        3: '3D Fix',
        4: 'DGPS',
        5: 'RTK Float',
        6: 'RTK Fixed',
        7: 'Static',
        8: 'PPP',
    }
    return mapping.get(int(fix_type), f'Unknown({fix_type})')

def rover_mode_from_custom_mode(cm: int) -> str:
    modes = {
        0: 'MANUAL',
        1: 'ACRO',
        3: 'STEERING',
        4: 'HOLD',
        5: 'FOLLOW',
        6: 'SIMPLE',
        10: 'AUTO',
        11: 'RTL',
        15: 'GUIDED',
        11: 'INITIALISING'
    }
    return modes.get(cm, 'UNKNOWN')

def format_hms(seconds: float | None) -> str:
    if seconds is None:
        return "-"
    s = int(seconds)
    h = s // 3600
    m = (s % 3600) // 60
    sec = s % 60
    return f"{h:02d}:{m:02d}:{sec:02d}"

def rover_status_snapshot() -> str:
    now = time.time()
    elapsed = (now - rover_start_time) if rover_start_time else None
    gps_age = (now - rover_last_gps_time) if rover_last_gps_time else None
    lines = []
    lines.append(f"Mode: {last_mode or 'Unknown'}")
    lines.append(f"Armed: {'Yes' if rover_armed else 'No' if rover_armed is not None else '-'}")
    if rover_lat is not None and rover_lon is not None:
        lines.append(f"Lat/Lon: {rover_lat:.6f}, {rover_lon:.6f}")
    else:
        lines.append("Lat/Lon: -")
    if rover_alt_m is not None:
        lines.append(f"Alt: {rover_alt_m:.1f} m")
    else:
        lines.append("Alt: -")
    fix_label = gps_fix_label(last_gps1_fix) if last_gps1_fix is not None else '-'
    hdop_str = f"{rover_hdop:.2f}" if rover_hdop is not None else '-'
    vdop_str = f"{rover_vdop:.2f}" if rover_vdop is not None else '-'
    age_str = f"{gps_age:.1f}s" if gps_age is not None else '-'
    lines.append(f"GPS1: {fix_label} HDOP={hdop_str} VDOP={vdop_str} Age={age_str}")
    gps2_age = (now - rover2_last_gps_time) if rover2_last_gps_time else None
    fix2_label = gps_fix_label(last_gps2_fix) if last_gps2_fix is not None else '-'
    hdop2_str = f"{rover2_hdop:.2f}" if rover2_hdop is not None else '-'
    vdop2_str = f"{rover2_vdop:.2f}" if rover2_vdop is not None else '-'
    age2_str = f"{gps2_age:.1f}s" if gps2_age is not None else '-'
    lines.append(f"GPS2: {fix2_label} HDOP={hdop2_str} VDOP={vdop2_str} Age={age2_str}")
    if rover_ground_speed is not None:
        lines.append(f"Speed: {rover_ground_speed:.2f} m/s")
    else:
        lines.append("Speed: -")
    # Temperature reading
    try:
        t_c = read_temp_c()
        t_f = t_c * 9.0 / 5.0 + 32.0
        lines.append(f"Temp: {t_c:.2f} °C / {t_f:.2f} °F")
    except Exception:
        lines.append("Temp: -")
    if rover_batt_voltage is not None:
        lines.append(f"Battery: {rover_batt_voltage:.2f} V")
    else:
        lines.append("Battery: -")
    lines.append(f"WP Seq: {rover_wp_seq if rover_wp_seq is not None else '-'}")
    # GCS link status
    q_str = f"{gcs_link_quality_pct:.0f}%" if gcs_link_quality_pct is not None else '-'
    rssi_str = f"{gcs_link_rssi}" if gcs_link_rssi is not None else '-'
    remrssi_str = f"{gcs_link_remrssi}" if gcs_link_remrssi is not None else '-'
    txbuf_str = f"{gcs_link_txbuf}" if gcs_link_txbuf is not None else '-'
    lines.append(f"GCS Link: Q={q_str} RSSI={rssi_str} RemRSSI={remrssi_str} TxBuf={txbuf_str}")
    if rover_failsafe_active and rover_failsafe_desc:
        if rover_failsafe_reason:
            lines.append(f"Failsafe: {rover_failsafe_reason}")
        else:
            lines.append(f"Failsafe: {rover_failsafe_desc}")
    else:
        lines.append("Failsafe: -")
    lines.append(f"Elapsed: {format_hms(elapsed)}")
    # Append last 10 status messages if available
    try:
        if last_status_texts:
            tail = list(reversed(last_status_texts[-10:]))
            lines.append("Messages:")
            lines.append("\n".join(tail))
    except Exception:
        pass
    return "\n".join(lines)

def extract_failsafe_reason(lower_txt: str) -> str | None:
    # Attempt to map common failsafe messages to concise reasons.
    patterns = [
        ('rc', 'RC Link Loss'),
        ('radio', 'RC Link Loss'),
        ('gcs', 'GCS Link Loss'),
        ('ekf', 'EKF'),
        ('gps', 'GPS'),
        ('battery', 'Battery'),
        ('terrain', 'Terrain'),
        ('crash', 'Crash'),
        ('leak', 'Leak'),
        ('parachute', 'Parachute'),
        ('watchdog', 'Watchdog'),
        ('compass', 'Compass'),
        ('imu', 'IMU'),
        ('internal error', 'Internal Error'),
        ('throttle', 'Throttle'),
        ('navigation', 'Navigation'),
        ('relay', 'Relay'),
        ('motor', 'Motor'),
    ]
    for key, reason in patterns:
        if key in lower_txt:
            return reason
    # Generic fallback: attempt to extract word after 'failsafe'
    try:
        idx = lower_txt.find('failsafe')
        if idx != -1:
            remainder = lower_txt[idx+8:].strip(': ,-')
            if remainder:
                # Take first token (up to space or punctuation)
                token = ''
                for ch in remainder:
                    if ch.isalnum() or ch in ('_', '-'):
                        token += ch
                    else:
                        break
                if token:
                    return token.capitalize()
    except Exception:
        pass
    return None

async def cmd_status(update: Update, context: ContextTypes.DEFAULT_TYPE):
    if update.effective_chat and getattr(update.effective_chat, 'id', None) is not None:
        set_chat_id(update.effective_chat.id)
    if update.message is None:
        return
    snapshot = rover_status_snapshot()
    await update.message.reply_text(snapshot)

# Backwards-compatible alias
async def cmd_rover(update: Update, context: ContextTypes.DEFAULT_TYPE):
    await cmd_status(update, context)

async def cmd_messages(update: Update, context: ContextTypes.DEFAULT_TYPE):
    if update.effective_chat and getattr(update.effective_chat, 'id', None) is not None:
        set_chat_id(update.effective_chat.id)
    if update.message is None:
        return
    try:
        if not last_status_texts:
            await update.message.reply_text("No status messages yet.")
            return
        # Show last 10 messages
        tail = last_status_texts[-10:]
        text = "Last messages (newest last):\n" + "\n".join(tail)
        # Telegram messages have length limits; ensure reasonable size
        if len(text) > 3500:
            text = text[-3500:]
        await update.message.reply_text(text)
    except Exception as e:
        await update.message.reply_text(f"Failed to render messages: {e}")

async def cmd_map(update: Update, context: ContextTypes.DEFAULT_TYPE):
    if update.effective_chat and getattr(update.effective_chat, 'id', None) is not None:
        set_chat_id(update.effective_chat.id)
    if update.message is None:
        return
    if rover_lat is None or rover_lon is None:
        await update.message.reply_text("Location unknown yet. Waiting for GPS…")
        return
    # Use HDOP to approximate accuracy in meters if available (roughly hdop*5m)
    acc = rover_hdop * 5.0 if rover_hdop is not None else None
    bot_send_location_http(rover_lat, rover_lon, silent=True, accuracy=acc)
    await update.message.reply_text(f"Sent map location: {rover_lat:.6f}, {rover_lon:.6f}")

async def cmd_reboot(update: Update, context: ContextTypes.DEFAULT_TYPE):
    if update.effective_chat and getattr(update.effective_chat, 'id', None) is not None:
        set_chat_id(update.effective_chat.id)
    if update.message is None:
        return
    # Ask for confirmation via inline keyboard
    try:
        kb = InlineKeyboardMarkup([
            [
                InlineKeyboardButton("Confirm Reboot", callback_data="REBOOT_CONFIRM"),
                InlineKeyboardButton("Cancel", callback_data="REBOOT_CANCEL"),
            ]
        ])
        await update.message.reply_text("Are you sure you want to reboot the flight controller?", reply_markup=kb)
    except Exception:
        await update.message.reply_text("Failed to present confirmation UI.")

async def cmd_skip(update: Update, context: ContextTypes.DEFAULT_TYPE):
    if update.effective_chat and getattr(update.effective_chat, 'id', None) is not None:
        set_chat_id(update.effective_chat.id)
    if update.message is None:
        return
    try:
        if rover_wp_seq is None:
            await update.message.reply_text("Current waypoint unknown.")
            return
        next_seq = int(rover_wp_seq) + 1
        tsys, tcomp = _target_ids()
        master_tx.mav.mission_set_current_send(tsys, tcomp, next_seq)
        await update.message.reply_text(f"Skipped to waypoint {next_seq}.")
        debug_print(f"[mavinject] MISSION_SET_CURRENT sent seq={next_seq}")
    except Exception as e:
        await update.message.reply_text(f"Failed to skip waypoint: {e}")
        debug_print(f"[mavinject] Skip failed: {e}")

def _target_ids():
    tsys = autopilot_sysid if autopilot_sysid is not None else 1
    tcomp = autopilot_compid if autopilot_compid is not None else 1
    return tsys, tcomp

def _send_mode(custom_mode: int) -> bool:
    try:
        tsys, tcomp = _target_ids()
        master_tx.mav.command_long_send(
            tsys,
            tcomp,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            1,  # param1: set using custom mode
            float(custom_mode),  # param2: custom mode number
            0, 0, 0, 0, 0
        )
        debug_print(f"[mavinject] Mode command sent custom_mode={custom_mode}")
        return True
    except Exception as e:
        debug_print(f"[mavinject] Mode command failed: {e}")
        return False

async def cmd_mode_manual(update: Update, context: ContextTypes.DEFAULT_TYPE):
    if update.effective_chat and getattr(update.effective_chat, 'id', None) is not None:
        set_chat_id(update.effective_chat.id)
    ok = _send_mode(0)
    if update.message:
        await update.message.reply_text("Set mode: MANUAL" if ok else "Failed to set MANUAL")

async def cmd_mode_auto(update: Update, context: ContextTypes.DEFAULT_TYPE):
    if update.effective_chat and getattr(update.effective_chat, 'id', None) is not None:
        set_chat_id(update.effective_chat.id)
    ok = _send_mode(10)
    if update.message:
        await update.message.reply_text("Set mode: AUTO" if ok else "Failed to set AUTO")

async def cmd_mode_hold(update: Update, context: ContextTypes.DEFAULT_TYPE):
    if update.effective_chat and getattr(update.effective_chat, 'id', None) is not None:
        set_chat_id(update.effective_chat.id)
    ok = _send_mode(4)
    if update.message:
        await update.message.reply_text("Set mode: HOLD" if ok else "Failed to set HOLD")

async def cmd_arm(update: Update, context: ContextTypes.DEFAULT_TYPE):
    if update.effective_chat and getattr(update.effective_chat, 'id', None) is not None:
        set_chat_id(update.effective_chat.id)
    try:
        tsys, tcomp = _target_ids()
        master_tx.mav.command_long_send(
            tsys,
            tcomp,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        if update.message:
            await update.message.reply_text("Arming requested.")
        return
    except Exception as e:
        if update.message:
            await update.message.reply_text(f"Failed to arm: {e}")

async def cmd_disarm(update: Update, context: ContextTypes.DEFAULT_TYPE):
    if update.effective_chat and getattr(update.effective_chat, 'id', None) is not None:
        set_chat_id(update.effective_chat.id)
    try:
        tsys, tcomp = _target_ids()
        master_tx.mav.command_long_send(
            tsys,
            tcomp,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        if update.message:
            await update.message.reply_text("Disarming requested.")
        return
    except Exception as e:
        if update.message:
            await update.message.reply_text(f"Failed to disarm: {e}")

async def cmd_menu(update: Update, context: ContextTypes.DEFAULT_TYPE):
    if update.effective_chat and getattr(update.effective_chat, 'id', None) is not None:
        set_chat_id(update.effective_chat.id)
    if update.message is None:
        return
    arm_label = "Disarm" if (rover_armed is True) else "Arm"
    arm_action = "DISARM" if (rover_armed is True) else "ARM"
    keyboard = [
        [
            InlineKeyboardButton("MANUAL", callback_data="MODE_MANUAL"),
            InlineKeyboardButton("AUTO", callback_data="MODE_AUTO"),
            InlineKeyboardButton("HOLD", callback_data="MODE_HOLD"),
        ],
        [
            InlineKeyboardButton(arm_label, callback_data=arm_action),
            InlineKeyboardButton("Status", callback_data="STATUS"),
            InlineKeyboardButton("Temp", callback_data="TEMP"),
            InlineKeyboardButton("Skip WP", callback_data="SKIP"),
        ],
        [
            InlineKeyboardButton("Save WP", callback_data="SAVE_WP"),
            InlineKeyboardButton("Reboot", callback_data="REBOOT_CONFIRM"),
            InlineKeyboardButton("Quit", callback_data="MENU_QUIT"),
        ],
    ]
    reply_markup = InlineKeyboardMarkup(keyboard)
    await update.message.reply_text("Control Menu", reply_markup=reply_markup)

def build_menu_keyboard() -> InlineKeyboardMarkup:
    arm_label = "Disarm" if (rover_armed is True) else "Arm"
    arm_action = "DISARM" if (rover_armed is True) else "ARM"
    keyboard = [
        [
            InlineKeyboardButton("MANUAL", callback_data="MODE_MANUAL"),
            InlineKeyboardButton("AUTO", callback_data="MODE_AUTO"),
            InlineKeyboardButton("HOLD", callback_data="MODE_HOLD"),
        ],
        [
            InlineKeyboardButton(arm_label, callback_data=arm_action),
            InlineKeyboardButton("Status", callback_data="STATUS"),
            InlineKeyboardButton("Temp", callback_data="TEMP"),
            InlineKeyboardButton("Skip WP", callback_data="SKIP"),
        ],
        [
            InlineKeyboardButton("Save WP", callback_data="SAVE_WP"),
            InlineKeyboardButton("Reboot", callback_data="REBOOT_CONFIRM"),
            InlineKeyboardButton("Quit", callback_data="MENU_QUIT"),
        ],
    ]
    return InlineKeyboardMarkup(keyboard)

async def respond_menu(query, text: str):
    try:
        await query.edit_message_text(text, reply_markup=build_menu_keyboard())
    except Exception:
        try:
            await query.edit_message_text(text)
        except Exception:
            pass

async def on_menu_callback(update: Update, context: ContextTypes.DEFAULT_TYPE):
    query = update.callback_query
    if query is None:
        return
    await query.answer()
    data = query.data or ""
    # Map actions
    if data == "MODE_MANUAL":
        ok = _send_mode(0)
        await respond_menu(query, "Set mode: MANUAL" if ok else "Failed to set MANUAL")
    elif data == "MODE_AUTO":
        ok = _send_mode(10)
        await respond_menu(query, "Set mode: AUTO" if ok else "Failed to set AUTO")
    elif data == "MODE_HOLD":
        ok = _send_mode(4)
        await respond_menu(query, "Set mode: HOLD" if ok else "Failed to set HOLD")
    elif data == "ARM":
        try:
            tsys, tcomp = _target_ids()
            master_tx.mav.command_long_send(
                tsys,
                tcomp,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1, 0, 0, 0, 0, 0, 0
            )
            await respond_menu(query, "Arming requested.")
        except Exception as e:
            await respond_menu(query, f"Failed to arm: {e}")
    elif data == "DISARM":
        try:
            tsys, tcomp = _target_ids()
            master_tx.mav.command_long_send(
                tsys,
                tcomp,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0, 0, 0, 0, 0, 0, 0
            )
            await respond_menu(query, "Disarming requested.")
        except Exception as e:
            await respond_menu(query, f"Failed to disarm: {e}")
    elif data == "STATUS":
        await respond_menu(query, rover_status_snapshot())
    elif data == "TEMP":
        try:
            t_c = read_temp_c()
            t_f = t_c * 9.0 / 5.0 + 32.0
            await respond_menu(query, f"Temperature: {t_c:.2f} °C / {t_f:.2f} °F")
        except Exception as e:
            await respond_menu(query, f"Failed to read temperature: {e}")
    elif data == "SKIP":
        try:
            if rover_wp_seq is None:
                await respond_menu(query, "Current waypoint unknown.")
            else:
                next_seq = int(rover_wp_seq) + 1
                tsys, tcomp = _target_ids()
                master_tx.mav.mission_set_current_send(tsys, tcomp, next_seq)
                await respond_menu(query, f"Skipped to waypoint {next_seq}.")
                debug_print(f"[mavinject] Menu SKIP: set current seq={next_seq}")
        except Exception as e:
            await respond_menu(query, f"Failed to skip waypoint: {e}")
    elif data == "SAVE_WP":
        try:
            global saved_wp_seq
            if rover_wp_seq is None:
                await respond_menu(query, "Current waypoint unknown.")
            else:
                saved_wp_seq = int(rover_wp_seq)
                await respond_menu(query, f"Saved current waypoint {saved_wp_seq}.")
                debug_print(f"[mavinject] Menu SAVE_WP: saved seq={saved_wp_seq}")
        except Exception as e:
            await respond_menu(query, f"Failed to save waypoint: {e}")
    elif data == "REBOOT_CONFIRM":
        try:
            target_sys = autopilot_sysid if autopilot_sysid is not None else 1
            target_comp = autopilot_compid if autopilot_compid is not None else 1
            master_tx.mav.command_long_send(
                target_sys,
                target_comp,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                0,
                1, 0, 0, 0, 0, 0, 0
            )
            await respond_menu(query, f"Reboot command sent (sys={target_sys} comp={target_comp}).")
            debug_print(f"[mavinject] Reboot command sent (sys={target_sys} comp={target_comp})")
        except Exception as e:
            await respond_menu(query, f"Failed to send reboot command: {e}")
            debug_print(f"[mavinject] Reboot command failed: {e}")
    elif data == "REBOOT_CANCEL":
        await respond_menu(query, "Reboot cancelled.")
    elif data == "MENU_QUIT":
        try:
            await query.edit_message_text("Menu closed.")
        except Exception:
            pass
    else:
        await respond_menu(query, "Unknown action")

def main():
    global app
    # Print configuration summary at startup
    try:
        token = TELEGRAM_BOT_TOKEN or ''
        token_masked = (token[:8] + '…') if token else 'unset'
        chat_clean = (TELEGRAM_CHAT_ID or 'unset').strip('\\\n\r ')
        cfg = {
            'MAVLINK_TX_ENDPOINT': MAVLINK_TX_ENDPOINT,
            'MAVLINK_RX_ENDPOINT': MAVLINK_RX_ENDPOINT,
            'MAVLINK_NAME': MAVLINK_NAME.strip(),
            'TELEGRAM_CHAT_ID': chat_clean,
            'TELEGRAM_BOT_TOKEN': token_masked,
            'EXTENDED_STATUS': '1' if EXTENDED_STATUS else '0',
            'DEBUG': '1' if DEBUG else '0',
            'HEARTBEAT_PERIODIC': os.getenv('HEARTBEAT_PERIODIC', '0'),
            'MODE_DEBOUNCE_SECONDS': os.getenv('MODE_DEBOUNCE_SECONDS', '2.0'),
            'AUTOPILOT_SYSID': os.getenv('AUTOPILOT_SYSID', 'auto'),
            'AUTOPILOT_COMPID': os.getenv('AUTOPILOT_COMPID', 'auto'),
            'POLLING_DISABLED': os.getenv('TELEGRAM_DISABLE_POLLING', '0'),
            'FAILSAFE_NOTIFY_LOUD': os.getenv('FAILSAFE_NOTIFY_LOUD', '1'),
        }
        print(f"[mavinject] Config: {cfg}")
    except Exception:
        pass
    if TEMP_SENDER_ENABLED:
        t_send = threading.Thread(target=mavlink_loop, daemon=True)
        t_send.start()
        debug_print("Temperature sender thread enabled.")
    else:
        debug_print("Temperature sender thread disabled by env.")

    disable_polling = os.getenv('TELEGRAM_DISABLE_POLLING', '0') == '1'
    if TELEGRAM_BOT_TOKEN and not disable_polling:
        try:
            app = Application.builder().token(TELEGRAM_BOT_TOKEN).build()
            app.add_handler(CommandHandler("start", cmd_start))
            app.add_handler(CommandHandler("temp", cmd_temp))
            app.add_handler(CommandHandler("ping", cmd_ping))
            app.add_handler(CommandHandler("status", cmd_status))
            app.add_handler(CommandHandler("rover", cmd_rover))
            app.add_handler(CommandHandler("map", cmd_map))
            app.add_handler(CommandHandler("reboot", cmd_reboot))
            app.add_handler(CommandHandler("skip", cmd_skip))
            app.add_handler(CommandHandler("messages", cmd_messages))
            app.add_handler(CommandHandler("menu", cmd_menu))
            app.add_handler(CommandHandler("mode_manual", cmd_mode_manual))
            app.add_handler(CommandHandler("mode_auto", cmd_mode_auto))
            app.add_handler(CommandHandler("mode_hold", cmd_mode_hold))
            app.add_handler(CommandHandler("arm", cmd_arm))
            app.add_handler(CommandHandler("disarm", cmd_disarm))
            app.add_handler(CallbackQueryHandler(on_menu_callback))
            if TELEGRAM_CHAT_ID:
                try:
                    set_chat_id(int((TELEGRAM_CHAT_ID or '').strip()))
                except ValueError:
                    pass
            # One-time startup notification (silent)
            if get_chat_id() is not None:
                bot_send_message_http(
                    f"[MAVInject started. TX={MAVLINK_TX_ENDPOINT} RX={MAVLINK_RX_ENDPOINT}]",
                    silent=True
                )
            t_mon = threading.Thread(target=mavlink_monitor_loop, daemon=True)
            t_mon.start()
            debug_print("Telegram bot polling mode active.")
            app.run_polling(close_loop=False)
        except Exception as e:
            print(f"[mavinject] Polling init failed, falling back to HTTP-only: {e}")
            app = None
            if TELEGRAM_CHAT_ID:
                try:
                    set_chat_id(int((TELEGRAM_CHAT_ID or '').strip()))
                except ValueError:
                    pass
            if get_chat_id() is not None:
                bot_send_message_http(
                    f"[MAVInject started (fallback). TX={MAVLINK_TX_ENDPOINT} RX={MAVLINK_RX_ENDPOINT}]",
                    silent=True
                )
            t_mon = threading.Thread(target=mavlink_monitor_loop, daemon=True)
            t_mon.start()
            # Keep main thread alive
            try:
                while True:
                    time.sleep(60)
            except KeyboardInterrupt:
                pass
    elif TELEGRAM_BOT_TOKEN and disable_polling:
        print("[mavinject] Polling disabled by env; using HTTP-only mode.")
        if TELEGRAM_CHAT_ID:
            try:
                set_chat_id(int((TELEGRAM_CHAT_ID or '').strip()))
            except ValueError:
                pass
        if get_chat_id() is not None:
            bot_send_message_http(
                f"[MAVInject started (no-poll). TX={MAVLINK_TX_ENDPOINT} RX={MAVLINK_RX_ENDPOINT}]",
                silent=True
            )
        t_mon = threading.Thread(target=mavlink_monitor_loop, daemon=True)
        t_mon.start()
        try:
            while True:
                time.sleep(60)
        except KeyboardInterrupt:
            pass
    else:
        debug_print("TELEGRAM_BOT_TOKEN not set; Telegram bot disabled.")
        try:
            while True:
                time.sleep(60)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
