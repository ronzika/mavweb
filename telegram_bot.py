from __future__ import annotations

import html
import io
import dataclasses
import json
import math
import os
import queue
import threading
import time
import urllib.parse
from typing import Any, Iterable

import requests
import streamlit as st
from dotenv import load_dotenv
from pymavlink import mavutil
from PIL import Image, ImageDraw, ImageFont

from shared_state import get_shared_state


load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), ".env"), override=True)


def _env_bool(name: str, default: bool = False) -> bool:
    value = (os.getenv(name, "") or "").strip().lower()
    if not value:
        return default
    return value in {"1", "true", "yes", "on"}


def _safe_int(value: Any) -> int | None:
    try:
        return int(str(value).strip())
    except Exception:
        return None


def _safe_float(value: Any) -> float | None:
    try:
        return float(str(value).strip())
    except Exception:
        return None


def _is_valid_mapbox_key(value: Any) -> bool:
    token = str(value or "").strip()
    return bool(token) and token.lower() not in {"none", "null", "undefined"} and token.startswith(("pk.", "sk."))


def _encode_polyline(points: Iterable[tuple[float, float]]) -> str:
    result: list[str] = []
    prev_lat = 0
    prev_lng = 0
    for lat, lng in points:
        lat_i = int(round(lat * 1e5))
        lng_i = int(round(lng * 1e5))
        for value in (lat_i - prev_lat, lng_i - prev_lng):
            value = ~(value << 1) if value < 0 else (value << 1)
            while value >= 0x20:
                result.append(chr((0x20 | (value & 0x1F)) + 63))
                value >>= 5
            result.append(chr(value + 63))
        prev_lat = lat_i
        prev_lng = lng_i
    return "".join(result)


def _downsample_points(points: list[list[float]], limit: int) -> list[list[float]]:
    if len(points) <= limit:
        return points
    step = max(1, math.ceil(len(points) / limit))
    return points[::step]


def _distance_m(lon1: float, lat1: float, lon2: float, lat2: float) -> float:
    r = 6378137.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2)**2
    return 2 * r * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def _mission_progress_snapshot(snapshot: dict[str, Any]) -> tuple[str, float | None]:
    total_wps = _safe_int(snapshot.get("mission_dl_total")) or 0
    wp_current = _safe_int(snapshot.get("wp_current")) or 0
    mode = str(snapshot.get("mode") or "").upper()

    if total_wps <= 0:
        pts = snapshot.get("mission_points") or []
        if isinstance(pts, list):
            total_wps = len(pts)

    if total_wps <= 1:
        return (f"Mission progress: {wp_current}", None)

    denom = max(total_wps - 1, 1)
    frac = max(0.0, min(1.0, wp_current / denom))
    shown_cur = min(max(wp_current + 1, 1), total_wps)
    text = f"Mission progress {shown_cur}/{total_wps} ({frac * 100.0:.0f}%)"
    if mode:
        text = f"{text}  Mode: {mode}"
    return text, frac


def _safe_point_list(points: Iterable[Any]) -> list[tuple[float, float]]:
    result: list[tuple[float, float]] = []
    for point in points:
        try:
            if isinstance(point, (list, tuple)) and len(point) >= 2:
                lon = float(point[0])
                lat = float(point[1])
                if math.isfinite(lon) and math.isfinite(lat):
                    result.append((lon, lat))
        except Exception:
            continue
    return result


def _draw_local_map(snapshot: dict[str, Any]) -> bytes | None:
    lat = _safe_float(snapshot.get("lat"))
    lon = _safe_float(snapshot.get("lon"))
    if lat is None or lon is None:
        return None

    history = _safe_point_list(snapshot.get("history") or [])
    mission_points = _safe_point_list(snapshot.get("mission_points") or [])
    wp_current = _safe_int(snapshot.get("wp_current")) or 0
    progress_text, progress_frac = _mission_progress_snapshot(snapshot)

    width = 900
    map_height = 600
    banner_height = 92
    canvas = Image.new("RGBA", (width, map_height + banner_height), (8, 12, 18, 255))
    draw = ImageDraw.Draw(canvas)

    # Determine the map bounds from available mission/history data so the view stays centered.
    all_points = [(lon, lat)] + history + mission_points
    lons = [p[0] for p in all_points]
    lats = [p[1] for p in all_points]
    min_lon, max_lon = min(lons), max(lons)
    min_lat, max_lat = min(lats), max(lats)

    center_lon = (min_lon + max_lon) / 2
    center_lat = (min_lat + max_lat) / 2
    
    base_lon_span = max(max_lon - min_lon, 0.00008) * 1.24
    base_lat_span = max(max_lat - min_lat, 0.00008) * 1.24
    
    lon_scale = math.cos(math.radians(center_lat))
    span_x = base_lon_span * lon_scale
    span_y = base_lat_span
    
    canvas_w = width - 80
    canvas_h = map_height - 80
    canvas_ratio = canvas_w / canvas_h
    
    if span_x / span_y > canvas_ratio:
        span_y = span_x / canvas_ratio
    else:
        span_x = span_y * canvas_ratio
        
    adj_lon_span = span_x / lon_scale
    adj_lat_span = span_y
    
    min_lon = center_lon - adj_lon_span / 2
    max_lon = center_lon + adj_lon_span / 2
    min_lat = center_lat - adj_lat_span / 2
    max_lat = center_lat + adj_lat_span / 2

    def to_xy(point_lon: float, point_lat: float) -> tuple[int, int]:
        x = int(round((point_lon - min_lon) / max(max_lon - min_lon, 1e-9) * (width - 80))) + 40
        y = int(round((max_lat - point_lat) / max(max_lat - min_lat, 1e-9) * (map_height - 80))) + 40
        return x, y

    # Map area background.
    draw.rectangle([0, 0, width, map_height], fill=(14, 20, 30, 255))
    grid_color = (34, 42, 54, 255)
    for x in range(40, width - 39, 80):
        draw.line([x, 30, x, map_height - 30], fill=grid_color, width=1)
    for y in range(40, map_height - 39, 80):
        draw.line([30, y, width - 30, y], fill=grid_color, width=1)

    # Mission path overlay.
    if len(mission_points) > 1:
        split_idx = max(1, wp_current)
        completed = mission_points[1:split_idx]
        pending = mission_points[max(1, split_idx - 1):]
        if len(completed) > 1:
            draw.line([to_xy(lon_p, lat_p) for lon_p, lat_p in completed], fill=(0, 230, 118, 255), width=5)
        if len(pending) > 1:
            draw.line([to_xy(lon_p, lat_p) for lon_p, lat_p in pending], fill=(245, 245, 245, 220), width=4)

    # Breadcrumb trail.
    if len(history) > 1:
        draw.line([to_xy(lon_p, lat_p) for lon_p, lat_p in history], fill=(255, 235, 59, 255), width=4)

    # Mission points and current rover marker.
    if mission_points:
        for idx, (lon_p, lat_p) in enumerate(mission_points[1:], start=1):
            x, y = to_xy(lon_p, lat_p)
            color = (0, 230, 118, 255) if idx < wp_current else (245, 245, 245, 255)
            draw.ellipse([x - 5, y - 5, x + 5, y + 5], fill=color, outline=(18, 24, 30, 255))

    rover_x, rover_y = to_xy(lon, lat)
    draw.ellipse([rover_x - 8, rover_y - 8, rover_x + 8, rover_y + 8], fill=(255, 23, 68, 255), outline=(255, 255, 255, 255), width=2)
    draw.line([rover_x, rover_y - 18, rover_x, rover_y + 18], fill=(255, 23, 68, 255), width=2)
    draw.line([rover_x - 18, rover_y, rover_x + 18, rover_y], fill=(255, 23, 68, 255), width=2)

    # Bottom banner with progress.
    banner_top = map_height
    draw.rectangle([0, banner_top, width, canvas.height], fill=(11, 18, 28, 255))
    draw.line([0, banner_top, width, banner_top], fill=(60, 74, 92, 255), width=2)

    font = ImageFont.load_default()
    draw.text((18, banner_top + 14), "Mission snapshot", fill=(235, 241, 248, 255), font=font)
    draw.text((18, banner_top + 32), progress_text, fill=(196, 207, 219, 255), font=font)

    bar_x = 18
    bar_y = banner_top + 58
    bar_w = width - 36
    bar_h = 16
    draw.rounded_rectangle([bar_x, bar_y, bar_x + bar_w, bar_y + bar_h], radius=8, fill=(39, 48, 59, 255))
    if progress_frac is not None:
        fill_w = max(10, int(bar_w * progress_frac))
    else:
        fill_w = max(10, bar_w // 8)
    draw.rounded_rectangle([bar_x, bar_y, bar_x + fill_w, bar_y + bar_h], radius=8, fill=(72, 211, 122, 255))

    out = io.BytesIO()
    canvas.save(out, format="PNG")
    return out.getvalue()


@dataclasses.dataclass
class TelegramConfig:
    token: str = ""
    default_chat_id: int | None = None
    disable_polling: bool = False
    mapbox_key: str = ""
    map_style: str = "satellite-streets-v12"


class TelegramBridge:
    def __init__(self, config: TelegramConfig):
        self.config = config
        self.state = get_shared_state()
        self.session = requests.Session()
        self.stop_event = threading.Event()
        self.chat_id_lock = threading.Lock()
        self.chat_id = config.default_chat_id
        self.last_snapshot: dict[str, Any] | None = None
        self.dashboard_message_id: int | None = None
        self.last_dashboard_text: str | None = None
        self.threads: list[threading.Thread] = []
        self.poll_offset = 0
        
        self.relay_labels: dict[int, str] = {
            i: (os.getenv(f"RELAY{i}", "") or "").strip()
            for i in range(1, 7)
            if (os.getenv(f"RELAY{i}", "") or "").strip()
        }

    def start(self) -> "TelegramBridge":
        if not self.config.token:
            return self

        if self.config.default_chat_id is not None:
            self._set_chat_id(self.config.default_chat_id)

        self._spawn(self._event_loop, "telegram-event-loop")
        self._spawn(self._snapshot_loop, "telegram-snapshot-loop")
        if not self.config.disable_polling:
            self._spawn(self._poll_updates_loop, "telegram-updates-loop")
        return self

    def stop(self) -> None:
        self.stop_event.set()

    def _spawn(self, target, name: str) -> None:
        thread = threading.Thread(target=target, name=name, daemon=True)
        thread.start()
        self.threads.append(thread)

    def _set_chat_id(self, chat_id: int | None) -> None:
        with self.chat_id_lock:
            self.chat_id = chat_id

    def _get_chat_id(self) -> int | None:
        with self.chat_id_lock:
            return self.chat_id

    def _api_url(self, method: str) -> str:
        return f"https://api.telegram.org/bot{self.config.token}/{method}"

    def _request(self, method: str, *, params: dict[str, Any] | None = None, data=None, files=None) -> dict[str, Any] | None:
        if not self.config.token:
            return None
        try:
            response = self.session.post(self._api_url(method), params=params, data=data, files=files, timeout=20)
            response.raise_for_status()
            payload = response.json()
            if not payload.get("ok", False):
                return None
            return payload
        except Exception:
            return None

    def send_message(self, text: str, *, silent: bool = True, reply_markup: dict[str, Any] | None = None, parse_mode: str | None = None) -> dict[str, Any] | None:
        chat_id = self._get_chat_id()
        if chat_id is None:
            return None
        payload: dict[str, Any] = {
            "chat_id": str(chat_id),
            "text": text,
            "disable_notification": "true" if silent else "false",
        }
        if reply_markup is not None:
            payload["reply_markup"] = json.dumps(reply_markup)
            
        if parse_mode is not None:
            payload["parse_mode"] = parse_mode
        elif "```" in text or "**" in text or "`" in text or "*" in text or "<pre>" in text or "<b>" in text:
            # Removed automatic markdown inference to rely on explicit typing where important.
            pass
             
        try:
            response = self.session.post(self._api_url("sendMessage"), data=payload, timeout=20)
            response.raise_for_status()
            return response.json()
        except requests.exceptions.HTTPError as exc:
            import logging
            logging.error(f"Telegram sendMessage failed: {exc.response.text}")
            return None
        except Exception as exc:
            import logging
            logging.error(f"Telegram sendMessage exception: {exc}")
            return None

    def edit_message_text(self, message_id: int, text: str, *, reply_markup: dict[str, Any] | None = None) -> None:
        chat_id = self._get_chat_id()
        if chat_id is None:
            return
        payload: dict[str, Any] = {
            "chat_id": str(chat_id),
            "message_id": str(message_id),
            "text": text,
            "parse_mode": "HTML",
        }
        if reply_markup is not None:
            payload["reply_markup"] = json.dumps(reply_markup)
        try:
            self.session.post(self._api_url("editMessageText"), data=payload, timeout=20).raise_for_status()
        except Exception:
            pass

    def send_location(self, lat: float, lon: float, *, silent: bool = True, accuracy: float | None = None) -> None:
        chat_id = self._get_chat_id()
        if chat_id is None:
            return
        payload: dict[str, Any] = {
            "chat_id": str(chat_id),
            "latitude": f"{lat:.7f}",
            "longitude": f"{lon:.7f}",
            "disable_notification": "true" if silent else "false",
        }
        if accuracy is not None:
            payload["horizontal_accuracy"] = f"{max(0.0, min(1500.0, float(accuracy))):.1f}"
        try:
            self.session.post(self._api_url("sendLocation"), data=payload, timeout=20).raise_for_status()
        except Exception:
            pass

    def send_photo(self, image_bytes: bytes, *, caption: str | None = None, silent: bool = True) -> None:
        chat_id = self._get_chat_id()
        if chat_id is None:
            return
        data = {
            "chat_id": str(chat_id),
            "disable_notification": "true" if silent else "false",
        }
        if caption:
            data["caption"] = caption
        files = {"photo": ("map.png", image_bytes, "image/png")}
        try:
            self.session.post(self._api_url("sendPhoto"), data=data, files=files, timeout=30).raise_for_status()
        except Exception:
            pass

    def _event_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                event = self.state.telegram_event_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            # Removed scrolling message updates to prevent chat clutter
            pass

    def _snapshot_loop(self) -> None:
        while not self.stop_event.is_set():
            snapshot = self.state.get()
            if self.last_snapshot is None:
                self.last_snapshot = snapshot
                if snapshot.get("link_active"):
                    self._cmd_dashboard()
                time.sleep(2.0)
                continue

            self._emit_snapshot_changes(self.last_snapshot, snapshot)
            self._update_dashboard(snapshot)
            self.last_snapshot = snapshot
            time.sleep(2.0)

    def _build_dashboard_text(self, snapshot: dict[str, Any]) -> str:
        mode = snapshot.get("mode") or "UNKNOWN"
        link_quality = snapshot.get("link_quality", 0)
        armed = "ARMED" if snapshot.get("armed") else "DISARMED"
        
        gps1_fix = snapshot.get("gps1_fix", 0)
        gps1_sats = snapshot.get("satellites_visible", 0)
        gps2_fix = snapshot.get("gps2_fix", 0)
        gps2_sats = snapshot.get("gps2_satellites_visible", 0)
        
        battery_v = snapshot.get("battery_v", 0.0)
        battery_pct = snapshot.get("battery_pct", 0)
        speed_ms = snapshot.get("speed_ms", 0.0)
        
        wp_current = snapshot.get("wp_current", 0)
        total_wps = snapshot.get("mission_dl_total", 0)
        mission_points = _safe_point_list(snapshot.get("mission_points") or [])
        if total_wps <= 0:
            if mission_points:
                total_wps = len(mission_points)
                
        dist_str = ""
        lat = _safe_float(snapshot.get("lat"))
        lon = _safe_float(snapshot.get("lon"))
        if lat is not None and lon is not None and wp_current > 0 and wp_current < len(mission_points):
            target_pt = mission_points[wp_current]
            dist = _distance_m(lon, lat, target_pt[0], target_pt[1])
            dist_ft = dist * 3.28084
            dist_str = f" ({dist_ft:.0f} ft)"

        heading = snapshot.get("heading_deg", 0)
        try:
            heading_display = f"{int(heading)}"
        except (ValueError, TypeError):
            heading_display = "0"
        
        failsafe_active = snapshot.get("failsafe_active", False)
        failsafe_desc = snapshot.get("failsafe_desc", "")
        failsafe_line = f"Failsafe | {html.escape(failsafe_desc)}\n" if failsafe_active else ""
        
        all_messages = list(snapshot.get("messages") or [])
        # Only display messages coming from the rover (exclude Telegram bot messages)
        fc_messages = [m for m in all_messages if "[Telegram]" not in m]
        messages = fc_messages[:5]
        msg_text = "\n".join(f"- {html.escape(m)}" for m in messages) if messages else "No recent messages"
        
        return f"""<b>Rover Dashboard</b>
<pre>
State    | Value
---------|-------------------
Mode     | {html.escape(mode)}
Link Q   | {link_quality}%
Status   | {armed}
Heading  | {heading_display} deg
{failsafe_line}GPS 1    | Fix: {gps1_fix}, Sats: {gps1_sats}
GPS 2    | Fix: {gps2_fix}, Sats: {gps2_sats}
Battery  | {battery_v:.1f}V ({battery_pct}%)
Waypoint | {wp_current} / {total_wps}{dist_str}
Speed    | {speed_ms:.2f} m/s
</pre>
<b>Recent Messages:</b>
{msg_text}"""

    def _build_dashboard_keyboard(self, snapshot: dict[str, Any]) -> dict[str, Any]:
        armed = snapshot.get("armed")
        arm_text = "Disarm" if armed else "Arm"
        arm_action = "DISARM" if armed else "ARM"
        
        keyboard = [
            [
                {"text": "Manual", "callback_data": "MODE_MANUAL"},
                {"text": "Auto", "callback_data": "MODE_AUTO"},
                {"text": "Hold", "callback_data": "MODE_HOLD"},
            ],
            [
                {"text": arm_text, "callback_data": f"TOGGLE_{arm_action}"},
                {"text": "Save WP", "callback_data": "SAVE_WP"},
                {"text": "Map", "callback_data": "REQUEST_MAP"},
            ],
            [
                {"text": "<< Prev WP", "callback_data": "PREV_WP"},
                {"text": "Next WP >>", "callback_data": "NEXT_WP"},
            ],
            [
                {"text": "<< Rate -10%", "callback_data": "SPEED_DEC"},
                {"text": "Rate +10% >>", "callback_data": "SPEED_INC"},
            ],
            [
                {"text": "Reset Mission", "callback_data": "RESET_MISSION"},
                {"text": "Clear Mission", "callback_data": "CLEAR_MISSION"},
            ]
        ]
        
        if self.relay_labels:
            relays = snapshot.get("relays", {})
            for i, label in self.relay_labels.items():
                is_on = relays.get(i, False)
                btn_label = f"{label} (Disable)" if is_on else f"{label} (Enable)"
                target_state = 0 if is_on else 1
                keyboard.append([
                    {"text": btn_label, "callback_data": f"RELAY_SET_{i}_{target_state}"}
                ])
                
        return {
            "inline_keyboard": keyboard
        }

    def _update_dashboard(self, snapshot: dict[str, Any]) -> None:
        if self.dashboard_message_id is None:
            return
            
        dash_text = self._build_dashboard_text(snapshot)
        reply_markup = self._build_dashboard_keyboard(snapshot)
        current_keyboard_json = json.dumps(reply_markup, sort_keys=True)
        last_keyboard = getattr(self, "last_dashboard_keyboard", None)
        
        if dash_text != self.last_dashboard_text or current_keyboard_json != last_keyboard:
            self.edit_message_text(self.dashboard_message_id, dash_text, reply_markup=reply_markup)
            self.last_dashboard_text = dash_text
            self.last_dashboard_keyboard = current_keyboard_json

    def _emit_snapshot_changes(self, previous: dict[str, Any], current: dict[str, Any]) -> None:
        if not previous.get("link_active") and current.get("link_active"):
            # Push a new dashboard when the rover connects
            self._cmd_dashboard()

    def _poll_updates_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                params = {"timeout": 20, "offset": self.poll_offset}
                response = self.session.get(self._api_url("getUpdates"), params=params, timeout=25)
                response.raise_for_status()
                payload = response.json()
            except Exception:
                time.sleep(2.0)
                continue

            if not payload.get("ok", False):
                time.sleep(2.0)
                continue

            for update in payload.get("result", []):
                try:
                    self.poll_offset = max(self.poll_offset, int(update.get("update_id", 0)) + 1)
                    self._handle_update(update)
                except Exception:
                    continue

    def _handle_update(self, update: dict[str, Any]) -> None:
        message = update.get("message") or update.get("edited_message")
        if message:
            chat = message.get("chat") or {}
            chat_id = _safe_int(chat.get("id"))
            if chat_id is not None:
                self._set_chat_id(chat_id)
            text = str(message.get("text") or "").strip()
            if text.startswith("/"):
                self._handle_command(text)
            return

        callback = update.get("callback_query")
        if callback:
            message = callback.get("message") or {}
            chat = message.get("chat") or {}
            chat_id = _safe_int(chat.get("id"))
            if chat_id is not None:
                self._set_chat_id(chat_id)
            data = str(callback.get("data") or "").strip()
            if data:
                self._handle_callback(callback, data)

    def _handle_callback(self, callback: dict[str, Any], data: str) -> None:
        callback_id = callback.get("id")
        if callback_id:
            self._request("answerCallbackQuery", data={"callback_query_id": callback_id})
            
        if data == "REBOOT_CONFIRM":
            self._send_reboot()
            self.send_message("Reboot command sent.", silent=False)
        elif data == "REBOOT_CANCEL":
            self.send_message("Reboot cancelled.", silent=True)
        elif data == "RESET_CONFIRM":
            self._send_reset_mission()
        elif data == "RESET_CANCEL":
            self.send_message("Mission reset cancelled.", silent=True)
        elif data == "CLEAR_MISSION_CONFIRM":
            self._send_clear_mission()
        elif data == "CLEAR_MISSION_CANCEL":
            self.send_message("Mission clear cancelled.", silent=True)
        elif data == "MODE_MANUAL":
            self._send_mode(0, "MANUAL")
        elif data == "MODE_AUTO":
            self._send_mode(10, "AUTO")
        elif data == "MODE_HOLD":
            self._send_mode(4, "HOLD")
        elif data == "TOGGLE_ARM":
            self._cmd_arm()
        elif data == "TOGGLE_DISARM":
            self._cmd_disarm()
        elif data == "PREV_WP":
            self._cmd_change_wp(-1)
        elif data == "NEXT_WP":
            self._cmd_change_wp(1)
        elif data == "SPEED_DEC":
            self._cmd_adjust_speed(0.9)
        elif data == "SPEED_INC":
            self._cmd_adjust_speed(1.1)
        elif data == "RESET_MISSION":
            # We skip confirmation for inline dashboard for fluidity, or we can just call _cmd_reset('confirm')
            self._send_reset_mission()
        elif data == "CLEAR_MISSION":
            self._cmd_clear_mission("confirm")
        elif data == "SAVE_WP":
            self._cmd_save_wp()
        elif data == "REQUEST_MAP":
            self._cmd_map()
        elif data.startswith("RELAY_SET_"):
            try:
                parts = data.split("_")
                self._send_relay(int(parts[2]), int(parts[3]))
            except Exception:
                pass

    def _handle_command(self, text: str) -> None:
        parts = text.split(maxsplit=1)
        command = parts[0].split("@", 1)[0].lower()
        arg = parts[1].strip() if len(parts) > 1 else ""

        handlers = {
            "/start": self._cmd_start,
            "/ping": self._cmd_ping,
            "/status": self._cmd_status,
            "/rover": self._cmd_status,
            "/dashboard": self._cmd_dashboard,
            "/messages": self._cmd_messages,
            "/map": self._cmd_map,
            "/temp": self._cmd_temp,
            "/mode_manual": lambda: self._send_mode(0, "MANUAL"),
            "/mode_auto": lambda: self._send_mode(10, "AUTO"),
            "/mode_hold": lambda: self._send_mode(4, "HOLD"),
            "/arm": self._cmd_arm,
            "/disarm": self._cmd_disarm,
            "/skip": self._cmd_skip,
            "/reset": lambda: self._cmd_reset(arg),
            "/reboot": lambda: self._cmd_reboot(arg),
            "/menu": self._cmd_menu,
            "/save_wp": self._cmd_save_wp,
        }

        handler = handlers.get(command)
        if handler is not None:
            handler()
        else:
            self.send_message(f"Unknown command: {command}", silent=True)

    def _cmd_start(self) -> None:
        self.send_message("Bot is ready. Use /status, /map, or /menu.", silent=True)

    def _cmd_ping(self) -> None:
        self.send_message("Ping received. Sending test notification.", silent=True)
        self.send_message("Test notification from mavweb.", silent=True)

    def _cmd_dashboard(self) -> None:
        snapshot = self.state.get()
        dash_text = self._build_dashboard_text(snapshot)
        reply_markup = self._build_dashboard_keyboard(snapshot)
        
        result = self.send_message(dash_text, silent=True, reply_markup=reply_markup, parse_mode="HTML")
        if result and result.get("ok"):
            self.dashboard_message_id = result.get("result", {}).get("message_id")
            self.last_dashboard_text = dash_text
            self.last_dashboard_keyboard = json.dumps(reply_markup, sort_keys=True)

    def _cmd_status(self) -> None:
        self.send_message(self._status_text(self.state.get()), silent=True)

    def _cmd_messages(self) -> None:
        messages = list(self.state.get().get("messages") or [])
        if not messages:
            self.send_message("No status messages yet.", silent=True)
            return
        text = "Last messages (newest first):\n" + "\n".join(messages[:10])
        self.send_message(text[:3500], silent=True)

    def _cmd_map(self) -> None:
        snapshot = self.state.get()
        lat = snapshot.get("lat")
        lon = snapshot.get("lon")
        if lat is None or lon is None:
            self.send_message("Location unknown yet. Waiting for GPS…", silent=True)
            return

        image = self._build_map_image(snapshot)
        if image is None:
            self.send_message("Failed to render mission snapshot.", silent=True)
            return

        progress_text, _ = _mission_progress_snapshot(snapshot)
        self.send_photo(image, caption=f"{float(lat):.6f}, {float(lon):.6f}\n{progress_text}", silent=True)

    def _cmd_temp(self) -> None:
        snapshot = self.state.get()
        value = snapshot.get("mqtt_var1")
        temp = _safe_float(value)
        if temp is None:
            self.send_message("Temperature unavailable from mavweb state.", silent=True)
            return
        fahrenheit = temp * 9.0 / 5.0 + 32.0
        self.send_message(f"Temperature: {temp:.2f} °C / {fahrenheit:.2f} °F", silent=True)

    def _cmd_arm(self) -> None:
        if self._send_arm_disarm(True):
            self.send_message("Arm command sent.", silent=False)
        else:
            self.send_message("Failed to send arm command.", silent=True)

    def _cmd_disarm(self) -> None:
        if self._send_arm_disarm(False):
            self.send_message("Disarm command sent.", silent=False)
        else:
            self.send_message("Failed to send disarm command.", silent=True)

    def _cmd_change_wp(self, delta: int) -> None:
        snapshot = self.state.get()
        wp_current = snapshot.get("wp_current")
        conn = self.state.get_connection()
        if conn is None:
            self.send_message("No MAVLink connection available.", silent=True)
            return
        if wp_current is None:
            self.send_message("Current waypoint unknown.", silent=True)
            return
            
        total_wps = snapshot.get("mission_dl_total", 0)
        if total_wps <= 0:
            pts = snapshot.get("mission_points") or []
            if isinstance(pts, list):
                total_wps = len(pts)

        next_seq = int(wp_current) + delta
        if next_seq < 0:
            next_seq = 0
        if total_wps > 0 and next_seq >= total_wps: # Waypoints are 0-indexed effectively or total_wps is count. The prompt says advance by 1 unless greater than total
            next_seq = total_wps - 1 if total_wps > 0 else 0
            # Wait, the prompt says "advances the current waypoint by 1 unless the value is greater than the total number of waypoints"
            # Actually, standard Ardupilot is 0-indexed (WP 0 is home). Total WPs: e.g. 5 means 0, 1, 2, 3, 4. So max is total_wps - 1. But let's just bound it by total_wps to be safe, sometimes it can accept up to total_wps.
        
        # Let's strictly bound by total WPs. The prompt says "greater than the total number of waypoints in the current mission"
        if total_wps > 0 and next_seq > total_wps:
            next_seq = total_wps

        try:
            with self.state.acquire_mav_lock():
                conn.mav.mission_set_current_send(conn.target_system, conn.target_component, next_seq)
        except Exception as exc:
            self.send_message(f"Failed to change waypoint: {exc}", silent=True)

    def _cmd_adjust_speed(self, factor: float) -> None:
        snapshot = self.state.get()
        current_speed = snapshot.get("speed_ms", 0.0)
        conn = self.state.get_connection()
        if conn is None:
            self.send_message("No MAVLink connection available.", silent=True)
            return

        # Ensure we have a minimum base to multiply against if it's currently stopped
        base_speed = max(float(current_speed), 0.5)
        new_speed = round(base_speed * factor, 2)
        
        try:
            with self.state.acquire_mav_lock():
                conn.mav.command_long_send(
                    conn.target_system,
                    conn.target_component,
                    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                    0,     # Confirmation
                    1,     # Speed type (1 = Ground Speed)
                    new_speed, # Speed in m/s
                    -1,    # Throttle (-1 indicates no change)
                    0,     # Reserved
                    0, 0, 0
                )
            self.state.append_message(f"[Telegram] Requested speed change to {new_speed} m/s")
        except Exception as exc:
            self.send_message(f"Failed to change speed: {exc}", silent=True)

    def _cmd_reset(self, arg: str = "") -> None:
        if arg.lower() in {"confirm", "yes", "ok"}:
            self._send_reset_mission()
            return

        reply_markup = {
            "inline_keyboard": [
                [
                    {"text": "Confirm Reset Mission", "callback_data": "RESET_CONFIRM"},
                    {"text": "Cancel", "callback_data": "RESET_CANCEL"},
                ]
            ]
        }
        self.send_message("Are you sure you want to reset the mission to WP 0?", silent=True, reply_markup=reply_markup)

    def _send_reset_mission(self) -> None:
        conn = self.state.get_connection()
        if conn is None:
            self.send_message("No MAVLink connection available.", silent=True)
            return

        try:
            with self.state.acquire_mav_lock():
                conn.mav.mission_set_current_send(
                    conn.target_system,
                    conn.target_component,
                    0,
                )
            self.state.append_message("[Telegram] Reset mission to WP 0")
            self.send_message("Mission reset to WP 0.", silent=False)
        except Exception as exc:
            error_msg = f"Failed to reset mission: {exc}"
            self.state.append_message(f"[Telegram] {error_msg}")
            self.send_message(error_msg, silent=True)

    def _cmd_clear_mission(self, arg: str = "") -> None:
        if arg.lower() in {"confirm", "yes", "ok"}:
            self._send_clear_mission()
            return

        reply_markup = {
            "inline_keyboard": [
                [
                    {"text": "Confirm Clear Mission", "callback_data": "CLEAR_MISSION_CONFIRM"},
                    {"text": "Cancel", "callback_data": "CLEAR_MISSION_CANCEL"},
                ]
            ]
        }
        self.send_message("Are you sure you want to completely clear the mission from the vehicle?", silent=True, reply_markup=reply_markup)

    def _send_clear_mission(self) -> None:
        conn = self.state.get_connection()
        if conn is None:
            self.send_message("No MAVLink connection available.", silent=True)
            return

        try:
            with self.state.acquire_mav_lock():
                conn.mav.mission_clear_all_send(
                    conn.target_system,
                    conn.target_component,
                )
            self.state.append_message("[Telegram] Cleared all waypoints from flight controller")
            self.send_message("Mission cleared.", silent=False)
        except Exception as exc:
            error_msg = f"Failed to clear mission: {exc}"
            self.state.append_message(f"[Telegram] {error_msg}")
            self.send_message(error_msg, silent=True)

    def _cmd_reboot(self, arg: str = "") -> None:
        if arg.lower() in {"confirm", "yes", "ok"}:
            self._send_reboot()
            self.send_message("Reboot command sent.", silent=False)
            return

        reply_markup = {
            "inline_keyboard": [
                [
                    {"text": "Confirm Reboot", "callback_data": "REBOOT_CONFIRM"},
                    {"text": "Cancel", "callback_data": "REBOOT_CANCEL"},
                ]
            ]
        }
        self.send_message("Are you sure you want to reboot the flight controller?", silent=True, reply_markup=reply_markup)

    def _cmd_menu(self) -> None:
        self.send_message(
            "Commands: /dashboard /status /map /messages /mode_manual /mode_auto /mode_hold /arm /disarm /skip /reset /reboot",
            silent=True,
        )

    def _cmd_save_wp(self) -> None:
        snapshot = self.state.get()
        wp_current = snapshot.get("wp_current")
        if wp_current is None:
            self.send_message("Current waypoint unknown.", silent=True)
            return
        self.state.save_wp_queue.put({"wp_current": int(wp_current), "ts": time.time()})
        self.send_message(f"Save waypoint requested for WP {int(wp_current)}.", silent=True)

    def _send_relay(self, relay_num: int, state: int) -> None:
        conn = self.state.get_connection()
        if conn is None:
            self.send_message("No MAVLink connection available.", silent=True)
            return
        
        try:
            ap_relay_index = relay_num - 1
            with self.state.acquire_mav_lock():
                conn.mav.command_long_send(
                    conn.target_system,
                    conn.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
                    0,
                    float(ap_relay_index),
                    float(state),
                    0, 0, 0, 0, 0,
                )
            
            # Keep synchronous state mapping up to date
            current_relays = self.state.get().get("relays", {})
            current_relays[relay_num] = bool(state)
            self.state.update({"relays": current_relays})
            
            # The dashboard and statustext handlers will update UI naturally, but we inject a local log.
            label = self.relay_labels.get(relay_num, f"Relay {relay_num}")
            state_str = "ON" if state else "OFF"
            self.state.append_message(f"[Telegram] Set {label} to {state_str}")
        except Exception as exc:
            self.send_message(f"Failed to set relay {relay_num}: {exc}", silent=True)

    def _send_mode(self, custom_mode: int, mode_name: str) -> None:
        conn = self.state.get_connection()
        if conn is None:
            self.send_message("No MAVLink connection available.", silent=True)
            return
        try:
            with self.state.acquire_mav_lock():
                conn.mav.command_long_send(
                    conn.target_system,
                    conn.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    0,
                    1,
                    float(custom_mode),
                    0,
                    0,
                    0,
                    0,
                    0,
                )
        except Exception as exc:
            self.send_message(f"Failed to send mode command: {exc}", silent=True)

    def _send_arm_disarm(self, arm: bool) -> bool:
        conn = self.state.get_connection()
        if conn is None:
            return False
        try:
            with self.state.acquire_mav_lock():
                conn.mav.command_long_send(
                    conn.target_system,
                    conn.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    1.0 if arm else 0.0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            return True
        except Exception:
            return False

    def _send_reboot(self) -> None:
        conn = self.state.get_connection()
        if conn is None:
            return
        try:
            with self.state.acquire_mav_lock():
                conn.mav.command_long_send(
                    conn.target_system,
                    conn.target_component,
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
        except Exception:
            pass

    def _status_text(self, snapshot: dict[str, Any]) -> str:
        messages = snapshot.get("messages") or []
        recent = "\n".join(messages[:5]) if messages else "(none)"
        return (
            f"Mode: {snapshot.get('mode')}\n"
            f"Armed: {'ARMED' if snapshot.get('armed') else 'DISARMED'}\n"
            f"GPS: {snapshot.get('gps1_fix')}\n"
            f"Lat/Lon: {snapshot.get('lat')}, {snapshot.get('lon')}\n"
            f"Speed: {snapshot.get('speed_ms')} m/s\n"
            f"Battery: {snapshot.get('battery_v')} V\n"
            f"WP: {snapshot.get('wp_current')}\n"
            f"Messages:\n{recent}"
        )

    def _build_map_image(self, snapshot: dict[str, Any]) -> bytes | None:
        lat = _safe_float(snapshot.get("lat"))
        lon = _safe_float(snapshot.get("lon"))
        if lat is None or lon is None:
            return None

        local_image = _draw_local_map(snapshot)
        if local_image is None:
            return None

        # Prefer Mapbox when available, but never require it.
        if not _is_valid_mapbox_key(self.config.mapbox_key):
            return local_image

        history = list(snapshot.get("history") or [])
        mission_points = list(snapshot.get("mission_points") or [])
        wp_current = int(snapshot.get("wp_current") or 0)

        history = _downsample_points(history, 120)
        mission_points = _downsample_points(mission_points, 120)
        all_points = [(lon, lat)] + history + mission_points

        overlays: list[str] = []
        if len(history) > 1:
            history_polyline = _encode_polyline([(float(p[1]), float(p[0])) for p in history if len(p) >= 2])
            overlays.append(f"path-3+ffeb3b-0.8({history_polyline})")

        if len(mission_points) > 1:
            completed = mission_points[1:max(1, wp_current)]
            pending = mission_points[max(1, wp_current - 1):]
            if len(completed) > 1:
                overlays.append(
                    f"path-3+00ff00-0.8({_encode_polyline([(float(p[1]), float(p[0])) for p in completed if len(p) >= 2])})"
                )
            if len(pending) > 1:
                overlays.append(
                    f"path-3+ffffff-0.8({_encode_polyline([(float(p[1]), float(p[0])) for p in pending if len(p) >= 2])})"
                )

        overlays.append(f"pin-s+ff1744({lon:.7f},{lat:.7f})")
        overlay_part = ",".join(overlays)
        style = self.config.map_style or "satellite-streets-v12"
        
        # Calculate optimal Mapbox bounds/scaling based on available points
        if len(all_points) > 1:
            url = (
                f"https://api.mapbox.com/styles/v1/mapbox/{style}/static/"
                f"{overlay_part}/auto/900x600@2x?padding=40,40,40,40&access_token={urllib.parse.quote(self.config.mapbox_key)}"
            )
        else:
            url = (
                f"https://api.mapbox.com/styles/v1/mapbox/{style}/static/"
                f"{overlay_part}/{lon:.7f},{lat:.7f},19/900x600@2x?access_token={urllib.parse.quote(self.config.mapbox_key)}"
            )

        if len(url) > 7500:
            history = _downsample_points(history, 60)
            mission_points = _downsample_points(mission_points, 60)
            overlays = []
            if len(history) > 1:
                history_polyline = _encode_polyline([(float(p[1]), float(p[0])) for p in history if len(p) >= 2])
                overlays.append(f"path-3+ffeb3b-0.8({history_polyline})")
            if len(mission_points) > 1:
                completed = mission_points[1:max(1, wp_current)]
                pending = mission_points[max(1, wp_current - 1):]
                if len(completed) > 1:
                    overlays.append(
                        f"path-3+00ff00-0.8({_encode_polyline([(float(p[1]), float(p[0])) for p in completed if len(p) >= 2])})"
                    )
                if len(pending) > 1:
                    overlays.append(
                        f"path-3+ffffff-0.8({_encode_polyline([(float(p[1]), float(p[0])) for p in pending if len(p) >= 2])})"
                    )
            overlays.append(f"pin-s+ff1744({lon:.7f},{lat:.7f})")
            overlay_part = ",".join(overlays)
            if len(all_points) > 1:
                url = (
                    f"https://api.mapbox.com/styles/v1/mapbox/{style}/static/"
                    f"{overlay_part}/auto/900x600@2x?padding=40,40,40,40&access_token={urllib.parse.quote(self.config.mapbox_key)}"
                )
            else:
                url = (
                    f"https://api.mapbox.com/styles/v1/mapbox/{style}/static/"
                    f"{overlay_part}/{lon:.7f},{lat:.7f},19/900x600@2x?access_token={urllib.parse.quote(self.config.mapbox_key)}"
                )

        try:
            response = self.session.get(url, timeout=30)
            response.raise_for_status()
            base_image = Image.open(io.BytesIO(response.content)).convert("RGBA")
            progress_text, progress_frac = _mission_progress_snapshot(snapshot)

            banner_height = 92
            composed = Image.new("RGBA", (base_image.width, base_image.height + banner_height), (8, 12, 18, 255))
            composed.paste(base_image, (0, 0))

            draw = ImageDraw.Draw(composed)
            banner_top = base_image.height
            draw.rectangle(
                [0, banner_top, composed.width, composed.height],
                fill=(11, 18, 28, 255),
            )
            draw.line([0, banner_top, composed.width, banner_top], fill=(60, 74, 92, 255), width=2)

            font = ImageFont.load_default()
            text_x = 18
            text_y = banner_top + 14
            draw.text((text_x, text_y), "Mission snapshot", fill=(235, 241, 248, 255), font=font)
            draw.text((text_x, text_y + 18), progress_text, fill=(196, 207, 219, 255), font=font)

            bar_x = 18
            bar_y = banner_top + 58
            bar_w = composed.width - 36
            bar_h = 16
            draw.rounded_rectangle([bar_x, bar_y, bar_x + bar_w, bar_y + bar_h], radius=8, fill=(39, 48, 59, 255))
            if progress_frac is not None:
                fill_w = max(10, int(bar_w * progress_frac))
                draw.rounded_rectangle([bar_x, bar_y, bar_x + fill_w, bar_y + bar_h], radius=8, fill=(72, 211, 122, 255))
            else:
                draw.rounded_rectangle([bar_x, bar_y, bar_x + max(10, bar_w // 8), bar_y + bar_h], radius=8, fill=(72, 211, 122, 255))

            out = io.BytesIO()
            composed.save(out, format="PNG")
            return out.getvalue()
        except Exception:
            return local_image


def _build_config() -> TelegramConfig:
    return TelegramConfig(
        token=(os.getenv("TELEGRAM_BOT_TOKEN", "") or "").strip(),
        default_chat_id=_safe_int(os.getenv("TELEGRAM_CHAT_ID", "")),
        disable_polling=_env_bool("TELEGRAM_DISABLE_POLLING", False),
        mapbox_key=(os.getenv("MAPBOX_API_KEY", "") or "").strip(),
        map_style=(os.getenv("TELEGRAM_MAP_STYLE", "") or "satellite-streets-v12").strip() or "satellite-streets-v12",
    )


@st.cache_resource
def start_telegram_bridge() -> TelegramBridge:
    bridge = TelegramBridge(_build_config())
    return bridge.start()