from typing import Any, Optional, Tuple
from pymavlink import mavutil
import time
import re

import sys
from config import Config
from state import AppContext
from utils import gps_fix_label, rover_mode_from_custom_mode, extract_failsafe_reason
from typing import List, Tuple

MAV_MODE_FLAG_SAFETY_ARMED = getattr(mavutil.mavlink, 'MAV_MODE_FLAG_SAFETY_ARMED', 128)

def open_connections(cfg: Config) -> Tuple[Any, Optional[Any]]:
    master_tx: Any = mavutil.mavlink_connection(
        cfg.MAVLINK_TX_ENDPOINT,
        dialect="ardupilotmega",
        source_system=255,
        source_component=getattr(mavutil.mavlink, 'MAV_COMP_ID_GCS', 255),
        force_mavlink_version=2
    )
    # Use the same socket for RX to ensure we receive replies on the originating port
    # master_rx = master_tx
    # But if RX endpoint is explicitly different, we should use it.
    # For now, let's restore the behavior of opening a separate RX connection if configured.
    try:
        master_rx = mavutil.mavlink_connection(
            cfg.MAVLINK_RX_ENDPOINT,
            dialect="ardupilotmega",
            source_system=255,
            source_component=1,
            force_mavlink_version=2
        )
    except Exception as e:
        print(f"Failed to open RX endpoint '{cfg.MAVLINK_RX_ENDPOINT}': {e}")
        master_rx = None
    
    return master_tx, master_rx

def send_heartbeat(ctx: AppContext):
    try:
        if ctx.master_tx is None or not hasattr(ctx.master_tx, 'mav'):
            return
        ctx.master_tx.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            0,
            0,
            0,
            0
        )
    except Exception:
        pass

def request_message_interval(ctx: AppContext, msg_id: int, hz: float):
    try:
        if ctx.master_tx is None or not hasattr(ctx.master_tx, 'mav'):
            return
        interval_us = int(1_000_000 / max(0.1, hz))
        tsys, tcomp = target_ids(ctx)
        ctx.master_tx.mav.command_long_send(
            tsys,
            tcomp,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            float(interval_us),
            0, 0, 0, 0, 0
        )
    except Exception:
        pass

def init_link(ctx: AppContext):
    # Kick the link: send a heartbeat and request key message intervals
    send_heartbeat(ctx)
    for mid, hz in [
        (mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1.0),
        (mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, 2.0),
        (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1.0),
        (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 2.0),
        (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2.0),
    ]:
        request_message_interval(ctx, mid, hz)

def download_mission(ctx: AppContext, mavconn: Any, timeout: float = 5.0, force: bool = False) -> List[Tuple[float, float]]:
    """
    Fetch mission items from the autopilot and return a list of (lat, lon) tuples.
    Uses a simple cache on ctx.rover.mission_items to avoid re-downloading when unchanged.
    """
    # Reuse cached points if available and not forcing refresh
    try:
        if not force and getattr(ctx.rover, 'mission_items', None):
            return list(ctx.rover.mission_items)
    except Exception:
        pass
    points: list[tuple[float, float]] = []
    try:
        sysid = ctx.targets.autopilot_sysid or getattr(ctx.master_tx, 'target_system', None) or 1
        compid = ctx.targets.autopilot_compid or getattr(ctx.master_tx, 'target_component', None) or 1

        # Request mission list
        # Request mission list and wait for count
        mavconn.mav.mission_request_list_send(sysid, compid)
        end_ts = time.time() + timeout
        count = None
        while time.time() < end_ts:
            msg = mavconn.recv_match(type=['MISSION_COUNT'], blocking=False)
            if msg:
                count = int(getattr(msg, 'count', 0))
                ctx.rover.wp_total = count
                break
            time.sleep(0.05)
        if not count or count <= 0:
            return points

        # Request each item (prefer INT), retrying if needed
        for seq in range(count):
            item = None
            # Try INT first, then non-INT
            for attempt in range(2):
                try:
                    if attempt == 0:
                        mavconn.mav.mission_request_int_send(sysid, compid, seq)
                    else:
                        mavconn.mav.mission_request_send(sysid, compid, seq)
                except Exception:
                    pass
                end_item = time.time() + 1.5
                while time.time() < end_item:
                    msg = mavconn.recv_match(type=['MISSION_ITEM_INT', 'MISSION_ITEM'], blocking=False)
                    if msg:
                        item = msg
                        break
                    time.sleep(0.02)
                if item:
                    break
            if not item:
                # Could not fetch this item; skip gracefully
                continue
            try:
                if item.get_type() == 'MISSION_ITEM_INT':
                    lat = float(getattr(item, 'x', 0)) / 1e7
                    lon = float(getattr(item, 'y', 0)) / 1e7
                else:
                    lat = float(getattr(item, 'x', 0.0))
                    lon = float(getattr(item, 'y', 0.0))
            except Exception:
                lat, lon = 0.0, 0.0
            # Ignore invalid/zero coordinates
            if abs(lat) < 1e-9 and abs(lon) < 1e-9:
                continue
            points.append((lat, lon))
        # Update cache and a coarse mission hash
        try:
            ctx.rover.mission_items = list(points)
            if points:
                first = points[0]
                last = points[-1]
                ctx.deb.mission_hash = f"{count}:{first[0]:.6f},{first[1]:.6f}:{last[0]:.6f},{last[1]:.6f}"
        except Exception:
            pass
        return points
    except Exception as e:
        if ctx.cfg.DEBUG:
            print(f"[mission] download failed: {e}")
        return points

def target_ids(ctx: AppContext) -> Tuple[int, int]:
    tsys = ctx.targets.autopilot_sysid if ctx.targets.autopilot_sysid is not None else (
        getattr(ctx.master_tx, 'target_system', None) or 1
    )
    tcomp = ctx.targets.autopilot_compid if ctx.targets.autopilot_compid is not None else (
        getattr(ctx.master_tx, 'target_component', None) or 1
    )
    try:
        if getattr(ctx.cfg, 'DEBUG', 0):
            print(f"[mavlink] target_ids -> sysid={tsys} compid={tcomp}")
    except Exception:
        pass
    return tsys, tcomp

def send_set_mode(ctx: AppContext, custom_mode: int) -> bool:
    try:
        if ctx.master_tx is None:
            return False
        tsys, tcomp = target_ids(ctx)
        try:
            if getattr(ctx.cfg, 'DEBUG', 0):
                print(f"[mavlink] send_set_mode custom_mode={int(custom_mode)} targets sys={tsys} comp={tcomp}")
        except Exception:
            pass
        # Prefer SET_MODE for ArduPilot when available
        try:
            ctx.master_tx.mav.set_mode_send(
                tsys,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                int(custom_mode)
            )
            return True
        except Exception:
            pass
        # Fallback to COMMAND_LONG
        ctx.master_tx.mav.command_long_send(
            tsys,
            tcomp,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            1,  # use custom mode
            float(custom_mode),
            0, 0, 0, 0, 0
        )
        # Diagnostic broadcast attempt: send to all components
        try:
            ctx.master_tx.mav.command_long_send(
                0,
                0,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                1,
                float(custom_mode),
                0, 0, 0, 0, 0
            )
        except Exception:
            pass
        return True
    except Exception:
        return False

def send_set_mode_name(ctx: AppContext, mode_name: str) -> bool:
    try:
        if ctx.master_tx is None:
            return False
        # Try to use autopilot-provided mapping from heartbeat
        mapping = None
        try:
            mapping = ctx.master_tx.mode_mapping()
        except Exception:
            mapping = None
        target_num = None
        if mapping and isinstance(mapping, dict):
            # mapping has names in upper-case
            target_num = mapping.get(mode_name.upper())
        if target_num is None:
            # Fallback for Rover defaults
            fallback = {
                'MANUAL': 0,
                'STEERING': 3,
                'HOLD': 4,
                'AUTO': 10,
            }
            target_num = fallback.get(mode_name.upper())
        if target_num is None:
            return False
        return send_set_mode(ctx, int(target_num))
    except Exception:
        return False

def handle_command_ack(ctx: AppContext, msg) -> Optional[str]:
    try:
        cmd_id = int(getattr(msg, 'command', -1))
    except Exception:
        cmd_id = -1
    try:
        result = int(getattr(msg, 'result', -1))
    except Exception:
        result = -1
    # Map known command ids to names
    name = None
    try:
        enum_cmd = mavutil.mavlink.enums.get('MAV_CMD', {})
        entry = enum_cmd.get(cmd_id)
        if entry and getattr(entry, 'name', None):
            name = entry.name
    except Exception:
        name = None
    if name is None:
        name = f"MAV_CMD_{cmd_id}"
    text = f"[ACK] {name}: result={result}"
    try:
        import time as _t
        ctx.last_ack_text = text
        ctx.last_ack_ts = _t.time()
    except Exception:
        pass
    return text

def send_arm(ctx: AppContext, arm: bool) -> bool:
    try:
        if ctx.master_tx is None or not hasattr(ctx.master_tx, 'mav'):
            return False
        tsys, tcomp = target_ids(ctx)
        try:
            if getattr(ctx.cfg, 'DEBUG', 0):
                print(f"[mavlink] send_arm arm={bool(arm)} targets sys={tsys} comp={tcomp}")
        except Exception:
            pass
        ctx.master_tx.mav.command_long_send(
            tsys,
            tcomp,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1 if arm else 0,
            0, 0, 0, 0, 0, 0
        )
        # Diagnostic broadcast attempt
        try:
            ctx.master_tx.mav.command_long_send(
                0,
                0,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1 if arm else 0,
                0, 0, 0, 0, 0, 0
            )
        except Exception:
            pass
        return True
    except Exception:
        return False

def send_mission_skip(ctx: AppContext) -> bool:
    try:
        if ctx.master_tx is None or not hasattr(ctx.master_tx, 'mav'):
            return False
        tsys, tcomp = target_ids(ctx)
        # Determine next waypoint index and clamp within mission range
        cur = ctx.rover.wp_seq or 0
        total = ctx.rover.wp_total or 0
        nxt = cur + 1
        if total and nxt >= total:
            # Already at or beyond last, keep last
            nxt = max(0, total - 1)
        # Prefer MISSION_SET_CURRENT for ArduPilot
        ctx.master_tx.mav.mission_set_current_send(tsys, tcomp, int(nxt))
        return True
    except Exception:
        return False

def send_reboot(ctx: AppContext) -> bool:
    try:
        if ctx.master_tx is None or not hasattr(ctx.master_tx, 'mav'):
            return False
        sysid = getattr(ctx.master_tx, 'target_system', None) or ctx.targets.autopilot_sysid
        compid = getattr(ctx.master_tx, 'target_component', None) or ctx.targets.autopilot_compid or 0
        if sysid is None:
            return False
        ctx.master_tx.mav.command_long_send(
            sysid, compid,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        return True
    except Exception:
        return False

def handle_heartbeat(ctx: AppContext, msg) -> Optional[str]:
    src_sys = getattr(msg, 'get_srcSystem', lambda: None)()
    src_comp = getattr(msg, 'get_srcComponent', lambda: None)()
    if ctx.targets.autopilot_sysid is None:
        ctx.targets.autopilot_sysid = ctx.cfg.AUTOPILOT_SYSID or src_sys
    if ctx.targets.autopilot_compid is None:
        ctx.targets.autopilot_compid = ctx.cfg.AUTOPILOT_COMPID or src_comp
    if (ctx.targets.autopilot_sysid and src_sys != ctx.targets.autopilot_sysid) or \
       (ctx.targets.autopilot_compid and src_comp != ctx.targets.autopilot_compid):
        return None

    cm_raw = getattr(msg, 'custom_mode', None)
    try:
        cm = int(cm_raw) if cm_raw is not None else None
    except Exception:
        cm = None

    mode_name = rover_mode_from_custom_mode(cm) if cm is not None else None
    try:
        ms = mavutil.mode_string_v10(msg)
        if ms and not ms.startswith('Mode('):
            mode_name = ms
    except Exception:
        pass
    if mode_name is None:
        mode_name = f"UNKNOWN(cm={cm if cm is not None else 'NA'})"

    ctx.rover.start_time = ctx.rover.start_time or time.time()
    try:
        base_mode = int(getattr(msg, 'base_mode', 0))
        ctx.rover.armed = bool(base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
    except Exception:
        pass

    now_ts = time.time()
    ctx.rover.last_heartbeat_ts = now_ts
    # Always update last_mode from the latest heartbeat to avoid stale/incorrect reporting
    prev_mode = ctx.rover.last_mode
    ctx.deb.last_custom_mode = cm if cm is not None else ctx.deb.last_custom_mode
    ctx.rover.last_mode = mode_name
    ctx.deb.last_mode_sent_ts = now_ts
    if prev_mode != mode_name:
        return f"[Mode Changed]: {mode_name}"
    if ctx.cfg.HEARTBEAT_PERIODIC and (now_ts - ctx.deb.last_heartbeat_notify_ts) >= 15:
        ctx.deb.last_heartbeat_notify_ts = now_ts
        return f"Heartbeat alive. Mode={mode_name}"
    return None

def handle_named_value(ctx: AppContext, msg) -> Optional[str]:
    try:
        n = msg.name.decode('ascii') if isinstance(msg.name, (bytes, bytearray)) else str(msg.name)
    except Exception:
        n = str(msg.name)
    name_str = ctx.cfg.MAVLINK_NAME[:10].ljust(10)
    if n.strip() == ctx.cfg.MAVLINK_NAME.strip() or n[:10].ljust(10) == name_str:
        now_ts = time.time()
        if now_ts - ctx.deb.last_named_value_ts >= ctx.cfg.MAVLINK_THROTTLE_SECONDS:
            ctx.deb.last_named_value_ts = now_ts
            return f"{n.strip()}: {msg.value:.2f}"
    return None

def handle_status_text(ctx: AppContext, msg) -> Optional[str]:
    try:
        sev = int(getattr(msg, 'severity', 0))
    except Exception:
        sev = 0
    try:
        raw = getattr(msg, 'text', None)
        txt = raw.decode('utf-8', errors='ignore') if isinstance(raw, (bytes, bytearray)) else str(raw) if raw is not None else None
    except Exception:
        txt = None
    txt = txt or f"{msg.get_type()} (sev={sev})"
    lower_txt = txt.lower()
    if 'failsafe' in lower_txt:
        clearing = any(kw in lower_txt for kw in ['end', 'ended', 'recovered', 'cleared'])
        if clearing:
            ctx.rover.failsafe_active = False
            ctx.rover.failsafe_desc = None
            ctx.rover.failsafe_reason = None
            return f"Failsafe cleared: {txt}"
        else:
            ctx.rover.failsafe_active = True
            ctx.rover.failsafe_desc = txt
            ctx.rover.failsafe_reason = extract_failsafe_reason(lower_txt)
            display_txt = txt if ctx.rover.failsafe_reason is None else f"{txt} (reason={ctx.rover.failsafe_reason})"
            return f"[FAILSAFE]: {display_txt}"
    try:
        if 'linkqualitygcs' in lower_txt:
            m = re.search(r'linkqualitygcs\s*[:=]?\s*(\d+)', lower_txt)
            if m:
                ctx.link.quality_pct = float(m.group(1))
    except Exception:
        pass
    # Only keep and surface "Reached waypoint" messages, appending total waypoints when known
    try:
        m = re.search(r'reached\s+waypoint\s*#?\s*(\d+)', lower_txt)
        if m:
            wp_idx = int(m.group(1))
            total = ctx.rover.wp_total if ctx.rover.wp_total is not None else None
            msg_txt = f"Reached waypoint {wp_idx}" if not total else f"Reached waypoint {wp_idx} of {int(total)}"
            ctx.rover.status_texts.append(msg_txt)
            if len(ctx.rover.status_texts) > 50:
                del ctx.rover.status_texts[:len(ctx.rover.status_texts)-50]
            # Do not emit a separate Telegram message; menu will be refreshed via MQTT status
            return None
    except Exception:
        pass
    # Suppress other STATUSTEXT messages
    return None

# (removed duplicate handlers; see definitions later in file)

def handle_gps1(ctx: AppContext, msg):
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
        if lat_i != 0: ctx.rover.lat = lat_i / 1e7
        if lon_i != 0: ctx.rover.lon = lon_i / 1e7
        if alt_i != 0: ctx.rover.alt_m = alt_i / 1000.0
        if eph != 0: ctx.rover.hdop = eph / 100.0
        if epv != 0: ctx.rover.vdop = epv / 100.0
        ctx.rover.last_gps_time = time.time()
    except Exception:
        pass
    prev = ctx.rover.gps1_fix
    if prev is None or fix != prev:
        ctx.rover.gps1_fix = fix
        entering = (prev != 6 and fix == 6)
        leaving = (prev == 6 and fix != 6)
        return f"[GPS1 fix changed]: {gps_fix_label(fix)}", (entering or leaving)
    return None

def handle_gps2(ctx: AppContext, msg):
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
        if lat_i != 0: ctx.rover.lat2 = lat_i / 1e7
        if lon_i != 0: ctx.rover.lon2 = lon_i / 1e7
        if alt_i != 0: ctx.rover.alt2_m = alt_i / 1000.0
        if eph != 0: ctx.rover.hdop2 = eph / 100.0
        if epv != 0: ctx.rover.vdop2 = epv / 100.0
        ctx.rover.last_gps2_time = time.time()
    except Exception:
        pass
    prev = ctx.rover.gps2_fix
    if prev is None or fix != prev:
        ctx.rover.gps2_fix = fix
        entering = (prev != 6 and fix == 6)
        leaving = (prev == 6 and fix != 6)
        return f"[GPS2 fix changed]: {gps_fix_label(fix)}", (entering or leaving)
    return None

def handle_vfr_hud(ctx: AppContext, msg) -> None:
    try:
        gs = float(getattr(msg, 'groundspeed', 0.0))
        if gs >= 0.0:
            ctx.rover.ground_speed = gs
    except Exception:
        pass
    try:
        hdg = float(getattr(msg, 'heading', 0.0))
        if hdg >= 0.0:
            ctx.rover.heading_deg = hdg
    except Exception:
        pass

def handle_mission_current(ctx: AppContext, msg) -> None:
    try:
        seq = int(getattr(msg, 'seq', -1))
        if seq >= 0:
            ctx.rover.wp_seq = seq
    except Exception:
        pass

def handle_mission_count(ctx: AppContext, msg) -> None:
    try:
        count = int(getattr(msg, 'count', -1))
        if count >= 0:
            ctx.rover.wp_total = count
    except Exception:
        pass
