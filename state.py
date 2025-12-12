from dataclasses import dataclass, field
from typing import Optional, List, Any

@dataclass
class LinkStats:
    quality_pct: Optional[float] = None
    rssi: Optional[int] = None
    remrssi: Optional[int] = None
    noise: Optional[int] = None
    remnoise: Optional[int] = None
    txbuf: Optional[int] = None
    rxerrors: Optional[int] = None
    fixed: Optional[int] = None

@dataclass
class RoverState:
    last_mode: Optional[str] = None
    armed: Optional[bool] = None
    last_heartbeat_ts: Optional[float] = None
    heading_deg: Optional[float] = None
    lat: Optional[float] = None
    lon: Optional[float] = None
    alt_m: Optional[float] = None
    hdop: Optional[float] = None
    vdop: Optional[float] = None
    ground_speed: Optional[float] = None
    batt_voltage: Optional[float] = None
    wp_seq: Optional[int] = None
    mission_total: Optional[int] = None
    wp_total: Optional[int] = None
    start_time: Optional[float] = None
    last_gps_time: Optional[float] = None
    gps1_fix: Optional[int] = None
    gps2_fix: Optional[int] = None
    lat2: Optional[float] = None
    lon2: Optional[float] = None
    alt2_m: Optional[float] = None
    hdop2: Optional[float] = None
    vdop2: Optional[float] = None
    last_gps2_time: Optional[float] = None
    heading_deg: Optional[float] = None
    wp_total: Optional[int] = None
    failsafe_active: bool = False
    failsafe_desc: Optional[str] = None
    failsafe_reason: Optional[str] = None
    status_texts: List[str] = field(default_factory=list)
    mission_items: List[tuple] = field(default_factory=list)

@dataclass
class DebounceState:
    last_named_value_ts: float = 0.0
    last_heartbeat_notify_ts: float = 0.0
    last_custom_mode: Optional[int] = None
    last_custom_mode_sent: Optional[int] = None
    last_mode_sent_ts: float = 0.0
    pending_custom_mode: Optional[int] = None
    pending_mode_name: Optional[str] = None
    pending_mode_first_seen: Optional[float] = None
    mission_hash: Optional[str] = None
    last_progress_idx: Optional[int] = None

@dataclass
class Targets:
    autopilot_sysid: Optional[int] = None
    autopilot_compid: Optional[int] = None

@dataclass
class AppContext:
    cfg: 'Config'
    rover: RoverState
    deb: DebounceState
    link: LinkStats
    targets: Targets
    chat_id: Optional[int] = None
    master_tx: Optional[Any] = None
    last_ack_text: Optional[str] = None
    last_ack_ts: Optional[float] = None
