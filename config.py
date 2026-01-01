from dataclasses import dataclass
import os
from dotenv import load_dotenv

load_dotenv()

@dataclass(frozen=True)
class Config:
    MAVLINK_TX_ENDPOINT: str = os.getenv("MAVLINK_TX_ENDPOINT", "udpout:127.0.0.1:14550")
    MAVLINK_RX_ENDPOINT: str = os.getenv("MAVLINK_RX_ENDPOINT", "udpin:0.0.0.0:14551")
    MAVLINK_NAME: str = os.getenv("MAVLINK_NAME", "DS18F     ")
    DEBUG: bool = os.getenv("DEBUG", "0") == "1"
    TEMP_SENDER_ENABLED: bool = os.getenv("TEMP_SENDER_ENABLED", "1") == "1"

    FAILSAFE_NOTIFY_LOUD: bool = os.getenv("FAILSAFE_NOTIFY_LOUD", "1") == "1"
    AUTOPILOT_SYSID: int | None = (lambda v: int(v) if v and v.isdigit() else None)(os.getenv("AUTOPILOT_SYSID", ""))
    AUTOPILOT_COMPID: int | None = (lambda v: int(v) if v and v.isdigit() else None)(os.getenv("AUTOPILOT_COMPID", ""))

    PLOT_WIDTH: int = int(os.getenv("PLOT_WIDTH", "1024"))
    PLOT_HEIGHT: int = int(os.getenv("PLOT_HEIGHT", "768"))
    PLOT_BG: str = os.getenv("PLOT_BG", "#FFFFFF")
    PLOT_ROUTE_COLOR: str = os.getenv("PLOT_ROUTE_COLOR", "#B4B4B4")
    PLOT_COMPLETED_COLOR: str = os.getenv("PLOT_COMPLETED_COLOR", "#00A03C")
    PLOT_POINT_COLOR: str = os.getenv("PLOT_POINT_COLOR", "#969696")