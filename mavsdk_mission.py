import asyncio
import os
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class MavsdkMissionConfig:
    system_address: str
    connect_timeout_s: float = 10.0


def get_mavsdk_system_address() -> str:
    # MAVSDK listens/connected separately from pymavlink.
    # Typical default for MAVSDK examples is udp://:14540.
    return (os.getenv("MAVSDK_SYSTEM_ADDRESS", "") or "udp://:14540").strip()


def _run_coro_blocking(coro):
    """Run an async coroutine from sync code.

    Streamlit scripts generally don't run an asyncio loop, but if they do (or a library does),
    we run the coroutine in a dedicated thread with its own event loop.
    """
    try:
        asyncio.get_running_loop()
    except RuntimeError:
        return asyncio.run(coro)

    import threading

    result = {"value": None, "error": None}

    def _runner():
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            result["value"] = loop.run_until_complete(coro)
        except Exception as e:  # pragma: no cover
            result["error"] = e
        finally:
            try:
                loop.close()
            except Exception:
                pass

    t = threading.Thread(target=_runner, daemon=True)
    t.start()
    t.join()

    if result["error"] is not None:
        raise result["error"]
    return result["value"]


async def _connect_drone(cfg: MavsdkMissionConfig):
    from mavsdk import System

    drone = System()
    await drone.connect(system_address=cfg.system_address)

    async def _wait_connected():
        async for state in drone.core.connection_state():
            if state.is_connected:
                return

    await asyncio.wait_for(_wait_connected(), timeout=cfg.connect_timeout_s)
    return drone


async def download_mission_points(cfg: MavsdkMissionConfig) -> list[list[float]]:
    """Download mission via MAVSDK Mission plugin.

    Returns [[lon, lat], ...] to match the app's PyDeck expectation.
    """
    drone = await _connect_drone(cfg)

    plan = await drone.mission.download_mission()
    items = getattr(plan, "mission_items", []) or []

    points: list[list[float]] = []
    for it in items:
        lat = getattr(it, "latitude_deg", None)
        lon = getattr(it, "longitude_deg", None)
        if lat is None or lon is None:
            continue
        try:
            lat_f = float(lat)
            lon_f = float(lon)
        except Exception:
            continue
        if abs(lat_f) < 1e-9 and abs(lon_f) < 1e-9:
            continue
        points.append([lon_f, lat_f])

    return points


def download_mission_points_sync(system_address: Optional[str] = None, *, timeout_s: float = 10.0) -> list[list[float]]:
    addr = (system_address or get_mavsdk_system_address()).strip()
    cfg = MavsdkMissionConfig(system_address=addr, connect_timeout_s=timeout_s)
    return _run_coro_blocking(download_mission_points(cfg))


def _parse_qgc_wpl_points(filepath: str) -> list[tuple[float, float, float]]:
    """Parse QGC WPL mission file into (lat, lon, rel_alt_m) points."""
    try:
        with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
            lines = [ln.strip() for ln in f if ln.strip() and not ln.strip().startswith("#")]
        if not lines or not lines[0].startswith("QGC WPL"):
            return []

        pts: list[tuple[float, float, float]] = []
        for ln in lines[1:]:
            cols = ln.split("\t")
            if len(cols) < 12:
                continue
            try:
                lat = float(cols[8])
                lon = float(cols[9])
                alt = float(cols[10])
            except Exception:
                continue
            if abs(lat) < 1e-9 and abs(lon) < 1e-9:
                continue
            pts.append((lat, lon, alt))
        return pts
    except Exception:
        return []


async def upload_qgc_wpl_mission(cfg: MavsdkMissionConfig, filepath: str) -> bool:
    """Upload a QGC WPL mission via MAVSDK Mission plugin.

    Notes:
    - MAVSDK Mission API is higher-level than raw MAVLink mission items. We map each row to
      a basic waypoint-style MissionItem.
    """
    from mavsdk.mission import MissionItem, MissionPlan

    pts = _parse_qgc_wpl_points(filepath)
    if not pts:
        return False

    drone = await _connect_drone(cfg)

    mission_items = []
    for (lat, lon, alt) in pts:
        # Conservative defaults; rover missions commonly ignore altitude anyway.
        mission_items.append(
            MissionItem(
                latitude_deg=lat,
                longitude_deg=lon,
                relative_altitude_m=alt,
                speed_m_s=0.0,
                is_fly_through=True,
                gimbal_pitch_deg=0.0,
                gimbal_yaw_deg=0.0,
                camera_action=MissionItem.CameraAction.NONE,
                loiter_time_s=0.0,
                camera_photo_interval_s=0.0,
                acceptance_radius_m=1.0,
                yaw_deg=0.0,
                camera_photo_distance_m=0.0,
            )
        )

    plan = MissionPlan(mission_items)
    await drone.mission.upload_mission(plan)
    return True


def upload_qgc_wpl_mission_sync(filepath: str, system_address: Optional[str] = None, *, timeout_s: float = 10.0) -> bool:
    addr = (system_address or get_mavsdk_system_address()).strip()
    cfg = MavsdkMissionConfig(system_address=addr, connect_timeout_s=timeout_s)
    return bool(_run_coro_blocking(upload_qgc_wpl_mission(cfg, filepath)))
