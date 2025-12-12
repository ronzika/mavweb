import time
from typing import Optional

def format_hms(seconds: Optional[float]) -> str:
    if seconds is None:
        return "-"
    s = int(seconds)
    h = s // 3600
    m = (s % 3600) // 60
    sec = s % 60
    return f"{h:02d}:{m:02d}:{sec:02d}"

def gps_fix_label(fix_type: int) -> str:
    mapping = {
        0: 'No GPS', 1: 'No Fix', 2: '2D Fix', 3: '3D Fix',
        4: 'DGPS', 5: 'RTK Float', 6: 'RTK Fixed', 7: 'Static', 8: 'PPP',
    }
    return mapping.get(int(fix_type), f'Unknown({fix_type})')

def rover_mode_from_custom_mode(cm: int) -> str:
    modes = {
        0: 'MANUAL', 1: 'ACRO', 2: 'STEERING', 3: 'HOLD', 4: 'LOITER',
        5: 'FOLLOW', 6: 'SIMPLE', 7: 'AUTO', 8: 'RTL', 9: 'SMART_RTL',
        10: 'GUIDED', 11: 'INITIALISING'
    }
    return modes.get(cm, 'UNKNOWN')

def extract_failsafe_reason(lower_txt: str) -> str | None:
    patterns = [
        ('rc', 'RC Link Loss'), ('radio', 'RC Link Loss'), ('gcs', 'GCS Link Loss'),
        ('ekf', 'EKF'), ('gps', 'GPS'), ('battery', 'Battery'), ('terrain', 'Terrain'),
        ('crash', 'Crash'), ('leak', 'Leak'), ('parachute', 'Parachute'), ('watchdog', 'Watchdog'),
        ('compass', 'Compass'), ('imu', 'IMU'), ('internal error', 'Internal Error'),
        ('throttle', 'Throttle'), ('navigation', 'Navigation'), ('relay', 'Relay'), ('motor', 'Motor'),
    ]
    for key, reason in patterns:
        if key in lower_txt:
            return reason
    try:
        idx = lower_txt.find('failsafe')
        if idx != -1:
            remainder = lower_txt[idx+8:].strip(': ,-')
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

def now() -> float:
    return time.time()
