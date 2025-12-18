"""
Mission file handling: upload, parse, preview
"""
import os
import datetime
import pandas as pd
from typing import List, Dict

def list_missions(missions_dir: str) -> List[Dict]:
    missions = []
    files = os.listdir(missions_dir)
    for f in files:
        if not f.startswith('.'):
            base = os.path.splitext(f)[0].replace('.', '')
            path = os.path.join(missions_dir, f)
            try:
                ts = os.path.getmtime(path)
                ts_str = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
            except Exception:
                ts_str = '?'
            missions.append({'Mission': base, 'Timestamp': ts_str, 'Filename': f, 'Path': path})
    return missions

def parse_waypoints_file(filepath: str) -> List[Dict]:
    points = []
    try:
        with open(filepath) as f:
            all_lines = [line.strip() for line in f if line.strip() and not line.startswith('#')]
        lines = all_lines[2:]
        for line in lines:
            cols = line.split('\t')
            if len(cols) >= 10:
                try:
                    lat = float(cols[8])
                    lon = float(cols[9])
                    points.append({'lat': lat, 'lon': lon})
                except Exception:
                    continue
    except Exception:
        pass
    return points
