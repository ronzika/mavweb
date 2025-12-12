# Incremental mission progress renderer (PNG), using cached base image and coords.
from __future__ import annotations
import hashlib, json, os
from typing import List, Tuple
from PIL import Image, ImageDraw


def _hex_to_rgb(s: str) -> tuple[int,int,int]:
    s = s.strip()
    if s.startswith("#"):
        s = s[1:]
    if len(s) == 6:
        return int(s[0:2], 16), int(s[2:4], 16), int(s[4:6], 16)
    return (255, 255, 255)


def _mission_hash(points: List[Tuple[float, float]]) -> str:
    h = hashlib.sha1()
    for lat, lon in points:
        h.update(f"{lat:.7f},{lon:.7f};".encode("utf-8"))
    return h.hexdigest()


def _project(points: List[Tuple[float, float]], width: int, height: int, margin: int = 20) -> List[Tuple[int, int]]:
    if not points:
        return []
    lats = [p[0] for p in points]
    lons = [p[1] for p in points]
    min_lat, max_lat = min(lats), max(lats)
    min_lon, max_lon = min(lons), max(lons)
    # avoid zero extent
    if max_lat - min_lat < 1e-9: max_lat = min_lat + 1e-9
    if max_lon - min_lon < 1e-9: max_lon = min_lon + 1e-9
    W, H = width, height
    x0, y0 = margin, margin
    x1, y1 = W - margin, H - margin
    span_x = (x1 - x0)
    span_y = (y1 - y0)
    coords: List[Tuple[int, int]] = []
    for lat, lon in points:
        x = x0 + int((lon - min_lon) / (max_lon - min_lon) * span_x)
        # invert Y so north is up
        y = y0 + int((max_lat - lat) / (max_lat - min_lat) * span_y)
        coords.append((x, y))
    return coords


def _ensure_dir(path: str):
    if path and not os.path.exists(path):
        os.makedirs(path, exist_ok=True)


def build_base_png(points: List[Tuple[float, float]], width: int, height: int,
                   bg_color: str, route_color: str, point_color: str,
                   cache_dir: str = "cache") -> Tuple[str, str]:
    """
    Returns (base_png_path, coords_json_path). Creates cache files if missing.
    """
    _ensure_dir(cache_dir)
    mh = _mission_hash(points)
    base_png = os.path.join(cache_dir, f"mission_{mh}_base.png")
    coords_json = os.path.join(cache_dir, f"mission_{mh}_coords.json")
    if os.path.exists(base_png) and os.path.exists(coords_json):
        return base_png, coords_json

    coords = _project(points, width, height)
    # Save coords for later incremental draws
    with open(coords_json, "w") as f:
        json.dump({"coords": coords, "width": width, "height": height}, f)

    # Draw base: white bg, full mission in gray, points as small circles
    img = Image.new("RGB", (width, height), _hex_to_rgb(bg_color))
    drw = ImageDraw.Draw(img)
    if len(coords) >= 2:
        drw.line(coords, fill=_hex_to_rgb(route_color), width=2)
    r = 3
    pc = _hex_to_rgb(point_color)
    for (x, y) in coords:
        drw.ellipse([(x - r, y - r), (x + r, y + r)], fill=pc, outline=pc, width=1)
    img.save(base_png, format="PNG", optimize=True)
    return base_png, coords_json


def update_progress_png(base_png: str, coords_json: str, current_idx: int, out_png: str,
                        completed_color: str, last_idx: int | None = None) -> int:
    """
    Draws newly completed segments from last_idx+1..current_idx on top of base.
    Returns the idx actually drawn (clamped).
    """
    if not os.path.exists(coords_json):
        raise FileNotFoundError(coords_json)
    with open(coords_json, "r") as f:
        data = json.load(f)
    coords = data.get("coords") or []
    if not coords:
        # nothing to draw; copy base
        Image.open(base_png).save(out_png, format="PNG", optimize=True)
        return -1
    # clamp
    n = len(coords)
    if current_idx is None: current_idx = 0
    if current_idx < 0: current_idx = 0
    if current_idx >= n: current_idx = n - 1
    if last_idx is None: last_idx = 0
    if last_idx < 0: last_idx = 0
    if last_idx >= current_idx: last_idx = max(0, min(current_idx - 1, n - 2))

    img = Image.open(base_png).convert("RGB")
    drw = ImageDraw.Draw(img)
    cc = _hex_to_rgb(completed_color)
    # Draw segments in green
    for i in range(max(1, last_idx + 1), current_idx + 1):
        x0, y0 = coords[i - 1]
        x1, y1 = coords[i]
        drw.line([(x0, y0), (x1, y1)], fill=cc, width=3)
    # Highlight current point (solid fill with completed color border)
    x, y = coords[current_idx]
    r = 5
    drw.ellipse([(x - r, y - r), (x + r, y + r)], fill=cc, outline=(0, 120, 255), width=2)
    img.save(out_png, format="PNG", optimize=True)
    return current_idx


def read_progress_state(path: str) -> dict:
    try:
        with open(path, "r") as f:
            return json.load(f)
    except Exception:
        return {}


def write_progress_state(path: str, state: dict):
    _ensure_dir(os.path.dirname(path) or ".")
    with open(path, "w") as f:
        json.dump(state, f)
