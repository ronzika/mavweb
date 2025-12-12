from threading import Thread
import time
import os, hashlib, json, time
import requests

from config import Config
from utils import gps_fix_label
from state import AppContext, RoverState, DebounceState, LinkStats, Targets
from mavlink_io import open_connections, handle_heartbeat, handle_named_value, handle_status_text, handle_gps1, handle_gps2
from mavlink_io import (
    open_connections, handle_heartbeat, handle_named_value, handle_status_text,
    handle_gps1, handle_gps2, handle_command_ack, init_link, send_heartbeat,
    handle_vfr_hud, handle_mission_current, handle_mission_count
)
from mavlink_io import download_mission
from utils_plot import build_base_png, update_progress_png, read_progress_state, write_progress_state
from queue import Queue

_plot_queue: Queue = Queue()
_plot_worker_started = False


def run_monitor(ctx: AppContext, master_rx, notify_fn):
    while True:
        drained = 0
        while True:
            msg = None
            if master_rx is not None:
                msg = master_rx.recv_match(blocking=False)
            if not msg:
                break
            drained += 1
            mtype = msg.get_type()
            if mtype == 'HEARTBEAT':
                text = handle_heartbeat(ctx, msg)
                if text:
                    notify_fn(text, loud=True)
            elif mtype == 'NAMED_VALUE_FLOAT':
                text = handle_named_value(ctx, msg)
                if text:
                    notify_fn(text, loud=False)
            elif mtype in ('STATUSTEXT', 'STATUSTEXT_LONG'):
                _ = handle_status_text(ctx, msg)
            elif mtype == 'COMMAND_ACK':
                text = handle_command_ack(ctx, msg)
                # Only show ACK messages when DEBUG is enabled
                if text and ctx.cfg.DEBUG:
                    notify_fn(text, loud=False)
            elif mtype == 'GPS_RAW_INT':
                res = handle_gps1(ctx, msg)
                if res:
                    text, loud = res
                    notify_fn(text, loud=loud)
            elif mtype in ('GPS2_RAW', 'GPS2_RAW_INT'):
                res = handle_gps2(ctx, msg)
                if res:
                    text, loud = res
                    notify_fn(text, loud=loud)
            elif mtype == 'VFR_HUD':
                handle_vfr_hud(ctx, msg)
            elif mtype == 'MISSION_CURRENT':
                handle_mission_current(ctx, msg)
            elif mtype == 'MISSION_COUNT':
                handle_mission_count(ctx, msg)
        time.sleep(0.5)



def handle_plot_mission(ctx: AppContext):
    # Enqueue a plot request for background processing
    try:
        _plot_queue.put({'ts': time.time(), 'seq': (ctx.rover.wp_seq or 0)})
    except Exception:
        pass

def _plot_worker(ctx: AppContext):
    while True:
        job = _plot_queue.get()
        try:
            # Download or reuse mission points
            pts = download_mission(ctx, ctx.master_tx, timeout=3.0) or []
            if not pts:
                continue
            # Build/reuse base, update progress
            cache_dir = os.path.join(os.getcwd(), "cache")
            base_png, coords_json = build_base_png(
                pts,
                width=ctx.cfg.PLOT_WIDTH,
                height=ctx.cfg.PLOT_HEIGHT,
                bg_color=ctx.cfg.PLOT_BG,
                route_color=ctx.cfg.PLOT_ROUTE_COLOR,
                point_color=ctx.cfg.PLOT_POINT_COLOR,
                cache_dir=cache_dir
            )
            # Progress state keyed by mission hash (coarse)
            mh = getattr(ctx.deb, 'mission_hash', None)
            if not mh:
                mh = hashlib.sha1((";".join([f"{p[0]:.7f},{p[1]:.7f}" for p in pts])).encode("utf-8")).hexdigest()
            state_path = os.path.join(cache_dir, "mission_progress_state.json")
            st = read_progress_state(state_path)
            last_idx = st.get("last_idx") if st.get("hash") == mh else None

            out_png = os.path.join(os.getcwd(), "mission_progress.png")
            drawn_idx = update_progress_png(
                base_png, coords_json, (ctx.rover.wp_seq or 0),
                out_png, completed_color=ctx.cfg.PLOT_COMPLETED_COLOR, last_idx=last_idx
            )
            write_progress_state(state_path, {"hash": mh, "last_idx": drawn_idx})

            # (Telegram photo sending removed)
        except Exception as e:
            if ctx.cfg.DEBUG:
                print(f"[plot-worker] error: {e}")
        finally:
            try:
                _plot_queue.task_done()
            except Exception:
                pass
    # Build/reuse base, update progress
    cache_dir = os.path.join(os.getcwd(), "cache")
    base_png, coords_json = build_base_png(
        pts,
        width=ctx.cfg.PLOT_WIDTH,
        height=ctx.cfg.PLOT_HEIGHT,
        bg_color=ctx.cfg.PLOT_BG,
        route_color=ctx.cfg.PLOT_ROUTE_COLOR,
        point_color=ctx.cfg.PLOT_POINT_COLOR,
        cache_dir=cache_dir
    )
    # Progress state keyed by mission hash
    mh = hashlib.sha1((";".join([f"{p[0]:.7f},{p[1]:.7f}" for p in pts])).encode("utf-8")).hexdigest()
    state_path = os.path.join(cache_dir, "mission_progress_state.json")
    st = read_progress_state(state_path)
    last_idx = st.get("last_idx") if st.get("hash") == mh else None

    out_png = os.path.join(os.getcwd(), "mission_progress.png")
    drawn_idx = update_progress_png(
        base_png, coords_json, (ctx.rover.wp_seq or 0),
        out_png, completed_color=ctx.cfg.PLOT_COMPLETED_COLOR, last_idx=last_idx
    )
    write_progress_state(state_path, {"hash": mh, "last_idx": drawn_idx})

    # (Telegram photo sending removed)


def main():
    global _plot_worker_started
    cfg = Config()
    ctx = AppContext(cfg=cfg, rover=RoverState(), deb=DebounceState(), link=LinkStats(), targets=Targets())
    # temp_reader removed
    # Reset plot/cache on startup (always)
    try:
        cache_dir = os.path.join(os.getcwd(), "cache")
        # Remove mission progress state and any generated images
        state_path = os.path.join(cache_dir, "mission_progress_state.json")
        out_png = os.path.join(os.getcwd(), "mission_progress.png")
        if os.path.exists(state_path):
            os.remove(state_path)
        if os.path.exists(out_png):
            os.remove(out_png)
        # Also clear any cached base images and coords
        if os.path.isdir(cache_dir):
            for fname in os.listdir(cache_dir):
                if fname.endswith("_base.png") or fname.endswith("_coords.json"):
                    try:
                        os.remove(os.path.join(cache_dir, fname))
                    except Exception:
                        pass
    except Exception:
        pass
    master_tx, master_rx = open_connections(cfg)
    ctx.master_tx = master_tx
    # Nudge FC to start streaming and recognize us
    init_link(ctx)

    def notify(text: str, loud: bool = False):
        pass  # Telegram notification removed

    # Telegram chat_id logic removed

    t_mon = Thread(target=run_monitor, args=(ctx, master_rx, notify), daemon=True)
    t_mon.start()

    # Optional: periodic heartbeat to keep link alive
    def hb_loop():
        try:
            while True:
                send_heartbeat(ctx)
                time.sleep(1.0)
        except Exception:
            pass
    t_hb = Thread(target=hb_loop, daemon=True)
    t_hb.start()

    # MQTT publisher thread removed

    # MQTT command consumer removed

    # Telegram bot logic removed; always start plot worker in headless mode
    if not _plot_worker_started:
        try:
            Thread(target=_plot_worker, args=(ctx,), daemon=True).start()
            _plot_worker_started = True
        except Exception as e:
            if cfg.DEBUG:
                print(f"[plot] worker start failed: {e}")
    try:
        while True:
            time.sleep(60)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
