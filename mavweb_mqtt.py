"""
MQTT publishing for MavWeb.

- Uses SharedState snapshot as the source of truth.
- Publishes a full JSON snapshot plus per-key topics.

Environment variables:
- MQTT_ENABLED=1|0
- MQTT_BROKER_URL=hostname/ip
- MQTT_BROKER_PORT=1883
- MQTT_USERNAME=...
- MQTT_PASSWORD=...
- MQTT_TOPIC_PREFIX=mavweb
"""

from __future__ import annotations

import json
import os
import time
from typing import Any, Optional
import streamlit as st
import paho.mqtt.client as mqtt

from shared_state import get_shared_state


def _env_bool(name: str, default: bool = False) -> bool:
    v = (os.getenv(name, "") or "").strip().lower()
    if v == "":
        return default
    return v in ("1", "true", "yes", "on")


def _json_default(o: Any) -> Any:
    # Best-effort encoder so "all available mavlink data" can be published.
    try:
        if hasattr(o, "isoformat"):
            return o.isoformat()
    except Exception:
        pass

    if isinstance(o, (bytes, bytearray)):
        return o.hex()

    # Fallback: string representation
    return str(o)


def _make_topics(prefix: str) -> dict[str, str]:
    base = prefix.strip().strip("/")
    if not base:
        base = "mavweb"
    return {
        "base": base,
        "state": f"{base}/state",
        "state_key": f"{base}/state",  # per-key will be f"{state_key}/{key}"
        "meta": f"{base}/meta",
    }


_client: Optional[mqtt.Client] = None
_client_connected: bool = False


def get_mqtt_client() -> Optional[mqtt.Client]:
    """
    Returns a connected MQTT client if MQTT is enabled; otherwise returns None.
    Connection is established lazily and cached at module level.
    """
    global _client, _client_connected

    if not _env_bool("MQTT_ENABLED", default=False):
        return None

    if _client is None:
        _client = mqtt.Client(client_id=os.getenv("MAVLINK_NAME", "mavweb") or "mavweb")

        username = (os.getenv("MQTT_USERNAME", "") or "").strip()
        password = os.getenv("MQTT_PASSWORD", "") or ""
        if username:
            _client.username_pw_set(username=username, password=password)

        def _on_connect(client, userdata, flags, rc, properties=None):
            # rc==0 => OK
            global _client_connected
            _client_connected = (rc == 0)

        def _on_disconnect(client, userdata, rc, properties=None):
            global _client_connected
            _client_connected = False

        _client.on_connect = _on_connect
        _client.on_disconnect = _on_disconnect

    if not _client_connected:
        host = (os.getenv("MQTT_BROKER_URL", "") or "").strip()
        port = int(os.getenv("MQTT_BROKER_PORT", "1883") or "1883")
        if not host:
            return None

        # Connect + start network loop once.
        _client.connect(host, port, keepalive=30)
        _client.loop_start()

        # Give on_connect a brief chance to flip _client_connected.
        # (Avoids "publish before connected" in typical startup paths.)
        t0 = time.time()
        while not _client_connected and (time.time() - t0) < 1.0:
            time.sleep(0.01)

    return _client if _client_connected else None

@st.fragment(run_every=2)
def _publish_stats(client: Optional[mqtt.Client] = None) -> bool:
    """
    Publishes all available MAVLink/rover state from SharedState to MQTT.

    Publishes:
    - {prefix}/state           -> full JSON snapshot
    - {prefix}/state/<key>     -> each top-level key as JSON
    - {prefix}/meta/ts         -> publish timestamp (unix seconds)

    Returns True if publish succeeded (connected + enqueued), else False.
    """
    if client is None:
        client = get_mqtt_client()
    if client is None:
        return False

    prefix = os.getenv("MQTT_TOPIC_PREFIX", "mavweb") or "mavweb"
    topics = _make_topics(prefix)

    state_snapshot = get_shared_state().get()
    now = time.time()

    payload_all = json.dumps(
        {"ts": now, "data": state_snapshot},
        default=_json_default,
        separators=(",", ":"),
    )

    # Full snapshot
    client.publish(topics["state"], payload_all, qos=0, retain=False)

    # Per-key snapshots (top-level keys)
    for k, v in state_snapshot.items():
        try:
            payload_k = json.dumps(v, default=_json_default, separators=(",", ":"))
        except Exception:
            payload_k = json.dumps(str(v), separators=(",", ":"))
        client.publish(f"{topics['state_key']}/{k}", payload_k, qos=0, retain=False)

    # Meta timestamp
    client.publish(f"{topics['meta']}/ts", str(now), qos=0, retain=False)

    return True