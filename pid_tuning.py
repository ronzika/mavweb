import os
import time
import datetime
import re
import json
import textwrap

import altair as alt
import pandas as pd
import requests
import streamlit as st
from streamlit_autorefresh import st_autorefresh

from dashboard_page import ensure_runtime_started
from shared_state import get_shared_state


PARAM_GROUPS = {
    "Steering": {
        "ATC_STR_RAT_P": (0.0, 20.0, 0.01),
        "ATC_STR_RAT_I": (0.0, 20.0, 0.01),
        "ATC_STR_RAT_D": (0.0, 2.0, 0.001),
        "ATC_STR_RAT_FF": (0.0, 5.0, 0.001),
    },
    "Speed": {
        "ATC_SPEED_P": (0.0, 5.0, 0.01),
        "ATC_SPEED_I": (0.0, 2.0, 0.01),
        "ATC_SPEED_D": (0.0, 1.0, 0.001),
        "ATC_SPEED_FF": (0.0, 2.0, 0.001),
    },
    "Throttle And Safety": {
        "CRUISE_SPEED": (0.0, 25.0, 0.1),
        "CRUISE_THROTTLE": (0.0, 100.0, 0.1),
        "ATC_ACCEL_MAX": (0.0, 10.0, 0.1),
        "ATC_DECEL_MAX": (0.0, 10.0, 0.1),
    },
    "L1 Navigation": {
        "NAVL1_PERIOD": (2.0, 60.0, 0.1),
        "NAVL1_DAMPING": (0.1, 2.0, 0.01),
        "WP_RADIUS": (0.1, 20.0, 0.1),
    },
}


def _flatten_params() -> dict:
    merged = {}
    for group_vals in PARAM_GROUPS.values():
        merged.update(group_vals)
    return merged


def _coerce_float(v, default=0.0):
    try:
        return float(v)
    except Exception:
        return default


def _to_samples_dataframe(samples: list[dict]) -> pd.DataFrame:
    if not samples:
        return pd.DataFrame()

    df = pd.DataFrame(samples)
    if "timestamp" in df.columns:
        df["timestamp"] = pd.to_datetime(df["timestamp"], unit="s", errors="coerce")
    return df.sort_values("timestamp")


def _decode_rover_pid_mask(mask_value) -> dict:
    try:
        mask_int = int(float(mask_value))
    except Exception:
        mask_int = None

    if mask_int is None:
        return {
            "raw": None,
            "steering": False,
            "speed": False,
            "known": False,
            "label": "unknown",
            "metrics": [],
        }

    steering = bool(mask_int & 1)   # Rover bit 0: Steering PID_TUNING
    speed = bool(mask_int & 2)      # Rover bit 1: Throttle/Speed PID_TUNING
    metrics = []
    if steering:
        metrics.append("steering")
    if speed:
        metrics.append("speed")

    if not metrics:
        label = "none"
    elif len(metrics) == 2:
        label = "steering+speed"
    else:
        label = metrics[0]

    return {
        "raw": mask_int,
        "steering": steering,
        "speed": speed,
        "known": True,
        "label": label,
        "metrics": metrics,
    }


def _resolve_pid_mask_from_cache(param_cache: dict) -> dict:
    entry = (param_cache or {}).get("GCS_PID_MASK") if isinstance(param_cache, dict) else None
    value = entry.get("value") if isinstance(entry, dict) else None
    return _decode_rover_pid_mask(value)


def _filter_dataframe_for_mask(df: pd.DataFrame, mask_info: dict) -> pd.DataFrame:
    if df.empty:
        return df

    if not (mask_info.get("steering") or mask_info.get("speed")):
        return pd.DataFrame()

    keep_cols = [
        c for c in ["timestamp", "mode", "armed", "gps_fix", "link_quality", "speed_ms", "signal_source", "gcs_pid_mask"]
        if c in df.columns
    ]

    if mask_info.get("steering"):
        keep_cols.extend([c for c in ["steering_desired", "steering_achieved", "steering_error"] if c in df.columns])
    if mask_info.get("speed"):
        keep_cols.extend([c for c in ["speed_desired", "speed_achieved", "speed_error"] if c in df.columns])

    # Preserve order while removing duplicates.
    keep_cols = list(dict.fromkeys(keep_cols))
    if not keep_cols:
        return pd.DataFrame()
    return df[keep_cols].copy()


def _filter_param_groups_for_mask(mask_info: dict) -> dict:
    visible = {}
    if mask_info.get("steering") and "Steering" in PARAM_GROUPS:
        visible["Steering"] = PARAM_GROUPS["Steering"]
    if mask_info.get("speed") and "Speed" in PARAM_GROUPS:
        visible["Speed"] = PARAM_GROUPS["Speed"]
    return visible


def _params_loaded_for_mask(param_cache: dict, mask_info: dict) -> tuple[bool, int, int]:
    visible_groups = _filter_param_groups_for_mask(mask_info)
    required_params = [pname for params in visible_groups.values() for pname in params.keys()]
    required_total = len(required_params)
    if required_total == 0:
        return False, 0, 0

    loaded_count = 0
    for pname in required_params:
        entry = param_cache.get(pname, {}) if isinstance(param_cache, dict) else {}
        if isinstance(entry, dict) and entry.get("value") is not None:
            loaded_count += 1

    return loaded_count == required_total, loaded_count, required_total


def _build_llm_prompt(mask_info: dict) -> str:
    metrics = list(mask_info.get("metrics") or [])
    if not metrics:
        metric_scope = "No valid PID metrics were selected by GCS_PID_MASK"
    elif len(metrics) == 1:
        metric_scope = f"Only the {metrics[0]} control-loop metrics were collected"
    else:
        metric_scope = "Both steering and speed control-loop metrics were collected"

    return (
        "You are reviewing rover PID tuning telemetry from a CSV capture. "
        f"{metric_scope}. "
        "Compare piddesired and pidachieved over time only for the metric types present in the CSV. "
        "Identify lag, overshoot, oscillation, steady-state error, and potential saturation behavior. "
        "Return:\n"
        "1) A concise health assessment for each collected control loop\n"
        "2) Specific tuning recommendations with rationale (P/I/D/FF where relevant)\n"
        "3) Any safety risks or unstable periods by timestamp windows\n"
        "4) Additional data you need to increase confidence\n\n"
        "Also include a final fenced JSON block (```json ... ```) with this exact structure:\n"
        "{\n"
        "  \"parameter_recommendations\": [\n"
        "    {\"name\": \"ATC_STR_RAT_P\", \"value\": 0.12, \"reason\": \"short reason\"}\n"
        "  ]\n"
        "}\n"
        "Use only numeric values for value. Include every parameter recommendation you make."
    )


def _build_session_summary(df: pd.DataFrame) -> dict:
    summary = {
        "sample_count": int(len(df)),
        "duration_s": 0.0,
        "steer_mae": None,
        "speed_mae": None,
        "steer_max_abs_err": None,
        "speed_max_abs_err": None,
        "max_speed_seen": None,
    }

    if df.empty:
        return summary

    try:
        ts_min = df["timestamp"].min()
        ts_max = df["timestamp"].max()
        if pd.notna(ts_min) and pd.notna(ts_max):
            summary["duration_s"] = max(0.0, float((ts_max - ts_min).total_seconds()))
    except Exception:
        pass

    if "steering_error" in df.columns:
        s = pd.to_numeric(df["steering_error"], errors="coerce").dropna()
        if not s.empty:
            summary["steer_mae"] = float(s.abs().mean())
            summary["steer_max_abs_err"] = float(s.abs().max())

    if "speed_error" in df.columns:
        s = pd.to_numeric(df["speed_error"], errors="coerce").dropna()
        if not s.empty:
            summary["speed_mae"] = float(s.abs().mean())
            summary["speed_max_abs_err"] = float(s.abs().max())

    if "speed_ms_raw" in df.columns:
        s = pd.to_numeric(df["speed_ms_raw"], errors="coerce").dropna()
        if not s.empty:
            summary["max_speed_seen"] = float(s.max())
    elif "speed_ms" in df.columns:
        s = pd.to_numeric(df["speed_ms"], errors="coerce").dropna()
        if not s.empty:
            summary["max_speed_seen"] = float(s.max())
    elif "speed_achieved" in df.columns:
        s = pd.to_numeric(df["speed_achieved"], errors="coerce").dropna()
        if not s.empty:
            summary["max_speed_seen"] = float(s.max())

    return summary


def _prepare_csv_for_llm(df: pd.DataFrame) -> tuple[str, str]:
    max_rows = int(os.getenv("PID_LLM_MAX_ROWS", "2500") or 2500)
    max_chars = int(os.getenv("PID_LLM_MAX_CHARS", "180000") or 180000)
    max_rows = max(200, min(max_rows, 20000))
    max_chars = max(10000, min(max_chars, 1000000))

    working = df.copy()
    note = ""
    if len(working) > max_rows:
        head_rows = max_rows // 2
        tail_rows = max_rows - head_rows
        working = pd.concat([working.head(head_rows), working.tail(tail_rows)], axis=0)
        note = (
            f"CSV was downsampled for LLM context limits: kept first {head_rows} and last {tail_rows} "
            f"rows out of {len(df)} total rows."
        )

    csv_text = working.to_csv(index=False)
    if len(csv_text) > max_chars:
        # Hard guard for context size and API payload limits.
        trimmed = csv_text[:max_chars]
        last_newline = trimmed.rfind("\n")
        if last_newline > 0:
            trimmed = trimmed[:last_newline]
        csv_text = trimmed
        extra = f"CSV was truncated to {len(csv_text)} characters."
        note = f"{note} {extra}".strip()

    return csv_text, note


def _build_param_snapshot_text(param_cache: dict, param_names: list[str]) -> str:
    lines = ["parameter,value,updated_ts"]
    for name in param_names:
        entry = param_cache.get(name, {}) if isinstance(param_cache, dict) else {}
        value = entry.get("value")
        updated_ts = entry.get("updated_ts")
        if value is None:
            lines.append(f"{name},MISSING,")
        else:
            lines.append(f"{name},{value},{updated_ts or ''}")
    return "\n".join(lines)


def _looks_like_ardupilot_param(name: str) -> bool:
    if not isinstance(name, str):
        return False
    nm = name.strip().upper()
    if len(nm) < 4:
        return False
    if "_" not in nm:
        return False
    return bool(re.fullmatch(r"[A-Z][A-Z0-9_]*", nm))


def _wrap_reason_text(text: str, width: int = 56) -> str:
    raw = str(text or "").strip()
    if not raw:
        return ""
    parts = [textwrap.fill(part, width=width) if part.strip() else "" for part in raw.splitlines()]
    return "\n".join(parts)


def _editor_row_height_for_reasons(reasons: list[str], base_px: int = 34, line_px: int = 16, max_px: int = 120) -> int:
    max_lines = 1
    for reason in reasons or []:
        txt = str(reason or "")
        line_count = max(1, txt.count("\n") + 1)
        if line_count > max_lines:
            max_lines = line_count
    # Keep compact rows for single-line content; expand only when wrapped lines exist.
    return min(max_px, base_px + ((max_lines - 1) * line_px))


def _extract_llm_recommendations(output_text: str, param_names: list[str] | None = None) -> dict[str, dict[str, str]]:
    if not output_text:
        return {}

    recommendations: dict[str, dict[str, str]] = {}

    def _set_recommendation(name: str, value_text: str, reason_text: str = ""):
        existing = recommendations.get(name)
        if existing is not None:
            # Keep first parsed value, but fill missing reason when we later discover one.
            if reason_text and not str(existing.get("reason") or "").strip():
                existing["reason"] = reason_text
            return
        recommendations[name] = {
            "value": str(value_text),
            "reason": str(reason_text or "").strip(),
        }

    # Prefer structured JSON when available.
    json_payloads = [
        m.group(1).strip()
        for m in re.finditer(r"```json\s*([\s\S]*?)\s*```", output_text, flags=re.IGNORECASE)
    ]

    # Also support unfenced JSON payloads (for example when model returns plain JSON text).
    stripped_text = output_text.strip()
    if stripped_text.startswith("{") or stripped_text.startswith("["):
        json_payloads.append(stripped_text)

    key_idx = output_text.find('"parameter_recommendations"')
    if key_idx != -1:
        start = output_text.rfind("{", 0, key_idx)
        end = output_text.rfind("}")
        if start != -1 and end != -1 and end > start:
            json_payloads.append(output_text[start:end + 1].strip())

    # Preserve order and avoid re-parsing identical payloads.
    json_payloads = list(dict.fromkeys(json_payloads))

    for json_payload in json_payloads:
        try:
            data = json.loads(json_payload)
        except Exception:
            continue

        items = []
        if isinstance(data, dict):
            recs = data.get("parameter_recommendations")
            if isinstance(recs, list):
                items.extend(recs)
            else:
                # Accept a direct mapping: {"ATC_STR_RAT_P": 0.12, ...}
                for k, v in data.items():
                    if isinstance(k, str) and isinstance(v, (int, float, str)):
                        items.append({"name": k, "value": v})
        elif isinstance(data, list):
            items.extend(data)

        for item in items:
            if isinstance(item, dict):
                name = str(item.get("name") or item.get("param") or "").strip().upper()
                value = item.get("value", item.get("suggested_value"))
                reason = str(item.get("reason") or item.get("rationale") or "").strip()
                if not name:
                    continue
                value_num = _coerce_float(value, None)
                if value_num is None:
                    continue
                if _looks_like_ardupilot_param(name):
                    _set_recommendation(name, str(value_num), reason)

    candidate_names = list(param_names or [])
    for pname in candidate_names:
        escaped = re.escape(pname)
        patterns = [
            rf"\b{escaped}\b\s*[:=]\s*(-?\d+(?:\.\d+)?)",
            rf"\b{escaped}\b\s+(?:to|->|=>)\s*(-?\d+(?:\.\d+)?)",
        ]
        for pattern in patterns:
            match = re.search(pattern, output_text, flags=re.IGNORECASE)
            if match:
                _set_recommendation(pname, match.group(1), "")
                break

    # Fallback: find generic ARDUPILOT_PARAM=value style lines.
    generic_patterns = [
        r"\b([A-Z][A-Z0-9_]*_[A-Z0-9_]+)\b\s*[:=]\s*(-?\d+(?:\.\d+)?)",
        r"\b([A-Z][A-Z0-9_]*_[A-Z0-9_]+)\b\s+(?:to|->|=>)\s*(-?\d+(?:\.\d+)?)",
    ]
    for pattern in generic_patterns:
        for name, val in re.findall(pattern, output_text, flags=re.IGNORECASE):
            upper_name = str(name).upper()
            if _looks_like_ardupilot_param(upper_name):
                _set_recommendation(upper_name, str(_coerce_float(val, 0.0)), "")

    return recommendations


def _submit_to_openrouter(csv_text: str, model: str, param_snapshot_text: str = "", prompt_text: str = "") -> dict:
    api_key = (os.getenv("OPENROUTER_API_KEY", "") or "").strip()
    if not api_key:
        raise RuntimeError("OPENROUTER_API_KEY is not configured")

    base_url = (os.getenv("OPENROUTER_BASE_URL", "https://openrouter.ai/api/v1") or "").strip().rstrip("/")
    url = f"{base_url}/chat/completions"

    prompt = prompt_text or _build_llm_prompt({"metrics": []})
    payload = {
        "model": model,
        "messages": [
            {"role": "system", "content": "You are a controls engineer focused on rover tuning."},
            {
                "role": "user",
                "content": (
                    f"{prompt}\n"
                    "Current flight-controller parameter values for this run:\n"
                    f"{param_snapshot_text}\n\n"
                    "CSV data follows. Use exact column names in your analysis.\n\n"
                    f"{csv_text}"
                ),
            },
        ],
        "temperature": 0.2,
    }

    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json",
    }

    max_attempts = int(os.getenv("OPENROUTER_MAX_RETRIES", "3") or 3)
    max_attempts = max(1, min(max_attempts, 6))
    resp = None
    for attempt in range(1, max_attempts + 1):
        resp = requests.post(url, headers=headers, json=payload, timeout=60)
        if resp.status_code < 500 and resp.status_code != 429:
            break
        if attempt < max_attempts:
            backoff_s = min(8.0, 1.5 * (2 ** (attempt - 1)))
            time.sleep(backoff_s)

    if resp is None:
        raise RuntimeError("OpenRouter request failed before response creation")
    if resp.status_code >= 400:
        raise RuntimeError(f"OpenRouter request failed ({resp.status_code}): {resp.text[:300]}")

    raw_body = resp.text or ""
    try:
        data = resp.json()
    except Exception:
        # Defensive fallback for non-JSON provider responses.
        data = {}

    if not isinstance(data, dict):
        data = {}

    if not data and raw_body:
        return {
            "output_text": raw_body[:20000],
            "reasoning_text": "",
            "refusal_text": "",
            "status_code": int(resp.status_code),
            "response_id": None,
            "response_model": model,
            "finish_reason": "raw_body_fallback",
            "usage": {},
            "raw_preview": raw_body[:1000],
        }

    choices = data.get("choices") or []
    if not choices:
        if raw_body:
            return {
                "output_text": raw_body[:20000],
                "reasoning_text": "",
                "refusal_text": "",
                "status_code": int(resp.status_code),
                "response_id": data.get("id"),
                "response_model": data.get("model") or model,
                "finish_reason": "no_choices_raw_body_fallback",
                "usage": data.get("usage") if isinstance(data.get("usage"), dict) else {},
                "raw_preview": raw_body[:1000],
            }
        raise RuntimeError("OpenRouter returned no choices")

    first = choices[0] if isinstance(choices[0], dict) else {}
    message = first.get("message") if isinstance(first, dict) else {}
    if not isinstance(message, dict):
        message = {}

    content_val = message.get("content")
    text_out = ""
    if isinstance(content_val, str):
        text_out = content_val.strip()
    elif isinstance(content_val, list):
        # Some providers return content as message parts.
        chunks = []
        for part in content_val:
            if isinstance(part, dict):
                txt = part.get("text")
                if isinstance(txt, str) and txt.strip():
                    chunks.append(txt.strip())
        text_out = "\n\n".join(chunks).strip()

    reasoning_text = ""
    reasoning_val = message.get("reasoning")
    if isinstance(reasoning_val, str):
        reasoning_text = reasoning_val.strip()

    if not reasoning_text:
        details = message.get("reasoning_details")
        if isinstance(details, list):
            detail_bits = []
            for item in details:
                if isinstance(item, dict):
                    txt = item.get("text")
                    if isinstance(txt, str) and txt.strip():
                        detail_bits.append(txt.strip())
            if detail_bits:
                reasoning_text = "\n\n".join(detail_bits)

    refusal_text = ""
    refusal_val = message.get("refusal")
    if isinstance(refusal_val, str):
        refusal_text = refusal_val.strip()

    if not text_out and reasoning_text:
        text_out = reasoning_text

    if not text_out and refusal_text:
        text_out = f"Model refusal: {refusal_text}"

    if not text_out and message:
        # Last-resort fallback to preserve visibility.
        text_out = str(message)

    if not text_out:
        raise RuntimeError("OpenRouter returned no usable content in content/reasoning/refusal fields")

    return {
        "output_text": text_out,
        "reasoning_text": reasoning_text,
        "refusal_text": refusal_text,
        "status_code": int(resp.status_code),
        "response_id": data.get("id"),
        "response_model": data.get("model"),
        "finish_reason": first.get("finish_reason") if isinstance(first, dict) else None,
        "usage": data.get("usage") if isinstance(data.get("usage"), dict) else {},
        "raw_preview": raw_body[:1000],
    }


st.markdown(
    """
    <style>
    .block-container {
        padding-top: 0.75rem;
    }
    div[data-testid="stVerticalBlockBorderWrapper"] {
        margin-top: 0 !important;
    }
    div[data-testid="stElementContainer"]:has(> div[data-testid="stToggle"]) {
        margin-bottom: 0 !important;
    }
    .pid-recording-title-row {
        display: flex;
        align-items: center;
        gap: 0.5rem;
    }
    .pid-recording-title-row h3 {
        margin: 0;
    }
    .pid-live-title-row {
        display: flex;
        align-items: center;
        gap: 0.5rem;
    }
    .pid-live-title-row h3 {
        margin: 0;
    }
    .pid-recording-title-row h3 .pid-recording-dot {
        display: inline-block;
        width: 0.55em;
        height: 0.55em;
        margin-left: 0.4em;
        border-radius: 50%;
        background: #d32f2f;
        box-shadow: 0 0 0 rgba(211, 47, 47, 0.55);
        animation: pidPulse 1.8s ease-in-out infinite;
        vertical-align: middle;
    }
    .pid-live-title-row h3 .pid-live-dot {
        display: inline-block;
        width: 0.55em;
        height: 0.55em;
        margin-left: 0.4em;
        border-radius: 50%;
        vertical-align: middle;
    }
    .pid-live-title-row h3 .pid-live-dot.live {
        background: #2e7d32;
        box-shadow: 0 0 0 rgba(46, 125, 50, 0.5);
        animation: pidLivePulse 1.8s ease-in-out infinite;
    }
    .pid-live-title-row h3 .pid-live-dot.stale {
        background: #d32f2f;
        box-shadow: none;
        animation: none;
    }
    div[data-testid="stVerticalBlockBorderWrapper"]:has(.pid-live-title-row) [data-testid="stMetricValue"] {
        font-size: 1.5rem;
    }
    @keyframes pidPulse {
        0% {
            transform: scale(0.85);
            box-shadow: 0 0 0 0 rgba(211, 47, 47, 0.55);
        }
        70% {
            transform: scale(1.0);
            box-shadow: 0 0 0 10px rgba(211, 47, 47, 0.0);
        }
        100% {
            transform: scale(0.85);
            box-shadow: 0 0 0 0 rgba(211, 47, 47, 0.0);
        }
    }
    @keyframes pidLivePulse {
        0% {
            transform: scale(0.85);
            box-shadow: 0 0 0 0 rgba(46, 125, 50, 0.5);
        }
        70% {
            transform: scale(1.0);
            box-shadow: 0 0 0 10px rgba(46, 125, 50, 0.0);
        }
        100% {
            transform: scale(0.85);
            box-shadow: 0 0 0 0 rgba(46, 125, 50, 0.0);
        }
    }
    </style>
    """,
    unsafe_allow_html=True,
)

st.title(":material/tune: PID Tuning")
st.caption("Capture desired vs achieved telemetry, tune rover parameters, and submit session CSV to an LLM.")

if "pid_llm_last_status" not in st.session_state:
    st.session_state["pid_llm_last_status"] = "idle"
if "pid_llm_last_error" not in st.session_state:
    st.session_state["pid_llm_last_error"] = ""
if "pid_llm_status_ts" not in st.session_state:
    st.session_state["pid_llm_status_ts"] = 0.0
if "pid_llm_submit_requested" not in st.session_state:
    st.session_state["pid_llm_submit_requested"] = False

llm_is_running = str(st.session_state.get("pid_llm_last_status") or "") == "running"
llm_submit_armed = bool(st.session_state.get("pid_llm_submit_requested", False))
ensure_runtime_started()

state = get_shared_state()
snap = state.get()
rec_active_global = bool(snap.get("pid_recording_active"))

# Keep stream-status indicators current while this page is open.
# Use faster refresh while recording, slower refresh while idle.
if not llm_is_running and not llm_submit_armed:
    refresh_interval_ms = 1000 if rec_active_global else 2000
    st_autorefresh(interval=refresh_interval_ms, key="pid_tuning_refresh")

if not snap.get("link_active"):
    st.warning("Telemetry link appears offline. Open Dashboard first to initialize MAVLink worker if needed.")

with st.container(border=True):
    now_s = time.time()
    pid_last_tuning_msg_ts = float(snap.get("pid_last_tuning_msg_ts") or 0.0)
    pid_streaming_live = bool(snap.get("link_active")) and pid_last_tuning_msg_ts > 0.0 and (now_s - pid_last_tuning_msg_ts) <= 3.0
    live_dot_state = "live" if pid_streaming_live else "stale"
    live_dot_label = "PID stream live" if pid_streaming_live else "PID stream not live"
    st.markdown(
        (
            '<div class="pid-live-title-row"><h3>Live PID Signals'
            f'<span class="pid-live-dot {live_dot_state}" aria-label="{live_dot_label}"></span>'
            "</h3></div>"
        ),
        unsafe_allow_html=True,
    )

    signal_source = str(snap.get("pid_signal_source") or "unknown")

    cache_for_mask = dict((snap.get("param_cache") or {}))
    mask_info = _resolve_pid_mask_from_cache(cache_for_mask)
    mask_raw = mask_info.get("raw")
    if mask_info.get("known"):
        st.caption(f"GCS_PID_MASK={mask_raw} | active metrics: {mask_info.get('label')}")
    else:
        st.warning(
            "GCS_PID_MASK is unavailable. Fetch parameters first so the PID page can show, record, and submit only valid metric types."
        )

    now_vals = {
        "steering_desired": snap.get("pid_steering_desired"),
        "steering_achieved": snap.get("pid_steering_achieved"),
        "speed_desired": snap.get("pid_speed_desired"),
        "speed_achieved": snap.get("pid_speed_achieved"),
    }

    if mask_info.get("steering") and mask_info.get("speed"):
        c1, c2, c3, c4 = st.columns(4)
        c1.metric("Steer Desired", f"{_coerce_float(now_vals['steering_desired']):.3f}")
        c2.metric("Steer Achieved", f"{_coerce_float(now_vals['steering_achieved']):.3f}")
        c3.metric("Speed Desired", f"{_coerce_float(now_vals['speed_desired']):.3f}")
        c4.metric("Speed Achieved", f"{_coerce_float(now_vals['speed_achieved']):.3f}")
    elif mask_info.get("steering"):
        c1, c2 = st.columns(2)
        c1.metric("Steer Desired", f"{_coerce_float(now_vals['steering_desired']):.3f}")
        c2.metric("Steer Achieved", f"{_coerce_float(now_vals['steering_achieved']):.3f}")
    elif mask_info.get("speed"):
        c1, c2 = st.columns(2)
        c1.metric("Speed Desired", f"{_coerce_float(now_vals['speed_desired']):.3f}")
        c2.metric("Speed Achieved", f"{_coerce_float(now_vals['speed_achieved']):.3f}")

    steer_desired_raw = now_vals.get("steering_desired")
    steer_achieved_raw = now_vals.get("steering_achieved")
    if steer_desired_raw is None:
        st.warning(
            "Steering desired is unavailable from current telemetry. "
            "Recordings and LLM analysis may be less reliable until desired steering is present."
        )
    elif signal_source == "proxy":
        st.warning(
            "Steering desired is synthetic (proxy-derived), not a direct flight-controller desired signal. "
            "Use caution when interpreting steering tuning conclusions."
        )

    samples = list(snap.get("pid_samples") or [])
    df = _to_samples_dataframe(samples)
    filtered_df = _filter_dataframe_for_mask(df, mask_info)

    if not filtered_df.empty:
        steering_cols = [c for c in ["steering_desired", "steering_achieved", "steering_error"] if c in filtered_df.columns]
        speed_cols = [c for c in ["speed_desired", "speed_achieved", "speed_error"] if c in filtered_df.columns]

        if mask_info.get("steering") and steering_cols:
            steer_non_null = int(filtered_df[steering_cols].notna().sum().sum())
            if steer_non_null == 0:
                st.warning(
                    "Steering metrics are enabled by GCS_PID_MASK, but captured steering desired/achieved values are empty. "
                    "Confirm PID_TUNING stream and vehicle steering activity."
                )
        if mask_info.get("speed") and speed_cols:
            speed_non_null = int(filtered_df[speed_cols].notna().sum().sum())
            if speed_non_null == 0:
                st.warning(
                    "Speed metrics are enabled by GCS_PID_MASK, but captured speed desired/achieved values are empty. "
                    "Confirm NAV_CONTROLLER_OUTPUT/speed telemetry is present."
                )

        if mask_info.get("steering") and steering_cols:
            steering_df = filtered_df[["timestamp"] + steering_cols].melt(
                id_vars=["timestamp"],
                value_vars=steering_cols,
                var_name="series",
                value_name="value",
            )
            steering_df["line_style"] = steering_df["series"].apply(
                lambda s: "dashed" if s == "steering_error" else "solid"
            )
            steering_color = alt.Scale(
                domain=["steering_achieved", "steering_desired", "steering_error"],
                range=["#1e88e5", "#2e7d32", "#e53935"],
            )
            steering_chart = (
                alt.Chart(steering_df)
                .mark_line()
                .encode(
                    x=alt.X("timestamp:T", title="Time"),
                    y=alt.Y("value:Q", title="Steering"),
                    color=alt.Color("series:N", scale=steering_color, title="Signal"),
                    strokeDash=alt.StrokeDash(
                        "line_style:N",
                        scale=alt.Scale(domain=["solid", "dashed"], range=[[1, 0], [8, 6]]),
                        legend=None,
                    ),
                )
                .properties(height=220)
            )
            st.altair_chart(steering_chart, use_container_width=True)
        if mask_info.get("speed") and speed_cols:
            speed_df = filtered_df[["timestamp"] + speed_cols].melt(
                id_vars=["timestamp"],
                value_vars=speed_cols,
                var_name="series",
                value_name="value",
            )
            speed_df["line_style"] = speed_df["series"].apply(
                lambda s: "dashed" if s == "speed_error" else "solid"
            )
            speed_color = alt.Scale(
                domain=["speed_achieved", "speed_desired", "speed_error"],
                range=["#1e88e5", "#2e7d32", "#e53935"],
            )
            speed_chart = (
                alt.Chart(speed_df)
                .mark_line()
                .encode(
                    x=alt.X("timestamp:T", title="Time"),
                    y=alt.Y("value:Q", title="Speed"),
                    color=alt.Color("series:N", scale=speed_color, title="Signal"),
                    strokeDash=alt.StrokeDash(
                        "line_style:N",
                        scale=alt.Scale(domain=["solid", "dashed"], range=[[1, 0], [8, 6]]),
                        legend=None,
                    ),
                )
                .properties(height=220)
            )
            st.altair_chart(speed_chart, use_container_width=True)
    else:
        st.info("No PID samples recorded yet.")

    rec_active = rec_active_global
    recording_dot_html = '<span class="pid-recording-dot" aria-label="recording active"></span>' if rec_active else ""
    st.markdown(
        f'<div class="pid-recording-title-row"><h3>Recording{recording_dot_html}</h3></div>',
        unsafe_allow_html=True,
    )
    rec_label = "Recording" if rec_active else "Stopped"
    st.write(f"State: {rec_label}")

    params_loaded, loaded_count, required_total = _params_loaded_for_mask(cache_for_mask, mask_info)
    link_active = bool(snap.get("link_active"))
    can_start_recording = bool(pid_streaming_live and link_active)
    if not params_loaded:
        if required_total == 0:
            st.caption("Load rover parameters (including GCS_PID_MASK) to improve metric validation and LLM output quality.")
        else:
            st.caption(
                "Load rover parameters (use Fetch All Params) to improve metric validation and LLM output quality "
                f"({loaded_count}/{required_total} required params loaded)."
            )
    if not link_active:
        st.caption("Telemetry link must be active to enable recording.")
    if link_active and not pid_streaming_live:
        st.caption("PID_TUNING stream must be live (green Live PID Signals dot) to enable recording.")

    rc1, rc2, rc3 = st.columns(3)
    if rc1.button("Start Recording", use_container_width=True, disabled=((not can_start_recording) or rec_active)):
        state.update({
            "pid_recording_active": True,
            "pid_recording_started_ts": time.time(),
            "pid_recording_stopped_ts": 0.0,
            "pid_samples": [],
        })
        st.rerun()

    if rc2.button("Stop Recording", use_container_width=True, disabled=(not rec_active)):
        state.update({
            "pid_recording_active": False,
            "pid_recording_stopped_ts": time.time(),
        })
        st.rerun()

    if rc3.button("Clear Samples", use_container_width=True):
        state.update({"pid_samples": []})
        st.rerun()

    if not filtered_df.empty:
        with st.expander("Session Summary", expanded=False):
            summary = _build_session_summary(filtered_df)
            if summary.get("max_speed_seen") is None:
                if "speed_ms_raw" in df.columns:
                    raw_speed = pd.to_numeric(df["speed_ms_raw"], errors="coerce").dropna()
                    if not raw_speed.empty:
                        summary["max_speed_seen"] = float(raw_speed.max())
                elif "speed_ms" in df.columns:
                    raw_speed = pd.to_numeric(df["speed_ms"], errors="coerce").dropna()
                    if not raw_speed.empty:
                        summary["max_speed_seen"] = float(raw_speed.max())
                elif "speed_achieved" in df.columns:
                    raw_speed = pd.to_numeric(df["speed_achieved"], errors="coerce").dropna()
                    if not raw_speed.empty:
                        summary["max_speed_seen"] = float(raw_speed.max())
            sm1, sm2, sm3 = st.columns(3)
            sm1.metric("Samples", f"{summary['sample_count']}")
            sm2.metric("Duration", f"{summary['duration_s']:.1f}s")
            sm3.metric("Steer MAE", "N/A" if summary["steer_mae"] is None else f"{summary['steer_mae']:.3f}")

            sm4, sm5, sm6, sm7 = st.columns(4)
            sm4.metric("Speed MAE", "N/A" if summary["speed_mae"] is None else f"{summary['speed_mae']:.3f}")
            sm5.metric(
                "Steer Max |Err|",
                "N/A" if summary["steer_max_abs_err"] is None else f"{summary['steer_max_abs_err']:.3f}",
            )
            sm6.metric(
                "Speed Max |Err|",
                "N/A" if summary["speed_max_abs_err"] is None else f"{summary['speed_max_abs_err']:.3f}",
            )
            sm7.metric(
                "Max Speed Seen (m/s)",
                "N/A" if summary["max_speed_seen"] is None else f"{summary['max_speed_seen']:.3f}",
            )

            csv_bytes = filtered_df.to_csv(index=False).encode("utf-8")
            ts_tag = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            st.download_button(
                "Download Session CSV",
                data=csv_bytes,
                file_name=f"pid_session_{ts_tag}.csv",
                mime="text/csv",
                use_container_width=True,
            )

with st.container(border=True):
    st.subheader("Rover Parameters")

    if st.button("Fetch All Params", use_container_width=True):
        state.param_op_queue.put({"action": "request_list"})
        st.toast("Parameter list request sent")

    all_params = _flatten_params()

    cache = dict((state.get().get("param_cache") or {}))

    llm_output_text = str(st.session_state.get("pid_llm_result") or "")
    llm_reasoning_text = str(st.session_state.get("pid_llm_reasoning") or "")
    llm_meta_state = st.session_state.get("pid_llm_meta") or {}
    llm_recommendations = _extract_llm_recommendations(llm_output_text, list(all_params.keys()))
    llm_refresh_signature = (
        json.dumps(llm_recommendations, sort_keys=True, default=str),
        llm_reasoning_text,
        json.dumps(llm_meta_state, sort_keys=True, default=str),
    )
    prev_llm_refresh_signature = st.session_state.get("pid_param_table_llm_signature")
    if prev_llm_refresh_signature != llm_refresh_signature:
        if "pid_param_table_editor" in st.session_state:
            del st.session_state["pid_param_table_editor"]
        st.session_state["pid_param_table_llm_signature"] = llm_refresh_signature

    rows = []
    visible_groups = _filter_param_groups_for_mask(mask_info)
    if not visible_groups:
        st.info("No parameter groups are enabled by current GCS_PID_MASK.")

    row_param_names = set()
    for group_name, params in visible_groups.items():
        for pname in params.keys():
            current = cache.get(pname, {})
            row_param_names.add(pname)
            rows.append(
                {
                    "": False,
                    "group": group_name,
                    "param": pname,
                    "value": current.get("value"),
                    "LLM": (llm_recommendations.get(pname, {}) or {}).get("value", ""),
                    "Reason": _wrap_reason_text((llm_recommendations.get(pname, {}) or {}).get("reason", "")),
                    "updated": current.get("updated_ts"),
                }
            )

    # Include LLM-suggested params even when they are outside the current mask-filtered groups.
    for pname, rec_data in llm_recommendations.items():
        if pname in row_param_names:
            continue
        current = cache.get(pname, {})
        rows.append(
            {
                "": False,
                "group": "LLM Suggested",
                "param": pname,
                "value": current.get("value") if isinstance(current, dict) else None,
                "LLM": (rec_data or {}).get("value", ""),
                "Reason": _wrap_reason_text((rec_data or {}).get("reason", "")),
                "updated": (current.get("updated_ts") if isinstance(current, dict) else None),
            }
        )

    table_df = pd.DataFrame(rows)
    edited_table_df = pd.DataFrame()
    if not table_df.empty:
        if "updated" in table_df.columns:
            updated_dt = pd.to_datetime(table_df["updated"], unit="s", errors="coerce")
            table_df["updated"] = updated_dt.dt.strftime("%H:%M:%S").fillna("")
        reason_values = table_df["Reason"].tolist() if "Reason" in table_df.columns else []
        adaptive_row_height = _editor_row_height_for_reasons(reason_values)
        edited_table_df = st.data_editor(
            table_df,
            width="stretch",
            height=280,
            row_height=adaptive_row_height,
            hide_index=True,
            disabled=["group", "param", "value", "LLM", "Reason", "updated"],
            column_config={
                "": st.column_config.CheckboxColumn("", default=False),
                "LLM": st.column_config.TextColumn(
                    "LLM",
                    help="LLM-recommended numeric value",
                    width="small",
                ),
                "Reason": st.column_config.TextColumn(
                    "Reason",
                    help="LLM rationale for the recommendation",
                    width="large",
                ),
            },
            key="pid_param_table_editor",
        )
        if llm_output_text and not llm_recommendations:
            st.caption("No explicit PARAM=VALUE recommendations were detected in the current LLM output.")

        if st.button("Use Selected LLM Recommendations", use_container_width=True):
            if bool(state.get().get("armed")):
                st.error("Parameter writes are blocked while rover is armed")
            else:
                selected_df = edited_table_df[edited_table_df[""] == True]
                if selected_df.empty:
                    st.warning("Select at least one checked row to apply LLM recommendations.")
                else:
                    queued = 0
                    failures = []
                    for _, row in selected_df.iterrows():
                        target_param = str(row.get("param") or "").strip()
                        rec_text = str(row.get("LLM") or "").strip()
                        rec_value = _coerce_float(rec_text, None)
                        if not target_param:
                            failures.append("A selected row has no parameter name")
                            continue
                        if rec_value is None:
                            failures.append(f"{target_param}: LLM value is not numeric")
                            continue

                        entry = cache.get(target_param, {}) if isinstance(cache, dict) else {}
                        param_type = int(entry.get("type", 9)) if isinstance(entry, dict) else 9
                        state.param_op_queue.put(
                            {
                                "action": "set_param",
                                "name": target_param,
                                "value": float(rec_value),
                                "param_type": param_type,
                            }
                        )
                        queued += 1

                    if queued > 0:
                        st.success(f"Queued {queued} LLM parameter write(s) to flight controller.")
                    if failures:
                        st.error("Failed to queue some selected rows: " + "; ".join(failures[:6]))

    op_status = state.get().get("param_last_op") or {}
    if op_status:
        st.caption(f"Param op status: {op_status.get('status', 'idle')} - {op_status.get('message', '')}")

st.divider()
st.subheader("LLM Analysis")

default_prompt = _build_llm_prompt(mask_info)
with st.expander("Default Prompt Used", expanded=False):
    st.text_area("Prompt", value=default_prompt, height=180, disabled=True)

llm_model = st.text_input(
    "OpenRouter Model",
    value=(os.getenv("OPENROUTER_MODEL", "openai/gpt-4o-mini") or "openai/gpt-4o-mini"),
    help="OpenRouter model ID using OpenAI-compatible chat completions",
)

if "pid_llm_result" not in st.session_state:
    st.session_state["pid_llm_result"] = ""
if "pid_llm_reasoning" not in st.session_state:
    st.session_state["pid_llm_reasoning"] = ""
if "pid_llm_steps" not in st.session_state:
    st.session_state["pid_llm_steps"] = []
if "pid_llm_meta" not in st.session_state:
    st.session_state["pid_llm_meta"] = {}
if "pid_llm_runs" not in st.session_state:
    st.session_state["pid_llm_runs"] = []
# Auto-recover from stale in-progress states after a timeout window.
status_timeout_s = float(os.getenv("PID_LLM_STATUS_TIMEOUT_S", "180") or 180)
status_timeout_s = max(30.0, min(status_timeout_s, 1800.0))
running_age_s = 0.0
if str(st.session_state.get("pid_llm_last_status") or "") == "running":
    running_age_s = max(0.0, time.time() - float(st.session_state.get("pid_llm_status_ts") or 0.0))
    if running_age_s > status_timeout_s:
        st.session_state["pid_llm_last_status"] = "error"
        st.session_state["pid_llm_last_error"] = (
            f"Submission state timed out after {int(running_age_s)}s. "
            "You can retry safely."
        )
        st.session_state["pid_llm_steps"] = list(st.session_state.get("pid_llm_steps") or []) + [
            f"Auto-reset stale running state after {int(running_age_s)}s"
        ]

last_status = str(st.session_state.get("pid_llm_last_status") or "idle")
if last_status == "success":
    st.success("Last submission completed successfully. Scroll down to LLM Output or LLM Run History.")
elif last_status == "error":
    last_error = str(st.session_state.get("pid_llm_last_error") or "Unknown error")
    st.error(f"Last submission failed: {last_error}")
elif last_status == "running":
    st.info(f"Submission in progress... ({int(running_age_s)}s elapsed)")

    if st.button("Reset Stuck Submission State", use_container_width=False):
        st.session_state["pid_llm_last_status"] = "idle"
        st.session_state["pid_llm_last_error"] = ""
        st.session_state["pid_llm_status_ts"] = 0.0
        st.rerun()

submit_disabled = filtered_df.empty
if str(st.session_state.get("pid_llm_last_status") or "") == "running":
    submit_disabled = True

submit_clicked = st.button("Submit Data to AI Model", disabled=submit_disabled, use_container_width=True)
if submit_clicked:
    # Clear previous LLM output and reset parameter-table recommendation UI for a fresh run.
    st.session_state["pid_llm_result"] = ""
    st.session_state["pid_llm_reasoning"] = ""
    st.session_state["pid_llm_steps"] = []
    st.session_state["pid_llm_meta"] = {}
    if "pid_param_table_editor" in st.session_state:
        del st.session_state["pid_param_table_editor"]

    # Phase 1: arm submit and rerun with refresh disabled.
    st.session_state["pid_llm_submit_requested"] = True
    st.rerun()

if bool(st.session_state.get("pid_llm_submit_requested", False)):
    # Phase 2: execute submit in a stable run.
    st.session_state["pid_llm_submit_requested"] = False
    if filtered_df.empty:
        st.error("No recorded samples available")
    else:
        st.session_state["pid_llm_last_status"] = "running"
        st.session_state["pid_llm_last_error"] = ""
        st.session_state["pid_llm_status_ts"] = time.time()
        steps = []
        steps.append(
            f"Prepared to submit {len(filtered_df)} recorded rows for metric scope: {mask_info.get('label')}"
        )
        param_cache = dict((state.get().get("param_cache") or {}))
        param_names = list(_flatten_params().keys())
        param_snapshot_text = _build_param_snapshot_text(param_cache, param_names)
        present_count = sum(
            1 for pname in param_names
            if isinstance(param_cache.get(pname), dict) and (param_cache.get(pname) or {}).get("value") is not None
        )
        steps.append(f"Parameter snapshot prepared: {present_count}/{len(param_names)} values present")
        csv_text, prep_note = _prepare_csv_for_llm(filtered_df)
        steps.append(f"CSV payload prepared with {len(csv_text)} characters")
        if prep_note:
            steps.append(prep_note)
        try:
            call_started = time.time()
            with st.spinner("Submitting CSV to OpenRouter..."):
                llm_resp = _submit_to_openrouter(
                    csv_text=csv_text,
                    model=llm_model,
                    param_snapshot_text=param_snapshot_text,
                    prompt_text=default_prompt,
                )
            elapsed_s = max(0.0, time.time() - call_started)
            steps.append(f"OpenRouter responded in {elapsed_s:.2f}s")
            steps.append(
                f"Parsed response id={llm_resp.get('response_id')} finish_reason={llm_resp.get('finish_reason')}"
            )

            st.session_state["pid_llm_result"] = llm_resp.get("output_text", "")
            st.session_state["pid_llm_reasoning"] = llm_resp.get("reasoning_text", "")
            st.session_state["pid_llm_steps"] = steps
            st.session_state["pid_llm_meta"] = {
                "status_code": llm_resp.get("status_code"),
                "response_model": llm_resp.get("response_model"),
                "finish_reason": llm_resp.get("finish_reason"),
                "usage": llm_resp.get("usage") or {},
                "raw_preview": llm_resp.get("raw_preview") or "",
            }
            steps.append(
                f"Usable output length={len(str(st.session_state['pid_llm_result'] or ''))} characters"
            )
            st.session_state["pid_llm_last_status"] = "success"
            st.session_state["pid_llm_last_error"] = ""
            st.session_state["pid_llm_status_ts"] = time.time()

            run_record = {
                "ts": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "model": llm_model,
                "prompt": default_prompt,
                "steps": list(steps),
                "output": st.session_state["pid_llm_result"],
                "reasoning": st.session_state["pid_llm_reasoning"],
                "meta": dict(st.session_state["pid_llm_meta"]),
            }
            history = list(st.session_state.get("pid_llm_runs") or [])
            history.insert(0, run_record)
            st.session_state["pid_llm_runs"] = history[:10]

            # Re-render from top so Rover Parameters picks up fresh LLM suggestions immediately.
            st.rerun()
        except Exception as llm_err:
            err_text = str(llm_err)
            fail_steps = steps + [f"Submission failed: {err_text}"]
            st.session_state["pid_llm_steps"] = fail_steps
            st.session_state["pid_llm_last_status"] = "error"
            st.session_state["pid_llm_last_error"] = err_text
            st.session_state["pid_llm_status_ts"] = time.time()

            fail_record = {
                "ts": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "model": llm_model,
                "prompt": default_prompt,
                "steps": list(fail_steps),
                "output": "",
                "reasoning": "",
                "meta": {
                    "status_code": None,
                    "response_model": llm_model,
                    "finish_reason": "error",
                    "usage": {},
                },
            }
            history = list(st.session_state.get("pid_llm_runs") or [])
            history.insert(0, fail_record)
            st.session_state["pid_llm_runs"] = history[:10]
            st.error(f"LLM request failed: {llm_err}")

result_text = st.session_state.get("pid_llm_result") or ""
result_reasoning = st.session_state.get("pid_llm_reasoning") or ""
result_steps = st.session_state.get("pid_llm_steps") or []
result_meta = st.session_state.get("pid_llm_meta") or {}

if result_steps:
    st.markdown("### LLM Execution Steps")
    for idx, item in enumerate(result_steps, start=1):
        st.write(f"{idx}. {item}")

if result_meta:
    usage = result_meta.get("usage") or {}
    st.caption(
        "Response metadata: "
        f"status={result_meta.get('status_code')} "
        f"model={result_meta.get('response_model')} "
        f"finish_reason={result_meta.get('finish_reason')} "
        f"prompt_tokens={usage.get('prompt_tokens', 'n/a')} "
        f"completion_tokens={usage.get('completion_tokens', 'n/a')}"
    )

if result_reasoning:
    with st.expander("LLM Steps (Model Reasoning Field)", expanded=False):
        st.text_area("Reasoning", value=result_reasoning, height=220, disabled=True)

if result_text:
    st.markdown("### LLM Output")
    st.text_area("LLM Output (plain text)", value=result_text, height=220, disabled=True)
    st.markdown(result_text)
    st.download_button(
        "Download Analysis Markdown",
        data=result_text.encode("utf-8"),
        file_name=f"pid_analysis_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.md",
        mime="text/markdown",
        use_container_width=True,
    )

run_history = list(st.session_state.get("pid_llm_runs") or [])
if run_history:
    st.markdown("### LLM Run History")
    if st.button("Clear LLM History", use_container_width=False):
        st.session_state["pid_llm_runs"] = []
        st.rerun()

    for idx, run in enumerate(run_history, start=1):
        title = f"Run {idx} - {run.get('ts', 'unknown time')} - {run.get('model', 'unknown model')}"
        with st.expander(title, expanded=(idx == 1)):
            st.markdown("#### Prompt Used")
            st.text_area(
                f"Prompt {idx}",
                value=str(run.get("prompt") or ""),
                height=120,
                disabled=True,
                key=f"llm_prompt_{idx}",
            )

            st.markdown("#### Steps")
            for step_i, step_text in enumerate(run.get("steps") or [], start=1):
                st.write(f"{step_i}. {step_text}")

            run_meta = run.get("meta") or {}
            run_usage = run_meta.get("usage") or {}
            st.caption(
                "Metadata: "
                f"status={run_meta.get('status_code')} "
                f"finish_reason={run_meta.get('finish_reason')} "
                f"prompt_tokens={run_usage.get('prompt_tokens', 'n/a')} "
                f"completion_tokens={run_usage.get('completion_tokens', 'n/a')}"
            )

            reasoning = str(run.get("reasoning") or "").strip()
            if reasoning:
                st.markdown("#### Reasoning")
                st.text_area(
                    f"Reasoning {idx}",
                    value=reasoning,
                    height=160,
                    disabled=True,
                    key=f"llm_reasoning_{idx}",
                )

            st.markdown("#### Output")
            st.markdown(str(run.get("output") or ""))
