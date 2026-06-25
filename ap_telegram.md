# AP Telegram Backend

## Purpose

This document describes the latest Telegram implementation in this repository for the purpose of extracting or rebuilding it as a standalone Telegram bot backend that can run in Docker.

The source of truth for this document is `mavinject.py`.

This is not a history document. It intentionally does not treat `main.py` or `mavmonitor.py` as the target implementation:

- `mavinject.py` is the standalone bot backend to model.
- `main.py` has Telegram logic removed and should not be used as the implementation reference.
- `mavmonitor.py` is a MAVProxy plugin with a different runtime model and should be treated as legacy for this effort.

## What The Backend Does

At a high level, the standalone Telegram backend in `mavinject.py` does five jobs:

1. Loads environment configuration.
2. Opens a single bidirectional MAVLink UDP connection.
3. Optionally publishes a temperature reading to MAVLink once per second.
4. Runs a Telegram bot interface for status and control commands.
5. Monitors incoming MAVLink traffic and converts selected vehicle events into Telegram notifications.

The implementation is a single Python process with multiple concurrent responsibilities:

- Main thread: starts the runtime and either runs Telegram polling or idles in HTTP-only mode.
- MAVLink monitor thread: consumes incoming MAVLink messages from the shared UDP connection and emits Telegram notifications.
- Temperature sender thread: optional background loop that emits `NAMED_VALUE_FLOAT` messages.
- Telegram polling loop: created by `python-telegram-bot` when polling is enabled.

## Source Of Truth

Primary implementation file:

- `mavinject.py`

Supporting context:

- `requirements.txt`: required Python packages.
- `config.py`: overlapping env var defaults used by other runtime code.
- `main.py`: confirms Telegram was removed from the alternate runtime.
- `mavmonitor.py`: older plugin-style Telegram implementation, not the target for Docker extraction.

## Runtime Architecture

### Startup Sequence

The standalone backend starts in module scope and then enters `main()`.

Execution order:

1. `load_dotenv()` loads local environment variables.
2. Global configuration values are read from `os.getenv(...)`.
3. A single bidirectional UDP MAVLink `master` connection is created via `mavutil.mavlink_connection(...)`.
4. If connection creation fails, startup should fail fast and surface a clear configuration/network error.
5. Optional temperature sensor support is initialized.
6. Global runtime state is initialized for Telegram, mode tracking, GPS state, rover telemetry, failsafe tracking, and recent status text history.
7. `main()` chooses one of three startup modes:
   - Telegram polling mode.
   - Telegram HTTP-only mode.
   - Telegram disabled mode.

### Threading Model

The current implementation is thread-based, not async-first.

- Telegram command handlers are async because `python-telegram-bot` v20+ uses async handlers.
- The MAVLink monitor is a plain daemon thread.
- The temperature sender is a plain daemon thread.
- Shared mutable state is stored in module-level globals.
- `chat_id` is protected by a `threading.Lock`; most other globals are not.

For a Docker-ready rewrite, this is the first area to improve. The current code works as a pragmatic script, but a standalone backend should isolate state more cleanly.

## Environment Configuration

The extracted backend needs to preserve the current environment surface unless intentionally simplified.

### Required For Telegram

#### `TELEGRAM_BOT_TOKEN`

- Purpose: Telegram bot authentication token.
- Default: none.
- Required for Telegram behavior: yes.
- Docker note: inject via environment variable or secret, not `.env` committed into an image.

#### `TELEGRAM_CHAT_ID`

- Purpose: default destination chat for proactive outbound messages.
- Default: none.
- Required for polling mode: not strictly, because the bot can learn the chat from inbound messages.
- Required for HTTP-only mode: effectively yes, because there is no inbound polling path to learn the chat dynamically.
- Docker note: set explicitly if you want notifications without waiting for a `/start` or other inbound command.

#### `TELEGRAM_DISABLE_POLLING`

- Purpose: disables `python-telegram-bot` polling and forces HTTP-only notification mode.
- Default: `0`.
- Behavior:
  - `0`: use polling if `TELEGRAM_BOT_TOKEN` is set.
  - `1`: do not start Telegram polling; only send outbound HTTP notifications.
- Docker note: useful when inbound bot commands are not needed, or when polling should be managed differently.

### Required For MAVLink Connectivity

#### `MAVLINK_UDP_ENDPOINT`

- Purpose: single bidirectional MAVLink UDP connection used for both monitoring and outbound commands.
- Default: `udp:127.0.0.1:14550`.
- Docker note: this endpoint must be reachable from inside the container network and support bidirectional MAVLink traffic.

#### `MAVLINK_NAME`

- Purpose: 10-character label for the published `NAMED_VALUE_FLOAT` temperature message.
- Default: `DS18F     `.
- Behavior: value is trimmed or padded to 10 bytes before transmission.
- Docker note: only relevant if the temperature sender remains enabled in the extracted backend.

### Notification And Monitoring Controls

#### `MAVLINK_THROTTLE_SECONDS`

- Purpose: throttle interval for forwarding `NAMED_VALUE_FLOAT` readings to Telegram.
- Default: `10`.

#### `EXTENDED_STATUS`

- Purpose: enables extra Telegram notifications for relay-related `COMMAND_LONG` traffic and `COMMAND_ACK` messages.
- Default: `0`.

#### `DEBUG`

- Purpose: enables debug logging via `debug_print(...)`.
- Default: `0`.

#### `TEMP_SENDER_ENABLED`

- Purpose: controls whether the process sends a synthetic or sensor-backed temperature reading to MAVLink every second.
- Default: `1`.
- Docker note: often disable this in a pure Telegram backend container unless temperature injection is part of the service contract.

#### `HEARTBEAT_PERIODIC`

- Purpose: when enabled, sends a silent Telegram heartbeat notification every 15 seconds from the monitor loop.
- Default: `0`.

#### `MODE_DEBOUNCE_SECONDS`

- Purpose: requires a new `custom_mode` to remain stable before broadcasting a mode change.
- Default: `2.0`.

#### `FAILSAFE_NOTIFY_LOUD`

- Purpose: controls whether failsafe alerts are sent with notifications enabled instead of silent delivery.
- Default: `1`.

#### `FORWARD_ALL_TEXT`

- Purpose: forwards any MAVLink message with a `text` attribute as a generic Telegram status line.
- Default: `0`.
- Risk: can create noisy, low-signal alerts.

### Autopilot Target Resolution

#### `AUTOPILOT_SYSID`

- Purpose: fixed MAVLink target system ID for filtering and command routing.
- Default: unset or auto-learn from first heartbeat.

#### `AUTOPILOT_COMPID`

- Purpose: fixed MAVLink target component ID for filtering and command routing.
- Default: unset or auto-learn from first heartbeat.

These values matter for both inbound filtering and outbound commands. In the current implementation, if they are not set, they are learned from the first matching heartbeat seen by the monitor.

### Temperature Fallback

#### `MAVINJECT_DUMMY_TEMP_C`

- Purpose: fallback Celsius value when neither a hardware sensor nor CPU temperature is available.
- Default: `99.0`.
- Docker note: containers usually do not have the same hardware temperature environment as the host, so this fallback or CPU temp behavior must be considered explicitly.

## Dependencies

Current Python dependencies in `requirements.txt` relevant to the Telegram backend are:

- `pymavlink==2.4.49`
- `python-dotenv==1.0.1`
- `python-telegram-bot>=20,<22`
- `w1thermsensor==2.3.0` as optional hardware-backed temperature support

Additional notes:

- `Pillow` is present in the repo dependency set but is not required for the Telegram backend described here.
- `w1thermsensor` is likely problematic in generic Docker deployments unless the container is deliberately given device access and the host kernel is configured for 1-wire sensors.

## Internal State Model

The current script keeps most runtime state in module-level globals. A coding agent extracting this backend should preserve the semantics, even if it replaces the storage model.

### Chat State

- `app`: telegram `Application` instance when polling is enabled.
- `chat_id`: current destination chat.
- `chat_id_lock`: lock protecting `chat_id` reads and writes.

### Mode Tracking

- `last_mode`
- `last_custom_mode`
- `last_custom_mode_sent`
- `last_mode_sent_ts`
- `pending_custom_mode`
- `pending_mode_name`
- `pending_mode_first_seen`
- `last_heartbeat_notify_ts`

These fields implement mode-change detection and debounce behavior.

### Autopilot Targeting

- `autopilot_sysid`
- `autopilot_compid`

These fields determine which incoming heartbeats count as the controlled autopilot and where outbound commands are sent.

### Rover Telemetry Snapshot

- Position: `rover_lat`, `rover_lon`, `rover_alt_m`
- GPS quality: `rover_hdop`, `rover_vdop`, `last_gps1_fix`
- Second GPS: `rover2_lat`, `rover2_lon`, `rover2_alt_m`, `rover2_hdop`, `rover2_vdop`, `last_gps2_fix`
- Motion and power: `rover_ground_speed`, `rover_batt_voltage`
- Mission state: `rover_wp_seq`, `saved_wp_seq`, `rover_start_time`
- Link status: `gcs_link_quality_pct`, `gcs_link_rssi`, `gcs_link_remrssi`, `gcs_link_noise`, `gcs_link_remnoise`, `gcs_link_txbuf`, `gcs_link_rxerrors`, `gcs_link_fixed`
- Vehicle flags: `rover_armed`

### Failsafe And Status Buffering

- `rover_failsafe_active`
- `rover_failsafe_desc`
- `rover_failsafe_reason`
- `last_status_texts`

`last_status_texts` is effectively a bounded ring buffer, retaining the most recent 50 entries.

## Telegram Send Paths

There are two outbound send mechanisms.

### Polling-App Send Path

`bot_send_message_async(...)`

- Uses `app.bot.send_message(...)`.
- Requires a live `Application` instance and a known `chat_id`.
- Exists, but the implementation relies more heavily on the HTTP helper for cross-thread notifications.

### HTTP Send Path

`bot_send_message_http(...)`

- Sends to `https://api.telegram.org/bot<TOKEN>/sendMessage`.
- Used broadly by the monitor thread and startup logic.
- Requires `TELEGRAM_BOT_TOKEN` and a known `chat_id`.
- Swallows send errors except for optional debug logging.

`bot_send_location_http(...)`

- Sends to `https://api.telegram.org/bot<TOKEN>/sendLocation`.
- Accepts optional `horizontal_accuracy` in meters and clamps it to Telegram's expected range.

For a standalone backend, this split should likely be normalized behind a single messaging adapter so the rest of the system does not care whether delivery is via polling context or direct HTTP.

## Command Interface

The bot exposes the following Telegram commands in polling mode.

### `/start`

- Captures `effective_chat.id` and stores it as the active `chat_id`.
- Replies with a simple readiness message.
- No MAVLink side effects.

### `/temp`

- Captures chat ID.
- Reads the current temperature via `read_temp_c()`.
- Replies with both Celsius and Fahrenheit.

### `/ping`

- Captures chat ID.
- Replies in chat that a test notification is being attempted.
- Then sends a separate silent HTTP notification with static text.

### `/status`

- Captures chat ID.
- Replies with `rover_status_snapshot()`.
- This snapshot includes mode, armed state, position, GPS health, speed, temperature, battery, waypoint, link stats, failsafe state, elapsed time, and recent status messages.

### `/rover`

- Alias for `/status`.
- Maintained for backward compatibility.

### `/messages`

- Captures chat ID.
- Returns the last 10 status text messages from the in-memory buffer.
- Truncates output if needed to stay within Telegram message size limits.

### `/map`

- Captures chat ID.
- If location is unknown, replies with a waiting message.
- Otherwise sends the current rover location via Telegram `sendLocation` and then replies with the numeric coordinates.
- Uses `HDOP * 5` meters as an approximate horizontal accuracy when available.

### `/reboot`

- Captures chat ID.
- Does not immediately send the MAVLink reboot command.
- Sends an inline keyboard requesting confirmation.

### `/skip`

- Captures chat ID.
- If `rover_wp_seq` is known, sends `mission_set_current_send(..., next_seq)` to skip to the next waypoint.
- Replies with success or failure text.

### `/menu`

- Captures chat ID.
- Displays an inline control menu.
- Menu buttons provide mode changes, arm/disarm, status, temperature, skip waypoint, save waypoint, reboot, and quit.

### `/mode_manual`

- Captures chat ID.
- Sends `MAV_CMD_DO_SET_MODE` with `custom_mode = 0`.
- Replies with success or failure text.

### `/mode_auto`

- Captures chat ID.
- Sends `MAV_CMD_DO_SET_MODE` with `custom_mode = 10`.
- Replies with success or failure text.

### `/mode_hold`

- Captures chat ID.
- Sends `MAV_CMD_DO_SET_MODE` with `custom_mode = 4`.
- Replies with success or failure text.

### `/arm`

- Captures chat ID.
- Sends `MAV_CMD_COMPONENT_ARM_DISARM` with arm parameter `1`.
- Replies with success or failure text.

### `/disarm`

- Captures chat ID.
- Sends `MAV_CMD_COMPONENT_ARM_DISARM` with arm parameter `0`.
- Replies with success or failure text.

## Inline Menu Callback Surface

The menu callback handler implements the following callback actions.

### `MODE_MANUAL`

- Sends manual mode change.
- Rewrites the menu message with the result.

### `MODE_AUTO`

- Sends auto mode change.
- Rewrites the menu message with the result.

### `MODE_HOLD`

- Sends hold mode change.
- Rewrites the menu message with the result.

### `ARM`

- Sends arm command.
- Rewrites the menu message with the result.

### `DISARM`

- Sends disarm command.
- Rewrites the menu message with the result.

### `STATUS`

- Rewrites the menu message with `rover_status_snapshot()`.

### `TEMP`

- Rewrites the menu message with a fresh temperature reading.

### `SKIP`

- Skips to the next waypoint if the current one is known.

### `SAVE_WP`

- Saves the current waypoint sequence into `saved_wp_seq`.
- Important: this only stores the waypoint in memory. It does not persist to disk and does not send a MAVLink command.

### `REBOOT_CONFIRM`

- Sends `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`.
- Uses the resolved autopilot target IDs.

### `REBOOT_CANCEL`

- Rewrites the menu message to indicate cancellation.

### `MENU_QUIT`

- Closes the menu message by editing it to `Menu closed.`

## MAVLink Command Surface

The standalone Telegram backend can actively control the vehicle. An extracted service must preserve these outbound MAVLink behaviors if command parity is required.

### Mode Changes

Implemented by `_send_mode(custom_mode)` using:

- `MAV_CMD_DO_SET_MODE`
- `param1 = 1` to indicate custom mode selection
- `param2 = custom_mode`

Current mapped values:

- `0`: MANUAL
- `10`: AUTO
- `4`: HOLD

### Arm/Disarm

Implemented with `MAV_CMD_COMPONENT_ARM_DISARM`.

- Arm: first parameter `1`
- Disarm: first parameter `0`

### Waypoint Skip

Implemented with `master.mav.mission_set_current_send(target_sys, target_comp, next_seq)`.

### Flight Controller Reboot

Implemented with `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` and parameter `1`.

## Outbound Notification Behavior

The monitor loop is the core of the backend's proactive alerting behavior.

### Startup Notifications

The backend sends a silent startup message in all enabled Telegram modes when a `chat_id` is known:

- Polling mode: `[MAVInject started. UDP=...]`
- Polling init fallback: `[MAVInject started (fallback). UDP=...]`
- HTTP-only mode: `[MAVInject started (no-poll). UDP=...]`

### Heartbeat Handling And Mode Detection

For `HEARTBEAT` messages the monitor loop:

1. Learns or validates autopilot system/component identity.
2. Ignores heartbeats from other systems/components.
3. Derives a human-readable rover mode from `custom_mode` when possible.
4. Tracks arm state from `base_mode`.
5. Emits mode-change notifications.

Mode notification rules:

- First recognized mode after startup is sent immediately and loudly.
- Later mode changes are debounced using `MODE_DEBOUNCE_SECONDS`.
- Periodic heartbeat notifications are silent and only sent when `HEARTBEAT_PERIODIC=1`.

### `NAMED_VALUE_FLOAT` Forwarding

When a received `NAMED_VALUE_FLOAT` matches `MAVLINK_NAME`, the backend forwards it to Telegram, subject to `MAVLINK_THROTTLE_SECONDS`.

This is how the optional temperature sender becomes user-visible in Telegram.

### Status Text Forwarding

For status-text-like MAVLink messages, the backend:

1. Extracts severity and text.
2. Appends the rendered line to the bounded `last_status_texts` buffer.
3. Detects failsafe-related text.
4. Sends Telegram notifications.

Notification rules:

- Failsafe activation: sends `[FAILSAFE]: ...`
- Failsafe clear: sends `Failsafe cleared: ...`
- Other status text: sends `Status: ...` silently

Failsafe reasoning is enriched through keyword matching such as RC, GCS, EKF, GPS, Battery, Terrain, Compass, IMU, and others.

### GPS Fix Change Alerts

For both GPS1 and GPS2 streams, the backend tracks fix type changes and sends Telegram alerts.

Loudness rule:

- Entering RTK Fixed: loud
- Leaving RTK Fixed: loud
- Other fix changes: silent

### Extended Status Alerts

When `EXTENDED_STATUS=1`, the backend additionally forwards:

- Relay-related `COMMAND_LONG` messages as `[Relay N]: ON/OFF`
- `COMMAND_ACK` messages as `[Command ACK]: ...`

These are always sent silently in the current implementation.

### Generic Text Forwarding

When `FORWARD_ALL_TEXT=1`, any MAVLink message with a `text` attribute may be forwarded as `Status?: ...`.

This should be considered a debug or diagnostic feature, not a high-confidence production behavior.

## Temperature Reading Behavior

The backend uses `read_temp_c()` as the single temperature abstraction.

Resolution order:

1. Use `W1ThermSensor` if available and enabled.
2. Otherwise read CPU temperature from `/sys/class/thermal/thermal_zone0/temp`.
3. Otherwise use `MAVINJECT_DUMMY_TEMP_C`.

This matters for Docker extraction because step 2 and step 1 may not work in a container without deliberate host integration.

## Startup Modes

### Mode 1: Polling Mode

Condition:

- `TELEGRAM_BOT_TOKEN` set
- `TELEGRAM_DISABLE_POLLING != 1`

Behavior:

- Builds Telegram `Application`
- Registers all command handlers and callback handler
- Seeds `chat_id` from `TELEGRAM_CHAT_ID` when available
- Sends silent startup notification if chat is known
- Starts MAVLink monitor thread
- Calls `app.run_polling(close_loop=False)`

### Mode 2: HTTP-Only Fallback After Polling Failure

Condition:

- Polling setup throws an exception

Behavior:

- Logs the failure
- Sets `app = None`
- Seeds `chat_id` from env if available
- Sends silent fallback startup notification
- Starts MAVLink monitor thread
- Keeps the main thread alive with a sleep loop

### Mode 3: HTTP-Only By Configuration

Condition:

- `TELEGRAM_BOT_TOKEN` set
- `TELEGRAM_DISABLE_POLLING == 1`

Behavior:

- Does not create Telegram `Application`
- Seeds `chat_id` from env if available
- Sends silent startup notification
- Starts MAVLink monitor thread
- Keeps the main thread alive with a sleep loop

### Mode 4: Telegram Disabled

Condition:

- `TELEGRAM_BOT_TOKEN` missing

Behavior:

- No Telegram interface
- No monitor thread startup in the current code path
- Process idles in a sleep loop

This last behavior is important. If the extracted backend is supposed to always monitor MAVLink even without Telegram enabled, that must be an intentional design change.

## Dockerization Guidance

### Recommended Container Role

The cleanest Docker role for this implementation is:

- One container process dedicated to Telegram plus MAVLink monitoring and control.

The backend should not assume local GPIO, local kernel sensor drivers, or access to host thermal files unless that is explicitly part of the deployment contract.

### Recommended Environment Strategy

- Use real environment variables or secret injection.
- Do not depend on `.env` existing inside the image.
- Treat `TELEGRAM_BOT_TOKEN` as a secret.
- Consider setting `TEMP_SENDER_ENABLED=0` by default in containerized deployments unless temperature injection is required.

### Networking Considerations

- The single MAVLink UDP endpoint must be reachable from inside the container.
- `udp:` targets should point to the correct host or service name visible from the container network.
- If traffic crosses container boundaries, ensure networking supports bidirectional UDP flow.

### Hardware And Host Integration Risks

- `w1thermsensor` may fail without host device mapping and kernel support.
- CPU temperature files may not reflect the host system or may not exist.
- For a pure bot backend, fallback temperature handling should be made explicit rather than incidental.

### Polling vs HTTP-Only In Docker

Polling mode is appropriate when:

- The container should receive Telegram commands directly.
- You want full bot control functionality.

HTTP-only mode is appropriate when:

- The service is alert-only.
- Another system owns Telegram command intake.
- Polling is not operationally desirable.

## Recommended Extraction Architecture

The current implementation is monolithic. A coding agent should extract it into explicit components.

### Suggested Modules

#### `config`

- Parses and validates environment variables.
- Converts strings into strongly typed runtime config.

#### `mavlink_client`

- Owns single bidirectional UDP connection setup.
- Exposes receive loop and outbound command methods.
- Encapsulates target sysid/compid resolution.

#### `telegram_client`

- Owns bot startup, polling, and direct HTTP fallback.
- Exposes `send_message`, `send_location`, and command registration.
- Hides delivery implementation details from the rest of the app.

#### `state_store`

- Holds rover telemetry, debounce state, status history, and failsafe state.
- Replaces module-level globals.

#### `event_monitor`

- Consumes MAVLink messages.
- Updates state.
- Emits notification intents.

#### `command_router`

- Maps Telegram commands and callbacks to application actions.
- Returns structured replies.

#### `app`

- Wires dependencies together.
- Handles startup and graceful shutdown.

### Minimum Interfaces To Preserve

An implementation rewrite should preserve these semantic interfaces even if names change.

#### Notification Interface

- Send text message to active/default chat.
- Send location to active/default chat.
- Support silent vs loud sends.

#### MAVLink Control Interface

- Set mode.
- Arm.
- Disarm.
- Skip waypoint.
- Reboot controller.

#### State Query Interface

- Build full rover status snapshot.
- Return recent status messages.
- Return current position if known.

#### Event Processing Interface

- Process `HEARTBEAT`
- Process `GPS_RAW_INT`
- Process `GPS2_RAW` and `GPS2_RAW_INT`
- Process `NAMED_VALUE_FLOAT`
- Process status text
- Optionally process `COMMAND_LONG`, `COMMAND_ACK`, `VFR_HUD`, `SYS_STATUS`, `RADIO_STATUS`, `MISSION_CURRENT`, `SYSTEM_TIME`

## Behavioral Details A Rewrite Must Keep

These details are easy to miss but are part of current behavior.

- Chat ID can be learned from inbound Telegram traffic and overwritten later.
- Mode changes are debounced instead of emitting on every heartbeat transition.
- The first detected mode is emitted immediately.
- GPS fix alerts are only loud when entering or leaving RTK Fixed.
- Failsafe notifications distinguish activation from clearing.
- `/rover` is an alias for `/status`.
- `/map` both sends a location pin and replies with coordinates.
- `SAVE_WP` stores an in-memory bookmark only.
- Polling failure falls back to HTTP-only mode instead of terminating.
- When Telegram is disabled entirely, the current script does less than the enabled modes because the monitor thread is not started there.

## Non-Target Files

### `main.py`

`main.py` should not be used as the template for the standalone Telegram backend.

Reasons:

- The notify function is a no-op.
- Comments explicitly state Telegram logic was removed.
- The current architecture there is for a different runtime direction.

### `mavmonitor.py`

`mavmonitor.py` should not be used as the template for the standalone Docker backend.

Reasons:

- It is a MAVProxy `MPModule`, not a standalone service.
- It polls Telegram manually with `getUpdates`.
- It supports a narrower command surface centered on `/rover`.
- Its lifecycle is tied to MAVProxy rather than an independent containerized application.

## Suggested Smoke Tests For A New Backend

After extraction, a coding agent should validate at least the following scenarios.

### Startup

- Starts successfully with valid token and reachable MAVLink endpoints.
- Starts in polling mode when polling is enabled.
- Starts in HTTP-only mode when polling is disabled.
- Falls back cleanly if polling initialization fails.

### Telegram Commands

- `/start` stores chat ID.
- `/temp` returns temperature.
- `/status` and `/rover` return full snapshot.
- `/messages` returns recent status lines.
- `/map` sends location when GPS is known.
- `/mode_manual`, `/mode_auto`, `/mode_hold` send correct MAVLink mode commands.
- `/arm` and `/disarm` send correct arm/disarm commands.
- `/skip` advances waypoint.
- `/reboot` requires confirmation before sending reboot command.

### Notifications

- Startup notification arrives.
- Initial mode notification arrives.
- Debounced mode change notification arrives.
- GPS1 and GPS2 fix changes are detected.
- Failsafe activation and clear notifications are detected.
- Throttled named value forwarding works.

### Docker-Specific

- Container receives and sends MAVLink traffic on the configured bidirectional UDP endpoint.
- Secret injection works for Telegram token.
- Temperature behavior is explicit when hardware sensors are unavailable.
- Service handles network interruptions without crashing.

## MavWeb Integration Architecture

The Telegram implementation in mavweb should not duplicate the Streamlit dashboard state or open a second MAVLink consumer. The bot runs in the same Python process as mavweb and reuses the existing shared singleton state and connection.

### Runtime Shape

- `dashboard_page.py` starts the existing MAVLink worker and the Telegram bridge together.
- `shared_state.py` remains the source of truth for rover telemetry, mission geometry, and recent messages.
- `telegram_event_queue` carries message-style events from mavweb into the Telegram bridge.
- The Telegram bridge reads snapshots from `get_shared_state()` and uses `get_connection()` plus `acquire_mav_lock()` for outbound commands.

### Map Behavior

The Telegram `/map` command should render a static Mapbox image using the map geometry already maintained by mavweb:

- Breadcrumb trail from `history`
- Mission path from `mission_points`
- Current rover position from `lat` and `lon`

If the Mapbox key is missing or invalid, the bot falls back to a Telegram location pin instead of failing.

### Command Surface

The initial implementation keeps the mavinject command surface available from the shared mavweb state:

- `/status` and `/rover` read the current snapshot
- `/messages` reads the recent status buffer
- `/map` renders the static map or falls back to a location pin
- `/mode_manual`, `/mode_auto`, `/mode_hold`, `/arm`, `/disarm`, `/skip`, and `/reboot` send MAVLink commands through the shared connection
- `/save_wp` enqueues a waypoint save request through the shared state queue

### Notification Flow

- Status text appended by mavweb is also placed on `telegram_event_queue`.
- The Telegram bridge drains that queue and forwards the messages.
- A lightweight snapshot loop compares the current `SharedState` snapshot to the previous one so mode, arm, link, and GPS-fix changes can generate proactive notifications without stealing MAVLink packets from the worker.

## Coding-Agent Implementation Checklist

Use this list as the minimum parity target when rebuilding the backend.

- Recreate all Telegram commands from `mavinject.py`.
- Recreate all menu callback actions.
- Preserve the startup mode split: polling, fallback, HTTP-only, disabled.
- Preserve proactive alert behavior from the MAVLink monitor.
- Preserve autopilot sysid/compid filtering and targeting.
- Preserve mode debounce behavior.
- Preserve GPS fix change handling for both GPS1 and GPS2.
- Preserve failsafe detection and clear logic.
- Preserve recent status message buffering.
- Preserve silent vs loud send semantics.
- Preserve rover status snapshot content or replace it intentionally with a documented equivalent.
- Explicitly decide whether temperature injection belongs in the new standalone backend.
- Replace module globals with an explicit state store.
- Make shutdown and restart behavior deliberate instead of relying on daemon threads.
- Make Docker env, networking, and secrets handling explicit.

If the goal is a production-ready standalone backend, the ideal deliverable is not a direct transliteration of `mavinject.py`. It is a structured service that preserves behavior while removing global state, tightening error handling, and separating Telegram delivery from MAVLink event processing.