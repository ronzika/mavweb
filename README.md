
# mavweb

Streamlit-based Ground Control Station (GCS) web UI for an ArduPilot rover.

This project is intended to be run on a laptop/RPi and connected to a MAVLink stream (typically forwarded from Mission Planner or MAVProxy). It provides a live dashboard (map + telemetry), mission controls, mission management (upload), link quality visualization, and optional ArduPilot relay toggles.

## What You Get

### Live Telemetry Header

At the top of the main page you’ll see live metrics that update continuously:

- Mode: vehicle mode (e.g., MANUAL/AUTO/HOLD)
- Armed: armed state
- Speed: groundspeed
- GPS: fix string (No Fix / 2D / 3D / RTK, etc)
- Battery: voltage + percent (when provided)
- WP: current mission waypoint index (as reported by the vehicle)

### Sidebar: Commands

The sidebar contains command buttons that send MAVLink commands to the vehicle:

- MANUAL / AUTO / HOLD: mode changes
- ARM / Disarm: arm/disarm commands
- Skip WP: increments the current waypoint by +1 (disabled until current WP is known; clamped when mission total is known)
- Reset Mission: sets current mission item back to 0 (disabled while in AUTO); includes an “Are you sure?” confirmation
- Fetch Mission: requests the mission list from the vehicle to populate mission geometry/total

### Sidebar: Relays (Optional)

If you configure relay labels in .env (RELAY1..RELAY6), the sidebar will show toggles that control ArduPilot relays.

- Each toggle label comes from the env var value (e.g., RELAY1=Lights shows a “Lights” toggle)
- Toggling sends MAV_CMD_DO_SET_RELAY for the corresponding relay number

See “Relay Setup” below.

### Sidebar: Map Settings

- Map Style: choose the Mapbox style used for the map

### Map View

The map is the main situational-awareness view:

- Rover marker: a small triangle indicating heading
- Breadcrumb trail: a path showing recent rover movement
	- The breadcrumb trail is cleared when the MAVLink link disconnects
- Mission overlay (when available):
	- Mission path is shown as Completed vs Pending segments
	- Waypoints are shown as dots

### Mission Progress Bar (AUTO-only)

A mission progress bar is displayed above the map only when the rover is in AUTO mode.

- Progress is derived from current waypoint / total waypoints
- The total waypoint count is learned from mission download metadata (MISSION_COUNT) or from the locally provided mission points when available

### System Messages

The “System Messages” window is the equivalent of Mission Planner’s STATUSTEXT console.

It includes:

- STATUSTEXT messages forwarded from the vehicle
- A few additional app-side messages such as:
	- link establishment / reconnect events
	- UI actions (e.g., “Skipped from WP X to WP Y”, “Reset mission to WP 0”)
	- mission transfer status messages

### Link Quality + Sparkline

The UI shows a link-quality percentage and a small sparkline trend. This is conceptually similar to Mission Planner’s LinkQualityGCS.

How it’s calculated in this app:

- For each received MAVLink message, the message sequence number is read.
- If sequence numbers jump, the app estimates “lost” packets based on the gap.
- A rolling window (size 100) of received/lost markers is maintained.
- Link quality is computed as:

	$$\text{LQ} = \frac{\text{received}}{\text{received} + \text{lost}} \times 100$$

- The sparkline is a smoothed trend: the app averages recent LQ values and keeps a short history to plot.

Notes:

- This is an estimate based on sequence gaps and can be influenced by what messages are flowing and at what rates.
- During mission uploads, telemetry is temporarily throttled to prioritize mission traffic (which can change the mix of messages seen).

## Mission Management Page

The Mission Management page lets you:

- Upload .waypoints files into the local missions folder
- Select a mission and preview it
- Upload the selected mission to the flight controller

Mission map rendering without a fetch:

- When you upload a mission to the FC from this UI, the app also publishes the mission geometry into shared state so the main map can render it immediately without needing to fetch it back from the vehicle.

## Relay Setup

Relays are enabled by setting environment variables in your .env file:

- RELAY1 through RELAY6

Example:

- RELAY1=Lights
- RELAY2=Pump

Behavior:

- If an env var is present and non-empty, a sidebar toggle is shown.
- Toggling sends MAV_CMD_DO_SET_RELAY with relay number 1..6.

ArduPilot configuration note:

- The relay must be configured on the vehicle (SERVO/RELAY output mapping depends on your hardware/firmware setup).

## MAVLink Forwarding (Required)

This app expects to receive MAVLink on the configured endpoint (see MAVLINK_ENDPOINT in .env).

In most setups, you’ll forward MAVLink bidirectionally from a “primary” GCS (Mission Planner) or from MAVProxy.

Bidirectional forwarding matters because:

- The app needs telemetry from the vehicle
- The vehicle needs to receive the app’s outgoing commands (modes, relays, mission control)

### Option A: MAVProxy (recommended for always-on forwarding)

Run MAVProxy connected to your vehicle and add an outbound UDP endpoint for mavweb:

- Example (adjust endpoints/ports to your setup):
	- Connect MAVProxy to the vehicle
	- Add an outbound UDP stream to the host/port where mavweb listens

Make sure forwarding is bidirectional (MAVProxy should both receive from the vehicle and forward to mavweb, and also forward mavweb’s outgoing packets back to the vehicle).

### Option B: Mission Planner MAVLink Forwarding

Mission Planner can forward MAVLink to this app. In Mission Planner:

- Open the MAVLink forwarding menu (in the MAVLink settings area)
- Add a forwarding target using UDP outbound to the host/port you configured for mavweb
- Enable the “Write” option for the forwarding target
	- “Write” is required so commands sent by mavweb can go back to the vehicle

Important:

- Mission Planner typically requires you to enable this forwarding each time you open Mission Planner.

## Configuration

Common .env values:

- MAVLINK_ENDPOINT: where mavweb listens/connects for MAVLink (default is udpin:0.0.0.0:14550)
- MAPBOX_API_KEY: required for Mapbox map tiles
- DEBUG: set to 1 for verbose debug logging
- RELAY1..RELAY6: optional relay toggle labels

## Running

Install dependencies:

- Create a venv and install requirements
- Start Streamlit

Typical commands:

- python -m venv .venv
- source .venv/bin/activate
- pip install -r requirements.txt
- streamlit run app.py --server.address 0.0.0.0

