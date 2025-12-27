"""
Streamlit Missions Page
"""

import streamlit as st
import os
import pandas as pd
import pydeck as pdk
from missions import list_missions, parse_waypoints_file
from st_aggrid import AgGrid, GridOptionsBuilder, GridUpdateMode
import mavlink_utils
from shared_state import get_shared_state


#def render():
st.title('Missions')
missions_dir = os.path.join(os.path.dirname(__file__), 'missions')
st.subheader('Upload Missions')
uploaded_files = st.file_uploader(
	'Upload .waypoints files',
	type=['waypoints'],
	accept_multiple_files=True,
	key='missions_upload_dialog'
)
print(uploaded_files)

if uploaded_files:
	for uploaded_file in uploaded_files:
		save_path = os.path.join(missions_dir, uploaded_file.name)
		with open(save_path, 'wb') as f:
			f.write(uploaded_file.read())
	st.success(f"Uploaded {len(uploaded_files)} mission(s). Reload the page to see them in the table.")

# Only wrap mission loading in try/except
try:
	missions = list_missions(missions_dir)
except Exception as e:
	st.error(f'Error loading missions: {e}')
	missions = []

if len(missions) > 0:
	df = pd.DataFrame(missions)
	gb = GridOptionsBuilder.from_dataframe(df[['Mission', 'Timestamp', 'Filename']])
	gb.configure_selection('single', use_checkbox=False)
	grid_options = gb.build()
	# Use session state to store selected mission
	if 'selected_mission' not in st.session_state:
		st.session_state['selected_mission'] = None

	grid_response = AgGrid(
		df[['Mission', 'Timestamp', 'Filename']],
		gridOptions=grid_options,
		update_mode=GridUpdateMode.SELECTION_CHANGED,
		theme='streamlit',
		height=300,
		allow_unsafe_jscode=True,
		enable_enterprise_modules=False,
		key="missions_aggrid"
	)
	selected_rows = grid_response['selected_rows']
	# Ensure selected_rows is always a list of dicts
	if isinstance(selected_rows, pd.DataFrame):
		selected_rows = selected_rows.to_dict(orient='records')

	# Always use the current AgGrid selection for preview if available
	if selected_rows is not None and len(selected_rows) > 0:
		sel_row = selected_rows[0]
		selected_mission = sel_row['Filename']
		# Update session state for persistence (optional)
		st.session_state['selected_mission'] = selected_mission
	else:
		selected_mission = st.session_state.get('selected_mission', None)

	# Show preview if a mission is selected and exists in DataFrame
	if selected_mission is not None and selected_mission != '':
		filtered = df[df['Filename'] == selected_mission]
		if not filtered.empty:
			full_row = filtered.iloc[0]
			# Layout: Preview text and Load button side by side
			col1, col2 = st.columns([3, 2])
			with col1:
				st.subheader(f"Preview: {full_row['Mission']} ({full_row['Filename']})")
			with col2:
				if st.button("Load to Flight Controller", key=f"load_{full_row['Filename']}"):
					mission_items = mavlink_utils.parse_mission_file(full_row['Path'])
					if mission_items:
						state = get_shared_state()
						grace_s = float(os.getenv("MISSION_PROGRESS_GRACE_S", "3") or 3)
						state.set_upload_status('pending', 'Queued for upload...')
						state.update({'mission_ul_active': False, 'mission_ul_total': 0, 'mission_ul_sent': 0, 'mission_ul_done_ts': 0.0})
						state.upload_queue.put(mission_items)
						import time
						progress_ph = st.empty()
						status_ph = st.empty()
						start_time = time.time()
						final_status = None
						while time.time() - start_time < 60:
							data = state.get()
							ul_total = int(data.get('mission_ul_total') or len(mission_items) or 0)
							ul_sent = int(data.get('mission_ul_sent') or 0)
							if ul_total > 0:
								progress_ph.progress(min(1.0, ul_sent / ul_total), text=f"Uploading {ul_sent}/{ul_total}")
							else:
								progress_ph.progress(0.0, text="Uploading...")

							status = state.get_upload_status()
							if status['status'] == 'success':
								progress_ph.progress(1.0, text=f"Upload done {ul_total}/{ul_total}")
								status_ph.success(status['message'])
								final_status = 'success'
								break
							elif status['status'] == 'error':
								status_ph.error(status['message'])
								final_status = 'error'
								break
							time.sleep(0.2)
						else:
							status_ph.error("Upload timed out.")
							final_status = 'timeout'

						if final_status in ('success', 'error'):
							time.sleep(grace_s)
							progress_ph.empty()
							status_ph.empty()
					else:
						st.error("Failed to parse mission file.")
			points = parse_waypoints_file(full_row['Path'])
			if points:
				path_points = [[p['lon'], p['lat']] for p in points]
				waypoint_data = [{"pos": [p['lon'], p['lat']], "color": [255, 255, 255, 255]} for p in points]
				view_state = pdk.ViewState(
					latitude=points[0]['lat'],
					longitude=points[0]['lon'],
					zoom=20,
					pitch=0,
				)
				layers = []
				if len(path_points) > 1:
					layers.append(pdk.Layer(
						"PathLayer",
						data=[{"path": path_points}],
						get_path="path",
						get_color=[255, 255, 255, 200],
						width_min_pixels=1,
						get_width=0.05,
						width_units='"meters"',
						dash_justified=True,
					))
				layers.append(pdk.Layer(
					"ScatterplotLayer",
					data=waypoint_data,
					get_position="pos",
					get_color="color",
					get_radius=0.15,
					radius_units='"meters"',
				))
				deck = pdk.Deck(
					initial_view_state=view_state,
					layers=layers,
					tooltip=True
				)
				st.pydeck_chart(deck)
			else:
				st.info('No valid points found in this mission file.')
		else:
			st.info('Select a mission row to preview.')
	else:
		st.info('Select a mission row to preview.')
else:
	st.info('No missions found.')
