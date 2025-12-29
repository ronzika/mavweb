import streamlit as st
import os
import pandas as pd
import pydeck as pdk
from missions import list_missions, parse_waypoints_file
from st_aggrid import AgGrid, GridOptionsBuilder, GridUpdateMode
import mavlink_utils
from shared_state import get_shared_state


#def render():
st.title('Mission Management')
missions_dir = os.path.join(os.path.dirname(__file__), 'missions')

uploaded_files = st.file_uploader(
	'Mission File Upload',
	type=['waypoints'],
	accept_multiple_files=True,
	key='missions_upload_dialog'
)

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
		selected_mission = None
		st.session_state['selected_mission'] = None

	mission_selected = st.session_state.get('selected_mission') is not None

	if mission_selected:
		mission_name = os.path.splitext(st.session_state['selected_mission'])[0]
		st.sidebar.markdown(f"**Selected:** {mission_name}")

	if st.sidebar.button("Load to FC", use_container_width=True, disabled=not mission_selected):
		filename = st.session_state['selected_mission']
		mission_row = df[df['Filename'] == filename]
		if not mission_row.empty:
			path = mission_row.iloc[0]['Path']
			print(f"Uploading mission from {path}")
			
			# Publish mission points to shared state so the main map can render it immediately
			# (without needing to fetch mission from the FC).
			try:
				pts = parse_waypoints_file(path)
				if pts:
					path_points = [[p['lon'], p['lat']] for p in pts]
					state = get_shared_state()
					import time
					state.update({
						'mission_points': path_points,
						'mission_dl_total': len(path_points),
						'mission_dl_received': len(path_points),
						'mission_dl_active': False,
						'mission_dl_done_ts': time.time(),
					})
			except Exception as e:
				st.sidebar.warning(f"Mission preview update failed: {e}")
			
			mission_items = mavlink_utils.parse_mission_file(path)
			if mission_items:
				state = get_shared_state()
				grace_s = float(os.getenv("MISSION_PROGRESS_GRACE_S", "3") or 3)
				# Reset status before starting
				state.set_upload_status('pending', 'Queued for upload...')
				# Reset visible progress
				state.update({'mission_ul_active': False, 'mission_ul_total': 0, 'mission_ul_sent': 0, 'mission_ul_done_ts': 0.0})
				state.upload_queue.put(mission_items)
				
				# Progress bar (uses worker-updated shared state counters)
				import time
				progress_ph = st.sidebar.empty()
				status_ph = st.sidebar.empty()
				start_time = time.time()
				final_status = None
				while time.time() - start_time < 60:  # 60s timeout
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

				# Keep the final state visible briefly, then clear.
				if final_status in ('success', 'error'):
					time.sleep(grace_s)
					progress_ph.empty()
					status_ph.empty()
			else:
				st.sidebar.error("Failed to parse mission file.")

	st.sidebar.button("Download", use_container_width=True, disabled=not mission_selected)

	if st.sidebar.button("Delete", use_container_width=True, disabled=not mission_selected):
		if st.session_state.get('selected_mission'):
			st.session_state['delete_confirm'] = True
		else:
			st.sidebar.warning("No mission selected")

	if st.session_state.get('delete_confirm'):
		st.sidebar.warning(f"Delete {st.session_state['selected_mission']}?")
		col_yes, col_no = st.sidebar.columns(2)
		if col_yes.button("Yes", key="del_yes"):
			try:
				os.remove(os.path.join(missions_dir, st.session_state['selected_mission']))
				st.sidebar.success(f"Deleted {st.session_state['selected_mission']}")
				st.session_state['selected_mission'] = None
				st.session_state['delete_confirm'] = False
				st.rerun()
			except Exception as e:
				st.sidebar.error(f"Error: {e}")
		if col_no.button("No", key="del_no"):
			st.session_state['delete_confirm'] = False
			st.rerun()

	st.sidebar.divider()

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
					# Publish mission points to shared state so the main map can render it immediately
					# (without needing to fetch mission from the FC).
					try:
						pts = parse_waypoints_file(full_row['Path'])
						if pts:
							path_points = [[p['lon'], p['lat']] for p in pts]
							state = get_shared_state()
							import time
							state.update({
								'mission_points': path_points,
								'mission_dl_total': len(path_points),
								'mission_dl_received': len(path_points),
								'mission_dl_active': False,
								'mission_dl_done_ts': time.time(),
							})
					except Exception as e:
						st.warning(f"Mission preview update failed: {e}")

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
