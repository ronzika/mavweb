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


def render():
	st.title('Missions')
	missions_dir = os.path.join(os.path.dirname(__file__), 'missions')
	st.subheader('Upload Missions')
	uploaded_files = st.file_uploader(
		'Upload .waypoints files',
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
						success = mavlink_utils.upload_mission_to_fc(full_row['Path'])
						if success:
							st.success("Mission uploaded to flight controller!")
						else:
							st.error("Failed to upload mission.")
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
