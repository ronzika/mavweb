"""
Streamlit Dashboard Page
"""
import streamlit as st

def render():
	import pydeck as pdk
	import math
	from dashboard import get_shared_state

	st.title('Dashboard')
	data = get_shared_state().get()
	if not data:
		st.warning('No telemetry data received from flight controller.')
		return

	# Metrics row
	col1, col2, col3 = st.columns(3)
	col1.metric('Mode', data.get('mode', 'N/A'))
	col2.metric('Speed', f"{data.get('speed_ms', 'N/A')} m/s")
	col3.metric('Satellites', data.get('satellites_visible', 'N/A'))

	# Map
	lat = data.get('lat')
	lon = data.get('lon')
	if lat and lon:
		view_state = pdk.ViewState(latitude=lat, longitude=lon, zoom=20)
		layer = pdk.Layer(
			"ScatterplotLayer",
			data=[{"position": [lon, lat]}],
			get_position="position",
			get_color=[255, 0, 0, 160],
			get_radius=10,
		)
		deck = pdk.Deck(initial_view_state=view_state, layers=[layer])
		st.pydeck_chart(deck)
	else:
		st.info('Waiting for GPS location...')
