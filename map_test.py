import streamlit as st
import pandas as pd
import pydeck as pdk

st.title("Map Style Explorer")

# Sample data
data = {
    'latitude': [37.773972, 34.052235, 40.712776],
    'longitude': [-122.431297, -118.243683, -74.005974],
    'City': ['San Francisco', 'Los Angeles', 'New York City']
}
df = pd.DataFrame(data)

# Container for the map (displayed at the top)
map_container = st.container()

st.divider()
st.subheader("Map Settings")

# Form options below the map
with st.form("map_settings"):
    col1, col2 = st.columns(2)
    
    with col1:
        map_styles = {
            "Streets": "mapbox://styles/mapbox/streets-v11",
            "Outdoors": "mapbox://styles/mapbox/outdoors-v11",
            "Light": "mapbox://styles/mapbox/light-v10",
            "Dark": "mapbox://styles/mapbox/dark-v10",
            "Satellite": "mapbox://styles/mapbox/satellite-v9",
            "Satellite Streets": "mapbox://styles/mapbox/satellite-streets-v11",
            "Navigation Day": "mapbox://styles/mapbox/navigation-day-v1",
            "Navigation Night": "mapbox://styles/mapbox/navigation-night-v1",
        }
        selected_style = st.selectbox("Select Map Style", list(map_styles.keys()), index=0)
    
    with col2:
        pitch = st.slider("Pitch", 0, 60, 0, help="Tilt the map for 3D view")
        bearing = st.slider("Bearing", 0, 360, 0, help="Rotate the map")
        
    submitted = st.form_submit_button("Update Map")

# Render the map in the top container
with map_container:
    st.pydeck_chart(pdk.Deck(
        map_style=map_styles[selected_style],
        initial_view_state=pdk.ViewState(
            latitude=df['latitude'].mean(),
            longitude=df['longitude'].mean(),
            zoom=3,
            pitch=pitch,
            bearing=bearing,
        ),
        layers=[
            pdk.Layer(
                'ScatterplotLayer',
                data=df,
                get_position='[longitude, latitude]',
                get_color='[200, 30, 0, 160]',
                get_radius=200000,
                pickable=True,
            ),
            pdk.Layer(
                "TextLayer",
                data=df,
                get_position='[longitude, latitude]',
                get_text="City",
                get_color=[0, 0, 0, 200],
                get_size=16,
                get_alignment_baseline="'bottom'",
            )
        ],
        tooltip={"text": "{City}"}
    ), width="stretch", key=f"map_{selected_style}_{pitch}_{bearing}")
