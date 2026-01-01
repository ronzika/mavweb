import streamlit as st

st.set_page_config(page_title="Rover GCS", layout="wide", initial_sidebar_state="expanded")


pages = {
    "MavWeb GCS": [
        st.Page("dashboard_page.py", title="Dashboard", icon=":material/dashboard:", default=True),
        st.Page("mission.py", title="Mission", icon=":material/moving:",),
        st.Page("settings.py", title="Settings", icon=":material/settings:",),
    ]
}


selected = st.navigation(pages, position="sidebar", expanded=True)
selected.run()

