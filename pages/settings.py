import streamlit as st


import streamlit as st
from pathlib import Path
import config
from shared_state import get_shared_state

def load_env_vars():
    env_path = Path(__file__).resolve().parents[1] / '.env'
    env_vars = {}
    if env_path.exists():
        with open(env_path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                if '=' in line:
                    k, v = line.split('=', 1)
                    env_vars[k] = v
    return env_vars

def save_env_vars(new_vars):
    env_path = Path(__file__).resolve().parents[1] / '.env'
    with open(env_path, 'w') as f:
        for k, v in new_vars.items():
            f.write(f'{k}={v}\n')


st.title('Settings')
st.info('Edit and save environment variables. Changes will be written to .env.')
env_vars = load_env_vars()
current = {k: st.session_state.get(f'env_{k}', v) for k, v in env_vars.items()}
with st.form('env_form'):
    new_vars = {}
    for k in sorted(env_vars.keys()):
        v = env_vars[k]
        new_vars[k] = st.text_input(k, value=current[k], key=f'env_{k}')
    submitted = st.form_submit_button('Save')
    if submitted:
        save_env_vars(new_vars)
        st.session_state['Pages'] = 'Dashboard'
        st.success('.env updated! Reloading app...')
        st.rerun()

print(env_vars)
