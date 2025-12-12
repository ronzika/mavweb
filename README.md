# mavinject

Sends a temperature reading to MAVLink as NAMED_VALUE_FLOAT and provides a Telegram bot that replies with the current temperature on `/temp`.

## Setup

1. Create a virtual environment and install deps:

```bash
python3 -m venv env
source env/bin/activate
pip install -r requirements.txt
```

2. Configure environment:

```bash
cp .env.example .env
# edit .env to set TELEGRAM_BOT_TOKEN and any overrides
```

## Run

```bash
source env/bin/activate
python main.py
```

- If `TELEGRAM_BOT_TOKEN` is set, the bot will start; send `/temp` to your bot to get a temperature readout.
- MAVLink endpoint can be overridden via `MAVLINK_ENDPOINT`.
- The named value label defaults to `DS18F     ` (exactly 10 chars); override via `MAVLINK_NAME`.

MAVLink endpoint can be overridden via `MAVLINK_ENDPOINT`.
The named value label defaults to `DS18F     ` (exactly 10 chars); override via `MAVLINK_NAME`.
