#!/bin/bash
# Publishes temperature readings from a DS18B20 sensor to an MQTT broker on a Raspberry Pi.
MQTT_HOST="192.168.1.48"
MQTT_TOPIC="mavweb/temp"

publish_temp_json() {
    local temp_f="$1"
    local error_msg="$2"
    local ts
    ts=$(date -u +"%Y-%m-%dT%H:%M:%SZ")
    local payload
    local error_json="null"

    if [[ -n "$error_msg" ]]; then
        error_msg=${error_msg//\\/\\\\}
        error_msg=${error_msg//"/\\"}
        error_json=$(printf '"%s"' "$error_msg")
    fi

    payload=$(printf '{"temperature_f":%s,"timestamp":"%s","error":%s}' "$temp_f" "$ts" "$error_json")

    echo "Publishing payload: $payload"
    if command -v mosquitto_pub >/dev/null 2>&1; then
        mosquitto_pub -h "$MQTT_HOST" -t "$MQTT_TOPIC" -m "$payload" >/dev/null 2>&1
    fi
}

if ! command -v mosquitto_pub >/dev/null 2>&1; then
    echo "mosquitto_pub not found; install mosquitto-clients to enable MQTT publishing" >&2
fi

BASE_DIR="/sys/bus/w1/devices"
SENSOR=$(ls -d $BASE_DIR/28-* 2>/dev/null | head -n 1)
SENSOR_FILE="$SENSOR/w1_slave"

if [[ ! -f "$SENSOR_FILE" ]]; then
    err_msg="No w1thermsensor found under $BASE_DIR"
    echo "$err_msg"
    publish_temp_json "-99" "$err_msg"
    exit 0
fi

RAW=$(cat "$SENSOR_FILE")

if echo "$RAW" | grep -q "YES"; then
    temp_milli_c=$(echo "$RAW" | awk -F't=' '{print $2}')
    if [[ -z "$temp_milli_c" ]]; then
        err_msg="Sensor read error (temperature field missing)"
        echo "$err_msg"
        publish_temp_json "-99" "$err_msg"
        exit 0
    fi

    temp_c=$(awk "BEGIN {printf \"%.2f\", $temp_milli_c / 1000}")
    temp_f=$(awk "BEGIN {printf \"%.2f\", ($temp_c * 9/5) + 32}")

    echo "Temperature: ${temp_f} °F"
    publish_temp_json "$temp_f" ""
else
    err_msg="Sensor read error (CRC check failed)"
    echo "$err_msg"
    publish_temp_json "-99" "$err_msg"
fi


