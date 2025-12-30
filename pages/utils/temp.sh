#!/bin/bash
# Publishes temperature readings from a DS18B20 sensor to an MQTT broker on a Raspberry Pi.
MQTT_HOST="192.168.1.48"
MQTT_TOPIC="mavweb/temp"

if ! command -v mosquitto_pub >/dev/null 2>&1; then
    echo "mosquitto_pub not found; install mosquitto-clients to enable MQTT publishing" >&2
fi

BASE_DIR="/sys/bus/w1/devices"
SENSOR=$(ls -d $BASE_DIR/28-* 2>/dev/null | head -n 1)
SENSOR_FILE="$SENSOR/w1_slave"

if [[ ! -f "$SENSOR_FILE" ]]; then
    echo "No w1thermsensor found under $BASE_DIR"
    exit 1
fi

RAW=$(cat "$SENSOR_FILE")

if echo "$RAW" | grep -q "YES"; then
    temp_milli_c=$(echo "$RAW" | awk -F't=' '{print $2}')
    temp_c=$(awk "BEGIN {printf \"%.2f\", $temp_milli_c / 1000}")
    temp_f=$(awk "BEGIN {printf \"%.2f\", ($temp_c * 9/5) + 32}")

    echo "Temperature: ${temp_f} °F"

    if command -v mosquitto_pub >/dev/null 2>&1; then
        mosquitto_pub -h "$MQTT_HOST" -t "$MQTT_TOPIC" -m "$temp_f" >/dev/null 2>&1
    fi
else
    echo "Sensor read error (CRC check failed)"
fi


