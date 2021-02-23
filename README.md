# Use an ESP32 as an InfraRed camera

A work in progress.

```
# Initialize python virtualenv.
./setup.sh
source .venv/bin/activate

# First upload over serial:
pio run -t upload

# Upload over OTA:
pio run -t upload --upload-port 192.168.1.xx

# Monitor over serial:
pio device monitor
```
