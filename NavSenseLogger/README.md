# NavSense Logger

A deliberately small Prototype-0 phone/laptop logger for the NavSense FYP.

Phone:
- Android camera through CameraX
- accelerometer + gyroscope + magnetometer through Android SensorManager
- timestamps every sample with phone wall-clock milliseconds and monotonic nanoseconds
- sends records over TCP to the laptop
- camera frames are JPEG compressed (1280x720 target, quality 70)
- protocol is extensible: future wheel encoders are simply another `type:"encoder"` record

Laptop:
- `receiver.py` listens on TCP
- performs a TCP clock-offset calibration before logging
- displays the live camera (requires `opencv-python`) plus camera/IMU rates, bytes received and estimated one-way transport latency
- saves received camera frames and JSONL sensor records into a session directory

## Build

Open this folder in Android Studio. Let Gradle sync/download its Android/Kotlin dependencies, then:
Build > Build APK(s)

Install the generated debug APK on the phone.

## Run laptop receiver

Python 3.10+:

    pip install opencv-python numpy
python receiver.py --port 8765

Find the laptop's AUBdot1x IPv4 address.

On the phone enter that address and port 8765, then START.

Important:
- Both devices must be able to reach each other over AUBdot1x. Some university Wi-Fi networks isolate clients. If the phone cannot connect, this is likely a network policy issue rather than the app.
- The latency number is an estimate. Phone and laptop clocks are calibrated using a short TCP exchange before logging. This measures roughly end-to-end phone-to-laptop transport + phone JPEG encoding time, not camera exposure time or laptop processing time.
- This is Prototype 0, not a production logger. It intentionally prioritizes being easy to modify.

## Protocol

Each TCP record is:
    uint32 big-endian JSON-header length
    JSON header
    optional binary payload

Current record types:
    hello
    imu
    camera

Future:
    encoder

Example encoder record:
    {"type":"encoder","t_wall_ms":...,"t_mono_ns":...,"left_ticks":123,"right_ticks":127}

No protocol redesign is needed to add it.
