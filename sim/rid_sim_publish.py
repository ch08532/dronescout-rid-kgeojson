#!/usr/bin/env python3
"""
Publish simulated DroneScout MQTT traffic built from odid_sim output.

  ./odid_sim > sim.jsonl
  python3 rid_sim_publish.py sim.jsonl --broker localhost --sensor ds240-sim --loop

Behaves like a DroneScout sensor: topic /sensor/<id>/upload, one publish per
second containing every drone detected in that second (transmit_mode 2),
LZMA compression (or --plain), and a status message every 60 s.

Timestamps are restamped to the current wall-clock time (JSON timestamp,
Location.TimeStamp and System.Timestamp inside UASdata), so tracks are live
in IRIS. Use --no-restamp to keep the simulator's fixed time base.
"""
import argparse, base64, itertools, json, lzma, struct, time
from collections import OrderedDict
import paho.mqtt.client as mqtt

EPOCH_2019 = 1546300800
OFF_LOC_TS, OFF_SYS_TS, OFF_LOC_VALID, OFF_SYS_VALID = 132, 856, 894, 912

ap = argparse.ArgumentParser()
ap.add_argument("simfile")
ap.add_argument("--broker", default="localhost")
ap.add_argument("--port", type=int, default=1883)
ap.add_argument("--sensor", default="ds240-sim")
ap.add_argument("--rate", type=float, default=1.0, help="speed-up factor (1 = real time)")
ap.add_argument("--loop", action="store_true", help="repeat the scenario until stopped")
ap.add_argument("--plain", action="store_true", help="compression = none")
ap.add_argument("--no-restamp", action="store_true", help="keep simulator timestamps")
ap.add_argument("--status-every", type=int, default=60, help="seconds between status messages")
a = ap.parse_args()

steps = OrderedDict()
for line in open(a.simfile):
    r = json.loads(line)
    steps.setdefault(r["t"], []).append(r)

def restamp(b, now_ms):
    b = bytearray(b)
    if b[OFF_LOC_VALID]:
        fix_ms = now_ms - 300                                   # fix 0.3 s before receipt
        struct.pack_into("<f", b, OFF_LOC_TS, round((fix_ms / 1000.0) % 3600, 1))
    if b[OFF_SYS_VALID]:
        struct.pack_into("<I", b, OFF_SYS_TS, now_ms // 1000 - EPOCH_2019)
    return bytes(b)

c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"sim-{a.sensor}")
c.connect(a.broker, a.port)
c.loop_start()
topic = f"/sensor/{a.sensor}/upload"
period = 1.0 / a.rate
n = 0
try:
    for rows in (itertools.cycle(steps.values()) if a.loop else steps.values()):
        t0 = time.monotonic()
        objs = []
        for r in rows:
            b = bytes.fromhex(r["uas_hex"])
            now_ms = r["rx_ms"] if a.no_restamp else int(time.time() * 1000)
            if not a.no_restamp:
                b = restamp(b, now_ms)
            objs.append({"protocol": 1.0, "data": {
                "sensor ID": a.sensor, "RSSI": -60 - 5 * r["drone"], "channel": 0 if "BLE" in r["link"] else 6,
                "timestamp": now_ms, "MAC address": r["mac"], "type": r["link"],
                "UASdata": base64.b64encode(b).decode(),
                "extra": {"SN present": 1, "SN valid": 1, "manufacturer": r["manufacturer"],
                          "model": r["model"]}}})
        if n % a.status_every == 0:
            objs.append({"protocol": 1.0, "status": {
                "sensor ID": a.sensor, "timestamp": objs[0]["data"]["timestamp"],
                "firmware version": "sim", "model": "ds240", "status": "normal"}})
        body = ("".join(json.dumps(o) for o in objs) + "\n").encode()
        c.publish(topic, body if a.plain else lzma.compress(body), qos=1).wait_for_publish()
        n += 1
        if n % 10 == 0 or a.rate > 5:
            print(f"step {n}: {len(rows)} drones, {len(body)} bytes -> {topic}", flush=True)
        time.sleep(max(0.0, period - (time.monotonic() - t0)))
except KeyboardInterrupt:
    pass
finally:
    c.loop_stop()
    c.disconnect()
