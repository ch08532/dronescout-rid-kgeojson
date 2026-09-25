# dronescout-rid-kgeojson

DroneScout Remote ID → IRIS KGeoJSON, built on [Bento](https://github.com/warpstreamlabs/bento).

This pipeline subscribes to the MQTT feed from BlueMark DroneScout ds230/ds240 Remote ID receivers and decodes the binary Remote ID data. It converts each drone position into an **IRIS KGeoJSON** track message, following the IRIS KGeoJSON ICD, and publishes it on **MQTT** for IRIS. Sensor status and other messages are published on a separate MQTT topic tree. Sending tracks as UDP datagrams to the IRIS KGeoJSON Service is available as an **option**, off by default. Tracks can go to IRIS over MQTT, UDP, or both.

> **Status:** the output follows the IRIS KGeoJSON Service ICD. Tested end to end with a simulator (MQTT, and the optional UDP output). Not yet validated against a real sensor or a real IRIS instance.

---

## 1. Design

### Data flow

```
DroneScout sensor (or sim/rid_sim_publish.py for testing)
   │ publishes → /sensor/<sensorID>/upload     (JSON, plain or LZMA)
   ▼
Source MQTT broker
   ▼
Bento ─ mqtt input (subscribes /sensor/+/upload)
   │
   ├─ subprocess: rid_decode.py
   │     1. LZMA decompress (if compressed)
   │     2. split concatenated JSON objects (transmit_mode 2 batches)
   │     3. classify each object: data / aircraft / status / location / mobile network
   │     4. data → base64-decode UASdata → parse 920-byte ODID_UAS_Data struct
   │     → returns one JSON array per payload
   │
   ├─ unarchive (json_array) → one message per record
   │
   ├─ switch
   │     error / unknown / decode error → log WARN, drop
   │     rid with valid position        → kgeojson.blobl → KGeoJSON track
   │     rid without position (ID only) → drop
   │     status, aircraft,
   │     sensor_location,
   │     mobile_network                 → pass through
   │
   ├─ catch: any processing failure → log ERROR, drop (never published raw)
   ▼
Outputs
   ├─ KGeoJSON tracks → MQTT   rid/kgeojson/<sensor>/<track key>      (default; off with IRIS_MQTT_ENABLED=false)
   │                  → UDP    IRIS host:TrackPortNumber              (optional; on with IRIS_UDP_ENABLED=true)
   └─ everything else → MQTT   rid/<kind>/<sensor>                    (status, aircraft, ...)
```

### Components

| File | Role |
|---|---|
| `rid_pipeline.yaml` | Bento config. Handles MQTT input, routing, error handling, the MQTT output, and the optional UDP output. |
| `rid_decode.py` | Decoder. Bento starts it as a long-running child process and exchanges data over stdin/stdout. It has no network access and uses only the Python standard library. |
| `kgeojson.blobl` | Bloblang mapping from a decoded Remote ID record to a KGeoJSON track message, including unit conversions. |
| `kgeojson.schema.json` | JSON Schema transcribed from the IRIS KGeoJSON ICD. Used to validate test output (the ICD's own example also validates against it). |
| `sim/` | Sensor simulator for testing without hardware, including a pre-generated Oslo Gardermoen scenario (`sim.jsonl`). See section 5. |

### Design decisions

- **Bento owns all network I/O**: MQTT in, MQTT out and the optional UDP output. Reconnects, QoS and backpressure are handled by Bento, not custom code.
- **MQTT is the primary path to IRIS.** KGeoJSON tracks have their own topic tree (`rid/kgeojson/...`), separate from status and other messages (`rid/<kind>/...`), so IRIS can subscribe to `rid/kgeojson/#` and receive only valid KGeoJSON.
- **UDP is optional**, for IRIS deployments that use the KGeoJSON Service's UDP input. When enabled, each track is sent as one JSON object per datagram with no trailing newline. Tracks are about 1 KB, well under the UDP size limit. It can run alongside MQTT, or replace it for tracks (`IRIS_MQTT_ENABLED=false`). Status and other messages always go to MQTT.
- **Only KGeoJSON tracks go on the KGeoJSON topic and the UDP output.** Status, sensor location, mobile network and ADS-B/UAT messages go to their own MQTT topics.
- **The decoder is a subprocess, not an embedded script.** Struct parsing is awkward in Bloblang, and a small standalone Python process is easy to test on its own. Payloads are passed length-prefixed (`uint32` big-endian), so LZMA binary data can't be corrupted by newline framing.
- **The vendor's `mqtt_sub.py` is not used.** It is a complete subscriber application (console, CSV and SBS output) and has no "bytes in, JSON out" interface. `rid_decode.py` reimplements only the parsing, based on the struct definition in the DroneScout manual (v2.7, §3). Keep the vendor repo as a reference and for cross-checking.
- **Tracks are only sent when the drone has reported a valid position.** KGeoJSON requires latitude and longitude, so ID-only detections are dropped.
- **Bad data is logged and dropped, never published raw.** Unparseable payloads and records with decode errors (such as a wrong `UASdata` size) produce a WARN log. Any unexpected processing failure is caught and logged as ERROR. If one object in a batched payload is corrupt, the valid objects before it are still processed.

---

## 2. Input format (from the DroneScout manual v2.7, §3)

The sensor publishes JSON messages with `"protocol": 1.0` and one of these top-level keys: `data`, `status`, `location`, `mobile network` or `aircraft`.

- With `compression = lzma`, the whole payload is LZMA-compressed.
- With `transmit_mode = 2`, a single MQTT publish can contain several JSON objects back to back (`}{`).
- `data.UASdata` is base64 of the raw C struct `ODID_UAS_Data` from `opendroneid.h`, at opendroneid-core-c commit `4785de4570e2ecd418543d130d16147108181d0e`.

### ODID_UAS_Data layout (920 bytes, little-endian, 4-byte enums)

These offsets were verified against the real header compiled with gcc (see `sim/`).

**Identity**

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 / 32 | i32 | BasicID[0/1].UAType | 0 none, 1 aeroplane, 2 heli/multirotor, 3 gyroplane, 4 hybrid VTOL, 6 glider, 15 other |
| 4 / 36 | i32 | BasicID[0/1].IDType | 0 none, 1 serial (CTA-2063-A), 2 CAA reg, 3 UTM UUID, 4 session ID |
| 8 / 40 | char[21] | BasicID[0/1].UASID | ASCII, null-padded |
| 776 | i32 | SelfID.DescType | |
| 780 | char[24] | SelfID.Desc | Free text, e.g. flight purpose |
| 864 | i32 | OperatorID.OperatorIdType | |
| 868 | char[21] | OperatorID.OperatorId | |

**UAS position (`Location`)**

| Offset | Type | Field | Units / invalid value |
|---|---|---|---|
| 64 | i32 | Status | 0 undeclared, 1 ground, 2 airborne, 3 emergency, 4 RID failure |
| 68 | f32 | Direction | ° true; invalid 361 |
| 72 | f32 | SpeedHorizontal | m/s; invalid 255 |
| 76 | f32 | SpeedVertical | m/s, positive is up; invalid 63 |
| 80 | f64 | Latitude | °; invalid 0 |
| 88 | f64 | Longitude | °; invalid 0 |
| 96 | f32 | AltitudeBaro | m; invalid −1000 |
| 100 | f32 | AltitudeGeo | m WGS84 HAE; invalid −1000 |
| 104 | i32 | HeightType | 0 above takeoff, 1 AGL |
| 108 | f32 | Height | m; invalid −1000 |
| 112–128 | i32 ×5 | Horiz/Vert/Baro/Speed/TS accuracy | enums, see `rid_decode.py` |
| 132 | f32 | TimeStamp | seconds past the UTC hour; invalid 65535 |

**Operator position (`System`)**

| Offset | Type | Field | Notes |
|---|---|---|---|
| 808 | i32 | OperatorLocationType | 0 takeoff, 1 live GNSS, 2 fixed |
| 812 | i32 | ClassificationType | 0 undeclared, 1 EU |
| 816 | f64 | OperatorLatitude | invalid 0 |
| 824 | f64 | OperatorLongitude | invalid 0 |
| 852 | f32 | OperatorAltitudeGeo | m HAE; invalid −1000 |
| 856 | u32 | Timestamp | seconds since 2019-01-01 UTC |

**Valid flags (u8, 1 = block present):** BasicID 892–893, Location 894, AuthValid 895–910, SelfID 911, System 912, OperatorID 913.

The decoder rejects any `UASdata` that isn't exactly 920 bytes. If a firmware update changes the ODID commit and the layout, messages will fail loudly with an error instead of producing bad coordinates.

### Timestamps

- `rx_time` is when the sensor received the signal (`data.timestamp`). The sensor has no RTC, so this is only correct if NTP or GNSS time sync is working on the sensor (manual §1.13).
- `time` is the drone's own fix time. F3411 only transmits seconds past the hour at 0.1 s resolution, so the decoder takes the hour from `rx_time`. If the fix is more than 30 minutes ahead of `rx_time`, it is assigned to the previous hour.

---

## 3. Output format

### KGeoJSON track (MQTT topic `rid/kgeojson/<sensor>/<track key>`; optionally UDP)

Follows the IRIS KGeoJSON Service ICD. Example produced by the simulator at Oslo Gardermoen (`GEOID_UNDULATION_M=38.3`, drone orbiting at 100 m AGL over ground at 208 m MSL, so 308 m ≈ 1010 ft MSL):

```json
{
  "timestamp": 1790000119800,
  "trackID": "rid:SIM0OSL0000000000001",
  "source": "Remote ID OSL",
  "type": "contact",
  "displayName": "SIM0OSL0000000000001",
  "latitude": 60.1925799,
  "longitude": 11.1184315,
  "altitude": 1009.5,
  "status": "active",
  "course": 144.0,
  "speed": 15.1,
  "properties": [
    { "UAS ID": "SIM0OSL0000000000001" },
    { "ID type": "serial_number" },
    { "UA type": "helicopter_or_multirotor" },
    { "Flight status": "airborne" },
    { "Manufacturer": "DJI" },
    { "Model": "Mavic 3 Enterprise" },
    { "Operator ID": "NOR-SIM-0001" },
    { "Operator lat": "60.1903067" },
    { "Operator lon": "11.1220867" },
    { "Operator location type": "takeoff" },
    { "Self ID": "Orbit" },
    { "Altitude source": "HAE-geoid" },
    { "Altitude HAE (m)": "346.0" },
    { "Altitude baro (m)": "308.0" },
    { "Height (m)": "100.0" },
    { "Height ref": "ground" },
    { "Vertical rate (m/s)": "0.0" },
    { "Horizontal accuracy (m)": "3" },
    { "RSSI (dBm)": "-60" },
    { "Link": "BLE legacy" },
    { "MAC": "60:60:1F:00:00:01" },
    { "Sensor": "ds240-osl" },
    { "Received (UTC)": "2026-09-21T14:15:20.123Z" }
  ]
}
```

### Field mapping

| ICD field | ICD units | Source / conversion |
|---|---|---|
| `timestamp` | epoch ms | Drone's position fix time (see Timestamps in section 2). Falls back to the sensor receive time if the drone sent no valid timestamp. |
| `trackID` | string | `rid:` + UAS serial (first valid BasicID). Falls back to the MAC address until a BasicID has been received. The prefix avoids collisions with tracks from other sources. |
| `source` | string | `Remote ID <sensor ID>`. Override with `KG_SOURCE`. |
| `type` | enum | `contact`. Override with `KG_TYPE`. |
| `displayName` | string | UAS serial (or MAC). |
| `latitude`, `longitude` | degrees | Location block, WGS84. |
| `altitude` | **feet MSL** | `(HAE − N) × 3.28084`, where N is the geoid undulation set in `GEOID_UNDULATION_M`. If the drone sends no HAE, barometric (pressure) altitude is used instead, and `Altitude source` says `baro`. Omitted if neither is available. |
| `status` | enum | Always `active`. Track expiry is left to the service's `ExpiryTimeout`. |
| `course` | degrees | Location direction (° true). Omitted if the drone reports it as unknown. |
| `speed` | **knots** | Horizontal speed in m/s × 1.943844. Omitted if unknown. |
| `roll`, `pitch`, `yaw`, `battery` | | Not sent. Remote ID doesn't carry these. |
| `rssi` | percent | Not sent. The sensor reports dBm, and there's no agreed mapping to percent. The dBm value is included in `properties` instead. |
| `properties` | string values | Remote ID details for the IRIS metadata panel: IDs, airframe, operator, altitudes, accuracy, RF. Values are strings, and entries with no value are left out. |

### Altitude: set the geoid undulation for each site

Remote ID reports geodetic altitude (WGS84 HAE), but the ICD wants MSL. The difference is the local geoid undulation N (HAE − MSL). In Norway it is positive and tens of metres; at Oslo Gardermoen the EGM96 value is about +38.3 m. Confirm the value for each airport with Kartverket, since the Norwegian height system (NN2000) can differ from EGM96 by up to about a metre.

Look up N for the sensor site (for example with the NRCan GPS-H tool or an EGM2008 calculator) and set `GEOID_UNDULATION_M`. A single value per site is accurate enough, because N changes very little across one sensor's detection range. If you leave it at the default of 0, altitudes will be off by about |N|, which could be over 100 ft.

### Pass-through messages (MQTT topic `rid/<kind>/<sensor>`)

`status`, `sensor_location`, `mobile_network` and `aircraft` (ADS-B/UAT) are republished to MQTT as received, with a `kind` field added. They never go on the KGeoJSON topic or the UDP output.

---

## 4. Setup and running

### Requirements

- Bento (tested with v1.21.2): <https://github.com/warpstreamlabs/bento/releases>
- Python 3.8+ (standard library only)
- An MQTT broker that the sensor publishes to (Mosquitto or similar)
- IRIS subscribed to the destination MQTT broker (topic `rid/kgeojson/#` by default)
- Optional: the IRIS KGeoJSON Service's `TrackPortNumber` reachable over UDP from the Bento host, if the UDP output is used

### Sensor configuration (`/root/dronescout.conf`)

```ini
[mqtt]
host = <your broker>
port = 1883            ; 8883 with ssl = 1
compression = none     ; lzma also works; none is easier to debug on a LAN
transmit_mode = 2      ; default; the manual advises against mode 1 (spoofing risk)
aggregate_data = 1
[rid]
extra = 1
```

Edit it with `overlayroot-chroot`, then `nano /root/dronescout.conf`, `exit` and `reboot` (manual §2). The default topic is `/sensor/<sensorID>/upload`; note the leading slash.

To check that the sensor is publishing before starting Bento:

```sh
mosquitto_sub -h <broker> -t '/sensor/#' -v
```

If nothing arrives, check `/tmp/mqtt.log` on the sensor. Its last line should read `INFO MQTT: connecting and publishing data`.

### IRIS

Point IRIS at the destination MQTT broker and subscribe it to the KGeoJSON topic (`rid/kgeojson/#` by default; change it with `KG_TOPIC_PREFIX`). If the UDP option is used, note the KGeoJSON Service's `TrackPortNumber` in its `settings\Config.xml`.

Either way, set `ExpiryTimeout` to suit Remote ID update rates. The sensor sends about 4 updates per second with the default `transmit_mode_2_interval_ms = 250`, but drones can drop out briefly, so a few seconds of timeout is reasonable.

### Run

Put the pipeline files in one directory and run:

```sh
GEOID_UNDULATION_M=<N for the site, metres> \
SRC_MQTT_URL=tcp://broker:1883 \
DST_MQTT_URL=tcp://broker:1883 \
SRC_CLIENT_ID=bento-rid-in-01 \
DST_CLIENT_ID=bento-rid-out-01 \
bento -c rid_pipeline.yaml
```

| Variable | Default | Purpose |
|---|---|---|
| `GEOID_UNDULATION_M` | `0` | Geoid undulation N at the site in metres (HAE − MSL). **Set this per site.** |
| `KG_SOURCE` | `Remote ID <sensor ID>` | `source` shown in IRIS |
| `KG_TYPE` | `contact` | Symbology type (`contact`, `own`, `reference`, `origin`) |
| `SRC_MQTT_URL` | `tcp://localhost:1883` | Broker the sensors publish to |
| `DST_MQTT_URL` | `tcp://localhost:1883` | Broker IRIS subscribes to (KGeoJSON tracks, status) |
| `KG_TOPIC_PREFIX` | `rid/kgeojson` | KGeoJSON topic prefix → `<prefix>/<sensor>/<track key>` |
| `AUX_TOPIC_PREFIX` | `rid` | Other messages → `<prefix>/<kind>/<sensor>` |
| `IRIS_MQTT_ENABLED` | `true` | Set to `false` to stop publishing tracks on MQTT (e.g. UDP-only to IRIS). Status and other messages still go to MQTT. |
| `IRIS_UDP_ENABLED` | `false` | Set to `true` to send tracks over UDP |
| `IRIS_UDP_ADDR` | `127.0.0.1:5000` | UDP target: IRIS KGeoJSON Service host and `TrackPortNumber` |
| `SRC_CLIENT_ID` / `DST_CLIENT_ID` | `bento-rid-in` / `bento-rid-out` | MQTT client IDs. **Must be unique per instance.** |
| `RID_PYTHON` | `python3` | Python command Bento uses to run the decoder. On Windows use `python` or `py`. |
| `RID_DECODER` | `./rid_decode.py` | Path to the decoder |
| `KGEOJSON_MAP` | `./kgeojson.blobl` | Path to the mapping |

To check what IRIS will receive:

```sh
mosquitto_sub -h <broker> -t 'rid/kgeojson/#' -v    # KGeoJSON tracks
mosquitto_sub -h <broker> -t 'rid/#' -v             # everything
```

**Delivery modes to IRIS:**

| Mode | Settings |
|---|---|
| MQTT only (default) | nothing to set |
| MQTT and UDP | `IRIS_UDP_ENABLED=true IRIS_UDP_ADDR=<iris-host>:<TrackPortNumber>` |
| UDP only | `IRIS_UDP_ENABLED=true IRIS_MQTT_ENABLED=false IRIS_UDP_ADDR=<iris-host>:<TrackPortNumber>` |

Don't set `IRIS_MQTT_ENABLED=false` without enabling UDP, or tracks go nowhere (status messages still publish). To check it without IRIS running, listen on the port:

```sh
nc -ul 5000        # with IRIS_UDP_ENABLED=true IRIS_UDP_ADDR=127.0.0.1:5000
```

### TLS (production)

Set `ssl = 1` on the sensor. In Bento, replace the plain input with a TLS input like the one below; apply the same `tls` block to the output. This example passes `bento lint` but has not been tested against a live TLS broker.

```yaml
input:
  mqtt:
    urls: ["ssl://broker.example:8883"]
    topics: ["/sensor/+/upload"]
    client_id: "bento-rid-in-01"
    qos: 1
    user: "${SRC_MQTT_USER:}"
    password: "${SRC_MQTT_PASS:}"
    tls:
      enabled: true
      root_cas_file: /etc/bento/certs/ca.crt
      client_certs:
        - cert_file: /etc/bento/certs/client.crt
          key_file: /etc/bento/certs/client.key
```

---

## 5. Testing with the simulator (no sensor needed)

`sim/` produces realistic sensor traffic using the real Open Drone ID library, so the pipeline and IRIS can be tested without a sensor or drones.

### Do I need to build the C simulator?

**No, not to run the default demo.** The simulator has two parts that run at different times:

| Part | When it runs | What it needs |
|---|---|---|
| `odid_sim.c` (scenario generator) | **Offline, once**, only to create or change a scenario. Writes `sim.jsonl`. | A C compiler (gcc, or MinGW on Windows) and curl |
| `rid_sim_publish.py` (fake sensor) | **Every demo.** Reads `sim.jsonl` and publishes it to MQTT in real time. | Python 3 and `paho-mqtt` |

The default Oslo Gardermoen scenario is already generated and included as `sim/sim.jsonl`, so on any machine (Windows, Linux or macOS) you only need Python, an MQTT broker and Bento.

Build and run `odid_sim` only if you want a different scenario: another airport (`--lat`, `--lon`, `--ground-msl`, `--geoid`) or a different length (`--seconds`). After regenerating, commit the new `sim.jsonl` so other machines can use it without a compiler.

### How it works

1. `odid_sim.c` (run offline, only when generating a scenario) is compiled against `opendroneid.h`/`.c` at the same commit the DroneScout firmware uses. For each drone, every second, it encodes real 25-byte F3411 messages (BasicID, Location, System, OperatorID, SelfID), then decodes them on the receiver side with the library's own `decodeOpenDroneID()`, which is how the sensor builds `UASdata`. BLE legacy drones deliver one message type per frame (ID and operator arrive less often); the other links deliver a full set every time. It writes one JSON line per drone per second with the ground truth and the `UASdata` bytes, and prints the compiler's struct layout to `layout.txt`.
2. `rid_sim_publish.py` acts as the sensor. Every second it publishes one message containing all drones detected in that second (like `transmit_mode 2`) to `/sensor/<id>/upload`, LZMA-compressed or plain, plus a status message every 60 s. By default it **restamps all times to the current clock** (the JSON timestamp, and the fix time and system time inside `UASdata`), so tracks are live in IRIS.

### Default scenario: Oslo Airport Gardermoen (ENGM)

Four drones around the airport reference point (60.1939 N, 11.1004 E, approximate), ground at 208 m (681 ft) MSL, geoid N = 38.3 m. **All drones stay at or below 120 m (394 ft) AGL.**

| Track | Pattern | Where | AGL | Speed | Link / airframe |
|---|---|---|---|---|---|
| `rid:SIM0OSL…0001` | Orbit, r = 250 m | 1.2 km east of the ARP, near the eastern runway | 100 m (328 ft) | ~8 m/s | BLE legacy / DJI Mavic 3 Enterprise |
| `rid:SIM0OSL…0002` | Transit, 6 km legs west↔east | 3.5 km north of the ARP | 110 m (361 ft) | ~20 m/s | WiFi beacon / Autel EVO II Pro |
| `rid:SIM0OSL…0003` | Take off, climb 3 m/s, hover, descend, land | 2 km south-west | 0–115 m (0–377 ft) | vertical only | BLE long range / Skydio X10 |
| `rid:SIM0OSL…0004` | Survey lawnmower, 400 × 300 m | 2.5 km south | 60 m (197 ft) | ~6–10 m/s | WiFi NaN / Wingtra WingtraOne |

- UAS IDs start with `SIM` and operator IDs with `NOR-SIM`, so simulated tracks can't be mistaken for real detections if a real sensor is running at the same time.
- Every pattern returns to its starting point at the end of the scenario (600 s by default), so `--loop` runs continuously without jumps.
- The operator position (take-off point) is set for each drone and appears in the IRIS metadata panel.
- Assumes flat ground at airport elevation, and barometric altitude ≈ MSL (standard atmosphere).

### Run it (Linux, macOS, WSL)

Using the included default scenario (no build needed):

```sh
cd sim
pip install paho-mqtt
python3 rid_sim_publish.py sim.jsonl --broker <host> --loop     # live, real time, runs until Ctrl-C
```

Only to generate a new scenario (needs gcc + curl; overwrites `sim.jsonl`):

```sh
cd sim
sh build.sh --lat <lat> --lon <lon> --ground-msl <m> --geoid <m>   # writes sim.jsonl + layout.txt
```

Run the pipeline with the same geoid value the simulator used, so altitudes come out right:

```sh
GEOID_UNDULATION_M=38.3 KG_SOURCE="Remote ID OSL" bento -c rid_pipeline.yaml
```

Simulator options (`sh build.sh <options>` or `./odid_sim <options>`):

| Option | Default | Meaning |
|---|---|---|
| `--lat`, `--lon` | 60.1939, 11.1004 | Scenario centre (drone positions are offsets from it) |
| `--ground-msl` | 208 | Ground elevation, m MSL |
| `--geoid` | 38.3 | Geoid undulation N, m (must match the pipeline's `GEOID_UNDULATION_M`) |
| `--seconds` | 600 | Scenario length (≥ 60) |

Publisher options: `--sensor <id>`, `--rate <x>` (speed-up; 1 = real time), `--loop`, `--plain`, `--no-restamp` (keep the simulator's fixed time base; used for accuracy testing), `--status-every <s>`, `--port`.

### Run it on Windows

**Option 1: native Windows (no compiler needed).** Uses the included `sim/sim.jsonl`; the C simulator is not built or run.

1. Install Python 3 from python.org, then run `py -m pip install paho-mqtt`.
2. Install Mosquitto for Windows from mosquitto.org, or use any MQTT broker you already have. The Mosquitto service listens on localhost:1883 by default.
3. Download `bento_1.21.2_windows_amd64.tar.gz` from the Bento GitHub releases, extract it with `tar -xzf bento_1.21.2_windows_amd64.tar.gz`, and put `bento.exe` in the repo folder.
4. PowerShell window 1 (pipeline):

   ```powershell
   cd dronescout-rid-kgeojson
   $env:RID_PYTHON = "py"            # or "python"
   $env:GEOID_UNDULATION_M = "38.3"
   $env:KG_SOURCE = "Remote ID OSL"
   $env:SRC_MQTT_URL = "tcp://localhost:1883"
   $env:DST_MQTT_URL = "tcp://localhost:1883"
   .\bento.exe -c rid_pipeline.yaml
   ```

5. PowerShell window 2 (simulated sensor):

   ```powershell
   cd dronescout-rid-kgeojson\sim
   py rid_sim_publish.py sim.jsonl --broker localhost --loop
   ```

6. PowerShell window 3 (check the output): `& "C:\Program Files\mosquitto\mosquitto_sub.exe" -h localhost -t "rid/kgeojson/#" -v`

Only if you want a different scenario (another site, length or geoid): build `odid_sim.exe` with MSYS2's MinGW gcc (MSVC won't work, because the simulator uses `getopt_long`). In an MSYS2 MinGW64 shell, `sh build.sh` works as-is. Alternatively, download `opendroneid.h` and `opendroneid.c` (URLs in `build.sh`), then run `gcc -O1 -o odid_sim.exe odid_sim.c opendroneid.c -lm`.

**Option 2: WSL2.** Install Ubuntu under WSL2 and follow the Linux instructions. Everything runs unchanged.

**Windows status:** the simulator cross-compiles cleanly with MinGW-w64, including the compile-time check that the struct is 920 bytes. Two Windows-specific issues were found and fixed: `long` is 32-bit on Windows, which overflowed millisecond timestamps, and Python's text-mode stdout would have written `\r\n` line endings to Bento. The Windows builds have **not been run on a Windows machine yet**. The Python and Bento parts are cross-platform and should work as described.

### Test results (Bento 1.21.2, Mosquitto, x86_64)

**Oslo scenario, accuracy run** (600 s scenario at 60× speed, simulator time base so each output can be matched to ground truth):
- 2400 of 2400 tracks delivered (4 drones × 600 s), no loss, no Bento errors, all valid against `kgeojson.schema.json`.
- Lat/lon within 1e-7° of truth. Altitude in ft MSL within about 1 ft of truth, which is the F3411 altitude resolution (0.5 m).
- Maximum height above ground: 115 m (377 ft). Altitudes ranged from 681 ft MSL (drone on the ground) to 1059 ft MSL.

**Oslo scenario, live run** (real time, restamped, 20 s):
- 80 of 80 tracks (4 drones × 20 s), all valid.
- Track timestamps were current, arriving 350 ms after the fix time (the simulator puts the fix 300 ms before receipt, plus about 50 ms through the pipeline).

**Earlier runs:**
- Delivery modes: MQTT only (default) sent no UDP; MQTT and UDP sent tracks to both; UDP only sent all tracks to UDP, none to MQTT, while status still went to MQTT. With `KG_TOPIC_PREFIX` changed, tracks moved to the new prefix.
- Each UDP datagram contained exactly one JSON object (under 1 KB). The ICD's own example also validates against the schema.
- An ID-only detection was dropped. A payload with a wrong-size `UASdata` and a corrupt JSON tail was logged as WARN and dropped, while the valid objects in the same payload were still processed.
- Bugs found and fixed during testing: wrong System offsets in an early version of the decoder; Bento's default `lines` codec sent the trailing newline as a separate UDP datagram (UDP now uses `all-bytes`); a failed mapping would have passed the raw record through (now caught and dropped).
- Throughput in the sandbox was roughly 150–200 tracks per second, far above what a demo needs (4 drones at 1 Hz = 4 tracks/s).

### Not yet verified

- **A real IRIS KGeoJSON Service.** Output matches the ICD and its example, but hasn't been ingested by IRIS yet.
- **Real sensor data.** The layout is confirmed against the real header, but a capture from a real sensor with a real drone is still the final check. To capture: `mosquitto_sub -h <broker> -t '/sensor/#' -C 20 > capture.bin`
- **Cross-check against the vendor subscriber.** Run `mqtt_sub.py` from the BluemarkInnovations repo against the same feed and compare IDs and positions.
- **TLS against a live broker.**

---

## 6. Troubleshooting

| Symptom | Likely cause |
|---|---|
| Nothing appears in IRIS | Check that IRIS is subscribed to the right broker and topic (`mosquitto_sub -t 'rid/kgeojson/#'` should show tracks). If using UDP: check `IRIS_UDP_ENABLED` (and that `IRIS_MQTT_ENABLED=false` wasn't set without it), `IRIS_UDP_ADDR` against `TrackPortNumber`, and firewalls; listen with `nc -ul <port>` to confirm Bento is sending. |
| Tracks in IRIS appear and disappear | Updates are arriving slower than the service's `ExpiryTimeout`. Increase it, or check the sensor's detection rate. |
| Altitudes in IRIS are off by a roughly constant amount (often 100+ ft) | `GEOID_UNDULATION_M` isn't set for the site. |
| Repeated `Connection lost due to: EOF` in the Bento log, with missing messages | Another client is using the same MQTT `client_id`. Give each instance unique IDs. |
| Nothing on `rid/#` | Check that the sensor is publishing (`mosquitto_sub -t '/sensor/#'`). Check the subscription topic; it has a leading slash. |
| `dropped: {"kind":"error",...}` warnings | Corrupt payload, or the `UASdata` size isn't 920 bytes (the firmware's ODID version may have changed). |
| No track for a drone that is clearly detected | The drone isn't sending a valid Location (no GNSS fix, or Location messages not received yet). ID-only detections are dropped by design. |
| `timestamp` is years off | The sensor's clock isn't synced (no NTP, and `set_system_time` is off). |
| `Failed to read subprocess output: file already closed` when Bento stops | Normal during shutdown; it can be ignored. |

---

## 7. Next steps

1. Test against a real IRIS instance over MQTT. Confirm with the IRIS team the topic IRIS subscribes to, and the `source`, `type` and `displayName` conventions.
2. Validate with one real sensor capture.
3. Set `GEOID_UNDULATION_M` for each deployment site.
4. Optional: map ADS-B/UAT `aircraft` messages to KGeoJSON too. They already contain lat/lon, altitude, speed and track, so it's a second small mapping.