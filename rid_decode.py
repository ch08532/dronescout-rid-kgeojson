#!/usr/bin/env python3
"""
DroneScout MQTT payload -> normalized JSON records.

Runs as a Bento `subprocess` processor:
  stdin : length-prefixed (uint32 BE) raw MQTT payloads (plain or LZMA)
  stdout: one line per payload = JSON array of records (may be [])

Handles: LZMA, trailing \\0/\\n, concatenated JSON objects (transmit_mode 2),
and the 920-byte ODID_UAS_Data struct in data.UASdata
(opendroneid-core-c commit 4785de45, ARM64 layout, little-endian).
"""
import base64, json, lzma, struct, sys
from datetime import datetime, timezone

ODID_SIZE = 920

UA_TYPE = ["none", "aeroplane", "helicopter_or_multirotor", "gyroplane",
           "hybrid_lift", "ornithopter", "glider", "kite", "free_balloon",
           "captive_balloon", "airship", "free_fall_parachute", "rocket",
           "tethered_powered_aircraft", "ground_obstacle", "other"]
ID_TYPE = ["none", "serial_number", "caa_registration", "utm_uuid", "session_id"]
STATUS = ["undeclared", "ground", "airborne", "emergency", "rid_system_failure"]
HEIGHT_REF = ["takeoff", "ground"]
OP_LOC_TYPE = ["takeoff", "live_gnss", "fixed"]
# accuracy enums -> upper bound in metres (or m/s); None = unknown
H_ACC = [None, 18520, 7408, 3704, 1852, 926, 555.6, 185.2, 92.6, 30, 10, 3, 1]
V_ACC = [None, 150, 45, 25, 10, 3, 1]
S_ACC = [None, 10, 3, 1, 0.3]
EPOCH_2019 = 1546300800


def enum(table, v):
    return table[v] if 0 <= v < len(table) else v


def cstr(b):
    return b.split(b"\0", 1)[0].decode("ascii", "replace").strip() or None


def inv(v, sentinel):
    return None if v == sentinel else round(v, 7)


def iso(ms):
    if ms is None:
        return None
    return datetime.fromtimestamp(ms / 1000, tz=timezone.utc).isoformat(
        timespec="milliseconds").replace("+00:00", "Z")


def full_fix_time(rx_ms, secs_past_hour):
    """Location.TimeStamp is seconds past the UTC hour; anchor to receive time."""
    if secs_past_hour is None or rx_ms is None:
        return None
    hour = rx_ms - rx_ms % 3_600_000
    t = hour + int(round(secs_past_hour * 10)) * 100   # 0.1 s resolution
    if t - rx_ms > 1_800_000:      # fix belongs to previous hour
        t -= 3_600_000
    return t


def decode_uas(b, rx_ms):
    if len(b) != ODID_SIZE:
        raise ValueError(f"UASdata is {len(b)} bytes, expected {ODID_SIZE}")
    out = {"ids": [], "valid": {}}

    for i in range(2):
        if b[892 + i]:
            ua, idt = struct.unpack_from("<ii", b, i * 32)
            out["ids"].append({"ua_type": enum(UA_TYPE, ua),
                               "id_type": enum(ID_TYPE, idt),
                               "id": cstr(b[i * 32 + 8:i * 32 + 29])})

    if b[894]:
        (st, track, vh, vv, lat, lon, alt_baro, alt_geo, href, height,
         hacc, vacc, bacc, sacc, tsacc, ts) = struct.unpack_from(
            "<i3f2d2fifiiiiif", b, 64)
        has_pos = not (lat == 0 and lon == 0)
        ts = None if ts >= 65535 else ts
        out["status"] = enum(STATUS, st)
        out["position"] = {
            "lat": round(lat, 7), "lon": round(lon, 7),
            "alt_geo_m": inv(alt_geo, -1000), "alt_baro_m": inv(alt_baro, -1000),
            "height_m": inv(height, -1000), "height_ref": enum(HEIGHT_REF, href),
        } if has_pos else None
        out["velocity"] = {"track_deg": inv(track, 361),
                           "speed_h_ms": inv(vh, 255), "speed_v_ms": inv(vv, 63)}
        out["accuracy"] = {"horizontal_m": enum(H_ACC, hacc) if hacc < len(H_ACC) else None,
                           "vertical_m": enum(V_ACC, vacc) if vacc < len(V_ACC) else None,
                           "baro_m": enum(V_ACC, bacc) if bacc < len(V_ACC) else None,
                           "speed_ms": enum(S_ACC, sacc) if sacc < len(S_ACC) else None,
                           "time_s": tsacc * 0.1 if tsacc else None}
        out["fix_time_ms"] = full_fix_time(rx_ms, ts)
        out["fix_time"] = iso(out["fix_time_ms"])

    if b[911]:
        out["self_id"] = cstr(b[780:804])

    if b[912]:
        olt, cls, olat, olon = struct.unpack_from("<ii2d", b, 808)
        oalt, osys_ts = struct.unpack_from("<fI", b, 852)
        out["operator_location"] = {
            "type": enum(OP_LOC_TYPE, olt),
            "lat": round(olat, 7), "lon": round(olon, 7),
            "alt_geo_m": inv(oalt, -1000),
            "system_time": iso((EPOCH_2019 + osys_ts) * 1000) if osys_ts else None,
        } if not (olat == 0 and olon == 0) else None

    if b[913]:
        out["operator_id"] = cstr(b[868:889])

    out["valid"] = {"basic_id": [b[892], b[893]], "location": b[894],
                    "self_id": b[911], "system": b[912], "operator_id": b[913]}
    return out


def split_objects(text):
    """Yield JSON objects from a payload that may contain several concatenated."""
    dec, i, n = json.JSONDecoder(), 0, len(text)
    while i < n:
        while i < n and text[i] in " \r\n\t\0":
            i += 1
        if i >= n:
            break
        obj, i = dec.raw_decode(text, i)
        yield obj


def payload_to_text(raw):
    try:
        return raw.decode("utf-8")
    except UnicodeDecodeError:
        return lzma.decompress(raw).decode("utf-8")


def normalize(obj):
    if "data" in obj:
        d = obj["data"]
        rx = d.get("timestamp")
        rec = {"kind": "rid", "sensor_id": d.get("sensor ID"),
               "rx_time_ms": rx, "rx_time": iso(rx),
               "rssi": d.get("RSSI"), "channel": d.get("channel"),
               "mac": d.get("MAC address"), "link": d.get("type"),
               "extra": d.get("extra")}
        try:
            rec["uas"] = decode_uas(base64.b64decode(d["UASdata"]), rx)
        except Exception as e:           # keep the record, flag the problem
            rec["uas"], rec["error"] = None, str(e)
        return rec
    for key, kind in (("aircraft", "aircraft"), ("status", "status"),
                      ("location", "sensor_location"),
                      ("mobile network", "mobile_network")):
        if key in obj:
            return {"kind": kind, **obj[key]}
    return {"kind": "unknown", "raw": obj}


def handle(raw):
    """Decode what we can; a bad object doesn't discard the good ones before it."""
    out = []
    try:
        for o in split_objects(payload_to_text(raw)):
            out.append(normalize(o))
    except Exception as e:
        out.append({"kind": "error", "error": str(e)})
    return out


def main():
    # binary stdout: avoids \r\n line endings on Windows
    inp, out = sys.stdin.buffer, sys.stdout.buffer
    while True:
        hdr = inp.read(4)
        if len(hdr) < 4:
            return
        (n,) = struct.unpack(">I", hdr)
        out.write(json.dumps(handle(inp.read(n)), separators=(",", ":")).encode() + b"\n")
        out.flush()


if __name__ == "__main__":
    main()
