#!/usr/bin/env python3
"""
Fardriver ND72680 telemetry decoder.

Connects to the controller over BLE, verifies every frame, decodes the values
and serves them as JSON on a local WebSocket for the dashboard.

Verified against a real capture from YuanQuFOC096 on 2026-09-08:
21 of 21 frames passed CRC, and pack voltage (79.7 V) cross-checked against
the separately-reported state of charge (80%) on a 24S LiFePO4 pack.

    pip install bleak websockets

    python3 fardriver_decoder.py                 # connect and print
    python3 fardriver_decoder.py --ws            # also serve the dashboard
    python3 fardriver_decoder.py --log ride.csv  # record every frame
    python3 fardriver_decoder.py --replay f.txt  # decode a saved hex dump
"""

import argparse
import asyncio
import json
import math
import re
import struct
import sys
import time

# --------------------------------------------------------------------------
# YOUR BIKE. These are the only numbers you should need to change.
# --------------------------------------------------------------------------
POLE_PAIRS   = 5       # QS138 pole pairs. Wrong value = wrong speed, nothing else.
GEAR_RATIO   = 4.0     # motor turns per wheel turn (your chain reduction)
WHEEL_CIRC_M = 1.9     # rolling circumference in metres
RPM_IS_ELECTRICAL = True   # set False if speed reads POLE_PAIRS times too low

DEVICE_NAME  = "YuanQuFOC096"
CHAR_UUID    = "0000ffec-0000-1000-8000-00805f9b34fb"

# The Fardriver app sends this repeatedly. Your controller streams without it,
# so it is off by default. Turn it on if the data ever stops mid-ride.
KEEPALIVE      = bytes.fromhex("AA13EC07095F18E7")
SEND_KEEPALIVE = False
KEEPALIVE_S    = 1.0

WS_HOST, WS_PORT = "localhost", 8765

# --------------------------------------------------------------------------
# Protocol
# --------------------------------------------------------------------------

# Byte 1 carries flags in the top two bits and an index in the low six.
# The index looks up a memory address, which says what the 12 payload bytes are.
FLASH_READ_ADDR = [
    0xE2, 0xE8, 0xEE, 0x00, 0x06, 0x0C, 0x12,
    0xE2, 0xE8, 0xEE, 0x18, 0x1E, 0x24, 0x2A,
    0xE2, 0xE8, 0xEE, 0x30, 0x5D, 0x63, 0x69,
    0xE2, 0xE8, 0xEE, 0x7C, 0x82, 0x88, 0x8E,
    0xE2, 0xE8, 0xEE, 0x94, 0x9A, 0xA0, 0xA6,
    0xE2, 0xE8, 0xEE, 0xAC, 0xB2, 0xB8, 0xBE,
    0xE2, 0xE8, 0xEE, 0xC4, 0xCA, 0xD0,
    0xE2, 0xE8, 0xEE, 0xD6, 0xDC, 0xF4, 0xFA,
]


def crc16(data, length=14, init=0x7F3C):
    """CRC-16/MODBUS with a non-standard seed. Stored little-endian in 14-15."""
    c = init
    for i in range(length):
        c ^= data[i]
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c


def crc_ok(f):
    return len(f) == 16 and crc16(f) == (f[15] << 8) | f[14]


def phase_amps(raw):
    """24-bit big-endian, and it needs a square root. Documented, not yet
    confirmed on your bike -- the capture was taken with the motor stopped."""
    return 1.953125 * math.sqrt(raw) if raw > 0 else 0.0


class State:
    """Everything decoded so far. Frames arrive one address at a time, so the
    dashboard always reads the most recent value for each field."""

    def __init__(self):
        self.d = {}
        self.frames = 0
        self.bad = 0
        self.watt_sum = 0.0
        self.watt_n = 0
        self.started = time.time()

    def feed(self, f):
        if not crc_ok(f):
            self.bad += 1
            return None
        self.frames += 1
        idx = f[1] & 0x3F
        if idx >= len(FLASH_READ_ADDR):
            return None
        addr = FLASH_READ_ADDR[idx]
        p = f[2:14]
        d = self.d

        if addr == 0xE2:
            d["rpm_raw"] = struct.unpack_from("<H", p, 6)[0]
            d["gear"] = (p[0] >> 2) & 0x03
            d["reverse"] = bool(p[1] & 0x01)
            d["brake"] = bool(p[0] & 0x20)
            mech = d["rpm_raw"] / POLE_PAIRS if RPM_IS_ELECTRICAL else d["rpm_raw"]
            d["rpm"] = mech
            d["speed"] = (mech / GEAR_RATIO) * WHEEL_CIRC_M * 60 / 1000

        elif addr == 0xE8:
            d["volts"] = struct.unpack_from("<h", p, 0)[0] / 10.0
            d["current"] = struct.unpack_from("<h", p, 4)[0] / 4.0
            d["power"] = d["volts"] * d["current"] / 1000.0
            d["regen"] = d["current"] < -0.5
            self.watt_sum += abs(d["power"]) * 1000
            self.watt_n += 1
            d["avg_watts"] = self.watt_sum / self.watt_n

        elif addr == 0xEE:
            d["phaseA"] = phase_amps((p[4] << 16) | (p[5] << 8) | p[6])
            d["phaseC"] = phase_amps((p[7] << 16) | (p[8] << 8) | p[9])

        elif addr == 0xD6:
            d["mosTemp"] = struct.unpack_from("<h", p, 10)[0]

        elif addr == 0xF4:
            d["motTemp"] = struct.unpack_from("<h", p, 0)[0]
            d["soc"] = p[3]

        elif addr == 0x7C:
            d["odo_raw"] = struct.unpack_from("<H", p, 10)[0]

        elif addr in (0x88, 0xA6):
            txt = bytes(p).split(b"\x00")[0].decode("ascii", "ignore").strip()
            if txt:
                d["model" if addr == 0xA6 else "serial"] = txt

        d["frames"] = self.frames
        d["link"] = True
        if "mosTemp" in d and "motTemp" in d:
            d["tempWarn"] = d["mosTemp"] > 110 or d["motTemp"] > 125
        if "soc" in d:
            d["battLow"] = d["soc"] < 20
        return addr

    def line(self):
        d = self.d
        g = lambda k, f="%s", dflt="--": (f % d[k]) if k in d else dflt
        return ("v=%-6s A=%-7s kW=%-6s rpm=%-6s km/h=%-6s mos=%-4s mot=%-4s "
                "soc=%-4s phA=%-6s frames=%d bad=%d" % (
                    g("volts", "%.1f"), g("current", "%.1f"), g("power", "%.2f"),
                    g("rpm", "%.0f"), g("speed", "%.1f"), g("mosTemp", "%d"),
                    g("motTemp", "%d"), g("soc", "%d"), g("phaseA", "%.0f"),
                    self.frames, self.bad))


# --------------------------------------------------------------------------
# WebSocket for the dashboard
# --------------------------------------------------------------------------
clients = set()


async def ws_handler(ws):
    clients.add(ws)
    try:
        await ws.wait_closed()
    finally:
        clients.discard(ws)


async def broadcast(state):
    """Push at 10 Hz. The dashboard accepts any subset of the keys."""
    while True:
        await asyncio.sleep(0.1)
        if clients and state.d:
            msg = json.dumps(state.d)
            for c in list(clients):
                try:
                    await c.send(msg)
                except Exception:
                    clients.discard(c)


# --------------------------------------------------------------------------
# Live BLE
# --------------------------------------------------------------------------
async def run_live(args):
    from bleak import BleakClient, BleakScanner

    state = State()
    logf = open(args.log, "w") if args.log else None
    if logf:
        logf.write("timestamp,hex,addr\n")

    print("Scanning for %s ..." % DEVICE_NAME)
    dev = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=20.0)
    if dev is None:
        print("Not found. Is the bike powered on and the Fardriver app closed?")
        return
    print("Found %s (%s). Connecting..." % (dev.name, dev.address))

    def on_frame(_, data):
        addr = state.feed(bytes(data))
        if logf:
            logf.write("%.3f,%s,%s\n" % (
                time.time(), bytes(data).hex(),
                ("0x%02X" % addr) if addr else ""))

    async with BleakClient(dev) as client:
        await client.start_notify(CHAR_UUID, on_frame)
        print("Subscribed. Ctrl-C to stop.\n")

        tasks = []
        if args.ws:
            import websockets
            await websockets.serve(ws_handler, WS_HOST, WS_PORT)
            tasks.append(asyncio.create_task(broadcast(state)))
            print("Dashboard feed on ws://%s:%d\n" % (WS_HOST, WS_PORT))

        last_ka = 0.0
        try:
            while True:
                await asyncio.sleep(0.25)
                if SEND_KEEPALIVE and time.time() - last_ka > KEEPALIVE_S:
                    await client.write_gatt_char(CHAR_UUID, KEEPALIVE, response=False)
                    last_ka = time.time()
                sys.stdout.write("\r" + state.line())
                sys.stdout.flush()
        except (KeyboardInterrupt, asyncio.CancelledError):
            pass
        finally:
            for t in tasks:
                t.cancel()
            await client.stop_notify(CHAR_UUID)
            if logf:
                logf.close()
            print("\nStopped. %d frames, %d bad." % (state.frames, state.bad))


# --------------------------------------------------------------------------
# Replay a saved dump (any text containing 16-byte hex frames)
# --------------------------------------------------------------------------
def run_replay(path):
    state = State()
    text = open(path).read()
    hexes = re.findall(r"(?:[0-9A-Fa-f]{2}[-: ]?){16}", text)
    print("Found %d candidate frames\n" % len(hexes))
    for h in hexes:
        f = bytes.fromhex(re.sub(r"[^0-9A-Fa-f]", "", h))
        if len(f) != 16 or f[0] != 0xAA:
            continue
        addr = state.feed(f)
        print("0x%02X  %s" % (addr, f.hex(" ")) if addr else "bad  " + f.hex(" "))
    print("\n%d good, %d failed CRC" % (state.frames, state.bad))
    print("\nDecoded state:")
    for k in sorted(state.d):
        print("  %-12s %s" % (k, state.d[k]))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--ws", action="store_true", help="serve JSON for the dashboard")
    ap.add_argument("--log", metavar="FILE", help="record every frame to CSV")
    ap.add_argument("--replay", metavar="FILE", help="decode a saved hex dump instead")
    a = ap.parse_args()

    if a.replay:
        run_replay(a.replay)
    else:
        try:
            asyncio.run(run_live(a))
        except KeyboardInterrupt:
            pass
