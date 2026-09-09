MOTORCYCLE CLUSTER
==================

moto-cluster-tablet.html
    The dashboard. Sized 1920x1200 for the Galaxy Tab A9+ 5G.
    Talks to the Fardriver controller over Bluetooth by itself.
    No PC needed.

    IMPORTANT: Chrome blocks Bluetooth on files opened from local
    storage. This file must be served over https. Upload it to
    GitHub Pages (free) and open that URL on the tablet.

    Bike settings are near the bottom of the file, in the script:
        POLE_PAIRS    = 5      QS138 pole pairs
        GEAR_RATIO    = 4.0    motor turns per wheel turn
        WHEEL_CIRC_M  = 1.9    rolling circumference, metres
    Only speed depends on these. Calibrate against GPS.

    Screen shape is one line in the stylesheet:
        :root { --ar: 1920 / 1200; }


fardriver_decoder.py
    PC-only tool. Same decoder in Python, for logging rides to CSV
    and analysing them. Not needed for the tablet.

        pip install bleak websockets
        python3 fardriver_decoder.py --log ride.csv


PROTOCOL, verified against this controller on 2026-09-08
--------------------------------------------------------
    Device        YuanQuFOC096
    Service       0xFFE0
    Characteristic 0xFFEC   (notify)
    Frame         16 bytes, starts 0xAA
    Byte 1        top 2 bits = flags (2 = status)
                  low 6 bits = index into flash_read_addr
    Checksum      CRC-16/MODBUS, poly 0xA001, seed 0x7F3C,
                  over bytes 0-13, stored little-endian in 14-15

    Values (offsets into the 12-byte payload):
        0xE8  volts     int16 LE @0  / 10
        0xE8  current   int16 LE @4  / 4
        0xE2  rpm       uint16 LE @6
        0xD6  mos temp  int16 LE @10
        0xF4  motor temp int16 LE @0
        0xF4  soc       uint8 @3
        0xEE  phase A   uint24 BE @4, value = 1.953125 * sqrt(raw)
        0xEE  phase C   uint24 BE @7, same formula

    Phase current is the one field not yet confirmed on this bike -
    the capture was taken with the motor stopped.
