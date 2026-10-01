"""Înregistrează IQ-ul de la rx_audio (USB) într-un WAV pe care HDSDR / SDR# / SDR++ îl redau direct.

    python tools/iq_record.py                 # 30 s, portul plăcii găsit singur, în folderul curent
    python tools/iq_record.py -t 120 -p COM12 -o D:\\IQ

Firmware: comanda q1 pornește fluxul (perechi I,Q pe 24 de biți, 48 kHz), q0 îl oprește.
Fișierul: WAV stereo 24 de biți (stânga = I, dreapta = Q), cu chunk `auxi` (frecvența centrală, ora)
și numele în formatul HDSDR (…_<kHz>kHz_RF.wav), ca HDSDR să afișeze frecvența corectă.
Nu schimba frecvența în timpul înregistrării: în fișier rămâne cea de la pornire.
"""
import argparse
import datetime as dt
import os
import struct
import sys
import time

import serial
import serial.tools.list_ports

FS = 48000
PAIR = 6  # octeți pe pereche I,Q (2 × 24 de biți)


def systemtime(t):
    """SYSTEMTIME Windows (16 octeți), cum îl așteaptă chunk-ul auxi."""
    return struct.pack("<8H", t.year, t.month, t.isoweekday() % 7, t.day,
                       t.hour, t.minute, t.second, t.microsecond // 1000)


def write_wav(path, data, freq, t_start, t_stop):
    auxi = (systemtime(t_start) + systemtime(t_stop)
            + struct.pack("<9I", freq, FS, 0, FS, 0, 0, 0, 0, 0) + b"\0" * 96)
    fmt = struct.pack("<HHIIHH", 1, 2, FS, FS * PAIR, PAIR, 24)
    body = (b"WAVE" + b"fmt " + struct.pack("<I", len(fmt)) + fmt
            + b"auxi" + struct.pack("<I", len(auxi)) + auxi
            + b"data" + struct.pack("<I", len(data)) + data)
    with open(path, "wb") as f:
        f.write(b"RIFF" + struct.pack("<I", len(body)) + body)


def find_port():
    """Portul plăcii: VID 2E8A (Raspberry Pi); PID 10F1 = rx_audio cu microfon USB."""
    ports = [p for p in serial.tools.list_ports.comports() if p.vid == 0x2E8A]
    ports.sort(key=lambda p: p.pid != 0x10F1)
    if not ports:
        sys.exit("placa nu e conectată (niciun port cu VID 2E8A)")
    return ports[0].device


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", help="implicit: găsit după VID/PID")
    ap.add_argument("-t", "--seconds", type=float, default=30)
    ap.add_argument("-o", "--outdir", default=".")
    a = ap.parse_args()

    port = a.port or find_port()
    print("port:", port)
    s = serial.Serial(port, 115200, timeout=1)
    s.dtr = True
    s.write(b"q0\n"); time.sleep(0.3); s.reset_input_buffer()

    s.write(b"q1\n")
    hdr = b""
    t0 = time.time()
    while not hdr.startswith(b"IQ24"):
        hdr = s.readline().strip()
        if time.time() - t0 > 3:
            sys.exit("placa nu răspunde la q1 (firmware vechi? port greșit?)")
    _, rate, freq = hdr.decode().split()
    freq = int(freq)
    assert int(rate) == FS

    t_start = dt.datetime.now(dt.timezone.utc)
    want = int(a.seconds * FS) * PAIR
    buf = bytearray()
    print(f"înregistrez {a.seconds:g} s pe {freq / 1e6:.6f} MHz ...")
    last = time.time()
    while len(buf) < want:
        buf += s.read(min(65536, want - len(buf)))
        if time.time() - last > 1:
            last = time.time()
            print(f"\r  {len(buf) / PAIR / FS:5.1f} s", end="", flush=True)
    t_stop = dt.datetime.now(dt.timezone.utc)
    s.write(b"q0\n"); time.sleep(0.3)
    tail = s.read(s.in_waiting).decode(errors="ignore")
    s.close()

    real = (t_stop - t_start).total_seconds()
    print(f"\r  {len(buf) / PAIR / FS:.1f} s de semnal în {real:.1f} s reale")
    for line in tail.splitlines():
        if "pierdute" in line:
            print(" ", line.strip())

    name = f"HDSDR_{t_start:%Y%m%d_%H%M%S}Z_{freq / 1000:.0f}kHz_RF.wav"
    path = os.path.join(a.outdir, name)
    write_wav(path, bytes(buf[:len(buf) // PAIR * PAIR]), freq, t_start, t_stop)
    print("salvat:", os.path.abspath(path))


if __name__ == "__main__":
    main()
