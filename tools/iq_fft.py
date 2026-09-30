#!/usr/bin/env python3
"""Captură IQ de la firmware-ul rb_bringup și spectru (checkpoint-ul „faza 2" din README).

Pornește fluxul binar (comanda 'b'), citește perechi int16 I/Q la 48 kHz, face FFT și
afișează vârful. Cu --plot desenează spectrul.

    pip install pyserial numpy matplotlib
    python iq_fft.py COM5 --secunde 2 --plot
"""
import argparse, sys, time
import numpy as np
import serial

FS = 48000

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--secunde", type=float, default=1.0)
    ap.add_argument("--frecventa", type=int, help="setează întâi frecvența RX1, în Hz")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--salveaza", help="salvează IQ brut (.npy)")
    a = ap.parse_args()

    s = serial.Serial(a.port, 115200, timeout=1)
    time.sleep(0.3)
    s.reset_input_buffer()
    if a.frecventa:
        s.write(f"f {a.frecventa}\n".encode()); time.sleep(0.2)
    s.write(b"b\n")                                   # pornește fluxul binar
    time.sleep(0.2)
    s.reset_input_buffer()                            # aruncă textul de stare rămas

    need = int(a.secunde * FS) * 4
    data = bytearray()
    t0 = time.time()
    while len(data) < need and time.time() - t0 < a.secunde * 3 + 2:
        data += s.read(min(65536, need - len(data)))
    s.write(b"b\n")                                   # oprește fluxul
    s.close()

    n = len(data) // 4
    if n < 1024:
        sys.exit(f"prea puține date: {len(data)} octeți — FPGA încărcat? SYNC pe OLED?")
    raw = np.frombuffer(bytes(data[:n * 4]), dtype="<i2").reshape(-1, 2).astype(np.float64)
    iq = raw[:, 0] + 1j * raw[:, 1]
    print(f"{n} eșantioane în {time.time() - t0:.2f} s (așteptat ~{FS}/s)")
    print(f"RMS I {raw[:, 0].std():.1f}  Q {raw[:, 1].std():.1f}  "
          f"DC I {raw[:, 0].mean():.1f}  Q {raw[:, 1].mean():.1f}")
    if a.salveaza:
        np.save(a.salveaza, iq)

    N = 1 << int(np.log2(min(len(iq), 65536)))
    win = np.blackman(N)
    spec = np.fft.fftshift(np.fft.fft(iq[:N] * win))
    db = 20 * np.log10(np.abs(spec) / N + 1e-12)
    f = np.fft.fftshift(np.fft.fftfreq(N, 1 / FS))
    k = int(np.argmax(db))
    print(f"vârf: {f[k]:+.1f} Hz față de frecvența RX, {db[k]:.1f} dB "
          f"(podea ~{np.median(db):.1f} dB)")
    if a.plot:
        import matplotlib.pyplot as plt
        plt.plot(f / 1000, db)
        plt.xlabel("kHz față de RX1"); plt.ylabel("dB"); plt.grid(True)
        plt.title("Radioberry_Pico2 — spectru IQ")
        plt.show()

if __name__ == "__main__":
    main()
