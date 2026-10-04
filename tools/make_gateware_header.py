#!/usr/bin/env python3
"""Transformă un gateware Radioberry (.rbf) în header C.

    python tools/make_gateware_header.py gateware/radioberry_CL025.rbf
    python tools/make_gateware_header.py gateware/ragchewberry_pio_CL025.rbf rb_bringup_pio "github.com/wp3dn/RagchewBerry components/gateware/bitstreams/CL025"

Al doilea argument = sketch-ul din firmware/ (implicit rb_bringup), al treilea = originea scrisă în
header. Scrie firmware/<sketch>/gateware_cl025.h (exclus din git: binar terț).
"""
import hashlib, os, sys

src = sys.argv[1] if len(sys.argv) > 1 else "gateware/radioberry_CL025.rbf"
root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sketch = sys.argv[2] if len(sys.argv) > 2 else "rb_bringup"
origin = sys.argv[3] if len(sys.argv) > 3 else "github.com/pa3gsb/Radioberry-2.x SBC/rpi-5/archive/releases/dev/CL025"
out = os.path.join(root, "firmware", sketch, "gateware_cl025.h")

d = open(src, "rb").read()
sha = hashlib.sha256(d).hexdigest()
with open(out, "w", newline="\n", encoding="utf-8") as f:
    f.write("// Gateware Radioberry CL025 — generat de tools/make_gateware_header.py\n")
    f.write(f"// sursa: {os.path.basename(src)}, {len(d)} octeti, sha256 {sha}\n")
    f.write(f"// Origine: {origin}\n")
    f.write("// Binar tert, folosire personala — nu se publica (vezi .gitignore).\n")
    f.write("#pragma once\n#include <stdint.h>\n")
    f.write(f"static const uint32_t GATEWARE_LEN = {len(d)};\n")
    f.write(f'static const char GATEWARE_SHA256[] = "{sha}";\n')
    f.write("static const uint8_t GATEWARE[] = {\n")
    for i in range(0, len(d), 24):
        f.write("  " + ",".join(f"0x{b:02X}" for b in d[i:i + 24]) + ",\n")
    f.write("};\n")
print(f"{out}: {len(d)} octeti, sha256 {sha[:16]}...")
