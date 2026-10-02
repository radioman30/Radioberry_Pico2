# PROTOCOL.md — interfața host ↔ FPGA Radioberry v2

**Sursa:** driverul PA3GSB pentru Raspberry Pi 5, modul „pio-mode” (decembrie 2025):
`pa3gsb/Radioberry-2.x` → `SBC/rpi-5/device_driver/pio-mode/driver/`
(`rb2-rx-stream.c`, `rb2-tx-stream.c`, `rb2-trx-control.[ch]`, `rb2-load-fpga.[ch]`).

Pi 5 face I/O-ul prin cipul **RP1, care are PIO de aceeași arhitectură ca RP2040/RP2350**.
Programele PIO de mai jos sunt deci portabile aproape 1:1 pe Pico 2.

> Corecție față de planul inițial (README §6, WIRING §1.1): fluxul IQ **nu vine pe SPI**.
> SPI e doar pentru comenzi. IQ-ul RX vine pe 4 linii de date paralele + ready + clock,
> citite de un state machine PIO cu DMA. Stare: **extras din cod, nevalidat pe bancul propriu.**

Numerele de pini sunt **GPIO BCM de pe header-ul Pi** (adică pinii pe care îi vede Radioberry).

---

## 1. Încărcarea gateware-ului (passive serial, la fiecare pornire)

FPGA-ul Cyclone 10 LP nu are configurație proprie la pornire — o încarcă host-ul.

| Semnal | GPIO | Dir. (host) |
|---|---|---|
| nCONFIG | 27 | ieșire |
| DATA0 | 13 | ieșire |
| DCLK | 24 | ieșire |
| nSTATUS | 26 | intrare |
| CONF_DONE | 22 | intrare |

Secvența (`rb2-load-fpga.c`):
1. nCONFIG=0, DATA=0, DCLK=0; așteaptă 1 s; nCONFIG=1.
2. Așteaptă nSTATUS=1 (timeout ~2 s → eroare).
3. Pentru fiecare octet din `.rbf`: 8 biți **LSB primul** — pune DATA, ~1 µs, puls DCLK.
4. Verifică nSTATUS=1 și CONF_DONE=1.
5. Încă 2 pulsuri DCLK (inițializare).

Pe Pico: `.rbf`-ul pentru **CL025** stă în flash (se livrează în `SBC/.../releases/.../CL025`).
Bit-bang-ul se poate face și cu PIO (mult mai rapid decât 1 µs/bit).

## 2. Control (SPI)

| Semnal | GPIO |
|---|---|
| CE0 | 8 |
| CE1 | 7 |
| SCLK | 11 |
| MISO | 9 |
| MOSI | 10 |

**Mod SPI 3 (CPOL=1, CPHA=1), max 48 MHz** — din `radioberry.dts` (`spi-mode = <3>`); confirmat pe
placă: în modul 0 gateware-ul răspunde doar cu zerouri.
Transfer full-duplex simplu (`rb2_trx_control(tx, rx, cnt)` = un `spi_sync`). Formatul mesajelor
(frecvență, câștig, filtre etc.) e în codul de firmware al driverului — **de extras** (pasul următor).

## 3. RX — flux IQ (PIO + DMA)

| Semnal | GPIO | Dir. (host) |
|---|---|---|
| RX_CLK | 6 | ieșire (side-set) |
| RX_RDY (sample ready) | 25 | intrare (`jmp pin`) |
| D0..D3 | 18, 19, 20, 21 | intrări (`in pins, 4`) |

Program PIO (18 instrucțiuni, din `rb2-rx-stream.c`):

```
.side_set 1 opt
.wrap_target
    jmp 15
L1: nop        side 1 [3]
    in  pins,4 side 0          ; ×7 perechi → 7 nibble-uri
    ...                        ; (instr. 1..14)
    nop        side 1 [2]      ; 15
    nop        side 0 [2]      ; 16
    jmp pin L1                 ; 17: dacă RDY=1 → citește un cuvânt
.wrap
```

Configurația state machine-ului, decodată din registre:

| Registru | Valoare | Înseamnă |
|---|---|---|
| CLKDIV | 0x00020000 | divizor întreg 2 |
| EXECCTRL | 0x5901f600 (+wrap) | JMP_PIN = 25 (RDY), side-set opțional activ |
| SHIFTCTRL | 0x01c10000 | autopush, prag **28 biți**, shift spre stânga |
| PINCTRL | 0x40091800 | IN_BASE = 18, SIDESET_BASE = 6, side-set 2 biți (1 + bitul „opt”) |

Deci: la fiecare RDY=1 host-ul generează 7 fronturi de ceas și citește 7 × 4 = **28 de biți**,
împinși automat în FIFO → DMA. De lămurit din gateware: cum se împart cei 28 de biți
(eșantion 24 biți + 4 biți de marcaj/canal?) și câte cuvinte formează o pereche I/Q.

## 4. TX (în afara scopului v1, notat pentru completitudine)

| Semnal | GPIO | Dir. (host) |
|---|---|---|
| TX_RDY | 12 | intrare |
| TX_DATA | 5 | ieșire |
| TX_CLK | 4 | ieșire |

## 5. Consecințe pentru Radioberry_Pico2

- **Faza 0 (analizor logic pe Pi) nu mai e necesară pentru pini și forma semnalelor** — sunt
  în codul de mai sus. Rămâne de verificat pe banc formatul cuvântului de 28 biți.
- **Bugetul de pini (doar RX):** gateware 5 + SPI 5 (4 dacă CE1 nu trebuie) + RX 6 = 16,
  plus I2S 3 + OLED 2 + encoder/butoane 5 = **26 = exact câți GPIO are Pico 2**.
  Fără rezervă; debug doar pe USB. Argument puternic pentru **Waveshare RP2350-PiZero**
  (RP2350B, 48 GPIO, header Pi): Radioberry se infige direct și pinii BCM de mai sus
  se pot folosi ca atare, PIO-ul nu ține de pini ficși.
- WIRING.md §1.1 (IQ pe SPI0) e **depășit** — de refăcut după tabelele de aici.

---

## 6. CE MERGE PE PLACĂ (1 oct 2026): protocolul clasic Pi 4 + gateware Hermes-Lite 2

**Validat pe RP2350-PiZero + Radioberry CL025:** 48 006 perechi IQ/s, I/Q zgomot real, versiune 73.3.

- **Gateware:** NU cel din `SBC/rpi-5/archive/.../CL025` (cu el FPGA-ul se configurează, dar SPI dă 0 și
  RDY nu urcă niciodată). Cel bun e cel pe care îl instalează scriptul Pi 4:
  `softerhardware/Hermes-Lite2/gateware/variants/radioberry_cl025/build/radioberry.rbf` (73.3).
  Sursa RTL e publică: `Hermes-Lite2/gateware/rtl/radioberry/radioberry_core.v`.
  Varianta PIO (4 biți + meta) are doar sursă (`variants/radioberry_pio_cl025`, `rtl/radioberry/pi-pio/`),
  fără `.rbf` compilat — ar trebui compilată cu Quartus.
- **SPI comenzi:** mod 3, 6 octeți = 48 biți: `[stare, C0, C1, C2, C3, C4]`; în gateware
  `run = bit 40` (stare bit 0), `cmd_addr = C0[6:1]`, `cmd_data = C1..C4`. Răspuns:
  `{resp, 0, 0, {0, fpgatype}, VERSION_MAJOR, VERSION_MINOR}`.
  ⚠️ Contorul de biți al slave-ului NU se resetează la CS sus — doar cadre de exact 48 biți, prima
  comandă abia după ieșirea din reset (~200 ms după încărcare).
- ⚠️ **Registrul 0 trebuie să aibă DUPLEX = 1 (C4 bit 2, `cmd_data = 0x00000004`).** Altfel receptorul 1
  folosește frecvența TX; dacă aceea nu e setată, NCO-ul rămâne la 0 Hz → I = DC constant, Q = 0 exact.
  Comenzile se retrimit ciclic (reg 0, TX, RX1, câștig), ca firmware-ul PA3GSB.
- ⚠️ **Câștigul RX (LNA AD9866): adresa 0x0A (C0 = 0x14), `cmd_data[6:0] = 0x40 | (dB + 12)`, -12..+48 dB.**
  La pornire gateware-ul e pe 0x40 = **-12 dB** (minimul) → fără comandă receptorul e practic surd.
  Măsurat: -12 dB → -109,5 dBFS; 0 → -95; +20 → -76; +36 → -62,6 (zgomot de antenă dominant).
- **RX (clasic):** RDY = BCM25 = FIFO > 256 eșantioane. Un eșantion = 6 fronturi ale ceasului RX (BCM6),
  un octet pe fiecare front (ieșire combinațională pe nivelul ceasului), pe 8 linii
  bit 7..0 = BCM 23, 20, 19, 18, 16, 13, 12, 5. Ordinea octeților: **Q hi, mid, lo, apoi I hi, mid, lo**
  (din `radioberry_core.v`: `tdata = qdata` după primul front, `idata` după al 4-lea). **I și Q din
  ACELAȘI cadru formează perechea** — verificat pe o înregistrare IQ (2 oct 2026): corelație I/Q 0,00 și
  imagini la nivelul zgomotului; perechea (I din cadrul anterior, Q curent), presupusă inițial, dădea
  corelație 0,54 și imagini doar ~10 dB sub semnal.
  **Orientarea:** I + jQ direct (Q **nenegat**) = frecvențe pozitive = USB. Verificat pe 3 oct 2026 cu FT8 de pe 60 m:
  IQ-ul înregistrat se decodează în WSJT-X (jt9) doar așa (12–16 mesaje pe interval); cu Q negat, 0 mesaje. (Filtrul SSB din rx_audio avea și el faza inversată; cele două erori se anulau la audio.) Ceasul RX trebuie ținut JOS de dinainte de încărcare (un front în plus după reset decalează
  contoarele up/down). Driverul Pi 4 citește 63 de eșantioane per RDY.
- **Pini RP2350-PiZero:** BCM 23,20,19,18,16,13,12,5 → GP 23,20,19,18,16,13,9,15; RDY GP25; CLK GP6.
  ⚠️ BCM16 și BCM23 sunt linii de date FPGA → conflict cu pinii I2S aleși înainte pe PiZero (GP16/GP23).
