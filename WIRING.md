# WIRING.md — Radioberry v2 + Raspberry Pi Pico 2 (RP2350)

**Sursa de adevar pentru cablaj.** Orice divergenta intre cod si acest fisier se
rezolva in favoarea fisierului, dupa verificare pe placa.

| | |
|---|---|
| Host (varianta A, bring-up) | Raspberry Pi Pico 2 (RP2350, 2x Cortex-M33 @150 MHz, FPU+DSP, 520 KB SRAM) |
| Host (varianta B, build final) | Waveshare RP2350-PiZero — header 40 pini format Pi, vezi **§8** |
| Frontend | Radioberry v2 — Cyclone 10 LP **10CL025** + AD9866, clock 73.728 MHz |
| Audio out | PCM5102A (modul I2S) |
| UI | OLED 1.3" I2C (SH1106) + encoder rotativ + 2 butoane |
| Stare | **NEVALIDAT pe placă** — harta v2 in §0 (1 oct 2026), extrasa din driverul PA3GSB |

---

## 0. Harta de pini v2 — Pico 2 ↔ Radioberry (1 oct 2026)  ← SURSA DE ADEVĂR

Refăcută după protocolul real (`PROTOCOL.md`): IQ-ul vine pe 4 linii paralele citite de PIO,
SPI e doar pentru comenzi, iar host-ul încarcă gateware-ul FPGA. Codul folosește exact
această hartă: `firmware/rb_bringup/pins.h`. **§1 de mai jos e versiunea veche, depășită.**

Toți cei 26 de GPIO ai Pico 2 sunt ocupați. Consola e pe USB, nu pe UART.

### 0.1 Pico 2 → header-ul Radioberry (40 pini, format Pi)

| Funcție | Pico GP | Pico pin fizic | Radioberry GPIO (BCM) | Header Pi pin fizic |
|---|---|---|---|---|
| FPGA nCONFIG | GP0 | 1 | 27 | 13 |
| FPGA DATA0 | GP1 | 2 | 13 | 33 |
| FPGA DCLK | GP22 | 29 | 24 | 18 |
| FPGA nSTATUS | GP26 | 31 | 26 | 37 |
| FPGA CONF_DONE | GP27 | 32 | 22 | 15 |
| RX D0 | GP2 | 4 | 18 | 12 |
| RX D1 | GP3 | 5 | 19 | 35 |
| RX D2 | GP4 | 6 | 20 | 38 |
| RX D3 | GP5 | 7 | 21 | 40 |
| RX RDY (eșantion gata) | GP9 | 12 | 25 | 22 |
| RX CLK | GP13 | 17 | 6 | 31 |
| SPI MISO | GP16 | 21 | 9 | 21 |
| SPI CE0 | GP17 | 22 | 8 | 24 |
| SPI SCLK | GP18 | 24 | 11 | 23 |
| SPI MOSI | GP19 | 25 | 10 | 19 |
| SPI CE1 (ținut sus) | GP28 | 34 | 7 | 26 |
| GND | GND | 3, 8, 13, 18, 23, 28, 33, 38 | GND | 6, 9, 14, 20, 25, 30, 34, 39 |

Leagă **cel puțin 3–4 fire de GND** între plăci, distribuite printre semnalele RX. Pe fluxul
de date contează mai mult masa decât lungimea firelor. Ține firele RX sub ~10 cm.

Alimentarea Radioberry rămâne cea din §2. Nivelurile sunt 3,3 V pe ambele părți (FPGA-ul
Radioberry e pe 3,3 V, ca Pi-ul), deci nu trebuie translatoare.

### 0.2 Audio și UI (pe Pico)

| Funcție | Pico GP | Pico pin fizic | Legătură |
|---|---|---|---|
| I2S DATA | GP6 | 9 | PCM5102A DIN |
| I2S BCK | GP7 | 10 | PCM5102A BCK |
| I2S LRCK | GP8 | 11 | PCM5102A LRCK |
| OLED SDA (I2C1) | GP14 | 19 | SH1106 SDA |
| OLED SCL (I2C1) | GP15 | 20 | SH1106 SCL |
| Encoder A | GP10 | 14 | encoder CLK/A |
| Encoder B | GP11 | 15 | encoder DT/B |
| Encoder SW | GP12 | 16 | apăsare: pasul de acord (10 Hz…100 kHz) |
| Buton 1 | GP20 | 26 | bring-up: flux IQ binar pe USB on/off |
| Buton 2 | GP21 | 27 | bring-up: reîncarcă FPGA-ul |
| 3V3 | 3V3 OUT | 36 | OLED VCC, PCM5102A VIN, comunul encoderului NU (vezi dedesubt) |

Encoderul și butoanele: pull-up intern în Pico, **contactul spre GND** (comunul encoderului la
GND, nu la 3V3). Strapurile PCM5102A rămân cele din §1.2 (SCK→GND, XSMT→3V3 etc.).
OLED-ul SH1106 la adresa 0x3C; offset-ul de 2 coloane îl tratează biblioteca U8g2.

### 0.3 RP2350-PiZero (build final)

**Mapare header → GPIO confirmată parțial (1 oct 2026)** din `board_pins.h` al lui PA3GSB,
proiectul `RP2350-SDR-CW-Interface` (rulează pe aceeași placă, cu un HAT audio WM8960):

| Pin fizic header | GPIO Pi (BCM) | GPIO RP2350-PiZero | Folosit la Radioberry |
|---|---|---|---|
| 12 / 35 / 38 / 40 | 18 / 19 / 20 / 21 | 18 / 19 / 20 / 21 (1:1) | RX D0–D3 |
| 31 | 6 | 6 (1:1) | RX CLK |
| 13 / 15 / 18 / 33 | 27 / 22 / 24 / 13 | 27 / 22 / 24 / 13 (1:1) | FPGA nCONFIG / CONF_DONE / DCLK / DATA0 |
| 24 / 26 | 8 / 7 | 8 / 7 (1:1) | SPI CE0 / CE1 |
| **19 / 21 / 23** | 10 / 9 / 11 | **11 / 12 / 10** | SPI MOSI / MISO / SCLK → **SPI1 hardware** (SCK 10, TX 11, RX 12) |
| 16 | 23 | 23 (1:1) | — |
| 3 / 5 | 2 / 3 | 2 / 3 (1:1) | I2C |
| 29 | 5 | **15** | (TX data la Pi 5: BCM5) |
| 22 | 25 | ? — probabil 25 | RX RDY |
| 37 | 26 | ? — probabil 26 | FPGA nSTATUS |

⚠️ În fișierul PA3GSB apare „J5 pin 20 → GPIO25”, dar pe header-ul Pi pinul 20 e GND —
probabil greșeală de tipar pentru pinul 22. **De verificat pe schema Waveshare** pinii 20, 22, 37.

Consecințe pentru firmware pe PiZero: SPI pe `spi1` (10/11/12, CS manual pe 8), RX PIO cu
IN_BASE 18 și side-set pe 6, jmp pin pe RDY. UI-ul (OLED, encoder, butoane) trebuie pus pe
pinii de header lăsați liberi de Radioberry — PA3GSB își pune encoderul pe GP15/6/13, dar 6 și 13
sunt ocupați aici.


Pe Waveshare RP2350-PiZero, Radioberry se infige direct în header, deci se folosesc chiar
GPIO-urile BCM din coloana a patra, dacă maparea header → GPIO a plăcii e 1:1 (de verificat pe
schema Waveshare). Programul PIO nu depinde de pini ficși, doar D0–D3 trebuie consecutivi
(18–21 sunt). OLED/encoder/butoane se mută pe GPIO-urile rămase libere ale RP2350B (48 total).

---

## 1. Harta de pini — Pico 2 *(VECHE, depășită de §0)*

Pinii sunt notati `GPn` (numar GPIO) / `#nn` (numar fizic pe header).

### 1.1 SPI0 — link catre FPGA  *(critic, cu DMA)*

> ⚠️ **DEPASIT (30 sep 2026):** SPI e doar pentru comenzi; IQ-ul vine pe 4 linii paralele +
> RDY + CLK prin PIO, iar gateware-ul se incarca de catre host. Vezi `PROTOCOL.md` §1–3;
> harta de pini de mai jos trebuie refacuta.

| GP | # | Semnal Pico | Radioberry (Pi 40-pin) | Pi phys |
|----|----|-------------|------------------------|---------|
| GP16 | 21 | SPI0 RX (MISO) | GPIO9  / MISO | 21 |
| GP17 | 22 | SPI0 CSn       | GPIO8  / CE0  | 24 |
| GP18 | 24 | SPI0 SCK       | GPIO11 / SCLK | 23 |
| GP19 | 25 | SPI0 TX (MOSI) | GPIO10 / MOSI | 19 |

Pinii sunt grupati intentionat contiguu — scurteaza traseele si simplifica
adaptorul. **Nu muta SPI0 pe alt grup fara motiv.**

> Radioberry mai foloseste probabil una-doua linii de control/handshake in afara
> de SPI (CE1 si/sau un `ready`/IRQ). Vezi §7 — pinii de rezerva sunt deja pusi
> deoparte in §1.5.

### 1.2 I2S — PCM5102A  *(prin PIO)*

Conventie `pico-extras / audio_i2s`: `clock_pin_base` = BCK, iar LRCK = BCK+1.

| GP | # | Semnal | PCM5102A |
|----|----|--------|----------|
| GP6 | 9  | DATA   | DIN  |
| GP7 | 10 | BCK    | BCK  |
| GP8 | 11 | LRCK   | LRCK |

**Straps obligatorii pe modulul PCM5102A** (motivul #1 pentru care modulele astea
raman mute):

| Pin modul | Leaga la | De ce |
|-----------|----------|-------|
| SCK  | **GND** | forteaza PLL-ul intern; lasat in aer => fara iesire |
| XSMT | **3V3** | unmute; multe module au jumper de lipit, verifica-l |
| FLT  | GND | filtru normal latency |
| DEMP | GND | de-emphasis off |
| FMT  | GND | I2S standard (nu left-justified) |
| VIN  | 3V3 din Pico (`3V3 OUT`, #36) | consum mic, e OK aici |

### 1.3 OLED 1.3" — I2C1

| GP | # | Semnal | OLED |
|----|----|--------|------|
| GP14 | 19 | I2C1 SDA | SDA |
| GP15 | 20 | I2C1 SCL | SCL |
| —    | 36 | 3V3 OUT  | VCC |
| —    | 38 | GND      | GND |

- Adresa uzuala `0x3C` (uneori `0x3D`).
- Modulul are de regula pull-up-uri 4k7 pe placa. Daca nu, pune 4k7 spre 3V3.
- **SH1106 != SSD1306:** SH1106 are RAM de 132 coloane pentru un ecran de 128, deci
  are nevoie de **offset de 2 coloane**. Daca imaginea e deplasata orizontal si are
  o dunga in margine, asta e cauza — nu cablajul.
- I2C la 400 kHz => 1024 octeti/cadru => ~15-20 fps pe ecran plin. Suficient pentru
  S-meter + text. Pentru waterfall, redeseneaza doar fereastra spectrului.

### 1.4 Encoder + butoane

**Toate cu pull-up intern, contact spre GND.** Nu folosi pull-down — vezi §5.3.

| GP | # | Semnal | Functie propusa |
|----|----|--------|-----------------|
| GP10 | 14 | ENC_A  | tuning |
| GP11 | 15 | ENC_B  | tuning |
| GP12 | 16 | ENC_SW | apasare encoder — pas de acord (10 Hz / 100 Hz / 1 k / 10 k) |
| GP20 | 26 | BTN1   | mod (LSB / USB / CW / AM) |
| GP21 | 27 | BTN2   | banda / meniu |

Debounce: encoder in PIO (sau IRQ + filtru), butoanele 20-30 ms software.
Pune 100 nF de la fiecare pin la GND daca firele sunt lungi.

### 1.5 Rezervat / liber

| GP | Destinatie |
|----|------------|
| GP0, GP1 | UART0 — consola de debug (115200) |
| GP22, GP26, GP27, GP28 | **rezervate** pentru liniile de control Radioberry inca neidentificate (§7) |
| GP2, GP3, GP4, GP5, GP9, GP13 | libere |

`GP23, GP24, GP25, GP29` sunt interne pe Pico 2 (SMPS PS, VBUS sense, LED, VSYS
sense) si **nu sunt disponibile** pe header.

---

## 2. Alimentare

```
   PSU 5V / 2A --+------------------------------> Radioberry, pin 5V (Pi #2 sau #4)
                 |
                 +--[ Schottky SS14 / 1N5817 ]--> Pico 2 VSYS (#39)

   GND PSU ------+--> Radioberry GND (Pi #6, #9, #14, #20, #25, #30, #34, #39)
                 +--> Pico GND (#3, #8, #13, #18, #23, #28, #38)
```

**Reguli:**

1. **NU** alimenta Radioberry din `3V3 OUT` (#36) al Pico. 10CL025 + AD9866 trag
   cateva sute de mA; regulatorul Pico nu duce si intri in brown-out.
2. **NU** alimenta Radioberry din VBUS prin USB-ul Pico — 500 mA nu ajung si iti
   pica si hostul.
3. Dioda Schottky pe VSYS permite sa tii USB-ul conectat pentru debug in acelasi
   timp cu sursa externa (schema recomandata in datasheet-ul Pico).
4. Masa comuna, fir gros (min. AWG22) si scurt. Mai multe puncte de masa intre cele
   doua placi, nu unul singur.
5. Ordinea de pornire: sursa 5 V intai, apoi USB. La oprire invers.

---

## 3. Conectare mecanica

Radioberry are header **mama** 40 pini (format HAT). Ca sa ajungi la el:

- header **tata** 40 pini + fire scurte, sau
- cablu panglica Pi + placa de breakout, sau (recomandat dupa ce merge)
- o plachetta de adaptare Pico -> HAT.

Placa nu se monteaza fizic pe Pico — doar semnalele din §1.1 + alimentarea din §2
sunt conectate. Restul pinilor de pe headerul HAT raman neconectati.

---

## 4. Integritatea semnalului

La 48 kHz IQ ai nevoie de doar **~2.3 Mbit/s** (24-bit I + 24-bit Q).
La 192 kHz, ~9.2 Mbit/s. Ai marja enorma — **foloseste-o.**

- Porneste SPI la **4 MHz**. Creste doar dupa ce fluxul e stabil.
- Fire sub **10 cm**. Peste 15 cm la >10 MHz devine loterie.
- Fiecare fir de semnal impletit cu un fir de masa, sau macar un GND intre SCK si
  MISO in banda de fire.
- Daca vezi sample-uri corupte intermitent: intai coboara clockul, abia apoi
  suspecteaza software-ul.

---

## 5. Capcane cunoscute

### 5.1 PCM5102A mut
SCK trebuie la masa, XSMT trebuie sus. Verifica ambele **inainte** de a depana in cod.

### 5.2 SH1106 deplasat cu 2 coloane
Vezi §1.3. Nu e problema de cablaj.

### 5.3 RP2350 — erata E9
Pinii configurati ca intrare cu **pull-down intern** pot ramane latched la ~2 V si
nu mai citesc niciodata `0`. De aceea encoderul si butoanele din §1.4 sunt cu
**pull-up si contact spre GND**. Nu inversa.

### 5.4 VSYS vs VBUS
Nu lega niciodata 5 V extern direct la VBUS (#40) — pui sursa in paralel cu USB-ul
PC-ului. Doar VSYS, prin Schottky.

---

## 6. Buget de periferice

| Periferic | Alocat | Liber ramas |
|-----------|--------|-------------|
| SPI0 | FPGA (DMA RX+TX) | — |
| SPI1 | — | liber (display SPI, daca schimbi OLED-ul) |
| I2C0 | — | liber |
| I2C1 | OLED | — |
| PIO0 | I2S out | 3 SM libere |
| PIO1 | encoder | 3 SM libere |
| DMA  | 2 canale SPI0 + 1 I2S | 9 libere |
| Core0 | DSP (demodulare, filtre, AGC) | — |
| Core1 | UI, encoder, OLED | — |

Estimare incarcare DSP: FIR 100 tapuri la 48 kHz pe I+Q ~= 10 MMAC/s ~= **sub 10%**
dintr-un M33 la 150 MHz cu FPU. Nu calculul e problema in proiectul asta.

---

## 7. DE VERIFICAT — nimic din sectiunea asta nu e confirmat

Protocolul host<->FPGA al Radioberry **nu exista ca specificatie**, ci doar ca sursa
C in repo-ul PA3GSB. Pana la validare, tabelul din §1.1 e o ipoteza bazata pe
maparea SPI0 standard de pe Raspberry Pi.

- [ ] Radioberry porneste si functioneaza pe Pi (`192.168.0.188`) cu software stock
      -> confirma ca **placa e sanatoasa** inainte de orice port
- [ ] Capteaza cu analizor logic secventa de init Pi->FPGA: pini folositi, polaritate
      CS, mod SPI (CPOL/CPHA), frecventa clock, format comanda
- [ ] Identifica liniile de control in afara de SPI (CE1? ready/IRQ?) -> aloca din §1.5
- [ ] Confirma daca AD9866 se configureaza **prin** FPGA (probabil) sau direct
- [ ] Confirma latimea si endianness-ul sample-urilor IQ
- [ ] Verifica adresa I2C reala a OLED-ului (`0x3C` vs `0x3D`)
- [ ] Verifica daca modulul OLED are deja pull-up-uri I2C

### Milestone-uri (nu sari peste ordine)

1. Citire registru de versiune din FPGA de pe Pico -> **link-ul functioneaza**
2. Config AD9866 + NCO, flux IQ la 48 kHz
3. **Checkpoint critic:** arunca sample-urile brute pe USB CDC, FFT in Python pe PC.
   Purtatoarea unui generator trebuie sa apara exact unde trebuie.
   **Nicio linie de DSP inainte de asta.**
4. Decimare + demodulare SSB (Weaver — mai simplu decat Hilbert aici) + AGC + I2S
5. UI la final

---

## 8. Varianta B de host — Waveshare RP2350-PiZero

<https://www.waveshare.com/rp2350-pizero.htm> · wiki: `waveshare.com/wiki/RP2350-PiZero`

**Placa asta rezolva prin constructie doua dintre problemele documentului de fata:
§3 (conectare mecanica) si §4 (integritatea semnalului).** Are un header 40 pini in
format Raspberry Pi, deci Radioberry se imbina **direct**, fara fire volante.

### 8.1 Ce e confirmat din schema (`RP2350-PiZero.pdf`)

| | |
|---|---|
| MCU | **RP2350B** (QFN-80, **48 GPIO**), 2x M33 + 2x Hazard3 RISC-V, 150 MHz |
| Flash | W25Q128JVSI = **16 MB** |
| PSRAM | footprint U1 prezent (`PSRAM_CS`) dar **nepopulat** — raman 520 KB SRAM |
| Header | J5, 2x20 pini, format Raspberry Pi |
| microSD | soclu TF-110 (J1), pe GPIO-uri inalte (zona 30-40) |
| DVI/HDMI | conector HDMI, perechi diferentiale pe zona GPIO41-46 (HSTX) |
| PIO-USB | al doilea Type-C (J4) — **USB host sau device** |
| Baterie | incarcator **ETA6096** + conector PH2.0 (J3), ORing VBAT/VSYS pe MBR230 |
| 3V3 | buck TMI3112H |
| Debug | header H1 cu **SWCLK / SWDIO / GND** |
| Quartz | 12 MHz |

Periferiile de pe placa folosesc GPIO-urile **inalte** (30-47), deci cele 28 de
GPIO de pe headerul de 40 de pini raman libere. RP2350B are pini destui cat sa nu
mai existe presiune de alocare ca in §1.

Bonus: FAQ-ul Waveshare pentru placa asta documenteaza chiar **erata RP2350-E9**
din §5.3. Aceeasi capcana, confirmata de producator.

### 8.2 De ce e interesanta pentru proiectul asta

1. **Radioberry se infige direct.** Dispare tot §3 si aproape tot §4.
2. **PIO-USB host** = exact calea prin care „Ragchewberry" scoate audio pe casti
   USB (README §6.1), cu exemple gata facute de la Waveshare.
3. **DVI/HDMI** — daca vrei vreodata waterfall pe un ecran adevarat, nu pe OLED.
4. **microSD** — dump-uri IQ direct pe card in faza 2, in loc de USB CDC.
5. **SWD real** pe H1 — debug pas cu pas, nu `printf`.
6. **Incarcator LiPo integrat** — receptor portabil fara electronica in plus.
7. **16 MB flash** — tabele de filtre, fonturi, palete, fara sa numeri octetii.

### 8.3 NEVERIFICAT — de lamurit inainte de cumparare

**Maparea header <-> GPIO nu e confirmata.** Am extras netlist-ul din schema, dar
fara geometrie, deci nu pot spune ce pin fizic merge la ce GPIO. Ipoteza rezonabila
e mapare **1:1 intre numerotarea BCM si GPIO-ul RP2350B** (headerul e etichetat cu
numere BCM, iar netu-rile de pe el sunt `GPIO0..GPIO27` plus `CE0`, `CE1`,
`SPI_MOSI`, `SPI_MISO`, `SPI_SCLK`).

**Daca ipoteza e adevarata, apare o consecinta importanta.** SPI0-ul de pe Pi sta pe
BCM 8/9/10/11 = CE0 / MISO / MOSI / SCLK. Acelea ar deveni GPIO8-11 pe RP2350B,
care fac parte din grupul **SPI1**, dar cu functiile **decalate cu o pozitie**:

| Header (Pi) | Functie Pi | GPIO RP2350B | Functie SPI1 nativa |
|---|---|---|---|
| #24 | CE0 | GPIO8 | SPI1 **RX** |
| #21 | MISO | GPIO9 | SPI1 **CSn** |
| #19 | MOSI | GPIO10 | SPI1 **SCK** |
| #23 | SCLK | GPIO11 | SPI1 **TX** |

Adica **peripheralul SPI hardware nu se aliniaza** peste pinout-ul Pi. Nu e blocant
— RP2350 are 12 state machine PIO si un SPI in PIO e banal, cu DMA la fel de rapid.
Dar schimba planul din „SPI hardware + DMA" in „**PIO-SPI + DMA**".

- [ ] **Confirma maparea pin fizic -> GPIO** pe imaginea schemei sau cu multimetrul
      pe placa. Pana atunci, tabelul de mai sus e ipoteza, nu fapt.
- [ ] Confirma ce curent poate da raila de 3V3 (buck TMI3112H)
- [ ] Confirma daca headerul primeste 5 V dinspre placa sau il poate primi din afara

**Alimentarea nu se schimba:** Radioberry tot pe sursa lui de 5 V / 2 A. Nu trage
cateva sute de mA prin regulatorul plachetei, indiferent ce placa e dedesubt.

### 8.4 Recomandare

**Bring-up pe Pico 2 (§1-§7), build final pe RP2350-PiZero.**

Motivul e contraintuitiv dar solid: in fazele 0-2 ai nevoie de **analizor logic pe
semnale**, iar cand placa e infipta ca sandwich nu mai ajungi la ele. Firele volante
sunt un avantaj cat timp depanezi protocolul. La 4 MHz merg fara probleme.

Dupa ce IQ-ul e validat (faza 2 din README), muti pe PiZero si castigi mecanica,
integritatea semnalului, SD, USB host si bateria dintr-un foc.

---

## 9. Rezumat cablaj — varianta A (Pico 2) *(VECHI — vezi §0)*

| Pico GP | Pico # | Semnal | Merge la |
|---------|--------|--------|----------|
| GP0  | 1  | UART0 TX | debug |
| GP1  | 2  | UART0 RX | debug |
| GP6  | 9  | I2S DATA | PCM5102 DIN |
| GP7  | 10 | I2S BCK  | PCM5102 BCK |
| GP8  | 11 | I2S LRCK | PCM5102 LRCK |
| GP10 | 14 | ENC_A  | encoder A |
| GP11 | 15 | ENC_B  | encoder B |
| GP12 | 16 | ENC_SW | encoder push |
| GP14 | 19 | I2C1 SDA | OLED SDA |
| GP15 | 20 | I2C1 SCL | OLED SCL |
| GP16 | 21 | SPI0 MISO | Radioberry MISO (Pi #21) |
| GP17 | 22 | SPI0 CSn  | Radioberry CE0  (Pi #24) |
| GP18 | 24 | SPI0 SCK  | Radioberry SCLK (Pi #23) |
| GP19 | 25 | SPI0 MOSI | Radioberry MOSI (Pi #19) |
| GP20 | 26 | BTN1 | buton mod |
| GP21 | 27 | BTN2 | buton banda |
| —    | 36 | 3V3 OUT | OLED VCC, PCM5102 VIN |
| —    | 39 | VSYS | 5 V extern prin Schottky |
| —    | GND | masa | comuna cu Radioberry si PSU |
