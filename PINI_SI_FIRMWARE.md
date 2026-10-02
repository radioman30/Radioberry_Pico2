# Radioberry RX pe RP2350-PiZero — pini și firmware (stare la 2 oct 2026)

Firmware: `rx_audio`, commit **e40b5ea**, gateware FPGA **73.3** inclus în UF2.
Fișierul gata de scris pe placă: `release/rx_audio_2026-10-02_e40b5ea.uf2` (local, nu e pe GitHub —
conține gateware-ul PA3GSB/Hermes-Lite2, binar terț).

---

## 1. Cum scrii firmware-ul pe placă

**Varianta simplă (fără programe):**
1. Ține apăsat butonul **BOOT** de pe PiZero și bagă cablul USB în portul USB principal.
2. Dă drumul butonului. Apare un disc nou, **RP2350**.
3. Copiază fișierul `.uf2` pe disc. Placa se repornește singură cu noul firmware.

**Din sursă:** `powershell -File tools\build_rx_audio.ps1 -Flash` (compilează și scrie; găsește singur portul).

După flash, placa apare în Windows ca **port COM12** + **„Microphone (Radioberry RX audio)”**.
Setările salvate (frecvență, mod, filtre, volum, benzi) rămân după reflash cu aceeași versiune.

> Dacă după BOOT Windows nu mai deschide portul COM („device not functioning”): Device Manager →
> șterge dispozitivul USB al plăcii și reconectează (vezi §6).

---

## 2. Header-ul de 40 de pini — ce e legat unde

Radioberry e înfipt direct în header. „Radioberry” = pin folosit de placa radio, nu legi nimic acolo.
GP = numărul GPIO al RP2350 (pe PiZero NU e mereu egal cu BCM).

| Pin | BCM | GP | Folosit de | Ce legi tu |
|---|---|---|---|---|
| 1 | 3V3 | — | alimentare | PCM5102A **VIN**, OLED **VCC** |
| 2, 4 | 5V | — | alimentare Radioberry | — |
| 3 | 2 | **2** | I2C SDA | OLED **SDA** |
| 5 | 3 | **3** | I2C SCL | OLED **SCL** |
| 6, 9, 14, 20, 25, 30, 34, 39 | GND | — | masă | GND PCM5102A, OLED, encoder, butoane, condensator |
| 7 | 4 | **14** | I2S DATA | PCM5102A **DIN** |
| 8 | 14 | **4** | I2S BCK | PCM5102A **BCK** |
| 10 | 15 | **5** | I2S LRCK | PCM5102A **LCK** |
| 11 | 17 | 17 | ieșire FPGA (`pi_cwl`) | **NU LEGA** |
| 12 | 18 | 18 | Radioberry: date RX bit 4 | — |
| 13 | 27 | 27 | Radioberry: nCONFIG | — |
| 15 | 22 | 22 | Radioberry: CONF_DONE | — |
| 16 | 23 | 23 | Radioberry: date RX bit 7 | — |
| 17 | 3V3 | — | alimentare | ⚠️ chiar lângă pinul 18 — atenție la lipituri |
| **18** | 24 | **24** | Radioberry: DCLK (doar la pornire) | **rețeaua de butoane** prin 470 Ω (§3) |
| 19 | 10 | 11 | Radioberry: SPI MOSI | — |
| 21 | 9 | 12 | Radioberry: SPI MISO | — |
| 22 | 25 | 25 | Radioberry: RDY | — |
| 23 | 11 | 10 | Radioberry: SPI SCLK | — |
| 24 | 8 | 8 | Radioberry: SPI CE0 | — |
| 26 | 7 | 7 | Radioberry: SPI CE1 | — |
| 27 | 0 | **0** | encoder | encoder **A** (CLK) |
| 28 | 1 | **1** | encoder | encoder **B** (DT) |
| 29 | 5 | 15 | Radioberry: date RX bit 0 | — |
| 31 | 6 | 6 | Radioberry: ceas RX | — |
| 32 | 12 | 9 | Radioberry: date RX bit 1 | — |
| 33 | 13 | 13 | Radioberry: DATA0 / date RX bit 2 | — |
| 35 | 19 | 19 | Radioberry: date RX bit 5 | — |
| 36 | 16 | 16 | Radioberry: date RX bit 3 | — |
| 37 | 26 | 26 | Radioberry: nSTATUS | — |
| 38 | 20 | 20 | Radioberry: date RX bit 6 | — |
| 40 | 21 | 21 | ieșire FPGA (`pi_cwr`) | **NU LEGA** |

**PCM5102A:** SCK → GND, XSMT → 3V3, FLT/DEMP/FMT → GND. Căștile în mufa modulului.
**Encoder:** comunul la GND. Pinul „+” al modulului **nelegat** (are pull-up-uri de 10 kΩ care încurcă butoanele).

---

## 3. Butoanele — toate pe pinul 18 (GP24)

Pe header nu există pin analogic, așa că butoanele se recunosc după timpul de descărcare al unui
condensator prin rezistența fiecăruia.

```
 pin 18 (GP24) ──[ 470 Ω ]──┬──────────┬──────────┬──────────┐
                            │          │          │          │
                       tantal 1 µF  encoder SW    MOD       BANDA
                       (+ spre nod)    │          │          │
                            │          │       [1 kΩ]    [2,2 kΩ]
                            │          │          │          │
 GND ───────────────────────┴──────────┴──────────┴──────────┘
```

| Buton | Rezistor | Funcție | Timp măsurat pe banc |
|---|---|---|---|
| encoder SW | direct | meniu: parametrul următor; lung = înapoi la FRECV | 0 µs |
| MOD | 1 kΩ | USB → LSB → CW → AM → FM | 600–860 µs |
| BANDA | 2,2 kΩ | scurt = banda următoare, lung = banda anterioară | 2275–3050 µs |
| (FILTRU) | 4,7 kΩ | rezervat, nemontat | — |
| (PAS) | 10 kΩ | rezervat, nemontat — cere scos și pull-up-ul encoderului | — |

- **470 Ω e obligatoriu**: izolează condensatorul de DCLK la încărcarea FPGA-ului.
- **Modulele de butoane cu pull-up de 10 kΩ** (rezistor spre „+”): rezistorul se scoate sau „+” rămâne nelegat.
- Placa detectează singură rețeaua; pe ecran apare **„RC”** jos-dreapta. Fără ea, pinul 18 e buton simplu.
- Nu ține niciun buton apăsat la pornire.
- Verificare pe USB: `k` (stare + ultimul timp măsurat), `kd` (diagnostic condensator).

---

## 4. Istoricul modificărilor de pini

| Data | Ce s-a schimbat | De ce |
|---|---|---|
| 1 oct | PCM5102A mutat pe **GP4 / GP5 / GP14** (pinii 8 / 10 / 7) | primii pini aleși (GP16 / GP23) sunt linii de date ale Radioberry |
| 1 oct | encoder A/B pe **GP0 / GP1** (pinii 27 / 28), SW pe **GP24** (pinul 18) | singurii pini liberi; DCLK e liber după încărcarea FPGA-ului |
| 1 oct | butonul **BOOT** = banda următoare | header-ul nu mai avea pini |
| 2 oct | butoane **MOD + BANDA pe pinul 18** prin rețea RC (470 Ω, 1 µF, 1 kΩ, 2,2 kΩ); firul SW mutat pe nod | mai multe comenzi fără pini noi |
| 2 oct | scos pull-up-ul de 10 kΩ de pe modulele de butoane | reîncărca condensatorul, toate butoanele păreau „encoder” |
| 2 oct | butonul **BOOT nu mai are funcție** (doar pentru flash) | la cererea ta |
| 2 oct | USB: PID **2E8A:10F1**, portul devine **COM12** (era COM10) | PID-ul vechi (000F) era același cu modul BOOT → Windows încurca driverele |

---

## 5. Ce face firmware-ul

- **Recepție:** USB, LSB, CW, AM, **FM** (bandă îngustă), 10 kHz – 30 MHz, IQ 48 kHz de la FPGA.
- **Filtre:** SSB 1,8 / 2,0 / 2,5 / 2,7 kHz · CW 250 / 500 / 1000 Hz · AM 6 / 8 kHz · FM 8 / 11 kHz.
- **Benzi** (cu memorie): 160, 80, 60, 49 (AM), 40, 41 (AM), 31 (AM), 30, 20, 17, 15, 12, **CB** (FM), 10 m.
- **Ecran:** frecvență, bandă, mod, S-metru, spectru ±24 kHz, waterfall, meniu jos (pas, volum, câștig, filtru).
- **Setările** se salvează singure în flash la 5 s după ultima schimbare.
- **USB:**
  - **COM12** — comenzi text + **CAT Kenwood TS-2000** (WSJT-X, fldigi, Omni-Rig/HDSDR: 115200, DTR High, PTT None);
  - **microfon „Radioberry RX audio”** — audio demodulat 48 kHz, nivel fix (fără volumul căștilor);
  - **IQ brut** — `python tools/iq_record.py -t 60 -o iq_rec` → WAV pentru HDSDR / SDR# / SDR++.

**Comenzi pe COM12:** `s` stare · `f<Hz>` frecvență · `m<0-4>` mod · `v<0-100>` volum · `g<-12..48>` câștig ·
`w<Hz>` filtru · `k` / `kd` butoane · `x` inversează IQ · `q1`/`q0` flux IQ · `z<start,stop,pas kHz>` baleiaj.

---

## 6. Probleme cunoscute și soluții

| Simptom | Cauză | Soluție |
|---|---|---|
| COM-ul există dar nu răspunde („device not functioning”) | Windows a legat driverul greșit după BOOT | Device Manager → șterge dispozitivul plăcii → reconectează |
| Toate butoanele fac ce face encoderul | lipsește condensatorul / 470 Ω, sau un pull-up 10 kΩ pe nod | §3; verifică cu `kd` (bun: urcare zeci de ms) |
| Placa nu apare deloc pe USB | scurt la pinul 17 (3V3) lângă 18, tantal invers | verifică lipiturile de lângă pinul 18 |
| Frecvența din WSJT-X nu se schimbă | portul e ocupat de alt program (iq_record, alt terminal) | un singur program pe COM12 odată |

---

## 7. Plănuit: WM8960 (intrare + ieșire audio)

- I2C comun cu OLED-ul (GP2/GP3; WM8960 la adresa 0x1A).
- I2S refolosește pinii PCM5102A: BCK GP4, LRCK GP5, DACDAT GP14.
- **Mai trebuie 1 pin** pentru ADCDAT — de ales dintre GPIO-urile de pe USB-C-ul secundar sau slotul microSD.
- HAT-ul se leagă **cu fire**, nu se înfige peste Radioberry (folosește pinii de date RX 18–21).
