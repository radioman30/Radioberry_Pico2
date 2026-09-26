# Radioberry_Pico2

Receptor SDR HF standalone: **Radioberry v2 condus de un Raspberry Pi Pico 2**,
fara Linux, fara retea, fara PC. OLED 1.3", encoder, doua butoane, audio pe casti.

Scopul e sa scot Pi-ul din lant. Radioberry are deja un FPGA care face toata munca
grea (DDC, decimare); Pi-ul nu facea decat sa citeasca IQ pe SPI si sa-l impacheteze
in UDP HPSDR pentru un PC. Partea aia o poate face un RP2350, iar restul
demodularii incape lejer in cele doua nuclee M33.

**Stare:** faza 0 — recunoastere. Nimic nu e validat inca pe bancul propriu, dar
**arhitectura e confirmata de un tert care a ajuns deja la ea** — vezi §6.1.

---

## 1. Hardware

| Bloc | Piesa | Note |
|---|---|---|
| Host — bring-up | Raspberry Pi Pico 2 (RP2350) | 2x M33 @150 MHz, FPU + DSP, 520 KB SRAM |
| Host — build final | Waveshare **RP2350-PiZero** | RP2350B, 48 GPIO, 16 MB flash, header 40 pini **format Pi** — Radioberry se infige direct. Vezi `WIRING.md` §8 |
| Frontend + DDC | Radioberry v2 | Cyclone 10 LP **10CL025** + AD9866, clock 73.728 MHz |
| Audio out | PCM5102A | I2S prin PIO |
| UI | OLED 1.3" SH1106 I2C + encoder + 2 butoane | carcasa exista deja printata |
| Alimentare | PSU 5 V / 2 A extern | vezi `WIRING.md` §2 |

Cablajul complet, cu capcanele lui, e in **[`WIRING.md`](WIRING.md)** — acela e
sursa de adevar pentru pini. Documentul asta acopera doar planul si arhitectura.

---

## 2. De ce merge (bugetul de calcul)

Argumentul central al proiectului, ca sa fie clar de la inceput ca nu calculul e
problema:

| Marime | Valoare |
|---|---|
| Debit IQ la 48 kHz (24-bit I + Q) | ~2.3 Mbit/s |
| Debit IQ la 192 kHz | ~9.2 Mbit/s |
| FIR 100 tapuri, complex, la 48 kHz | ~19 MMAC/s |
| Capacitate un M33 @150 MHz cu FPU | ordinul a 150 MMAC/s |
| RAM necesar (buffere IQ + audio + FFT 1024) | zeci de KB din 520 |

Adica **sub 15% dintr-un singur nucleu** pentru lantul de demodulare. Al doilea
nucleu ramane integral pentru UI si spectru. SPI-ul are marja de un ordin de
marime peste ce e nevoie.

Dificultatea reala nu e in DSP, ci in §6.

---

## 3. Arhitectura

```
  antena
    |
    v
  AD9866  --76.8 MSPS-->  FPGA 10CL025
                            NCO + CIC + FIR compensator
                            (tot pe Radioberry, gateware existent)
                                  |
                                  |  SPI0 @ 4-20 MHz, DMA dublu-buffer
                                  v
  +---------------------------- RP2350 ----------------------------+
  |                                                                |
  |  CORE 0 (DSP)                        CORE 1 (UI)               |
  |  ------------                        -----------               |
  |  IQ in (48 kHz complex)              encoder (PIO) -> frecventa|
  |    v                                 butoane -> mod / banda    |
  |  filtru trece-banda COMPLEX          FFT 1024 -> spectru       |
  |  (asimetric, o singura banda lat.)   randare OLED I2C          |
  |    v                                                           |
  |  demodulare (SSB / CW / AM / NFM)    <--- FIFO --->            |
  |    v                                                           |
  |  AGC (attack rapid, decay lent)                                |
  |    v                                                           |
  |  I2S out (PIO) --> PCM5102A --> casti                          |
  +----------------------------------------------------------------+
```

### Demodulare — decizia de design

Fiindca de la FPGA vine deja **IQ complex la banda de baza**, cea mai directa cale
pentru SSB e:

1. filtru **trece-banda complex asimetric** (coeficienti complecsi, trece doar
   +300..+2700 Hz pentru USB, sau -2700..-300 pentru LSB)
2. `Re{}` din rezultat = audio

Selectia de banda laterala devine o simpla conjugare a coeficientilor — acelasi
cod, alta tabela. Costa 4 FIR-uri reale in loc de unul, dar la bugetul din §2 e
irelevant.

Alternativa Weaver (mixare +/-1.5 kHz + LPF reale) ramane valabila si foloseste
doar filtre reale; o pastrez ca plan B daca filtrul complex da batai de cap la
simetria benzii. **README-ul asta e autoritatea pe DSP**, nu nota din `WIRING.md`.

Celelalte moduri, din acelasi IQ:
- **CW** — acelasi trece-banda complex, dar ingust (300-500 Hz), cu offset de BFO
- **AM** — `sqrt(I^2 + Q^2)` + blocare DC
- **NFM** — demodulare de faza cu `(I*dQ - Q*dI) / (I^2 + Q^2)`, fara `atan2`

---

## 4. Plan pe faze

Fazele 1b si 1 sunt independente — se pot face in paralel.

### Faza 0 — recunoastere *(in curs)*

**Pasul 0, inainte de orice munca:** contacteaza autorul „Ragchewberry" (§6.1). A
rezolvat deja linkul RP2350 <-> Radioberry. Un raspuns de la el scurteaza faza asta
de la saptamani la zile. Costa un comentariu pe YouTube.

- [x] ~~Comentariu catre `@tresvoltios`~~ — **postat 2026-09-09**, se asteapta raspuns
- [ ] Radioberry pe Pi-ul existent (`192.168.0.188`) cu software stock, functional
      end-to-end -> **confirma ca placa e sanatoasa** inainte de orice port
- [ ] Analizor logic pe headerul Pi: captura secventei de init si a citirii de IQ
- [ ] Din captura: pini folositi, mod SPI (CPOL/CPHA), frecventa, format comanda,
      latimea si ordinea octetilor in sample-uri
- [ ] Citire paralela a sursei C din repo-ul PA3GSB, ca sa confirme captura

Pasii cu analizorul raman valabili chiar daca primesti sursa — ai nevoie de
intelegerea protocolului, nu doar de cod care merge.

**Livrabil:** `PROTOCOL.md` — protocolul host<->FPGA scris ca specificatie.

#### Analizorul logic — montaj si procedura

Nu e nevoie de cumparaturi: un **RP2040** din sertar (Marble Pico, Pico clasic) face
treaba.

> **NU folosi Pico 2 / RP2350 ca analizor.** Erata E9 loveste exact aici: pinii se
> blocheaza cand primesc nivel logic 1, deci nu poti citi date pe GPIO. Autorul lui
> `gusmanb/logicanalyzer` a **suspendat** suportul pentru Pico 2 din cauza asta.
> Aceeasi erata ca in `WIRING.md` §5.3, dar aici e fatala, nu doar incomoda.

**Firmware — doua variante bune:**

| | `gusmanb/logicanalyzer` | `pico-coder/sigrok-pico` |
|---|---|---|
| Canale | 24 | 21 digital + 3 analog (variante 26 / 32) |
| Rata | 100 MS/s | pana la ~120 MHz |
| Adancime | 131.071 esant. @ 8 canale, 65.535 @ 16, 32.767 @ 24 | — |
| Trigger | front, pattern rapid (5 ch), pattern complex (16 biti) | mai simplu |
| Software | GUI .NET propriu, **export spre sigrok** | direct in **PulseView** |
| RP2350 | **suspendat** (E9) | exista variante de firmware |

**Recomandare:** `gusmanb` pentru captura (trigger si adancime mai bune), export in
format sigrok, decodare SPI in **PulseView**. Decodorul SPI din PulseView e cel care
transforma fronturile in octeti — adica exact livrabilul fazei.

PulseView trebuie sa fie **mai nou de 0.4.2**, altfel nu recunoaste driverul.

**Socoteala de adancime:** iti trebuie 4 canale (SCK, MOSI, MISO, CS) -> mergi pe
modul 8 canale = 131k esantioane. La 100 MS/s = **1,3 ms de captura**. Daca SPI-ul
Pi merge la 15 MHz, un octet ia 533 ns, deci prinzi ~2400 de octeti intr-o captura.
Suficient cu mult pentru o secventa de init.

**Montaj:** SCK, MOSI, MISO, CS + **masa comuna** cu Pi-ul. Rezistente serie
100-330 ohmi pe fiecare semnal, fire scurte. Ambele placi sunt pe 3,3 V — fara level
shifting.

**Procedura:**

1. Citeste in sursa daemon-ului `radioberry` la ce viteza e setat `spidev`. Aia
   dicteaza rata de esantionare: **tinta e minim 4x frecventa SPI**.
2. Leaga sondele conform montajului de mai sus.
3. **Armeaza triggerul pe frontul cazator al CS, si abia apoi porneste serviciul
   radioberry.** Altfel initul s-a consumat demult cand te apuci tu de captat — e
   greseala clasica si pierzi o dupa-amiaza pe ea.
4. Decodeaza SPI in PulseView, scrie rezultatul in `PROTOCOL.md`.
5. Repeta cu trigger in regim stationar, ca sa prinzi si citirea de IQ, nu doar
   initul.

### Faza 1b — lantul audio *(independent de FPGA)*
- [ ] I2S prin PIO catre PCM5102A, ton sinusoidal generat pe Pico
- [ ] Verificat ca se aude curat, fara clicuri, la 48 kHz

Merita facut devreme: e usor, e independent, si elimina audio-ul din lista de
suspecti cand debughezi lantul RF.

### Faza 1 — link SPI catre FPGA
- [ ] Citire registru de versiune / ID din FPGA de pe Pico
- [ ] **Milestone: link-ul functioneaza**
- [ ] DMA dublu-buffer pe SPI0, fara pierderi de sample-uri

### Faza 2 — flux IQ *(checkpoint critic)*
- [ ] Configurare AD9866 prin FPGA
- [ ] Setare NCO (frecventa de acord) si rata de decimare -> IQ la 48 kHz
- [ ] Sample-uri brute pe USB CDC -> FFT in Python pe PC
- [ ] **Purtatoarea unui generator trebuie sa apara exact la frecventa corecta**

Nicio linie de DSP inainte de asta. Daca IQ-ul e gresit (endianness, semn, offset),
tot ce se construieste deasupra minte convingator.

### Faza 3 — receptie audibila
- [ ] Filtru trece-banda complex + `Re{}` -> USB/LSB
- [ ] AGC
- [ ] Legatura cu I2S din faza 1b
- [ ] **Milestone: se aude o statie reala pe 40 m**

### Faza 4 — UI
- [ ] Driver SH1106 (atentie la offsetul de 2 coloane)
- [ ] Encoder in PIO, pas de acord comutabil din apasare
- [ ] Ecran principal: frecventa, mod, S-meter, latime filtru
- [ ] Butoane: mod si banda
- [ ] Spectru: FFT 1024, fereastra ingusta redesenata incremental (I2C e lent)

### Faza 5 — moduri si finisaj
- [ ] CW cu BFO, AM, NFM
- [ ] Filtre comutabile pe latime
- [ ] Salvare stare in flash (frecventa, mod, volum)
- [ ] **Mod „protocol1"** — comutabil din meniu, firmware-ul se poarta ca server
      HPSDR pentru Thetis/Quisk pe PC in loc de DSP local (idee preluata din §6.1)
- [ ] **Mutare pe RP2350-PiZero** — Radioberry se infige direct in header, dispare
      cablajul volant. Se face **dupa** faza 2, nu inainte: cat depanezi protocolul
      ai nevoie de acces cu analizorul la semnale, imposibil intr-un sandwich.
      Detalii si necunoscute in `WIRING.md` §8
- [ ] Montaj in carcasa printata

---

## 5. In afara scopului (v1)

- **TX.** Radioberry poate emite (AD9866 are DAC), dar cere PA, filtre trece-jos pe
  banda, comutare T/R si intrare de microfon. Proiect separat, dupa ce RX-ul merge.
- Retea / HPSDR / streaming spre PC — exact ce incearca proiectul sa elimine.
- Moduri digitale (FT8, WSPR) — nu incap rezonabil aici.
- Waterfall pe tot ecranul — I2C la 400 kHz da ~15-20 fps pe ecran plin, deci doar
  o fereastra ingusta de spectru.

---

## 6. Riscuri

| Risc | Impact | Mitigare |
|---|---|---|
| Maparea header <-> GPIO pe RP2350-PiZero necunoscuta | s-ar putea sa nu se poata folosi SPI hardware | PIO-SPI (12 state machine libere). Nu blocheaza nimic, dar de confirmat inainte de cumparare — `WIRING.md` §8.3 |
| **Protocolul FPGA nu e documentat ca specificatie**, doar ca sursa C | proiectul moare in faza 1 | intreaba autorul din §6.1; in paralel, analizor logic pe un sistem functional (faza 0). Nu ghicit din cod. **Risc mult redus: cineva a facut-o deja.** |
| Integritate semnal SPI pe fire volante | sample-uri corupte intermitent | pornire la 4 MHz, fire <10 cm, retur de masa. Vezi `WIRING.md` §4 |
| Alimentare insuficienta | brown-out, comportament aleatoriu | PSU separat 5 V/2 A, niciodata din Pico. Vezi `WIRING.md` §2 |
| I2C prea lent pentru UI fluid | spectru sacadat | redesenare incrementala; daca nu ajunge, OLED SPI pe SPI1 (pini deja liberi) |

### 6.1 Prior art — „Ragchewberry" (@tresvoltios)

**Cineva a construit deja exact arhitectura asta.** Juan Agrinsoni (`@tresvoltios`
pe YouTube, canal cu 59 de clipuri) documenteaza in video un drum de ~7 luni:

| Etapa | Ce |
|---|---|
| acum ~7 luni | sbitx v2 + Quisk, driver soapy |
| acum ~4 luni | `hpsdrd` — server HPSDR protocol-1, FreeDV |
| acum ~1 luna | trece pe **RP2350** („Zbitx v1 RP2350"), apoi spectru pe display mic |
| acum ~4 sapt. | **comutator „local / protocol1"** in acelasi firmware |
| acum ~3 sapt. | decodor CW **de la PA3GSB** portat pe RP2350, meniuri, UI |
| acum ~2 sapt. | **PSK31 si RTTY, TX si RX**, pe RP2350 |
| acum ~3 zile | **„Radioberry RP2350 + usb headset"** |
| acum ~1 zi | **„Ragchewberry AM rx Radioberry RP2350"** |

Din imagini: placa verde Radioberry alaturi de un PCB cu OLED mic (afiseaza
frecventa + o banda ingusta de spectru) si un encoder mare. Practic montajul din
`WIRING.md`.

**Ce se invata din asta:**

1. **Arhitectura e valida.** Nu doar RX — omul face PSK31 si RTTY, inclusiv TX, pe
   acelasi RP2350. Bugetul de calcul din §2 se confirma in practica.
2. **Foloseste cod PA3GSB** (autorul Radioberry). Exista filiatie de cod pentru
   partea de link — nu e teren complet virgin.
3. **Comutatorul „local / protocol1" merita copiat.** Acelasi firmware ruleaza fie
   DSP local, fie ca server HPSDR pentru PC. Nu pierzi compatibilitatea cu
   Thetis/Quisk pentru pretul unui mod de lucru in plus. **De adaugat la roadmap
   dupa faza 3.**
4. **Audio pe USB headset**, nu I2S. RP2350 in mod USB host cu USB Audio Class.
   Nu schimbam planul pentru v1 — PCM5102 e mai simplu, il avem, si e independent
   de restul lantului (faza 1b). USB audio ramane optiune ulterioara.

**Nu exista repo public gasit.** Pagina „Despre" a canalului nu are niciun link, iar
cautarea pe GitHub scoate doar sbitx/zbitx, nimic „Ragchew"/„Ragchewberry". De aici
pasul 0 din faza 0: **intreaba-l.** Canal mic, raspunde la comentarii in doua zile.

### Planuri de rezerva daca faza 1 se blocheaza

1. **Modificarea gateware-ului.** 10CL025 e suportat de Quartus Prime Lite (gratuit),
   iar sursa Radioberry e publica. Se poate expune o interfata proprie, simpla, in
   locul celei existente. Mai multa munca, dar control total.
2. **Pico ca panou de comanda.** Pi-ul ramane host, Pico face doar UI peste UART.
   Functional, dar pierde tot ce e interesant la proiect.
3. **Abandon Radioberry** si trecere pe arhitectura Tayloe/QSD (gen PicoRX): Si5351
   + comutator + ADC intern. Mult mai simplu, performanta mai modesta.

---

## 7. Toolchain

- **pico-sdk 2.x** (obligatoriu 2.0+ pentru RP2350) + CMake + `arm-none-eabi-gcc`
- `pico-extras` pentru `audio_i2s`
- Python + numpy/matplotlib pe PC, pentru analiza capturilor din faza 2
- **PulseView** (>0.4.2, de pe sigrok.org) + firmware de analizor pe un RP2040 —
  faza 0, vezi subsectiunea din §4
- Quartus Prime Lite — doar daca se ajunge la planul de rezerva 1

---

## 8. Structura repo (propusa)

```
Radioberry_Pico2/
  README.md          <- planul (documentul asta)
  WIRING.md          <- sursa de adevar pentru pini
  PROTOCOL.md        <- livrabilul fazei 0, inca inexistent
  src/
    main.c
    rb_link.c/.h     faza 1 — SPI + DMA catre FPGA
    ad9866.c/.h      faza 2 — configurare frontend
    dsp/
      filters.c/.h   FIR complex, tabele de coeficienti
      demod.c/.h     SSB / CW / AM / NFM
      agc.c/.h
    audio_i2s.c/.h   faza 1b
    ui/
      sh1106.c/.h
      encoder.pio
      screens.c
  tools/
    iq_dump.py       faza 2 — captura USB CDC + FFT
    coeff_gen.py     generare tabele de filtre
```

---

## 9. Ordinea de citit

1. §6.1 — prior art; nu incepe munca fara sa fi trimis intrebarea
2. `WIRING.md` §7 — ce nu e confirmat inca
3. Sectiunea 4 de mai sus — fazele, in ordine
4. Nu sari peste checkpointul din faza 2

---

## 10. Referinte

**Prior art**
- Canal: <https://www.youtube.com/@tresvoltios/videos> — Juan Agrinsoni
- „Radioberry RP2350 + usb headset": <https://www.youtube.com/watch?v=MVh6SvB8Kpk>
  (28 s, fara descriere — informatia e in imagine)
- „Ragchewberry AM rx Radioberry RP2350" — cel mai recent, RX AM functional

**Ecosistem sbitx / zbitx** (de unde vine „Ragchew Machine")
- <https://github.com/afarhan/zbitxv2> — sursa oficiala zBitx v2 (VU2ESE)
- <https://github.com/Rhizomatica/sbitx-core> — software de control sbitx
- <https://github.com/dg0jde/zbitxd> — daemon systemd zBitx

**Radioberry**
- Proiectul PA3GSB — gateware FPGA + sursa host pe Pi. **Sursa de adevar pentru
  protocolul din faza 0.**

**RP2350**
- `pico-sdk` 2.x, `pico-extras` (`audio_i2s`)
- Erata E9 (pull-down pe intrari) — vezi `WIRING.md` §5.3

**Analizor logic pe RP2040** (faza 0)
- <https://github.com/gusmanb/logicanalyzer> — 24 ch / 100 MS/s, GUI propriu,
  export sigrok. **Fara suport Pico 2**, din cauza eratei E9
- <https://github.com/pico-coder/sigrok-pico> — in sigrok mainline, UF2 gata facute,
  are variante pentru RP2350
- <https://hackaday.io/project/190583-la-micro-logic-analyzer-for-rp2040> — μLA,
  alternativa
- PulseView de pe <https://sigrok.org> — **obligatoriu mai nou de 0.4.2**

**Waveshare RP2350-PiZero** (host varianta B)
- Produs: <https://www.waveshare.com/rp2350-pizero.htm>
- Wiki: <https://www.waveshare.com/wiki/RP2350-PiZero>
- Schema: <https://files.waveshare.com/wiki/RP2350-PiZero/RP2350-PiZero.pdf>
- Exemple PIO-USB (host) si DVI in pachetul de demo Waveshare
