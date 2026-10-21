// rx_audio — receptor SSB/CW/AM/FM: Radioberry v2 (CL025) + Waveshare RP2350-PiZero + WM8960 Audio HAT (rezervă: PCM5102A)
//
// Lanț: FPGA (gateware PIO 75.2 din RagchewBerry/WP3DN, 4 linii + meta, citit cu PIO + DMA) -> IQ 48 kHz
//       -> decimare /4 (12 kHz) -> filtru complex trece-bandă -> demodulare -> AGC -> ×4 -> I2S 48 kHz.
// Protocol și capcane: PROTOCOL.md §7 (§6 = vechiul protocol clasic pe 8 linii, gateware HL2 73.3).
// Doar pe RP2350-PiZero (Radioberry înfipt în header).
//
// UI (meniu cu un singur buton): apăsare scurtă = parametrul următor (FRECV > PAS > MOD > FILTRU > BANDA > VOL > GAIN);
//     rotire = schimbă parametrul evidențiat; apăsare lungă sau 6 s fără atingere = înapoi la FRECV.
//     Fiecare bandă își ține minte ultima frecvență și ultimul mod.
// 4 butoane (MOD, BANDA, FILTRU, PAS) pe același pin cu apăsarea encoderului (GP24, pin 18), fără ADC (pe
//     header nu e niciun pin analogic): rețea RC, butonul se recunoaște după timpul de descărcare. WIRING.md §0.6.
//     Rețeaua se detectează singură la pornire; fără ea, pinul 18 rămâne buton simplu, ca înainte.
// Setările (frecvență, mod, pas, filtre, volum, câștig, memoria benzilor) se salvează în flash la 5 s
//     după ultima schimbare și se reîncarcă la pornire.
// CAT: emulare Kenwood TS-2000 pe același port (comenzi cu „;”): WSJT-X, fldigi, Omni-Rig/HDSDR.
// USB (115200): s stare | f<Hz> frecvența | m<0-5> modul (USB LSB CW AM FM SAM) | l<dB> squelch: prag în dB peste zgomotul estimat, 1..40 (l0 = oprit) | n<0-3> reducere zgomot | c calibrare automată (în SAM, pe o stație AM) | c<ppb> calibrare manuală | v<0-100> volumul | g<-12..48> câștig RX dB | k butoane (diagnostic) | w<Hz> lățimea filtrului | x inversează IQ (LSB<->USB) | q1/q0 flux IQ binar pe USB (tools/iq_record.py)
//   TX: t stare | t1/t0 PTT | tt TUNE (ton +1 kHz) | td<0-15> drive (și în meniu, salvat) | tg<dB> microfon | tq orientare Q | tb1 BOOT=PTT
//   PTT și pe BOOT (ținut), butonul de 4,7 kΩ de pe rețeaua RC și CAT (TX;/RX;)
//   WM8960: a stare | ah<0x30-0x7F> căști (121 = 0 dB, 0 = mut) | as<reg> difuzor (0 = oprit) | am<0-63> PGA microfon | ab<0-3> boost | al1/al0 microfonul în căști (test) | au1/au2/au0 microfonul L/R pe USB în locul radioului (test)
//
// Ieșirea audio e izolată în audio_out_*(), ca să poată fi înlocuită (ex. CM108AH pe USB).

#include <Arduino.h>
#include <Wire.h>
#include <I2S.h>
#include <U8g2lib.h>
#include <math.h>
#include <EEPROM.h>
#include <Adafruit_TinyUSB.h>          // stiva USB „Adafruit TinyUSB": CDC + microfon USB
#include "usb_audio.h"
#include "wm8960.h"
struct Settings;                    // definită mai jos; aici doar pt. prototipurile generate de Arduino
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "gateware_cl025.h"

#if !defined(ARDUINO_WAVESHARE_RP2350_PIZERO)
#error "rx_audio e doar pentru Waveshare RP2350-PiZero (FQBN rp2040:rp2040:waveshare_rp2350_pizero)"
#endif

// ============================================================
// Pini RP2350-PiZero cu Radioberry înfipt (gateware PIO). „hdr" = pinul fizic al header-ului.
// Liberi de FPGA (netlist Radioberry.net): BCM 0,1,2,3,14,15 (+BCM4 intrare FPGA nefolosită,
// +BCM24/DCLK după configurare). INTERZIS: BCM17, BCM21 (ieșiri FPGA pi_cwl/pi_cwr).
// Față de protocolul clasic nu mai sunt linii de date RX: BCM 5, 12, 13, 16, 23 (GP 15, 9, 13, 16, 23).
// Rămân intrări până se confirmă că gateware-ul PIO nu le comandă.
// ============================================================
//                                   GP   BCM hdr
#define PIN_FPGA_NCONFIG   27   //  27   27  13
#define PIN_FPGA_DATA0     13   //  13   13  33
#define PIN_FPGA_DCLK      24   //  24   24  18  (după configurare = apăsarea encoderului)
#define PIN_FPGA_NSTATUS   26   //  26   26  37
#define PIN_FPGA_CONFDONE  22   //  22   22  15
#define PIN_RX_CLK          6   //   6    6  31   ceas RX (side-set PIO)
#define PIN_RX_RDY         25   //  25   25  22   RDY (jmp pin PIO)
#define PIN_RX_D0          18   //  18   18  12   date RX D0..D3 = GP18..21 (consecutivi, in pins PIO)
static const uint8_t FREED_GP[5] = { 15, 9, 13, 16, 23 };              // foste linii de date clasice
#define PIN_SPI_SCK        10   //  10   11  23   (SPI1 hardware)
#define PIN_SPI_MOSI       11   //  11   10  19
#define PIN_SPI_MISO       12   //  12    9  21
#define PIN_SPI_CE0         8   //   8    8  24
#define PIN_SPI_CE1         7   //   7    7  26
#define PIN_I2S_BCK         4   //   4   14   8   PCM5102A BCK
#define PIN_I2S_LRCK        5   //   5   15  10   PCM5102A LCK  (= BCK + 1)
#define PIN_I2S_DATA       14   //  14    4   7   PCM5102A DIN (doar fără WM8960)
#define PIN_I2S_DOUT       16   //  16   16  36   WM8960 DACDAT (căști)   — linie eliberată de gateware-ul PIO
#define PIN_I2S_DIN        23   //  23   23  16   WM8960 ADCDAT (microfon) — idem
#define PIN_OLED_SDA        2   //   2    2   3
#define PIN_OLED_SCL        3   //   3    3   5
#define PIN_ENC_A           0   //   0    0  27
#define PIN_ENC_B           1   //   1    1  28
#define PIN_ENC_SW         24   //  24   24  18   = DCLK, folosit doar după configurarea FPGA

// ============================================================
// Stare partajată
// ============================================================
enum { MODE_USB, MODE_LSB, MODE_CW, MODE_AM, MODE_FM, MODE_SAM, MODE_N };   // FM = NBFM (10 m, CB); SAM = AM sincron
static const char *MODE_NAME[MODE_N] = { "USB", "LSB", "CW", "AM", "FM", "SAM" };
static volatile uint32_t g_freq = 7074000;
static volatile int      g_mode = MODE_USB;
// Orientarea spectrului — convenția firmware-ului: frecvențe pozitive = USB (verificată cu FT8 în jt9, 3 oct 2026).
// Gateware-ul HL2 73.3 o dădea direct (I + jQ); gateware-ul PIO 75.2 dă spectrul INVERS (purtătoarea de pe
// 9640 kHz apare la -3193 Hz cu acord pe 9637 kHz, PROTOCOL.md §7) -> Q se neagă la citire (RB_Q_NEG).
// Comanda USB „x” comută în plus orientarea, pentru teste.
static const bool        RB_Q_NEG = true;
static volatile bool     g_iq_inv = false;
// Calibrarea frecvenței: eroarea ceasului Radioberry, în ppb (+ = placa recepționa mai sus decât afișa).
// Măsurat 3 oct: ~+23 ppm (stațiile de pe 9640/9630 kHz apăreau cu 222 Hz sub centru).
static volatile int32_t  g_cal_ppb = 0;
static inline uint32_t rb_freq() {                  // frecvența trimisă la FPGA = cea afișată, corectată
  int64_t f = g_freq;
  return (uint32_t)(f - (f * g_cal_ppb + (g_cal_ppb >= 0 ? 500000000LL : -500000000LL)) / 1000000000LL);
}
// butoane pe pinul 18 (vezi keys_read)
enum { K_NONE, K_SW, K_MOD, K_BAND, K_FILT, K_STEP };
static volatile bool     g_keys_rc = false;         // rețeaua RC e montată (detectat la pornire)
static volatile uint32_t g_keys_detect_us = 0, g_key_us = 0;
static volatile int      g_key_last = K_NONE;
static volatile bool     g_keys_pause = false;
static const char *KEY_NAME[] = { "-", "ENCODER", "MOD", "BANDA", "PTT", "PAS" };   // 4,7 kΩ: fost FILTRU, acum PTT (ținut)      // diagnosticul „kd" ține nucleul 1 departe de pin
static volatile int      g_vol = 30;                // 0..100
static volatile int      g_gain_db = 20;            // câștig LNA AD9866: -12..+48 dB
static volatile bool     g_gain_dirty = false;
static int               g_gain_sent = 0;           // câștigul trimis efectiv la FPGA (pentru squelch)
static volatile bool     g_freq_dirty = true, g_mode_dirty = true;
static volatile int      g_fpga = 0;                // 1 = OK
static volatile uint8_t  g_gw_major = 0, g_gw_minor = 0;
static volatile float    g_smeter_db = -140;
static volatile uint32_t g_iq_rate = 0, g_audio_drop = 0, g_audio_under = 0;
static volatile uint32_t g_oled_test_until = 0;  // comanda o: model de test pe OLED până la acest moment
static volatile bool     g_ui_ready = false;        // nucleul 1 poate lua pinul DCLK ca buton
// WM8960: 0 = încă nu s-a încercat, 1 = pornit (codec master, PIO duplex), -1 = lipsă -> PCM5102A
static volatile int      g_wm = 0;
static volatile int      g_wm_hp_req = -1, g_wm_spk_req = -1, g_wm_mic_req = -1;   // cereri I2C pt. nucleul 1
static volatile int      g_wm_hp = 0x73, g_wm_spk = 0x70, g_wm_pga = 0x27, g_wm_boost = 3;   // implicit: căști -6 dB, difuzor -9 dB
static volatile bool     g_mic_loop = false;        // test: microfonul WM8960 în căști în locul radioului
static volatile int      g_mic_usb = 0;             // test: 0 = USB primește radioul, 1 = microfonul stâng WM8960, 2 = dreptul
static volatile int      g_wm_retry = 0;            // ar: 1 = cerut, 2 = nucleul 1 a terminat init-ul (rezultat în g_wm_init)
static volatile int      g_wm_init = 0;
static volatile bool     g_i2c_scan = false;        // ai: scanare I2C pe nucleul 1
// 100 kHz, nu 400: cu OLED + WM8960 HAT pe fire (două seturi de pull-up), la 400 kHz transmisiile cădeau
// la întâmplare (timeout la registre WM8960, SDA blocat) — 22 oct.
#define I2C_HZ 400000   // înapoi la 400 kHz după reparația GND/pull-up (100 kHz făcea UI-ul lent: ~90 ms/cadru)
static volatile uint32_t g_i2c_fixes = 0;
static volatile uint32_t g_core1_beat = 0;         // crește la fiecare trecere prin loop1 (nucleul 1 trăiește?)
static volatile bool     g_wm_stop_req = false;    // aX: resetează codecul (oprește BCLK/LRCLK) — test de diafonie
static volatile bool     g_boot_ptt = false;        // BOOT ca PTT: oprit implicit (citirea îngheață scurt nucleul 0)          // deblocări I2C în funcționare
static volatile int      g_i2c_recover_pulses = -1;
// TX (detalii în secțiunea EMISIE)
enum { TXS_MIC_SSB, TXS_MIC_AM, TXS_TONE };
static volatile bool     g_tx_on = false;           // emite: MOX pus pe toate comenzile SPI
static volatile int      g_tx_src = 2;              // TXS_*
static volatile bool     g_ptt_boot = false, g_ptt_key = false, g_ptt_cat = false, g_ptt_usb = false, g_tune_req = false;
static volatile bool     g_tx_lock = false;         // refuz / timp maxim: se așteaptă eliberarea tuturor PTT-urilor
static volatile int      g_tx_drive = 4;            // 0..15 (RagchewBerry pornește cu 4)
static volatile bool     g_tx_drive_dirty = false;
static volatile int      g_tx_mic_db = 0;           // câștig digital al microfonului la emisie
static volatile bool     g_tx_qneg = true;          // Radioberry: spectrul TX inversat (ca la RagchewBerry)
static void tx_mic48(int16_t s);
static void tx_rx_probe(float i, float q);
static bool tx_band_ok(uint32_t f); // impulsuri SCL date la pornire ca să deblocheze SDA (-1 = n-a fost nevoie)

// ============================================================
// FPGA: încărcare passive serial (LSB primul)
// ============================================================
static int fpga_load() {
  gpio_set_dir(PIN_FPGA_DATA0, GPIO_OUT);
  gpio_set_dir(PIN_FPGA_DCLK, GPIO_OUT);
  gpio_put(PIN_FPGA_NCONFIG, 0); gpio_put(PIN_FPGA_DATA0, 0); gpio_put(PIN_FPGA_DCLK, 0);
  delay(10);
  if (gpio_get(PIN_FPGA_NSTATUS)) return -2;        // un FPGA alimentat trage nSTATUS jos
  gpio_put(PIN_FPGA_NCONFIG, 1);
  uint32_t t0 = millis();
  while (!gpio_get(PIN_FPGA_NSTATUS)) if (millis() - t0 > 2000) return -1;
  for (uint32_t n = 0; n < GATEWARE_LEN; n++) {
    uint8_t b = GATEWARE[n];
    for (int i = 0; i < 8; i++) {
      gpio_put(PIN_FPGA_DATA0, (b >> i) & 1);
      gpio_put(PIN_FPGA_DCLK, 1);
      __asm volatile("nop\nnop\nnop\nnop\nnop\nnop");
      gpio_put(PIN_FPGA_DCLK, 0);
    }
  }
  if (!gpio_get(PIN_FPGA_NSTATUS) || !gpio_get(PIN_FPGA_CONFDONE)) return -1;
  gpio_put(PIN_FPGA_DCLK, 1); gpio_put(PIN_FPGA_DCLK, 0);
  gpio_put(PIN_FPGA_DCLK, 1); gpio_put(PIN_FPGA_DCLK, 0);
  gpio_set_dir(PIN_FPGA_DATA0, GPIO_IN);            // BCM13 nu mai e folosit după configurare
  gpio_set_dir(PIN_FPGA_DCLK, GPIO_IN);             // BCM24 devine butonul encoderului
  gpio_pull_up(PIN_FPGA_DCLK);
  return 1;
}

// ============================================================
// SPI control: mod 3, 6 octeți [stare, C0, C1..C4]; răspuns: [4].[5] = versiune
// ============================================================
static const uint32_t RB_REG0 = 0x00000004;         // 48 kHz, 1 RX, DUPLEX=1 (altfel RX1 = frecv. TX)
// Câștig RX: adresa 0x0A (C0 = 0x14), cmd_data[6:0] = 0x40 | (dB + 12). Gateware-ul HL2 pornea pe -12 dB
// (surd fără comandă), cel PIO pornește pe câștig mare (~-47 dBFS cu antena) -> comanda se trimite la
// pornire și se retrimite ciclic, ca valoarea aleasă să fie mereu cea efectivă.
static inline uint32_t rb_gain_word() { return 0x40u | (uint32_t)(g_gain_db + 12); }
static void rb_cmd(uint8_t c0, uint32_t data) {
  if (g_tx_on) c0 |= 1;                              // MOX: bitul 0 pe fiecare comandă cât timp emite
  uint8_t tx[6] = { 0x05, c0, (uint8_t)(data >> 24), (uint8_t)(data >> 16), (uint8_t)(data >> 8), (uint8_t)data }, rx[6];
  gpio_put(PIN_SPI_CE0, 0);
  spi_write_read_blocking(spi1, tx, rx, 6);
  gpio_put(PIN_SPI_CE0, 1);
  if (rx[4] || rx[5]) { g_gw_major = rx[4]; g_gw_minor = rx[5]; }
}

// ============================================================
// DSP — totul la 12 kHz după decimare
// ============================================================
static const int FS_IN = 48000, DECIM = 4, FS = FS_IN / DECIM;
#define NDEC 41                                     // FIR decimare (trece-jos 4,5 kHz la 48 kHz)
#define NBP  127                                    // FIR complex trece-bandă la 12 kHz
static float h_dec[NDEC];
static float hb_r[NBP], hb_i[NBP];
static float dI[2 * NDEC], dQ[2 * NDEC]; static int dpos = 0, dphase = 0;
static float bI[2 * NBP], bQ[2 * NBP];   static int bpos = 0;
static float agc_env = 1e-4f, dc_x = 0, dc_y = 0, sm_acc = 0; static int sm_n = 0;

static void design_lowpass(float *h, int n, float fc, float fs) {
  int m = n - 1; float sum = 0;
  for (int k = 0; k < n; k++) {
    float x = k - m / 2.0f;
    float sinc = (x == 0) ? 2 * fc / fs : sinf(2 * (float)M_PI * fc / fs * x) / ((float)M_PI * x);
    float w = 0.54f - 0.46f * cosf(2 * (float)M_PI * k / m);   // Hamming
    h[k] = sinc * w; sum += h[k];
  }
  for (int k = 0; k < n; k++) h[k] /= sum;
}

// Lățimea filtrului, aleasă separat pentru fiecare mod (meniul FILTRU sau comanda w<Hz>).
static const int BW_SSB[] = { 1800, 2000, 2500, 2700 };
static const int BW_CW[]  = { 250, 500, 1000 };
static const int BW_AM[]  = { 6000, 8000 };
static const int BW_FM[]  = { 8000, 11000 };            // NBFM: deviație ±2,5 kHz (10 m, CB)
static volatile int g_bw_idx[MODE_N] = { 2, 2, 1, 1, 1, 1 };  // USB/LSB 2,5 kHz, CW 500 Hz, AM/SAM 8 kHz, FM 11 kHz

static int bw_count(int mode) {
  return mode == MODE_CW ? 3 : (mode == MODE_AM || mode == MODE_SAM || mode == MODE_FM) ? 2 : 4;
}
static int bw_hz(int mode) {
  int i = g_bw_idx[mode];
  return mode == MODE_CW ? BW_CW[i] : (mode == MODE_AM || mode == MODE_SAM) ? BW_AM[i] : mode == MODE_FM ? BW_FM[i] : BW_SSB[i];
}
// marginile benzii trecute, în Hz față de frecvența acordată
static void filter_edges(int mode, float *lo, float *hi) {
  float bw = bw_hz(mode);
  switch (mode) {
    case MODE_USB: *lo = 250;         *hi = 250 + bw; break;   // SSB: de la 250 Hz în sus
    case MODE_LSB: *lo = -250 - bw;   *hi = -250;     break;
    case MODE_CW:  *lo = 700 - bw / 2; *hi = 700 + bw / 2; break;
    default:       *lo = -bw / 2;     *hi = bw / 2;   break;   // AM, FM
  }
}

// filtru complex: trece-jos de lățime bw/2 mutat la f0 (f0<0 = banda de jos, pt. LSB)
static void design_bandpass(int mode) {
  float lo, hi;
  filter_edges(mode, &lo, &hi);
  float f0 = (lo + hi) / 2, half = (hi - lo) / 2;
  static float h[NBP];
  design_lowpass(h, NBP, half, FS);
  for (int k = 0; k < NBP; k++) {
    float ph = 2 * (float)M_PI * f0 * (k - (NBP - 1) / 2.0f) / FS;
    // dsp_push aplică coeficienții pe buffer de la cel mai VECHI eșantion la cel mai nou (corelație, nu convoluție),
    // deci faza se ia cu semn schimbat ca banda trecută să fie la +f0. Cu +sin, filtrul „USB” lăsa LSB și invers
    // (verificat numeric 3 oct: +1000 Hz atenuat 74 dB). Eroarea era compensată greșit prin negarea lui Q.
    hb_r[k] = h[k] * cosf(ph);
    hb_i[k] = -h[k] * sinf(ph);
  }
}

// un eșantion IQ la 48 kHz -> 0 sau 1 eșantion audio (la 12 kHz)
// ============================================================
// Squelch, reducere de zgomot, AM sincron. Ideile vin din PicoRX (github.com/dawsonjon/PicoRX, MIT,
// © 2024 Jonathan P Dawson); aici rescrise în virgulă mobilă pentru lanțul nostru de 12 kHz.
// ============================================================
static volatile int  g_sql = 0;                      // 0 = oprit; altfel pragul de deschidere în dB PESTE ZGOMOT (1..40)
static const int     SQL_MAX = 40;
static volatile int  g_nr = 0;                       // 0 = oprit; 1..3 = tăria reducerii de zgomot
static volatile bool g_sq_open = true;               // pentru ecran
static volatile float g_sq_nf = -140;                // zgomotul estimat în banda filtrului, dBFS (ecran / stare)

// Squelch relativ la zgomot: nivelul din banda filtrului, netezit ~17 ms, se compară cu zgomotul estimat + prag.
// Pragul fix în dBFS (până pe 3 oct) depindea de câștig și de bandă: la câștig mic zgomotul trecea peste el și
// squelch-ul pâlpâia. Zgomotul = „minimum statistics": media puterii pe blocuri de 0,1 s, apoi minimul ultimelor
// 8 s (80 de blocuri) — urmărește media zgomotului, nu golurile lui; coboară instant, urcă în ≤8 s. Cu 3 s,
// o stație care vorbea continuu devenea „zgomot" și își închidea singură squelch-ul (măsurat pe 40 m). Pe SSB/CW e
// suficient (vorbirea și manipularea au pauze). Pe AM/SAM/FM purtătoarea e continuă: cât squelch-ul e deschis,
// estimarea urcă doar cu 0,5 dB/s, ca purtătoarea să nu-și închidă singură squelch-ul. La schimbarea frecvenței,
// câștigului LNA estimarea se mută exact cu diferența (sq_gain_shift), fără să mai aștepte.
// Deschide peste zgomot + prag, închide la 3 dB sub, după 250 ms de reținere (să nu taie între cuvinte).
// Poarta se deschide/închide lin (~20 ms), fără clicuri.
#define SQ_NB 80
static float sq_pow = 1e-12f, sq_gate = 1, sq_nf = 0;
static float sq_blk[SQ_NB], sq_acc = 0; static int sq_acc_n = 0, sq_bi = 0, sq_bn = 0;
static volatile int sq_gain_shift = 0;            // dB de adăugat estimării (schimbare de câștig)
static int sq_hang = 0;
static void squelch_noise(float p) {
  int sh = sq_gain_shift;
  if (sh) { sq_gain_shift -= sh; for (int k = 0; k < sq_bn; k++) sq_blk[k] += sh; sq_nf += sh; g_sq_nf = sq_nf; }
  sq_acc += p;
  if (++sq_acc_n < FS / 10) return;
  sq_blk[sq_bi] = 10 * log10f(sq_acc / sq_acc_n + 1e-20f);
  sq_bi = (sq_bi + 1) % SQ_NB; if (sq_bn < SQ_NB) sq_bn++;
  sq_acc = 0; sq_acc_n = 0;
  float m = sq_blk[0];
  for (int k = 1; k < sq_bn; k++) if (sq_blk[k] < m) m = sq_blk[k];
  bool carrier = g_mode == MODE_AM || g_mode == MODE_SAM || g_mode == MODE_FM;
  if (sq_bn == 1 || m <= sq_nf || !(carrier && g_sq_open && g_sql)) sq_nf = m;
  else sq_nf = fminf(m, sq_nf + 0.05f);              // 0,5 dB/s (10 blocuri/s)
  g_sq_nf = sq_nf;
}
static float squelch_gate(float p) {
  sq_pow += 0.005f * (p - sq_pow);
  float lvl = 10 * log10f(sq_pow + 1e-20f);
  squelch_noise(p);
  int thr = g_sql;
  bool open = g_sq_open;
  if (!thr) open = true;
  else {
    if (lvl > sq_nf + thr) { open = true; sq_hang = FS / 4; }
    else if (sq_hang > 0) sq_hang--;
    else if (lvl < sq_nf + thr - 3) open = false;
  }
  g_sq_open = open;
  sq_gate += ((open ? 1.0f : 0.0f) - sq_gate) * 0.004f;
  return sq_gate;
}

// Reducere de zgomot spectrală pe audio (12 kHz): FFT 256 cu suprapunere 50 % (fereastră Hann, sumă = 1).
// Puterea pe fiecare bin se netezește în timp (0,7); zgomotul = minimul ei (urcă ~6 dB/s), × 2 ca să compenseze
// faptul că minimul stă sub medie. Câștig = 1 - α·zgomot/putere, limitat jos și netezit (mai puțin „zgomot
// muzical”). Întârziere 256 eșantioane = 21 ms. Simulat (ton intermitent în zgomot alb, SNR 3 dB): NR1 +9 dB,
// NR3 +14,5 dB SNR, zgomotul din pauze -10 / -18 dB, semnalul util -0,2 dB.
#define NR_N 256
#define NR_H (NR_N / 2)
static void fft256(float *re, float *im);
static float nr_in[NR_N], nr_ola[NR_H], nr_out[NR_H], nr_win[NR_N];
static float nr_noise[NR_H + 1], nr_gain[NR_H + 1], nr_pws[NR_H + 1];
static int nr_pos = 0;
static float nr_process(float x) {
  static bool init = false;
  if (!init) {
    for (int k = 0; k < NR_N; k++) nr_win[k] = 0.5f - 0.5f * cosf(2 * (float)M_PI * k / NR_N);
    for (int k = 0; k <= NR_H; k++) { nr_noise[k] = -1; nr_gain[k] = 1; nr_pws[k] = 0; }
    init = true;
  }
  float y = nr_out[nr_pos];
  nr_in[NR_H + nr_pos] = x;
  if (++nr_pos < NR_H) return y;
  nr_pos = 0;

  static float re[NR_N], im[NR_N];
  for (int k = 0; k < NR_N; k++) { re[k] = nr_in[k] * nr_win[k]; im[k] = 0; }
  fft256(re, im);
  int lvl = g_nr;
  float alpha = lvl == 1 ? 1.0f : lvl == 2 ? 1.6f : 2.4f;
  float gmin  = lvl == 1 ? 0.20f : lvl == 2 ? 0.12f : 0.07f;
  for (int k = 0; k <= NR_H; k++) {
    float pw = re[k] * re[k] + im[k] * im[k];
    float pws = nr_pws[k] = 0.7f * nr_pws[k] + 0.3f * pw;
    float nz = nr_noise[k];
    if (nz < 0) nz = pws = nr_pws[k] = pw;                   // primul cadru: pornește de la nivelul curent
    nz = (pws < nz) ? pws : nz * 1.015f;                     // 1.015^(12000/128) ≈ +6 dB/s
    if (nz < 1e-16f) nz = 1e-16f;
    nr_noise[k] = nz;
    float g = 1 - alpha * 2.0f * nz / (pws + 1e-20f);
    if (g < gmin) g = gmin;
    nr_gain[k] = 0.5f * nr_gain[k] + 0.5f * g;
    float gg = nr_gain[k];
    re[k] *= gg; im[k] *= gg;
    if (k > 0 && k < NR_H) { re[NR_N - k] *= gg; im[NR_N - k] *= gg; }
  }
  for (int k = 0; k < NR_N; k++) im[k] = -im[k];       // IFFT = conj(FFT(conj(X))) / N
  fft256(re, im);
  for (int k = 0; k < NR_H; k++) {
    nr_out[k] = nr_ola[k] + re[k] / NR_N;
    nr_ola[k] = re[k + NR_H] / NR_N;
  }
  memmove(nr_in, nr_in + NR_H, NR_H * sizeof(float));
  return y;
}

// AM sincron: PLL de ordinul 2 pe purtătoare (bandă ~30 Hz, prinde ±300 Hz); demodularea = partea în fază
// după rotirea cu faza purtătoarei. La fading selectiv nu mai distorsionează ca detectorul de anvelopă.
static float sam_ph = 0, sam_fr = 0, sam_dc_x = 0, sam_dc_y = 0;
static volatile float g_sam_offset_hz = 0;
static float sam_demod(float yr, float yi) {
  float c = cosf(sam_ph), sn = sinf(sam_ph);
  float zr = yr * c + yi * sn, zi = yi * c - yr * sn;  // y · e^(-j·φ)
  float e = atan2f(zi, zr);                            // eroarea de fază
  sam_fr += 0.000247f * e;                             // ωn = 2π·30/12000, ζ = 0,707
  const float FMAX = 2 * (float)M_PI * 300 / 12000;
  if (sam_fr > FMAX) sam_fr = FMAX; if (sam_fr < -FMAX) sam_fr = -FMAX;
  sam_ph += sam_fr + 0.0222f * e;
  if (sam_ph > (float)M_PI) sam_ph -= 2 * (float)M_PI; else if (sam_ph < -(float)M_PI) sam_ph += 2 * (float)M_PI;
  g_sam_offset_hz = sam_fr * 12000 / (2 * (float)M_PI);
  float a = zr - sam_dc_x + 0.995f * sam_dc_y;         // purtătoarea devine DC: se blochează
  sam_dc_x = zr; sam_dc_y = a;
  return a;
}

static float g_dsp_raw = 0;                          // ieșirea demodulatorului fără volumul căștilor (pt. USB)

static bool dsp_push(float I, float Q, float *out) {
  dI[dpos] = dI[dpos + NDEC] = I;
  dQ[dpos] = dQ[dpos + NDEC] = Q;
  if (++dpos >= NDEC) dpos = 0;
  if (++dphase < DECIM) return false;
  dphase = 0;
  float xi = 0, xq = 0;
  const float *pi = &dI[dpos], *pq = &dQ[dpos];
  for (int k = 0; k < NDEC; k++) { xi += h_dec[k] * pi[k]; xq += h_dec[k] * pq[k]; }

  bI[bpos] = bI[bpos + NBP] = xi;
  bQ[bpos] = bQ[bpos + NBP] = xq;
  if (++bpos >= NBP) bpos = 0;
  const float *ri = &bI[bpos], *rq = &bQ[bpos];
  float yr = 0, yi = 0;
  int mode = g_mode;
  if (mode == MODE_AM || mode == MODE_SAM || mode == MODE_FM) {
    for (int k = 0; k < NBP; k++) { yr += hb_r[k] * ri[k]; yi += hb_r[k] * rq[k]; }
  } else {
    for (int k = 0; k < NBP; k++) { yr += hb_r[k] * ri[k] - hb_i[k] * rq[k]; yi += hb_r[k] * rq[k] + hb_i[k] * ri[k]; }
  }
  float p = yr * yr + yi * yi;                       // putere în banda filtrului (S-metru)
  sm_acc += p; if (++sm_n >= FS / 10) { g_smeter_db = 10 * log10f(sm_acc / sm_n + 1e-20f); sm_acc = 0; sm_n = 0; }

  float a;
  if (mode == MODE_FM) {
    // discriminator în cuadratură: unghiul dintre eșantionul curent și cel anterior = frecvența instantanee
    static float pr = 0, pq = 0, de = 0;
    float d = atan2f(yi * pr - yr * pq, yr * pr + yi * pq);
    pr = yr; pq = yi;
    d *= 12000.0f / (2 * (float)M_PI * 3000.0f);     // ±3 kHz deviație -> ±1
    de += 0.41f * (d - de);                          // de-accentuare, pol la ~1 kHz
    float fm = de * 0.5f;
    if (g_nr) fm = nr_process(fm);
    g_dsp_raw = fm;                                  // FM are amplitudine constantă: fără AGC
    *out = g_dsp_raw * (g_vol / 100.0f) * squelch_gate(p);
    return true;
  } else if (mode == MODE_SAM) {
    a = sam_demod(yr, yi);
  } else if (mode == MODE_AM) {
    float env = sqrtf(p);
    a = env - dc_x + 0.995f * dc_y;                  // blocare DC
    dc_x = env; dc_y = a;
  } else {
    a = yr;                                          // SSB/CW: partea reală
  }
  if (g_nr) a = nr_process(a);                       // înaintea AGC: zgomotul de fond e stabil acolo
  // AGC: atac instant, revenire ~0,5 s
  float m = fabsf(a);
  agc_env = (m > agc_env) ? m : agc_env * 0.99983f;
  if (agc_env < 3e-6f) agc_env = 3e-6f;
  float g = 0.3f / agc_env;
  g_dsp_raw = a * g;                                 // USB-ul (WSJT-X etc.) primește fără squelch
  *out = g_dsp_raw * (g_vol / 100.0f) * squelch_gate(p);
  return true;
}

// ============================================================
// Ieșire audio, 48 kHz. Demodulatorul dă 12 kHz -> interpolare liniară ×4 -> coada aq -> I2S.
// Două variante, alese la pornire:
//  * WM8960 (dacă răspunde pe I2C): codecul e MASTER de ceas (cristalul lui de 24 MHz), RP2350 e slave.
//    Un program PIO duplex scrie DACDAT (GP16) și citește ADCDAT (GP23); 2 canale DMA în inel, fără IRQ
//    (ca la IQ: handler-ele DMA crăpau firmware-ul). Biblioteca I2S din core nu merge: intrarea ei slave
//    cere DIN = BCLK - 1 = GP3 (SCL-ul OLED-ului).
//  * PCM5102A (fallback, ca înainte): biblioteca I2S, master, BCK GP4 / LRCK GP5 / DIN GP14.
// Ceasul FPGA și ceasul audio diferă cu câțiva ppm: coada aq absoarbe; plină -> se aruncă, goală -> se repetă.
// ============================================================
static const int FS_OUT = 48000, UPS = FS_OUT / FS;
static I2S i2s(OUTPUT);
#define AQ 4096
static int16_t aq[AQ]; static int aq_w = 0, aq_r = 0;
static float up_prev = 0;

// PIO duplex slave, 16 biți/canal, 32 BCLK/cadru. Instrucțiunile se construiesc la rulare (pinii sunt în
// „wait gpio"). Datele: ies pe frontul căzător al BCLK, se citesc pe cel crescător; MSB la un BCLK după LRCLK.
//   0: wait 1 gpio LRCK      sincronizare o singură dată: LRCLK cade (= începe canalul stâng) ...
//   1: wait 0 gpio LRCK
//   2: wait 1 gpio BCK       ... frontul crescător al bitului de întârziere (LSB-ul dreptei anterioare)
//   3: pull noblock          <- wrap_target: un cadru = 1 cuvânt (stânga în biții 31:16); FIFO gol -> X (= 0)
//   4: set y, 31
//   5: wait 0 gpio BCK
//   6: out pins, 1
//   7: wait 1 gpio BCK
//   8: in pins, 1            autopush la 32 biți: cuvântul = stânga<<16 | dreapta
//   9: jmp y--, 5            <- wrap
#define WM_RING_BITS  15                              // 32 KB = 8192 cadre = 170 ms, în fiecare direcție
#define WM_RING_WORDS (1u << (WM_RING_BITS - 2))
static uint32_t wm_tx[WM_RING_WORDS] __attribute__((aligned(1u << WM_RING_BITS)));
static uint32_t wm_rx[WM_RING_WORDS] __attribute__((aligned(1u << WM_RING_BITS)));
static uint16_t wm_prog_instr[10];
static PIO  wm_pio; static uint wm_sm; static int wm_dma_tx = -1, wm_dma_rx = -1, wm_off = -1;
static uint32_t wm_tx_w = 0, wm_rx_rd = 0;           // indici în inele (următorul de scris / de citit)
static uint32_t wm_last = 0;                          // ultimul cadru scris (se repetă la goliciune)
static volatile uint32_t g_wm_frames = 0, g_wm_rate = 0;
static volatile int16_t  g_mic_pk_l = 0, g_mic_pk_r = 0;   // vârfuri microfon (se resetează la citire)

static uint32_t wm_tx_rd() { return ((uint32_t)dma_hw->ch[wm_dma_tx].read_addr - (uint32_t)wm_tx) / 4 % WM_RING_WORDS; }
static uint32_t wm_rx_wr() { return ((uint32_t)dma_hw->ch[wm_dma_rx].write_addr - (uint32_t)wm_rx) / 4 % WM_RING_WORDS; }

static bool wm_pio_start() {
  const uint B = PIN_I2S_BCK, L = PIN_I2S_LRCK;
  uint16_t *p = wm_prog_instr;
  p[0] = pio_encode_wait_gpio(true, L);  p[1] = pio_encode_wait_gpio(false, L); p[2] = pio_encode_wait_gpio(true, B);
  p[3] = pio_encode_pull(false, false);  p[4] = pio_encode_set(pio_y, 31);
  p[5] = pio_encode_wait_gpio(false, B); p[6] = pio_encode_out(pio_pins, 1);
  p[7] = pio_encode_wait_gpio(true, B);  p[8] = pio_encode_in(pio_pins, 1);
  p[9] = pio_encode_jmp_y_dec(5);        // adresă relativă: pio_add_program o relochează
  static const struct pio_program prog = { .instructions = wm_prog_instr, .length = 10, .origin = -1 };

  PIO pios[] = { pio0, pio1, pio2 };
  int off = -1, sm = -1;
  for (PIO pp : pios) {
    if (!pio_can_add_program(pp, &prog)) continue;
    if ((sm = pio_claim_unused_sm(pp, false)) < 0) continue;
    wm_pio = pp; off = wm_off = pio_add_program(pp, &prog); break;
  }
  if (off < 0) return false;
  wm_sm = (uint)sm;

  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_wrap(&c, off + 3, off + 9);
  sm_config_set_out_pins(&c, PIN_I2S_DOUT, 1);
  sm_config_set_in_pins(&c, PIN_I2S_DIN);
  sm_config_set_out_shift(&c, false, false, 32);     // MSB primul, fără autopull (pull noblock explicit)
  sm_config_set_in_shift(&c, false, true, 32);       // MSB primul, autopush la 32
  pio_gpio_init(wm_pio, PIN_I2S_DOUT);
  pio_sm_set_consecutive_pindirs(wm_pio, wm_sm, PIN_I2S_DOUT, 1, true);
  pio_gpio_init(wm_pio, PIN_I2S_DIN);
  pio_sm_set_consecutive_pindirs(wm_pio, wm_sm, PIN_I2S_DIN, 1, false);
  gpio_init(B); gpio_set_dir(B, GPIO_IN); gpio_init(L); gpio_set_dir(L, GPIO_IN);   // ceasurile vin de la codec
  gpio_disable_pulls(PIN_I2S_DIN);
  pio_sm_init(wm_pio, wm_sm, off, &c);
  pio_sm_exec(wm_pio, wm_sm, pio_encode_set(pio_x, 0));

  memset(wm_tx, 0, sizeof wm_tx);
  if ((wm_dma_tx = dma_claim_unused_channel(false)) < 0) return false;
  if ((wm_dma_rx = dma_claim_unused_channel(false)) < 0) return false;
  dma_channel_config t = dma_channel_get_default_config(wm_dma_tx);
  channel_config_set_transfer_data_size(&t, DMA_SIZE_32);
  channel_config_set_read_increment(&t, true);
  channel_config_set_write_increment(&t, false);
  channel_config_set_ring(&t, false, WM_RING_BITS);  // adresa de citire se învârte în inel
  channel_config_set_dreq(&t, pio_get_dreq(wm_pio, wm_sm, true));
  dma_channel_configure(wm_dma_tx, &t, &wm_pio->txf[wm_sm], wm_tx, dma_encode_endless_transfer_count(), true);
  dma_channel_config r = dma_channel_get_default_config(wm_dma_rx);
  channel_config_set_transfer_data_size(&r, DMA_SIZE_32);
  channel_config_set_read_increment(&r, false);
  channel_config_set_write_increment(&r, true);
  channel_config_set_ring(&r, true, WM_RING_BITS);
  channel_config_set_dreq(&r, pio_get_dreq(wm_pio, wm_sm, false));
  dma_channel_configure(wm_dma_rx, &r, wm_rx, &wm_pio->rxf[wm_sm], dma_encode_endless_transfer_count(), true);
  wm_tx_w = 3000; wm_rx_rd = 0;                       // rezerva pornește plină (liniște)
  pio_sm_set_enabled(wm_pio, wm_sm, true);
  return true;
}

static void audio_out_begin() {
  // nucleul 1 face probe + init pe I2C (bus comun cu OLED-ul); se așteaptă rezultatul
  uint32_t t0 = millis();
  while (g_wm == 0 && millis() - t0 < 3000) delay(5);
  if (g_wm == 1 && wm_pio_start()) return;
  if (g_wm == 1) g_wm = -2;                           // codec găsit, dar fără PIO/DMA liber
  i2s.setBCLK(PIN_I2S_BCK);
  i2s.setDATA(PIN_I2S_DATA);
  i2s.setBitsPerSample(16);
  i2s.setBuffers(8, 256);
  i2s.begin(FS_OUT);
}
static void aq_put(int16_t v) { aq[aq_w] = v; aq_w = (aq_w + 1) % AQ; }
static void audio_out_push(float x) {
  if (g_mic_loop && !g_tx_on) return;                // testul cu microfonul ține coada
  if (g_tx_on) x = 0;                                 // la emisie difuzorul tace (altfel microfonie)
  if (x > 1) x = 1; if (x < -1) x = -1;
  int n = (aq_w - aq_r + AQ) % AQ;
  if (n >= AQ - UPS) { g_audio_drop++; up_prev = x; return; }   // ceasul FPGA > ceasul audio: aruncă
  for (int k = 1; k <= UPS; k++) aq_put((int16_t)((up_prev + (x - up_prev) * k / UPS) * 32000));
  up_prev = x;
}
// microfon: citește cadrele noi din inelul ADC (vârfuri; în testul de buclă -> coada de ieșire)
static void audio_in_poll() {
  uint32_t wr = wm_rx_wr(), n = 0;
  int16_t pl = g_mic_pk_l, pr = g_mic_pk_r;
  while (wm_rx_rd != wr) {
    uint32_t f = wm_rx[wm_rx_rd]; wm_rx_rd = (wm_rx_rd + 1) % WM_RING_WORDS; n++;
    int16_t l = (int16_t)(f >> 16), r = (int16_t)f;
    if (abs(l) > pl) pl = abs(l); if (abs(r) > pr) pr = abs(r);
    if (g_tx_on) { if (g_tx_src != TXS_TONE) tx_mic48((int16_t)(((int32_t)l + r) / 2)); }
    else if (g_mic_loop && (aq_w - aq_r + AQ) % AQ < AQ - 1) aq_put((int16_t)(((int32_t)l + r) / 2));
    if (g_mic_usb) usb_audio_push48k(g_mic_usb == 1 ? l : r);
  }
  g_mic_pk_l = pl; g_mic_pk_r = pr; g_wm_frames += n;
}
static void audio_out_pump() {
  if (g_wm != 1) {
    while (aq_r != aq_w && i2s.availableForWrite() >= 4) {
      int16_t s = aq[aq_r]; aq_r = (aq_r + 1) % AQ;
      i2s.write16(s, s);
    }
    if (i2s.getUnderflow()) g_audio_under++;
    return;
  }
  audio_in_poll();
  // inelul DAC: DMA-ul citește la nesfârșit; scriem în fața lui, ținând 3000..4000 cadre (62..83 ms) avans.
  // Producătorul și consumatorul merg cu aceeași viteză, deci avansul stă lângă minim: minimul E rezerva.
  // Rezerva acoperă pauzele buclei principale (salvarea setărilor în flash, comenzi); depășită -> pârâit.
  uint32_t rd = wm_tx_rd();
  uint32_t ahead = (wm_tx_w - rd) % WM_RING_WORDS;
  if (ahead > WM_RING_WORDS * 3 / 4) {                // valoare de inel negativă: DMA-ul ne-a depășit, reluăm de la el
    wm_tx_w = (rd + 3000) % WM_RING_WORDS; ahead = 3000; g_audio_under++;
  }
  while (ahead < 4000) {
    if (aq_r != aq_w) {
      uint16_t s = (uint16_t)aq[aq_r]; aq_r = (aq_r + 1) % AQ;
      wm_last = ((uint32_t)s << 16) | s;              // mono pe ambele canale
    } else if (ahead >= 3000) break;                   // coada goală dar avem rezervă: așteptăm
    else g_audio_under++;                              // goliciune: se repetă ultimul cadru
    wm_tx[wm_tx_w] = wm_last; wm_tx_w = (wm_tx_w + 1) % WM_RING_WORDS; ahead++;
  }
}

// WM8960 apărut după pornire (HAT alimentat odată cu placa pornește mai încet, magistrală deblocată etc.):
// la 3 s se cere nucleului 1 o încercare; dacă reușește, sunetul trece de pe PCM5102A pe codec, fără restart.
static void wm_hot_poll() {
  static uint32_t t_try = 0;
  if (g_wm == -1 && g_wm_retry == 0 && millis() - t_try > 3000) { t_try = millis(); g_wm_retry = 1; }
  if (g_wm_retry != 2) return;
  if (g_wm_init == 1 && g_wm == -1) {
    i2s.end();                                        // PCM5102A eliberează GP4/GP5 (devin intrări de ceas)
    g_wm = wm_pio_start() ? 1 : -2;                   // ia și GP16/GP23 (pio_gpio_init)
  }
  g_wm_retry = 0;
}

// după o reinițializare a codecului ceasurile se opresc și repornesc: programul PIO se ia de la capăt
// (instrucțiunile 0-2 așteaptă începutul canalului stâng), altfel stânga/dreapta pot rămâne decalate
static void wm_pio_resync() {
  if (wm_off < 0) return;
  pio_sm_set_enabled(wm_pio, wm_sm, false);
  pio_sm_restart(wm_pio, wm_sm);
  pio_sm_exec(wm_pio, wm_sm, pio_encode_set(pio_x, 0));
  pio_sm_exec(wm_pio, wm_sm, pio_encode_jmp(wm_off));
  pio_sm_set_enabled(wm_pio, wm_sm, true);
}

static void wm_status() {
  if (g_wm != 1) {
    if (wm8960_fail_reg >= 0) Serial.printf("WM8960 init: prima scriere esuata = registrul 0x%02X (cod Wire %d, 2 = NACK adresa, 3 = NACK date)\n", wm8960_fail_reg, wm8960_fail_code);
    Serial.printf("audio: %s\n", g_wm == -2 ? "WM8960 gasit, dar fara PIO/DMA liber -> PCM5102A" : "WM8960 nu raspunde pe I2C (0x1A) -> PCM5102A pe GP14");
    return;
  }
  int16_t l = g_mic_pk_l, r = g_mic_pk_r; g_mic_pk_l = g_mic_pk_r = 0;
  auto db = [](int v) { return v > 0 ? 20 * log10f(v / 32768.0f) : -99.0f; };
  Serial.printf("audio: WM8960 master | cadre %lu/s | casti 0x%02X (%+d dB) | difuzor %s | mic PGA %d (%+.2f dB) boost %d | "
                "varf mic L %.1f R %.1f dBFS | avans DAC %lu | under %lu drop %lu | bucla mic %s\n",
                (unsigned long)g_wm_rate, g_wm_hp, g_wm_hp ? g_wm_hp - 0x79 : -99,
                g_wm_spk ? "pornit" : "oprit", g_wm_pga, -17.25 + 0.75 * g_wm_pga, g_wm_boost,
                (double)db(l), (double)db(r), (unsigned long)((wm_tx_w - wm_tx_rd()) % WM_RING_WORDS),
                (unsigned long)g_audio_under, (unsigned long)g_audio_drop, g_mic_loop ? "PORNITA" : "oprita");
}

// ============================================================
// RX PIO: cât timp RDY e sus, PIO-ul scoate cuvinte de 28 biți = meta (4) + eșantion cu semn (24),
// câte un nibble pe 7 fronturi ale ceasului RX. 1 receptor: meta 0 = I, apoi meta 1 = Q din aceeași
// pereche. Programul e al PA3GSB, identic cu rb-rx.pio din RagchewBerry; DMA ping-pong în RAM.
// ============================================================
static const uint16_t rx_iq_program_instr[] = {
  0x000f,  //  0: jmp    15
  0xbb42,  //  1: nop           side 1 [3]
  0x5004,  //  2: in pins, 4    side 0         <- meta
  0xbb42, 0x5004, 0xbb42, 0x5004, 0xbb42, 0x5004,
  0xbb42, 0x5004, 0xbb42, 0x5004, 0xbb42, 0x5004,   // 3..14: 6 nibble-uri = 24 biți eșantion
  0xba42,  // 15: nop           side 1 [2]
  0xb242,  // 16: nop           side 0 [2]
  0x00c1,  // 17: jmp pin, 1    (RDY=1 -> citește un cuvânt)
};
static const struct pio_program rx_iq_program = { .instructions = rx_iq_program_instr, .length = 18, .origin = -1 };

static PIO  rx_pio;
static uint rx_sm;
static int  rx_dma = -1;
// Un singur canal DMA, fără întreruperi: scrie la nesfârșit (TRANS_COUNT „endless", RP2350) într-un inel de
// 32 KB aliniat (ring pe adresa de scriere). Bucla principală citește adresa curentă de scriere a DMA-ului și
// procesează tot ce s-a adunat. Fără IRQ și fără re-armare: un handler pe DMA_IRQ_1 bloca firmware-ul (3 oct).
// 8192 cuvinte = 4096 perechi = 85 ms de rezervă (acoperă și scrierea setărilor în flash).
#define RX_RING_BITS  15
#define RX_RING_WORDS (1u << (RX_RING_BITS - 2))
static uint32_t rx_ring[RX_RING_WORDS] __attribute__((aligned(1u << RX_RING_BITS)));
static uint32_t rx_rd = 0;                           // următorul cuvânt de citit (index în inel)
static volatile uint32_t g_rx_overrun = 0;           // de câte ori DMA-ul a ajuns din urmă cititorul
static uint32_t g_rx_sync_err = 0;

static bool rx_pio_start() {
  // I2S-ul (pornit înainte) are deja un PIO; se ia primul bloc cu loc și un state machine liber
  PIO pios[] = { pio0, pio1, pio2 };
  int off = -1, sm = -1;
  for (PIO p : pios) {
    if (!pio_can_add_program(p, &rx_iq_program)) continue;
    sm = pio_claim_unused_sm(p, false);
    if (sm < 0) continue;
    rx_pio = p; off = pio_add_program(p, &rx_iq_program);
    break;
  }
  if (off < 0) return false;
  rx_sm = (uint)sm;

  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_wrap(&c, off, off + 17);
  sm_config_set_sideset(&c, 2, true, false);        // 1 bit + „opt"
  sm_config_set_sideset_pins(&c, PIN_RX_CLK);
  sm_config_set_in_pins(&c, PIN_RX_D0);
  sm_config_set_jmp_pin(&c, PIN_RX_RDY);
  sm_config_set_in_shift(&c, false, true, 28);      // stânga, autopush la 28 biți
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
  sm_config_set_clkdiv_int_frac(&c, 2, 0);          // ca la RagchewBerry; verificat 48 128 perechi/s, 0 erori

  pio_sm_set_pins_with_mask(rx_pio, rx_sm, 0, 1u << PIN_RX_CLK);   // ceasul rămâne jos la predare
  pio_gpio_init(rx_pio, PIN_RX_CLK);
  pio_sm_set_consecutive_pindirs(rx_pio, rx_sm, PIN_RX_CLK, 1, true);
  for (int p = PIN_RX_D0; p < PIN_RX_D0 + 4; p++) pio_gpio_init(rx_pio, p);
  pio_sm_set_consecutive_pindirs(rx_pio, rx_sm, PIN_RX_D0, 4, false);
  pio_sm_init(rx_pio, rx_sm, off, &c);

  if ((rx_dma = dma_claim_unused_channel(false)) < 0) return false;
  dma_channel_config dc = dma_channel_get_default_config(rx_dma);
  channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
  channel_config_set_read_increment(&dc, false);
  channel_config_set_write_increment(&dc, true);
  channel_config_set_ring(&dc, true, RX_RING_BITS);  // adresa de scriere se învârte în inel
  channel_config_set_dreq(&dc, pio_get_dreq(rx_pio, rx_sm, false));
  dma_channel_configure(rx_dma, &dc, rx_ring, &rx_pio->rxf[rx_sm], dma_encode_endless_transfer_count(), true);
  rx_rd = 0;
  pio_sm_set_enabled(rx_pio, rx_sm, true);
  return true;
}
static uint32_t cnt_iq = 0;

// Captură pentru spectru: nucleul 1 cere un bloc, nucleul 0 copiază 256 de perechi IQ consecutive.
#define FFT_N 256
static float cap_i[FFT_N], cap_q[FFT_N];
static volatile bool cap_req = false, cap_ready = false;
static int cap_n = 0;

// Flux IQ pe USB (comanda q1): perechi I,Q pe 24 de biți little-endian (6 octeți), 48 kHz, fără antet.
// Nucleul 0 le pune într-un tampon circular, loop() le scrie cât acceptă USB-ul. Tampon plin = se
// aruncă perechi întregi, deci fluxul rămâne aliniat la 6 octeți.
#define IQB_PAIRS 8192
static uint8_t iqb[IQB_PAIRS * 6];
static volatile uint32_t iqb_w = 0, iqb_r = 0;          // în octeți
static volatile bool g_stream = false;
static uint32_t g_stream_drop = 0;

static inline void iqb_push(int32_t I, int32_t Q) {
  uint32_t w = iqb_w, used = (w - iqb_r + sizeof(iqb)) % sizeof(iqb);
  if (used >= sizeof(iqb) - 6) { g_stream_drop++; return; }
  uint8_t *d = &iqb[w];
  d[0] = I; d[1] = I >> 8; d[2] = I >> 16; d[3] = Q; d[4] = Q >> 8; d[5] = Q >> 16;
  iqb_w = (w + 6) % sizeof(iqb);
}

static void iqb_pump() {
  while (iqb_r != iqb_w) {
    uint32_t r = iqb_r, w = iqb_w;
    uint32_t n = (w > r) ? w - r : sizeof(iqb) - r;          // până la capătul tamponului
    int room = Serial.availableForWrite();
    if (room <= 0) return;
    if (n > (uint32_t)room) n = room;
    n = Serial.write(&iqb[r], n);
    if (!n) return;
    iqb_r = (r + n) % sizeof(iqb);
  }
}

static void rx_pair(int32_t i, int32_t q) {
  if (RB_Q_NEG != g_iq_inv) q = -q;                  // convenția firmware-ului: +f = USB
  float out;
  float fi = i / 8388608.0f, fq = q / 8388608.0f;
  if (g_tx_on) tx_rx_probe(fi, fq);
  if (dsp_push(fi, fq, &out)) { audio_out_push(out); if (!g_mic_usb) usb_audio_push12k(g_dsp_raw); }
  if (cap_req) {
    cap_i[cap_n] = fi; cap_q[cap_n] = fq;
    if (++cap_n >= FFT_N) { cap_n = 0; __dmb(); cap_req = false; cap_ready = true; }
  }
  if (g_stream) iqb_push(i, q);                      // aceeași convenție ca spectrul
  cnt_iq++;
}

static void rx_poll() {
  static bool synced = false;
  static int32_t cur_i = 0;
  if (rx_dma < 0) return;
  // poziția de scriere a DMA-ului în inel (cuvântul la care va scrie următorul)
  uint32_t wr = ((uint32_t)dma_channel_hw_addr(rx_dma)->write_addr - (uint32_t)rx_ring) / 4 % RX_RING_WORDS;
  uint32_t avail = (wr - rx_rd) % RX_RING_WORDS;
  if (avail > RX_RING_WORDS * 3 / 4) g_rx_overrun++;   // cititorul a rămas periculos de în urmă
  if (avail > 1024) avail = 1024;                    // cel mult 512 perechi per apel: audio și USB nu așteaptă
  while (avail--) {
    uint32_t w = rx_ring[rx_rd];
    rx_rd = (rx_rd + 1) % RX_RING_WORDS;
    uint32_t meta = (w >> 24) & 0x0F;
    int32_t s = (int32_t)(w << 8) >> 8;              // extensie de semn pe 24 biți
    if (meta == 0) { cur_i = s; synced = true; }
    else if (meta == 1 && synced) { rx_pair(cur_i, s); synced = false; }
    else { g_rx_sync_err++; synced = false; }        // Q fără I înainte, sau meta necunoscut
  }
}

// ============================================================
// EMISIE (TX)
// Calea spre FPGA (din RagchewBerry/WP3DN: rb-tx.pio, main.c): FPGA-ul ridică TX_RDY (GP9) când vrea un
// eșantion; PIO-ul trimite atunci un cuvânt de 32 de biți, bit cu bit (MSB primul), date pe GP15, ceas pe GP14:
// I în biții 31:16, Q în 15:0, 48 kHz. MOX = bitul 0 din C0, pus pe TOATE comenzile SPI cât timp emite;
// drive = C0 0x12, nivel 0..15 în biții 31:28. Spectrul TX e inversat ca la RX: pentru USB se neagă Q.
// Surse: microfonul WM8960 (USB/LSB: filtru complex 300-2700 Hz la 12 kHz; AM), ton (TUNE, CW).
// Siguranță: doar în benzile de radioamator, timp maxim de emisie, oprire la eliberarea PTT-ului;
// după un refuz sau un timp maxim, PTT-ul trebuie eliberat complet înainte de o nouă emisie.
// PTT: BOOT ținut apăsat, butonul PTT de pe rețeaua RC (4,7 kΩ, fost FILTRU), CAT (TX;/RX;), USB (t1, tt).
// ============================================================
#define PIN_TX_CLK  14                               // BCM4,  pin 7
#define PIN_TX_DATA 15                               // BCM5,  pin 29
#define PIN_TX_RDY   9                               // BCM12, pin 32
static const char *TXS_NAME[] = { "MIC SSB", "MIC AM", "TON" };
static uint32_t g_tx_t0 = 0;
static float    g_tx_tone_hz = 1000;                 // TUNE: ton la +1 kHz (USB) / -1 kHz (LSB) față de acord
static float    g_tx_level = 0.5f;                   // amplitudinea tonului (0..0,95)
static volatile uint32_t g_tx_under = 0, g_tx_over = 0, g_tx_rate = 0;
static volatile float g_tx_mic_pk = 0;
static volatile float g_tx_probe_pos = -200, g_tx_probe_neg = -200;   // RX în timpul TX: puterea la +ton / -ton

// ---------- PIO + DMA (inel fără IRQ, ca la RX) ----------
static uint16_t tx_prog_instr[9];
static PIO  tx_pio; static uint tx_sm; static int tx_off = -1, tx_dma = -1;
static pio_sm_config tx_cfg;
// 32 KB = MAXIMUL inelului DMA (câmpul RING_SIZE are 4 biți: 2^15). Cu 16, inelul se dezactiva, DMA-ul citea liniar
// până la capătul RAM-ului și se oprea după 0,4-2 s (22 oct: FPGA-ul a primit gunoi, apoi 0 eșantioane/s).
#define TX_RING_BITS  15                             // 32 KB = 8192 eșantioane = 170 ms
#define TX_RING_WORDS (1u << (TX_RING_BITS - 2))
static_assert(TX_RING_BITS <= 15 && WM_RING_BITS <= 15 && RX_RING_BITS <= 15, "inelul DMA are maxim 2^15 octeti");
static uint32_t tx_ring[TX_RING_WORDS] __attribute__((aligned(1u << TX_RING_BITS)));
static uint32_t tx_w = 0;
// La pornire FPGA-ul își umple FIFO-ul TX dintr-o dată (mai repede de 48 kHz): cu 2048 de rezervă se goleau
// 1100-2100 eșantioane la fiecare apăsare (22 oct) -> se pornește cu 8192 (170 ms de liniște, apoi microfonul).
static const uint32_t TX_LEAD_MIN = 1024, TX_LEAD = 5000, TX_LEAD_MAX = 6000;

static uint32_t tx_rd() { return ((uint32_t)dma_hw->ch[tx_dma].read_addr - (uint32_t)tx_ring) / 4 % TX_RING_WORDS; }

static bool tx_hw_init() {
  // programul lui PA3GSB/WP3DN (rb-tx.pio), construit la rulare. side-set 1 bit + opt = ceasul (GP14)
  uint16_t *p = tx_prog_instr;
  p[0] = pio_encode_jmp_pin(2)   | pio_encode_sideset_opt(1, 1) | pio_encode_delay(2);   // idle: RDY=1 -> start
  p[1] = pio_encode_jmp(0)       | pio_encode_sideset_opt(1, 0) | pio_encode_delay(2);
  p[2] = pio_encode_nop()        | pio_encode_sideset_opt(1, 0) | pio_encode_delay(2);   // start: ceas jos
  p[3] = pio_encode_set(pio_x, 31);
  p[4] = pio_encode_pull(false, true);                                                    // pull block
  p[5] = pio_encode_out(pio_pins, 1);                                                     // bitloop
  p[6] = pio_encode_nop()        | pio_encode_sideset_opt(1, 1) | pio_encode_delay(2);   // ceas sus: FPGA citește
  p[7] = pio_encode_jmp_x_dec(5) | pio_encode_sideset_opt(1, 0) | pio_encode_delay(2);
  p[8] = pio_encode_jmp(0);
  static const struct pio_program prog = { .instructions = tx_prog_instr, .length = 9, .origin = -1 };
  PIO pios[] = { pio0, pio1, pio2 };
  int sm = -1;
  for (PIO pp : pios) {
    if (!pio_can_add_program(pp, &prog)) continue;
    if ((sm = pio_claim_unused_sm(pp, false)) < 0) continue;
    tx_pio = pp; tx_off = pio_add_program(pp, &prog); break;
  }
  if (tx_off < 0) return false;
  tx_sm = (uint)sm;
  tx_cfg = pio_get_default_sm_config();
  sm_config_set_wrap(&tx_cfg, tx_off, tx_off + 8);
  sm_config_set_sideset(&tx_cfg, 2, true, false);
  sm_config_set_sideset_pins(&tx_cfg, PIN_TX_CLK);
  sm_config_set_out_pins(&tx_cfg, PIN_TX_DATA, 1);
  sm_config_set_out_shift(&tx_cfg, false, false, 32);
  sm_config_set_jmp_pin(&tx_cfg, PIN_TX_RDY);
  sm_config_set_clkdiv(&tx_cfg, 12.0f);              // ca la RagchewBerry: ~65 000 cuvinte/s maxim > 48 000
  gpio_init(PIN_TX_CLK);  gpio_set_dir(PIN_TX_CLK, GPIO_OUT);  gpio_put(PIN_TX_CLK, 0);
  gpio_init(PIN_TX_DATA); gpio_set_dir(PIN_TX_DATA, GPIO_OUT); gpio_put(PIN_TX_DATA, 0);
  gpio_init(PIN_TX_RDY);  gpio_set_dir(PIN_TX_RDY, GPIO_IN);   gpio_pull_up(PIN_TX_RDY);
  if ((tx_dma = dma_claim_unused_channel(false)) < 0) return false;
  return true;
}

static void tx_hw_start() {
  memset(tx_ring, 0, sizeof tx_ring);
  tx_w = TX_LEAD;
  pio_sm_set_enabled(tx_pio, tx_sm, false);
  pio_sm_set_pins_with_mask(tx_pio, tx_sm, 0, (1u << PIN_TX_CLK) | (1u << PIN_TX_DATA));
  pio_gpio_init(tx_pio, PIN_TX_CLK); pio_gpio_init(tx_pio, PIN_TX_DATA);
  pio_sm_set_consecutive_pindirs(tx_pio, tx_sm, PIN_TX_CLK, 2, true);   // GP14, GP15
  pio_sm_init(tx_pio, tx_sm, tx_off, &tx_cfg);
  pio_sm_clear_fifos(tx_pio, tx_sm);
  dma_channel_config c = dma_channel_get_default_config(tx_dma);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, false);
  channel_config_set_ring(&c, false, TX_RING_BITS);
  channel_config_set_dreq(&c, pio_get_dreq(tx_pio, tx_sm, true));
  dma_channel_configure(tx_dma, &c, &tx_pio->txf[tx_sm], tx_ring, dma_encode_endless_transfer_count(), true);
  pio_sm_set_enabled(tx_pio, tx_sm, true);
}

static void tx_hw_stop() {
  dma_channel_abort(tx_dma);
  pio_sm_set_enabled(tx_pio, tx_sm, false);
  pio_sm_clear_fifos(tx_pio, tx_sm);
  pio_sm_set_pins_with_mask(tx_pio, tx_sm, 0, (1u << PIN_TX_CLK) | (1u << PIN_TX_DATA));
}

// un eșantion IQ în convenția firmware-ului (+f = USB) -> inel
static void tx_put(float i, float q) {
  if (g_tx_qneg) q = -q;                            // Radioberry: spectrul TX e inversat
  if (i > 0.95f) i = 0.95f; if (i < -0.95f) i = -0.95f;
  if (q > 0.95f) q = 0.95f; if (q < -0.95f) q = -0.95f;
  uint32_t ahead = (tx_w - tx_rd()) % TX_RING_WORDS;
  if (ahead > TX_LEAD_MAX) { g_tx_over++; return; } // microfonul (ceasul codecului) e mai rapid: aruncă
  int16_t si = (int16_t)lrintf(i * 32767.0f), sq = (int16_t)lrintf(q * 32767.0f);
  tx_ring[tx_w] = ((uint32_t)(uint16_t)si << 16) | (uint16_t)sq;
  tx_w = (tx_w + 1) % TX_RING_WORDS;
}

// ---------- DSP microfon: 48 kHz -> /4 -> 12 kHz (procesare) -> ×4 -> 48 kHz IQ ----------
#define NTXD 41                                      // decimare: trece-jos 3 kHz la 48 kHz
#define NTXB 127                                     // filtru complex 300-2700 Hz la 12 kHz (o singură bandă laterală)
#define NTXI 48                                      // interpolare ×4: trece-jos 3 kHz la 48 kHz (12 coeficienți/fază)
static float h_txd[NTXD], h_txb_r[NTXB], h_txb_i[NTXB], h_txi[NTXI];
static float txd_buf[2 * NTXD]; static int txd_pos = 0, txd_ph = 0;
static float txb_buf[2 * NTXB]; static int txb_pos = 0;
static float txi_i[2 * (NTXI / 4)], txi_q[2 * (NTXI / 4)]; static int txi_pos = 0;
static float tx_env = 0, tx_dc = 0, tx_dcy = 0;

static void tx_dsp_init() {
  design_lowpass(h_txd, NTXD, 3000, 48000);
  float lp[NTXB];
  design_lowpass(lp, NTXB, 1200, 12000);             // ±1200 Hz în jurul lui 1500 Hz = 300..2700 Hz
  const float w0 = 2 * (float)M_PI * 1500 / 12000;
  for (int k = 0; k < NTXB; k++) {                   // ×2: ieșirea analitică păstrează amplitudinea intrării reale
    float a = w0 * (k - (NTXB - 1) / 2.0f);
    h_txb_r[k] = 2 * lp[k] * cosf(a); h_txb_i[k] = 2 * lp[k] * sinf(a);
  }
  design_lowpass(h_txi, NTXI, 3000, 48000);
  for (int k = 0; k < NTXI; k++) h_txi[k] *= 4;      // câștigul pierdut la inserarea zerourilor
}
static void tx_dsp_reset() {
  memset(txd_buf, 0, sizeof txd_buf); memset(txb_buf, 0, sizeof txb_buf);
  memset(txi_i, 0, sizeof txi_i); memset(txi_q, 0, sizeof txi_q);
  txd_pos = txd_ph = txb_pos = txi_pos = 0; tx_env = 0; tx_dc = tx_dcy = 0;
}
// convoluție cu bufferul „cel mai nou primul": buf[pos + k] = x[n - k]
static inline void ring_push(float *buf, int n, int &pos, float x) { pos = (pos - 1 + n) % n; buf[pos] = buf[pos + n] = x; }

static void tx_emit12(float i, float q) {            // 1 eșantion la 12 kHz -> 4 la 48 kHz
  const int M = NTXI / 4;
  ring_push(txi_i, M, txi_pos, i); txi_q[txi_pos] = txi_q[txi_pos + M] = q;
  for (int ph = 0; ph < 4; ph++) {
    float yi = 0, yq = 0;
    for (int m = 0; m < M; m++) { float h = h_txi[ph + 4 * m]; yi += h * txi_i[txi_pos + m]; yq += h * txi_q[txi_pos + m]; }
    tx_put(yi, yq);
  }
}

static void tx_mic48(int16_t s) {
  float x = s / 32768.0f;
  ring_push(txd_buf, NTXD, txd_pos, x);
  if (++txd_ph < 4) return;
  txd_ph = 0;
  float a = 0;
  for (int k = 0; k < NTXD; k++) a += h_txd[k] * txd_buf[txd_pos + k];
  // 12 kHz: fără DC, câștig, limitator (atac instant, revenire ~0,2 s) -> vârfuri ≤ 0,3
  tx_dcy = a - tx_dc + 0.995f * tx_dcy; tx_dc = a; a = tx_dcy;
  a *= powf(10.0f, g_tx_mic_db / 20.0f);
  float m = fabsf(a);
  if (m > g_tx_mic_pk) g_tx_mic_pk = m;
  tx_env = m > tx_env ? m : tx_env * 0.9995f;
  if (tx_env > 0.3f) a *= 0.3f / tx_env;
  if (g_tx_src == TXS_MIC_AM) {
    tx_emit12(0.45f * (1 + 2.8f * a), 0);            // purtătoare 0,45, modulație ~85 % la vârf
    return;
  }
  ring_push(txb_buf, NTXB, txb_pos, a);
  float yi = 0, yq = 0;
  for (int k = 0; k < NTXB; k++) { float v = txb_buf[txb_pos + k]; yi += h_txb_r[k] * v; yq += h_txb_i[k] * v; }
  yi *= 2.8f; yq *= 2.8f;                            // vârf ~0,85
  if (g_mode == MODE_LSB) yq = -yq;                  // conjugat = banda laterală inferioară
  tx_emit12(yi, yq);
}

// RX în timpul TX (duplex): cât din propriul semnal apare la +ton și la -ton. Confirmă că iese RF și
// că banda laterală e cea corectă (USB: tonul trebuie să fie la +ton).
static float tx_ph_pos = 0; static float tx_acc_pr = 0, tx_acc_pi = 0, tx_acc_nr = 0, tx_acc_ni = 0; static int tx_acc_n = 0;
static void tx_rx_probe(float i, float q) {
  float w = 2 * (float)M_PI * g_tx_tone_hz / 48000.0f;
  tx_ph_pos += w; if (tx_ph_pos > 2 * (float)M_PI) tx_ph_pos -= 2 * (float)M_PI;
  float c = cosf(tx_ph_pos), s = sinf(tx_ph_pos);
  tx_acc_pr += i * c + q * s; tx_acc_pi += q * c - i * s;     // (i+jq)·e^{-jwt}
  tx_acc_nr += i * c - q * s; tx_acc_ni += q * c + i * s;     // (i+jq)·e^{+jwt}
  if (++tx_acc_n >= 4800) {
    float n = (float)tx_acc_n;
    g_tx_probe_pos = 10 * log10f((tx_acc_pr * tx_acc_pr + tx_acc_pi * tx_acc_pi) / (n * n) + 1e-20f);
    g_tx_probe_neg = 10 * log10f((tx_acc_nr * tx_acc_nr + tx_acc_ni * tx_acc_ni) / (n * n) + 1e-20f);
    tx_acc_pr = tx_acc_pi = tx_acc_nr = tx_acc_ni = 0; tx_acc_n = 0;
  }
}

static void tx_say(const char *msg) { if (!g_stream && !cat_active()) Serial.printf("TX: %s\n", msg); }

static void tx_start() {
  const char *why = nullptr;
  int src = TXS_TONE;
  if (g_tune_req) src = TXS_TONE;
  else if (g_mode == MODE_USB || g_mode == MODE_LSB) src = TXS_MIC_SSB;
  else if (g_mode == MODE_AM || g_mode == MODE_SAM) src = TXS_MIC_AM;
  else if (g_mode == MODE_CW) src = TXS_TONE;
  else why = "FM nu e implementat la emisie";
  if (g_fpga != 1 || tx_dma < 0) why = "FPGA / PIO TX indisponibil";
  else if (g_wm != 1) why = "fara WM8960 (GP14 e al PCM5102A, nu exista microfon)";
  else if (!tx_band_ok(g_freq)) why = "frecventa in afara HF (1,5-30 MHz)";
  if (why) { tx_say(why); g_tx_lock = true; return; }

  g_tx_src = src;
  tx_dsp_reset(); g_tx_under = g_tx_over = 0; g_tx_mic_pk = 0;
  g_tx_probe_pos = g_tx_probe_neg = -200; tx_acc_n = 0; tx_acc_pr = tx_acc_pi = tx_acc_nr = tx_acc_ni = 0;
  if (src == TXS_TONE) g_tx_tone_hz = g_mode == MODE_CW ? 700 : 1000;
  rb_cmd(0x02, rb_freq());                           // frecvența TX = acordul
  rb_cmd(0x12, (uint32_t)(g_tx_drive & 0x0F) << 28); // drive
  tx_hw_start();                                     // fluxul pornește cu MOX încă oprit
  g_tx_on = true; g_tx_t0 = millis();
  rb_cmd(0x00, RB_REG0);                             // MOX (bitul 0 se adaugă în rb_cmd)
  char m[96];
  snprintf(m, sizeof m, "PORNIT %lu Hz %s, sursa %s, drive %d/15", (unsigned long)g_freq, MODE_NAME[g_mode],
           TXS_NAME[src], g_tx_drive);
  tx_say(m);
}

static void tx_stop(const char *why) {
  g_tx_on = false;
  rb_cmd(0x00, RB_REG0);                             // întâi MOX jos (RF oprit), apoi fluxul
  tx_hw_stop();
  char m[96];
  snprintf(m, sizeof m, "OPRIT (%s) dupa %.1f s, sub %lu peste %lu", why, (millis() - g_tx_t0) / 1000.0,
           (unsigned long)g_tx_under, (unsigned long)g_tx_over);
  tx_say(m);
}

// apelat din loop(): PTT, ton, rezerva inelului (fără limită de timp — la cererea userului, 22 oct)
static void tx_poll() {
  bool req = g_ptt_boot || g_ptt_key || g_ptt_cat || g_ptt_usb || g_tune_req;
  if (g_tx_lock) { if (!req) g_tx_lock = false; req = false; }
  if (req && !g_tx_on) tx_start();
  else if (!req && g_tx_on) tx_stop("PTT eliberat");
  if (!g_tx_on) return;
  if (g_tx_drive_dirty) { g_tx_drive_dirty = false; rb_cmd(0x12, (uint32_t)(g_tx_drive & 0x0F) << 28); }
  uint32_t rd = tx_rd();
  static uint32_t last_rd = 0, cnt = 0, t_rate = 0;
  cnt += (rd - last_rd) % TX_RING_WORDS; last_rd = rd;
  if (millis() - t_rate >= 1000) { t_rate = millis(); g_tx_rate = cnt; cnt = 0; }
  uint32_t ahead = (tx_w - rd) % TX_RING_WORDS;
  if (ahead > TX_RING_WORDS - 512) { tx_w = (rd + TX_LEAD) % TX_RING_WORDS; ahead = TX_LEAD; g_tx_under++; }   // DMA-ul ne-a depășit
  if (g_tx_src == TXS_TONE) {                        // tonul se generează aici, cât e loc
    static float ph = 0;
    float w = 2 * (float)M_PI * g_tx_tone_hz / 48000.0f * (g_mode == MODE_LSB ? -1 : 1);
    while (ahead < TX_LEAD) {
      ph += w; if (ph > (float)M_PI) ph -= 2 * (float)M_PI; if (ph < -(float)M_PI) ph += 2 * (float)M_PI;
      tx_put(g_tx_level * cosf(ph), g_tx_level * sinf(ph)); ahead++;
    }
  } else {
    while (ahead < TX_LEAD_MIN) { tx_put(0, 0); ahead++; g_tx_under++; }   // microfonul a rămas în urmă
  }
}

static void tx_status() {
  if (g_tx_on)
    Serial.printf("TX EMITE %.1f s | %lu Hz %s | sursa %s | drive %d/15 | cuvinte %lu/s | sub %lu peste %lu | "
                  "mic varf %.1f dBFS (castig %+d dB) | RX propriu: +%.0f Hz %.1f dB, -%.0f Hz %.1f dB | Q %s\n",
                  (millis() - g_tx_t0) / 1000.0, (unsigned long)g_freq, MODE_NAME[g_mode], TXS_NAME[g_tx_src],
                  g_tx_drive, (unsigned long)g_tx_rate, (unsigned long)g_tx_under, (unsigned long)g_tx_over,
                  20 * log10f(g_tx_mic_pk + 1e-9f), g_tx_mic_db, (double)g_tx_tone_hz, (double)g_tx_probe_pos,
                  (double)g_tx_tone_hz, (double)g_tx_probe_neg, g_tx_qneg ? "negat" : "normal");
  else
    Serial.printf("TX oprit | drive %d/15 | mic %+d dB | frecventa %s | PTT: boot %d tasta %d cat %d usb %d tune %d%s\n",
                  g_tx_drive, g_tx_mic_db, tx_band_ok(g_freq) ? "OK" : "IN AFARA HF",
                  g_ptt_boot, g_ptt_key, g_ptt_cat, g_ptt_usb, g_tune_req, g_tx_lock ? " | BLOCAT pana la eliberarea PTT" : "");
  g_tx_mic_pk = 0;
}

// ============================================================
// Nucleul 0
// ============================================================
static void radio_start() {
  gpio_init(PIN_RX_CLK); gpio_set_dir(PIN_RX_CLK, GPIO_OUT); gpio_put(PIN_RX_CLK, 0);  // jos ÎNAINTE de încărcare
  gpio_init(PIN_RX_RDY); gpio_set_dir(PIN_RX_RDY, GPIO_IN);
  for (int k = 0; k < 4; k++) { gpio_init(PIN_RX_D0 + k); gpio_set_dir(PIN_RX_D0 + k, GPIO_IN); }
  for (uint8_t p : FREED_GP) {
    if (g_wm == 1 && (p == PIN_I2S_DOUT || p == PIN_I2S_DIN)) continue;   // sunt ale PIO-ului audio
    gpio_init(p); gpio_set_dir(p, GPIO_IN);
  }
  gpio_init(PIN_FPGA_NCONFIG);  gpio_set_dir(PIN_FPGA_NCONFIG, GPIO_OUT); gpio_put(PIN_FPGA_NCONFIG, 1);
  gpio_init(PIN_FPGA_DATA0);    gpio_set_dir(PIN_FPGA_DATA0, GPIO_OUT);
  gpio_init(PIN_FPGA_DCLK);     gpio_set_dir(PIN_FPGA_DCLK, GPIO_OUT);
  gpio_init(PIN_FPGA_NSTATUS);  gpio_set_dir(PIN_FPGA_NSTATUS, GPIO_IN);  gpio_pull_up(PIN_FPGA_NSTATUS);
  gpio_init(PIN_FPGA_CONFDONE); gpio_set_dir(PIN_FPGA_CONFDONE, GPIO_IN); gpio_pull_up(PIN_FPGA_CONFDONE);

  spi_init(spi1, 4000000);
  spi_set_format(spi1, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
  gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SPI_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SPI_MISO, GPIO_FUNC_SPI);
  gpio_init(PIN_SPI_CE0); gpio_set_dir(PIN_SPI_CE0, GPIO_OUT); gpio_put(PIN_SPI_CE0, 1);
  gpio_init(PIN_SPI_CE1); gpio_set_dir(PIN_SPI_CE1, GPIO_OUT); gpio_put(PIN_SPI_CE1, 1);

  g_fpga = fpga_load();
  if (g_fpga == 1) {
    delay(500);                                      // ieșirea din reset înainte de primul cadru SPI (ca RagchewBerry)
    rb_cmd(0x00, RB_REG0); rb_cmd(0x02, rb_freq()); rb_cmd(0x04, rb_freq()); rb_cmd(0x14, rb_gain_word());
    g_gain_sent = g_gain_db;
    if (!rx_pio_start()) g_fpga = -3;               // fără PIO/DMA liber: niciun eșantion
    tx_dsp_init();
    if (!tx_hw_init()) tx_dma = -1;                  // fără TX, recepția merge mai departe
  }
  g_ui_ready = true;
}

static void settings_load();

void setup() {
  usb_audio_begin();
  settings_load();
  Serial.begin(115200);
  design_lowpass(h_dec, NDEC, 4500, FS_IN);
  g_mode_dirty = true;                               // filtrele (și decimarea pt. FM) se fac în loop()
  audio_out_begin();
  radio_start();
}

// Baleiaj de diagnostic: pentru fiecare frecvență așteaptă 2 măsurători de S (2 × 100 ms) și le notează.
static volatile bool scan_on = false;
static uint32_t scan_f, scan_stop, scan_step, scan_t; static int scan_saved_mode;
static float scan_min = 0, scan_max = -200; static uint32_t scan_fmax = 0;

static void scan_start(const char *a) {
  long st = 5800, sp = 10000, ps = 10;
  sscanf(a, "%ld,%ld,%ld", &st, &sp, &ps);
  scan_f = st * 1000; scan_stop = sp * 1000; scan_step = ps * 1000;
  scan_saved_mode = g_mode; g_mode = MODE_AM; g_mode_dirty = true;
  g_freq = scan_f; g_freq_dirty = true;
  scan_min = 0; scan_max = -200; scan_t = millis(); scan_on = true;
  Serial.printf("baleiaj %ld..%ld kHz pas %ld kHz, castig %+d dB (AM +-4 kHz)\n", st, sp, ps, g_gain_db);
}

static void scan_tick() {
  if (!scan_on || millis() - scan_t < 220) return;
  float sdb = g_smeter_db;
  if (sdb < scan_min) scan_min = sdb;
  if (sdb > scan_max) { scan_max = sdb; scan_fmax = scan_f; }
  Serial.printf("%7.3f MHz  %6.1f dBFS\n", scan_f / 1e6, sdb);
  scan_f += scan_step;
  if (scan_f > scan_stop) {
    scan_on = false;
    Serial.printf("gata: minim %.1f dBFS, maxim %.1f dBFS la %.3f MHz (diferenta %.1f dB)\n",
                  scan_min, scan_max, scan_fmax / 1e6, scan_max - scan_min);
    g_mode = scan_saved_mode; g_mode_dirty = true;
    return;
  }
  g_freq = scan_f; g_freq_dirty = true; scan_t = millis();
}

// 'i': ce se întâmplă efectiv pe pinii audio (nivelul citit din pad, deci și dacă sunt în scurt)
static void pin_activity() {
  const int P[3] = { PIN_I2S_BCK, PIN_I2S_LRCK, PIN_I2S_DATA };
  const char *N[3] = { "BCK  (hdr 8)", "LCK  (hdr 10)", "DIN  (hdr 7)" };
  uint32_t hi[3] = { 0 }, tr[3] = { 0 }; int last[3];
  for (int k = 0; k < 3; k++) last[k] = gpio_get(P[k]);
  const int NS = 100000;
  uint32_t t0 = micros();
  for (int n = 0; n < NS; n++) {
    uint32_t a = gpio_get_all();
    for (int k = 0; k < 3; k++) { int v = (a >> P[k]) & 1; hi[k] += v; if (v != last[k]) { tr[k]++; last[k] = v; } }
  }
  uint32_t dt = micros() - t0;
  Serial.printf("activitate pini audio (%d citiri in %lu us):\n", NS, (unsigned long)dt);
  for (int k = 0; k < 3; k++)
    Serial.printf("  %s GP%d: sus %3lu%%  comutari %lu  (~%lu kHz)\n", N[k], P[k],
                  (unsigned long)(hi[k] * 100 / NS), (unsigned long)tr[k], (unsigned long)(tr[k] / 2 * 1000 / (dt ? dt : 1)));
}


// ============================================================
// CAT: emulare Kenwood TS-2000 (subset), pe același port ca restul comenzilor.
// Comenzile Kenwood: 2 litere mari + parametri, terminate cu ';'. Interogare = fără parametri.
// Doar recepție: TX/RX se acceptă și se ignoră (PTT în program: „None”/VOX).
// ============================================================
static volatile uint32_t g_cat_t = 0;                // ultima comandă CAT (în timpul CAT nu se scrie text)
static bool cat_active() { return g_cat_t && millis() - g_cat_t < 15000; }

static int mode_to_kw(int m) {                      // Kenwood: 1 LSB, 2 USB, 3 CW, 4 FM, 5 AM
  switch (m) { case MODE_LSB: return 1; case MODE_USB: return 2; case MODE_CW: return 3; case MODE_FM: return 4; default: return 5; }
}
static int kw_to_mode(int k) {
  switch (k) { case 1: return MODE_LSB; case 2: return MODE_USB; case 3: case 7: return MODE_CW;
               case 4: return MODE_FM; case 5: return MODE_AM; default: return -1; }
}

static void cat_cmd(const char *c) {
  g_cat_t = millis(); if (!g_cat_t) g_cat_t = 1;
  char a = c[0], b = c[1]; const char *p = c + 2; size_t n = strlen(p);
  char r[48];
  if ((a == 'F' && (b == 'A' || b == 'B'))) {       // VFO A/B: ambele = frecvența receptorului
    if (n) { uint32_t f = strtoul(p, nullptr, 10); if (f >= 10000 && f <= 30000000) { g_freq = f; g_freq_dirty = true; } }
    else { snprintf(r, sizeof(r), "F%c%011lu;", b, (unsigned long)g_freq); Serial.print(r); }
  } else if (a == 'I' && b == 'F' && !n) {          // stare: 38 de caractere, formatul TS-2000/TS-480
    snprintf(r, sizeof(r), "IF%011lu     +0000000000%d000000 ;", (unsigned long)g_freq, mode_to_kw(g_mode));
    Serial.print(r);
  } else if (a == 'M' && b == 'D') {
    if (n) { int m = kw_to_mode(atoi(p)); if (m >= 0 && m != g_mode) { g_mode = m; g_mode_dirty = true; } }
    else { snprintf(r, sizeof(r), "MD%d;", mode_to_kw(g_mode)); Serial.print(r); }
  } else if (a == 'I' && b == 'D') Serial.print("ID019;");
  else if (a == 'P' && b == 'S') { if (!n) Serial.print("PS1;"); }
  else if (a == 'A' && b == 'I') { if (!n) Serial.print("AI0;"); }
  else if ((a == 'F' && (b == 'R' || b == 'T')) && !n) { snprintf(r, sizeof(r), "F%c0;", b); Serial.print(r); }
  else if (a == 'F' && b == 'V' && !n) Serial.print("FV1.00;");
  else if (a == 'K' && b == 'S' && !n) Serial.print("KS020;");   // viteza manipulatorului: Hamlib o cere la pornire
  else if (a == 'S' && b == 'M') {                  // S-metru 0..30 (TS-2000), din dBFS: zgomot ~-110 -> 0
    int v = (int)((g_smeter_db + 110) / 2); if (v < 0) v = 0; if (v > 30) v = 30;
    snprintf(r, sizeof(r), "SM0%04d;", v); Serial.print(r);
  } else if (a == 'T' && b == 'X') g_ptt_cat = true;   // TX; TX0; TX1: emisie (sursa = microfonul / tonul modului)
  else if (a == 'R' && b == 'X') g_ptt_cat = false;
  else if (!n) Serial.print("?;");                  // interogare necunoscută
  // setare necunoscută: Kenwood nu răspunde nimic
}

// Comanda u: comandă FPGA-ul GP16 / GP23 (liniile eliberate, vizate pt. WM8960 DACDAT/ADCDAT)?
// Pinul e intrare; cu pull-up apoi pull-down: dacă nivelul urmează rezistența, nu-l ține nimeni.
// Plus comutări în 100 000 de citiri cu fluxul RX pornit (un FPGA care le comandă ar comuta sau ar sta fix).
static void freed_pin_test() {
  const int P[2] = { 16, 23 };
  const char *N[2] = { "GP16 (hdr 36)", "GP23 (hdr 16)" };
  for (int k = 0; k < 2; k++) {
    int p = P[k];
    gpio_set_dir(p, GPIO_IN);
    gpio_pull_up(p);   delayMicroseconds(200); int up = gpio_get(p);
    uint32_t tr = 0; int last = up;
    for (int n = 0; n < 100000; n++) { int v = gpio_get(p); if (v != last) { tr++; last = v; } }
    // erata RP2350 E9: cu intrarea activă, pull-down-ul nu coboară un pin rămas sus → descărcare cu IE oprit
    gpio_set_input_enabled(p, false); gpio_pull_down(p); delayMicroseconds(500);
    gpio_set_input_enabled(p, true); int dn = gpio_get(p);
    uint32_t tr2 = 0; last = dn;
    for (int n = 0; n < 100000; n++) { int v = gpio_get(p); if (v != last) { tr2++; last = v; } }
    gpio_disable_pulls(p);
    // revenire după 2 µs ținut jos (2 mA): ieșire FPGA = sus imediat; pull-up slab = sute de ns (RC cu ~10 pF)
    gpio_set_drive_strength(p, GPIO_DRIVE_STRENGTH_2MA);
    uint32_t loops[3];
    for (int r = 0; r < 3; r++) {
      uint32_t msk = 1u << p, cnt = 0;
      uint32_t irq = save_and_disable_interrupts();
      sio_hw->gpio_clr = msk; sio_hw->gpio_oe_set = msk;
      busy_wait_us_32(2);
      sio_hw->gpio_oe_clr = msk;
      while (!(sio_hw->gpio_in & msk) && cnt < 100000) cnt++;
      restore_interrupts(irq);
      loops[r] = cnt;
    }
    Serial.printf("  %s: revenire dupa tras jos = %lu/%lu/%lu bucle (~%lu ns)\n", N[k],
                  (unsigned long)loops[0], (unsigned long)loops[1], (unsigned long)loops[2],
                  (unsigned long)(loops[1] * 4 * 1000000000ull / clock_get_hz(clk_sys)));
    Serial.printf("  %s: pull-up -> %d, pull-down -> %d, comutari %lu/%lu  => %s\n", N[k], up, dn,
                  (unsigned long)tr, (unsigned long)tr2,
                  (up == 1 && dn == 0 && tr == 0 && tr2 == 0) ? "LIBER (nu-l comanda FPGA)" :
                  (tr == 0 && tr2 == 0 && loops[1] >= 2) ? "pull-up slab in FPGA, nu iesire => folosibil" :
                  "COMANDAT / suspect - NU folosi");
  }
}

static void handle_cmd(const char *s) {
  if (s[0] == 'i') { pin_activity(); return; }
  if (s[0] == 't') {                                  // TX: t | t1 t0 | tt | td<0-15> | tg<dB> | tq | tb
    long a = atol(s + 2);
    switch (s[1]) {
      case '1': g_ptt_usb = true; break;
      case '0': g_ptt_usb = g_ptt_cat = g_tune_req = false; break;
      case 't': g_tune_req = !g_tune_req; break;
      case 'd': g_tx_drive = constrain(a, 0, 15); g_tx_drive_dirty = true; break;
      case 'g': g_tx_mic_db = constrain(a, -20, 30); break;
      case 'q': g_tx_qneg = !g_tx_qneg; break;
      case 'b': g_boot_ptt = a != 0; break;           // tb1 / tb0: BOOT ca PTT
    }
    if (s[1] == '1' || s[1] == 't') {                 // lasă bucla să pornească emisia, apoi arată starea
      uint32_t t0 = millis();
      while (millis() - t0 < 300) { if (g_fpga == 1) rx_poll(); audio_out_pump(); tx_poll(); }
    }
    tx_status();
    return;
  }
  if (s[0] == 'o') { g_oled_test_until = millis() + 15000; Serial.println("OLED: model de test 15 s"); return; }
  if (s[0] == 'u') { if (g_wm == 1) Serial.println("u: GP16/GP23 sunt ale WM8960 acum"); else freed_pin_test(); return; }
  if (s[0] == 'a') {                                  // WM8960: a | ah<reg> | as<reg> | am<pga> | ab<boost> | al1/al0
    long a = atol(s + 2);
    if (g_wm == 1) switch (s[1]) {
      case 'h': g_wm_hp_req = a; break;
      case 's': g_wm_spk_req = a; break;
      case 'm': g_wm_pga = constrain(a, 0, 63); g_wm_mic_req = 1; break;
      case 'b': g_wm_boost = constrain(a, 0, 3); g_wm_mic_req = 1; break;
      case 'l': g_mic_loop = a != 0; break;
      case 'u': g_mic_usb = constrain(a, 0, 2); break;
      case 'X': g_wm_stop_req = true; break;          // test: codec resetat = fără ceasuri pe fire
      case 'R': {                                     // reinițializare completă (registre pierdute pe I2C)
        g_wm_retry = 1; uint32_t t0 = millis();
        while (g_wm_retry != 2 && millis() - t0 < 3000) { if (g_fpga == 1) rx_poll(); audio_out_pump(); }
        Serial.printf("WM8960 reinit: %s\n", g_wm_init == 1 ? "OK" : "ESUAT");
        g_wm_retry = 0;
        wm_pio_resync();
        break;
      }
    }
    if (s[1] == 'd') {                                // cine ține SDA jos? (rulează pe nucleul 0)
      uint32_t b0 = g_core1_beat; delay(500); uint32_t b1 = g_core1_beat;
      int sda_i2c = gpio_get(PIN_OLED_SDA), scl_i2c = gpio_get(PIN_OLED_SCL);
      gpio_set_function(PIN_OLED_SDA, GPIO_FUNC_SIO); gpio_set_dir(PIN_OLED_SDA, GPIO_IN);
      gpio_set_function(PIN_OLED_SCL, GPIO_FUNC_SIO); gpio_set_dir(PIN_OLED_SCL, GPIO_IN);
      delayMicroseconds(100);
      int sda_free = gpio_get(PIN_OLED_SDA), scl_free = gpio_get(PIN_OLED_SCL);
      gpio_set_function(PIN_OLED_SDA, GPIO_FUNC_I2C); gpio_set_function(PIN_OLED_SCL, GPIO_FUNC_I2C);
      Serial.printf("nucleul 1: %lu treceri in 0,5 s (%s) | cu I2C: SDA %d SCL %d | pinii eliberati de RP2350: SDA %d SCL %d => %s\n",
                    (unsigned long)(b1 - b0), b1 != b0 ? "ruleaza" : "BLOCAT", sda_i2c, scl_i2c, sda_free, scl_free,
                    sda_free ? (sda_i2c ? "magistrala libera" : "SDA era tinut de RP2350 (I2C blocat)") : "SDA tinut de un cip EXTERN / scurt la masa");
      return;
    }
    if (s[1] == 'i') {                                // rezultatul scanării îl scrie nucleul 1
      // nivelul liniilor, citit de aici: dacă nucleul 1 e blocat pe I2C, măcar vedem cine ține magistrala jos
      int sda = 0, scl = 0;
      for (int k = 0; k < 1000; k++) { sda += gpio_get(PIN_OLED_SDA); scl += gpio_get(PIN_OLED_SCL); delayMicroseconds(10); }
      Serial.printf("I2C linii (10 ms): SDA sus %d%%, SCL sus %d%%  (normal ~100%% in repaus; 0%% = tinuta la masa)\n", sda / 10, scl / 10);
      if (g_i2c_recover_pulses >= 0) Serial.printf("la pornire SDA era blocat: deblocat dupa %d impulsuri SCL\n", g_i2c_recover_pulses);
      Serial.printf("deblocari I2C in functionare: %lu\n", (unsigned long)g_i2c_fixes);
      g_i2c_scan = true; return;
    }
    if (s[1] == 'r') {
      if (g_wm == 1) { Serial.println("WM8960 merge deja"); return; }
      if (g_wm_retry == 0) g_wm_retry = 1;
      uint32_t t0 = millis();
      while (g_wm != 1 && millis() - t0 < 3000) { if (g_fpga == 1) rx_poll(); audio_out_pump(); wm_hot_poll(); }
    }
    if (s[1]) {                                       // nucleul 1 scrie pe I2C între două desene; sunetul curge mai departe
      uint32_t t0 = millis();
      while (millis() - t0 < 50) { if (g_fpga == 1) rx_poll(); audio_out_pump(); }
    }
    wm_status();
    return;
  }
  if (s[0] == 'z') { scan_start(s + 1); return; }
  long v = atol(s + 1);
  if (s[0] == 'f' && v >= 10000 && v <= 30000000) { g_freq = v; g_freq_dirty = true; }
  if (s[0] == 'q') {
    if (s[1] == '1') {
      iqb_r = iqb_w = 0; g_stream_drop = 0;
      Serial.printf("IQ24 48000 %lu\n", (unsigned long)g_freq);   // ultimul text; de aici doar binar
      Serial.flush();
      g_stream = true;
    } else {
      g_stream = false;
      delay(50); iqb_r = iqb_w;
      Serial.printf("\nIQ stop, perechi pierdute %lu\n", (unsigned long)g_stream_drop);
    }
    return;
  }
  if (g_stream) return;                               // în timpul fluxului nu se scrie text
  if (s[0] == 'k' && s[1] == 'd') {                  // diagnostic rețea RC: cât durează încărcarea/descărcarea
    g_keys_pause = true; delay(400);                  // nucleul 1 termină ce măsura și lasă pinul
    auto rise = [](uint32_t low_ms) -> uint32_t {     // ține pinul jos low_ms, apoi pull-up: timp până citește 1
      gpio_disable_pulls(PIN_ENC_SW);
      gpio_put(PIN_ENC_SW, 0); gpio_set_dir(PIN_ENC_SW, GPIO_OUT); delay(low_ms);
      gpio_set_dir(PIN_ENC_SW, GPIO_IN); gpio_pull_up(PIN_ENC_SW);
      uint32_t t0 = time_us_32(), dt;
      do { dt = time_us_32() - t0; } while (!gpio_get(PIN_ENC_SW) && dt < 400000);
      return dt;
    };
    uint32_t up20 = rise(20), up1 = rise(1);
    // ține pinul sus 20 ms, apoi pull-down: timp până citește 0 (fără niciun buton apăsat!)
    gpio_disable_pulls(PIN_ENC_SW);
    gpio_put(PIN_ENC_SW, 1); gpio_set_dir(PIN_ENC_SW, GPIO_OUT); delay(20);
    gpio_set_dir(PIN_ENC_SW, GPIO_IN); gpio_pull_down(PIN_ENC_SW);
    uint32_t t0 = time_us_32(), dn;
    do { dn = time_us_32() - t0; } while (gpio_get(PIN_ENC_SW) && dn < 400000);
    gpio_disable_pulls(PIN_ENC_SW); gpio_pull_up(PIN_ENC_SW);
    g_keys_pause = false;
    Serial.printf("kd: urcare dupa 20 ms jos = %lu us | dupa 1 ms jos = %lu us | coborare cu pull-down = %lu us\n",
                  (unsigned long)up20, (unsigned long)up1, (unsigned long)dn);
    return;
  }
  if (s[0] == 'k') {
    Serial.printf("butoane: %s (urcare la detectie %lu us) | ultima masurare %lu us -> %s\n",
                  g_keys_rc ? "retea RC" : "doar encoder", (unsigned long)g_keys_detect_us,
                  (unsigned long)g_key_us, KEY_NAME[g_key_last]);
    return;
  }
  if (s[0] == 'x') { g_iq_inv = !g_iq_inv; Serial.printf("IQ %s\n", g_iq_inv ? "inversat" : "normal"); return; }
  if (s[0] == 'w' && v > 0) {
    int m = g_mode, best = 0, bd = 1 << 30;
    for (int k = 0; k < bw_count(m); k++) {
      g_bw_idx[m] = k; int dk = abs(bw_hz(m) - v);
      if (dk < bd) { bd = dk; best = k; }
    }
    g_bw_idx[m] = best; g_mode_dirty = true;
    Serial.printf("filtru %s: %d Hz\n", MODE_NAME[m], bw_hz(m)); return;
  }
  if (s[0] == 'm' && v >= 0 && v < MODE_N)        { g_mode = v; g_mode_dirty = true; }
  if (s[0] == 'v' && v >= 0 && v <= 100)          g_vol = v;
  if (s[0] == 'l' && v >= 0 && v <= SQL_MAX)     g_sql = v;
  if (s[0] == 'n' && v >= 0 && v <= 3)            g_nr = v;
  if (s[0] == 'c') {
    if (s[1] == 0) {                                 // automat: decalajul măsurat de PLL-ul SAM pe o stație AM
      float off = g_sam_offset_hz;
      if (g_mode != MODE_SAM) { Serial.printf("calibrare: treci intai pe SAM, pe o statie AM puternica\n"); return; }
      if (fabsf(off) > 290 || g_smeter_db < -100) { Serial.printf("calibrare: SAM nu e prins (%.0f Hz, S %.0f dBFS)\n", (double)off, (double)g_smeter_db); return; }
      // purtătoarea apare la +off Hz -> placa e acordată cu off Hz prea jos -> eroarea scade cu off/f
      g_cal_ppb -= (int32_t)lroundf(off / (float)g_freq * 1e9f);
    } else if (v >= -200000 && v <= 200000) g_cal_ppb = v;
    g_freq_dirty = true;
    Serial.printf("calibrare: %+.3f ppm (%ld ppb)\n", g_cal_ppb / 1000.0, (long)g_cal_ppb);
    return;
  }
  if (s[0] == 'g' && v >= -12 && v <= 48)         { g_gain_db = v; g_gain_dirty = true; }
  Serial.printf("FPGA %s gw %u.%u | %lu Hz %s vol %d gain %+d dB | IQ %lu/s sync err %lu ovr %lu | S %.1f dBFS | drop %lu under %lu | mic USB %s (aruncate %lu) | SQL +%d dB (zgomot %.1f dBFS) %s | NR %d | SAM %+.0f Hz\n",
                g_fpga == 1 ? "OK" : g_fpga == -3 ? "ERR PIO/DMA" : "ERR", g_gw_major, g_gw_minor, (unsigned long)g_freq, MODE_NAME[g_mode],
                g_vol, g_gain_db, (unsigned long)g_iq_rate, (unsigned long)g_rx_sync_err, (unsigned long)g_rx_overrun,
                g_smeter_db, (unsigned long)g_audio_drop, (unsigned long)g_audio_under,
                usb_audio_streaming() ? "activ" : "oprit", (unsigned long)usb_audio_drops(),
                g_sql, (double)g_sq_nf, g_sql ? (g_sq_open ? "deschis" : "inchis") : "oprit", g_nr, (double)g_sam_offset_hz);
}

void loop() {
  static uint32_t t_keep = 0, t_rate = 0; static int keep = 0;
  static char line[24]; static int len = 0;

  if (g_fpga == 1) rx_poll();
  audio_out_pump();
  wm_hot_poll();
  tx_poll();

  if (g_mode_dirty) {
    g_mode_dirty = false;
    static int dec_mode = -1; int fm = g_mode == MODE_FM;
    if (fm != dec_mode) { design_lowpass(h_dec, NDEC, fm ? 5600 : 4500, FS_IN); dec_mode = fm; }
    design_bandpass(g_mode);
  }
  if (g_freq_dirty && g_fpga == 1) { g_freq_dirty = false; rb_cmd(0x02, rb_freq()); rb_cmd(0x04, rb_freq()); }
  if (g_gain_dirty && g_fpga == 1) {
    g_gain_dirty = false; rb_cmd(0x14, rb_gain_word());
    sq_gain_shift += g_gain_db - g_gain_sent; g_gain_sent = g_gain_db;   // zgomotul urcă/coboară cu câștigul
  }

  uint32_t now = millis();
  if (g_fpga == 1 && now - t_keep >= 100) {          // comenzi retrimise ciclic: reg0, TX, RX1, câștig
    t_keep = now;
    switch (keep) {
      case 0: rb_cmd(0x00, RB_REG0); break;
      case 1: rb_cmd(0x02, rb_freq()); break;
      case 2: rb_cmd(0x04, rb_freq()); break;
      default: rb_cmd(0x14, rb_gain_word()); break;
    }
    keep = (keep + 1) % 4;
  }
  if (now - t_rate >= 1000) { t_rate = now; g_iq_rate = cnt_iq; cnt_iq = 0; g_wm_rate = g_wm_frames; g_wm_frames = 0; }
  scan_tick();
  if (g_stream) iqb_pump();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == ';') {                                   // CAT Kenwood
      line[len] = 0;
      if (len >= 2 && isupper(line[0]) && isupper(line[1])) cat_cmd(line);
      len = 0;
    } else if (c == '\n' || c == '\r') { line[len] = 0; if (len) handle_cmd(line); len = 0; }
    else if (len < 23) line[len++] = c;
  }
}

// ============================================================
// Nucleul 1: OLED + encoder
// ============================================================
U8G2_SH1106_128X64_NONAME_F_2ND_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
static volatile int32_t enc_delta = 0;
static const uint32_t STEPS[] = { 10, 100, 1000, 10000, 100000 };
static int step_idx = 1;

// Benzi: radioamatori HF + radiodifuziune AM. Frecvența/modul se memorează la părăsirea benzii.
struct Band { const char *name; uint32_t lo, hi, f; int mode; };
static Band BANDS[] = {
  { "160m", 1810000, 2000000, 1840000, MODE_LSB },
  { "80m",  3500000, 3800000, 3700000, MODE_LSB },
  { "60m",  5351500, 5366500, 5357000, MODE_USB },
  { "49m",  5900000, 6200000, 6000000, MODE_AM },
  { "40m",  7000000, 7200000, 7100000, MODE_LSB },
  { "41m",  7200000, 7450000, 7300000, MODE_AM },
  { "31m",  9400000, 9900000, 9650000, MODE_AM },
  { "30m", 10100000, 10150000, 10120000, MODE_CW },
  { "20m", 14000000, 14350000, 14200000, MODE_USB },
  { "17m", 18068000, 18168000, 18130000, MODE_USB },
  { "15m", 21000000, 21450000, 21200000, MODE_USB },
  { "12m", 24890000, 24990000, 24940000, MODE_USB },
  { "CB",  26965000, 27405000, 27185000, MODE_FM },
  { "10m", 28000000, 29700000, 28500000, MODE_USB },
};
static const int NBANDS = sizeof(BANDS) / sizeof(BANDS[0]);

// banda în care e frecvența curentă (-1 = în afara benzilor)
static int band_of(uint32_t f) {
  for (int b = 0; b < NBANDS; b++) if (f >= BANDS[b].lo && f <= BANDS[b].hi) return b;
  return -1;
}

// emisie pe toată banda HF (cerut de user 22 oct; înainte: doar benzile de radioamator)
static bool tx_band_ok(uint32_t f) { return f >= 1500000 && f <= 30000000; }


static void band_step(int dir) {
  uint32_t f = g_freq;
  int cur = band_of(f);
  if (cur >= 0) { BANDS[cur].f = f; BANDS[cur].mode = g_mode; }   // memorează unde ai rămas
  int nb;
  if (cur >= 0) nb = ((cur + dir) % NBANDS + NBANDS) % NBANDS;
  else {                                       // în afara benzilor: următoarea peste/sub frecvență
    nb = dir > 0 ? 0 : NBANDS - 1;
    for (int b = 0; b < NBANDS; b++) if (BANDS[b].lo > f) { nb = dir > 0 ? b : (b + NBANDS - 1) % NBANDS; break; }
  }
  g_mode = BANDS[nb].mode; g_mode_dirty = true;
  g_freq = BANDS[nb].f; g_freq_dirty = true;
}

static void enc_isr() {
  static uint8_t prev = 0; static int8_t acc = 0;
  static const int8_t TAB[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };
  uint8_t cur = (gpio_get(PIN_ENC_A) << 1) | gpio_get(PIN_ENC_B);
  acc += TAB[(prev << 2) | cur]; prev = cur;
  if (acc >= 4) { enc_delta++; acc = 0; }
  if (acc <= -4) { enc_delta--; acc = 0; }
}

// ---------------- setări salvate în flash ----------------
// Scrierea șterge un sector de 4 KB (~50 ms, cu nucleul 0 oprit): se face rar, după 5 s fără modificări.
struct Settings {
  uint32_t magic;
  uint32_t freq;
  uint8_t  mode, step, vol; int8_t gain;
  uint8_t  bw[8];                                   // fix 8: loc pentru moduri noi fără schimbarea structurii
  int8_t   sql; uint8_t nr; uint8_t rsv[2]; int32_t cal_ppb;    // cal_ppb: fost rezervă (0); rsv[0] = drive TX + 1
  uint32_t band_f[NBANDS];
  uint8_t  band_mode[NBANDS];
  uint8_t  wm_hp, wm_spk, wm_pga, wm_boost;          // WM8960 (RBS4): registre căști/difuzor, PGA și boost microfon
  uint32_t sum;
};
static const uint32_t SET_MAGIC = 0x52425334;          // "RBS4"; schimbă-l când se schimbă structura
// RBS3 = aceeași structură fără câmpurile WM8960; se citește o dată, ca să nu se piardă setările vechi
struct SettingsV3 {
  uint32_t magic, freq; uint8_t mode, step, vol; int8_t gain; uint8_t bw[8];
  int8_t sql; uint8_t nr; uint8_t rsv[2]; int32_t cal_ppb;
  uint32_t band_f[NBANDS]; uint8_t band_mode[NBANDS]; uint32_t sum;
};
static const uint32_t SET_MAGIC_V3 = 0x52425333;
static volatile bool g_settings_loaded = false;       // nucleul 1 aplică setările WM8960 abia după încărcare
static Settings set_saved;

static uint32_t settings_sum(const Settings &x) {
  const uint8_t *b = (const uint8_t *)&x; uint32_t h = 2166136261u;
  for (size_t k = 0; k < offsetof(Settings, sum); k++) h = (h ^ b[k]) * 16777619u;   // FNV-1a
  return h;
}

static void settings_snapshot(Settings &x) {
  memset(&x, 0, sizeof(x));
  x.magic = SET_MAGIC; x.freq = g_freq; x.mode = g_mode; x.step = step_idx;
  x.vol = g_vol; x.gain = g_gain_db; x.sql = g_sql; x.nr = g_nr; x.cal_ppb = g_cal_ppb;
  x.wm_hp = g_wm_hp; x.wm_spk = g_wm_spk; x.wm_pga = g_wm_pga; x.wm_boost = g_wm_boost;
  x.rsv[0] = (uint8_t)(g_tx_drive + 1);             // drive TX + 1 (0 = setări vechi, rămâne implicit 4)
  for (int m = 0; m < MODE_N; m++) x.bw[m] = g_bw_idx[m];
  for (int b = 0; b < NBANDS; b++) { x.band_f[b] = BANDS[b].f; x.band_mode[b] = BANDS[b].mode; }
  x.sum = settings_sum(x);
}

static void settings_load() {
  EEPROM.begin(sizeof(Settings) < 256 ? 256 : 512);
  Settings x; EEPROM.get(0, x);
  bool ok = x.magic == SET_MAGIC && x.sum == settings_sum(x);
  if (ok) {
    if (x.wm_hp == 0 || (x.wm_hp >= 0x30 && x.wm_hp <= 0x7F))   g_wm_hp = x.wm_hp;
    if (x.wm_spk == 0 || (x.wm_spk >= 0x30 && x.wm_spk <= 0x7F)) g_wm_spk = x.wm_spk;
    if (x.wm_pga <= 63) g_wm_pga = x.wm_pga;
    if (x.wm_boost <= 3) g_wm_boost = x.wm_boost;
    if (x.rsv[0] >= 1 && x.rsv[0] <= 16) g_tx_drive = x.rsv[0] - 1;
  } else {                                           // setări RBS3: aceleași câmpuri până la band_mode
    SettingsV3 o; EEPROM.get(0, o);
    uint32_t h = 2166136261u; const uint8_t *b = (const uint8_t *)&o;
    for (size_t k = 0; k < offsetof(SettingsV3, sum); k++) h = (h ^ b[k]) * 16777619u;
    if (o.magic == SET_MAGIC_V3 && o.sum == h) {
      memcpy(&x, &o, offsetof(SettingsV3, sum));
      ok = true;
    }
  }
  if (ok) {                                          // altfel: prima pornire, rămân valorile implicite
    if (x.freq >= 10000 && x.freq <= 30000000) g_freq = x.freq;
    if (x.mode < MODE_N) g_mode = x.mode;
    if (x.step < 5) step_idx = x.step;
    if (x.vol <= 100) g_vol = x.vol;
    if (x.sql >= 0 && x.sql <= SQL_MAX) g_sql = x.sql;
    else if (x.sql < 0) g_sql = 8;                   // setare veche, prag fix în dBFS -> 8 dB peste zgomot
    if (x.nr <= 3) g_nr = x.nr;
    if (x.cal_ppb >= -200000 && x.cal_ppb <= 200000) g_cal_ppb = x.cal_ppb;
    if (x.gain >= -12 && x.gain <= 48) g_gain_db = x.gain;
    for (int m = 0; m < MODE_N; m++) if (x.bw[m] < bw_count(m)) g_bw_idx[m] = x.bw[m];
    for (int b = 0; b < NBANDS; b++) {
      if (x.band_f[b] >= BANDS[b].lo && x.band_f[b] <= BANDS[b].hi) BANDS[b].f = x.band_f[b];
      if (x.band_mode[b] < MODE_N) BANDS[b].mode = x.band_mode[b];
    }
  }
  settings_snapshot(set_saved);
  g_settings_loaded = true;
}

// apelat din loop1: salvează când setările s-au schimbat și apoi au stat neatinse 5 s
static void settings_poll() {
  static Settings last; static uint32_t t_change = 0; static bool pending = false;
  if (scan_on) return;                                   // baleiajul schimbă frecvența temporar
  Settings now; settings_snapshot(now);
  if (memcmp(&now, &last, sizeof(now)) != 0) { last = now; t_change = millis(); pending = true; return; }
  if (!pending || millis() - t_change < 5000) return;
  pending = false;
  if (memcmp(&now, &set_saved, sizeof(now)) == 0) return;
  EEPROM.put(0, now);
  EEPROM.commit();
  set_saved = now;
}

// Deblocare I2C. Când Pico se resetează (scriere firmware) în mijlocul unei transmisii, un slave alimentat
// în continuare (WM8960 ia 5V din header, nu se resetează) poate rămâne cu SDA ținut jos, așteptând ceasul:
// magistrala moare, OLED-ul nu mai pornește. Se dau impulsuri pe SCL până eliberează SDA, apoi STOP.
// (Văzut 21 oct: SDA 0%, SCL 100% după fiecare flash.)
static void i2c_bus_recover() {
  const uint SDA = PIN_OLED_SDA, SCL = PIN_OLED_SCL;
  gpio_init(SDA); gpio_init(SCL);
  gpio_pull_up(SDA); gpio_pull_up(SCL);
  gpio_put(SDA, 0); gpio_put(SCL, 0);                // „open-drain": 0 = ieșire jos, 1 = intrare (pull-up)
  gpio_set_dir(SDA, GPIO_IN); gpio_set_dir(SCL, GPIO_IN);
  delayMicroseconds(10);
  if (gpio_get(SDA)) return;
  int n = 0;
  while (!gpio_get(SDA) && n < 18) {
    gpio_set_dir(SCL, GPIO_OUT); delayMicroseconds(5);
    gpio_set_dir(SCL, GPIO_IN);  delayMicroseconds(5);
    n++;
  }
  // STOP: SDA jos cu SCL sus, apoi SDA sus
  gpio_set_dir(SDA, GPIO_OUT); delayMicroseconds(5);
  gpio_set_dir(SDA, GPIO_IN);  delayMicroseconds(5);
  g_i2c_recover_pulses = n;
}

// Deblocare în timpul funcționării: SDA ținut jos la 3 verificări la rând (la 100 ms, între tranzacții —
// I2C-ul e doar al nucleului 1) -> I2C oprit, impulsuri SCL + STOP, I2C și OLED repornite.
// Văzut 22 oct după pornire de la zero: WM8960, care termină pornirea după placă, rămâne cu SDA jos în mijlocul
// unei transmisii spre OLED; atingerea unui pin cu degetul dădea „ceasul" care îl elibera.
static void i2c_runtime_recover() {
  Wire1.end();
  i2c_bus_recover();
  Wire1.setSDA(PIN_OLED_SDA); Wire1.setSCL(PIN_OLED_SCL); Wire1.setClock(I2C_HZ); oled.setBusClock(I2C_HZ);
  oled.begin();                                      // pornește și Wire1; OLED-ul poate fi rămas la jumătatea unei comenzi
  g_i2c_fixes++;
}
static void i2c_watchdog() {
  static uint32_t t = 0; static int low = 0;
  if (millis() - t < 100) return;
  t = millis();
  if (gpio_get(PIN_OLED_SDA)) { low = 0; return; }
  if (++low >= 3) { low = 0; i2c_runtime_recover(); }
}

void setup1() {
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), enc_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), enc_isr, CHANGE);
  i2c_bus_recover();
  Wire1.setSDA(PIN_OLED_SDA); Wire1.setSCL(PIN_OLED_SCL); Wire1.setClock(I2C_HZ); oled.setBusClock(I2C_HZ);
  oled.begin();
  // WM8960 pe același I2C: nucleul 0 așteaptă rezultatul în audio_out_begin(). La pornire „la rece" (HAT-ul
  // alimentat odată cu placa) codecul răspunde abia după un timp -> se încearcă până la 2 s, cu deblocare.
  bool found = false;
  for (uint32_t t0 = millis(); millis() - t0 < 2000; delay(100)) {
    if (!gpio_get(PIN_OLED_SDA)) i2c_runtime_recover();
    if ((found = wm8960_probe(Wire1))) break;
  }
  g_wm = (found && wm8960_init(Wire1)) ? 1 : -1;
  if (g_wm == 1) {                                   // volumele și microfonul salvate în flash
    uint32_t t0 = millis();
    while (!g_settings_loaded && millis() - t0 < 1000) delay(1);
    wm8960_set_hp(Wire1, g_wm_hp); wm8960_set_spk(Wire1, g_wm_spk); wm8960_set_mic(Wire1, g_wm_pga, g_wm_boost);
  }
}

static void i2c_scan() {
  Serial.print("I2C (GP2/GP3):");
  int n = 0;
  for (int a = 0x08; a < 0x78; a++) {
    if (a == WM8960_ADDR) continue;                 // codecul nu se poate citi; verificat separat mai jos
    // citire de 1 octet: fără „probe"-ul bit-bang al Wire
    if (Wire1.requestFrom(a, 1) == 1) { Wire1.read(); Serial.printf(" 0x%02X%s", a, a == 0x3C ? "(OLED)" : ""); n++; }
  }
  if (g_wm == 1) {                                   // WM8960: rescrie volumul căștilor (inofensiv) și vezi dacă confirmă
    if (wm8960_set_hp(Wire1, g_wm_hp)) { Serial.print(" 0x1A(WM8960)"); n++; }
  }
  Serial.println(n ? "" : " nimic");
}

// cererile de la comenzile USB („a…") se execută aici: I2C-ul e al nucleului 1 (OLED-ul e pe același bus)
static void wm_requests() {
  if (g_i2c_scan) { g_i2c_scan = false; i2c_scan(); }
  if (g_wm_stop_req) {                               // reset = codec slave, fără ceasuri; se repetă până confirmă
    g_wm_stop_req = false;
    for (int k = 0; k < 10; k++) {
      if (!gpio_get(PIN_OLED_SDA)) i2c_runtime_recover();
      Wire1.beginTransmission(WM8960_ADDR); Wire1.write((uint8_t)(0x0F << 1)); Wire1.write((uint8_t)0);
      if (Wire1.endTransmission() == 0) { Serial.printf("WM8960 resetat (ceasuri oprite) la incercarea %d\n", k + 1); break; }
    }
  }
  if (g_wm_retry == 1) {
    g_wm_init = (wm8960_probe(Wire1) && wm8960_init(Wire1)) ? 1 : -1;
    if (g_wm_init == 1) { wm8960_set_hp(Wire1, g_wm_hp); wm8960_set_spk(Wire1, g_wm_spk); wm8960_set_mic(Wire1, g_wm_pga, g_wm_boost); }
    g_wm_retry = 2;
  }
  if (g_wm != 1) return;
  int v;
  if ((v = g_wm_hp_req) >= 0)  { g_wm_hp_req = -1;  if (v < 0x30) v = 0; if (v > 0x7F) v = 0x7F; g_wm_hp = v;  wm8960_set_hp(Wire1, v); }
  if ((v = g_wm_spk_req) >= 0) { g_wm_spk_req = -1; if (v < 0x30) v = 0; if (v > 0x7F) v = 0x7F; g_wm_spk = v; wm8960_set_spk(Wire1, v); }
  if (g_wm_mic_req >= 0)       { g_wm_mic_req = -1; wm8960_set_mic(Wire1, g_wm_pga, g_wm_boost); }
}

// ---------------- butoane pe un singur pin (GP24 = pin 18 = DCLK, după configurarea FPGA) ----------------
// pin 18 ── 470 Ω ── N ;  N ── 1 µF ── GND ;  N ── encoder SW ── GND (direct)
//   N ── MOD ── 1 kΩ ── GND ;  N ── BANDA ── 2,2 kΩ ── GND ;  N ── FILTRU ── 4,7 kΩ ── GND ;  N ── PAS ── 10 kΩ ── GND
// 470 Ω izolează condensatorul: la încărcarea FPGA, DCLK comută rapid și nu vede 1 µF.
// Repaus: pull-up-ul intern ține N sus. Apăsat: pinul citește 0 -> se încarcă C prin 470 Ω, apoi se măsoară
// cât durează până scade sub prag (calculat pt. prag 1,0..1,6 V):
//   SW ~0 | 1k 0,34-0,81 ms | 2k2 1,17-2,20 ms | 4k7 2,95-5,16 ms | 10k 6,8-11,5 ms
// Limitele dintre ferestre sunt la mijloc (geometric); comanda USB „k" arată timpul măsurat.
// Măsurat pe placă (2 oct 2026, cu un pull-up de ~10k rămas pe nod): SW 0 | MOD 600-860 | BANDA 2275-3050 µs.
// Limitele = medii geometrice între grupuri, cu marjă ≥1,6x; FILTRU/PAS nemăsurate (încă nemontate).
// 22 oct: butonul PTT (4,7 kΩ) se măsoară mult mai lent decât era calculat, din cauza pull-up-ului de ~10 kΩ rămas
// pe nod (cade în fereastra PAS). PAS nu e montat -> tot ce trece de 5 ms e PTT; limita urcă la 40 ms ca PTT-ul
// ținut să nu fie citit din când în când ca „eliberat".
static const uint32_t KEY_LIM_US[] = { 100, 1500, 5000, 10000, 40000 };  // SW | MOD | BANDA | PTT | PTT
static const int      KEY_OF[]     = { K_SW, K_MOD, K_BAND, K_FILT, K_FILT };

static void keys_detect() {
  // descarcă C, apoi lasă pull-up-ul (~50 kΩ) să-l încarce: cu 1 µF durează ~50 ms, fără C câteva µs
  gpio_put(PIN_ENC_SW, 0); gpio_set_dir(PIN_ENC_SW, GPIO_OUT); delay(20);
  gpio_set_dir(PIN_ENC_SW, GPIO_IN); gpio_pull_up(PIN_ENC_SW);
  uint32_t t0 = time_us_32(), dt;
  do { dt = time_us_32() - t0; } while (!gpio_get(PIN_ENC_SW) && dt < 300000);
  g_keys_detect_us = dt;
  g_keys_rc = dt > 2000 && dt < 300000;      // ținut apăsat la pornire (nu urcă deloc) = mod simplu, sigur
}

static int keys_read() {
  if (gpio_get(PIN_ENC_SW)) return K_NONE;               // nimic apăsat
  if (!g_keys_rc) return K_SW;                           // fără rețea: nu se comandă pinul (SW e direct la GND)
  gpio_put(PIN_ENC_SW, 1); gpio_set_dir(PIN_ENC_SW, GPIO_OUT);
  delayMicroseconds(3000);                               // încarcă C prin 470 Ω (τ ≤ 0,47 ms)
  gpio_set_dir(PIN_ENC_SW, GPIO_IN);
  uint32_t t0 = time_us_32(), dt;
  do { dt = time_us_32() - t0; } while (gpio_get(PIN_ENC_SW) && dt < KEY_LIM_US[4]);
  g_key_us = dt;
  for (int k = 0; k < 5; k++) if (dt < KEY_LIM_US[k]) return KEY_OF[k];
  return K_NONE;                                         // eliberat chiar în timpul măsurării
}

enum { UI_FREQ, UI_STEP, UI_MODE, UI_FILT, UI_BAND, UI_VOL, UI_GAIN, UI_SQL, UI_NR, UI_CAL, UI_DRIVE, UI_MIC, UI_N };
static int ui_sel = UI_FREQ;

// ---------------- spectru + waterfall ----------------
// IQ 48 kHz, FFT 256 (187,5 Hz/bin) -> 128 coloane (375 Hz/coloană) = ±24 kHz în jurul frecvenței.
#define SP_X 128
#define SP_Y0 11            // spectrul: rândurile 11..32
#define SP_H 22
#define WF_Y0 33            // waterfall: rândurile 33..55
#define WF_H 23
static float fft_re[FFT_N], fft_im[FFT_N], win[FFT_N];
static float sp_db[SP_X];                    // spectru netezit, dB
static uint8_t wf[WF_H][SP_X];              // intensitate 0..16
static int wf_top = 0;                       // rândul cel mai nou (buffer circular)
static float sp_floor = -100;

static void fft256(float *re, float *im) {
  for (int i = 1, j = 0; i < FFT_N; i++) {              // permutare bit-inversată
    int bit = FFT_N >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) { float t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; }
  }
  for (int len = 2; len <= FFT_N; len <<= 1) {
    float ang = -2 * (float)M_PI / len, wr = cosf(ang), wi = sinf(ang);
    for (int i = 0; i < FFT_N; i += len) {
      float cr = 1, ci = 0;
      for (int k = 0; k < len / 2; k++) {
        int u = i + k, v = i + k + len / 2;
        float tr = re[v] * cr - im[v] * ci, ti = re[v] * ci + im[v] * cr;
        re[v] = re[u] - tr; im[v] = im[u] - ti; re[u] += tr; im[u] += ti;
        float ncr = cr * wr - ci * wi; ci = cr * wi + ci * wr; cr = ncr;
      }
    }
  }
}

static void spectrum_update() {
  __dmb();
  for (int k = 0; k < FFT_N; k++) { fft_re[k] = cap_i[k] * win[k]; fft_im[k] = cap_q[k] * win[k]; }
  cap_ready = false;                                    // nucleul 0 poate umple următorul bloc
  fft256(fft_re, fft_im);
  float col[SP_X];
  for (int c = 0; c < SP_X; c++) {
    // fftshift: coloana 0 = -24 kHz, 64 = frecvența acordată, 127 = +24 kHz
    int b0 = (2 * c + FFT_N / 2) % FFT_N, b1 = (b0 + 1) % FFT_N;
    float p0 = fft_re[b0] * fft_re[b0] + fft_im[b0] * fft_im[b0];
    float p1 = fft_re[b1] * fft_re[b1] + fft_im[b1] * fft_im[b1];
    col[c] = 10 * log10f((p0 > p1 ? p0 : p1) + 1e-20f);
  }
  col[64] = 0.5f * (col[63] + col[65]);                 // ascunde vârful DC al convertorului
  // nivelul zgomotului = percentila 25 (robust la stații)
  float tmp[SP_X]; memcpy(tmp, col, sizeof(tmp));
  for (int i = 1; i < SP_X; i++) { float v = tmp[i]; int j = i - 1; while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; } tmp[j + 1] = v; }
  sp_floor = 0.8f * sp_floor + 0.2f * tmp[SP_X / 4];
  for (int c = 0; c < SP_X; c++) sp_db[c] = 0.5f * sp_db[c] + 0.5f * col[c];
  // rând nou în waterfall: 0..16 pe 40 dB peste zgomot
  wf_top = (wf_top + WF_H - 1) % WF_H;
  for (int c = 0; c < SP_X; c++) {
    float v = (col[c] - sp_floor - 3) * 16.0f / 37.0f;
    wf[wf_top][c] = (uint8_t)(v < 0 ? 0 : v > 16 ? 16 : v);
  }
}

static const uint8_t BAYER4[4][4] = { { 0, 8, 2, 10 }, { 12, 4, 14, 6 }, { 3, 11, 1, 9 }, { 15, 7, 13, 5 } };

static void spectrum_draw() {
  // spectru: 40 dB pe SP_H pixeli, de la zgomot în sus
  for (int c = 0; c < SP_X; c++) {
    int h = (int)((sp_db[c] - sp_floor + 2) * SP_H / 40.0f);
    if (h < 0) h = 0; if (h > SP_H) h = SP_H;
    if (h) oled.drawVLine(c, SP_Y0 + SP_H - h, h);
  }
  // marcaj frecvență acordată + banda filtrului (375 Hz/coloană)
  for (int y = SP_Y0; y < SP_Y0 + SP_H; y += 3) oled.drawPixel(64, y);
  float lo, hi;
  filter_edges(g_mode, &lo, &hi);
  int x0 = 64 + (int)floorf(lo / 375.0f), x1 = 64 + (int)ceilf(hi / 375.0f);
  oled.drawHLine(x0, SP_Y0, x1 - x0 + 1);
  // waterfall cu dithering ordonat (monocrom)
  for (int r = 0; r < WF_H; r++) {
    const uint8_t *row = wf[(wf_top + r) % WF_H];
    int y = WF_Y0 + r;
    for (int c = 0; c < SP_X; c++)
      if (row[c] > BAYER4[y & 3][c & 3]) oled.drawPixel(c, y);
  }
}

// text cu fundal inversat când e selectat (font 5x7)
static int draw_item(int x, int y, const char *txt, bool sel) {
  int w = (int)strlen(txt) * 5;
  if (sel) { oled.drawBox(x - 1, y - 7, w + 1, 8); oled.setDrawColor(0); oled.drawStr(x, y, txt); oled.setDrawColor(1); }
  else oled.drawStr(x, y, txt);
  return x + w + 4;
}

void loop1() {
  static uint32_t t_draw = 0, t_down = 0, t_act = 0; static bool down = false, long_done = false;
  g_core1_beat++;
  i2c_watchdog();
  wm_requests();
  static bool win_done = false;
  if (!win_done) { for (int k = 0; k < FFT_N; k++) win[k] = 0.5f - 0.5f * cosf(2 * (float)M_PI * k / (FFT_N - 1)); win_done = true; }

  int32_t d; noInterrupts(); d = enc_delta; enc_delta = 0; interrupts();
  if (d) {
    t_act = millis();
    switch (ui_sel) {
      case UI_FREQ: {
        int64_t f = (int64_t)g_freq + (int64_t)d * STEPS[step_idx];
        if (f < 10000) f = 10000; if (f > 30000000) f = 30000000;
        g_freq = (uint32_t)f; g_freq_dirty = true; break;
      }
      case UI_STEP: step_idx = constrain(step_idx + (int)d, 0, 4); break;
      case UI_MODE: g_mode = ((g_mode + (int)d) % MODE_N + MODE_N) % MODE_N; g_mode_dirty = true; break;
      case UI_FILT: { int m = g_mode; g_bw_idx[m] = constrain(g_bw_idx[m] + (int)d, 0, bw_count(m) - 1); g_mode_dirty = true; break; }
      case UI_BAND: band_step(d > 0 ? 1 : -1); break;
      case UI_VOL:  g_vol = constrain(g_vol + 2 * (int)d, 0, 100); break;
      case UI_GAIN: g_gain_db = constrain(g_gain_db + (int)d, -12, 48); g_gain_dirty = true; break;
      case UI_SQL: g_sql = constrain(g_sql + (int)d, 0, SQL_MAX); break;   // oprit, apoi 1..40 dB peste zgomot
      case UI_NR: g_nr = constrain(g_nr + (int)d, 0, 3); break;
      case UI_CAL: g_cal_ppb = constrain(g_cal_ppb + 100 * (int)d, -200000, 200000); g_freq_dirty = true; break;   // 0,1 ppm
      case UI_DRIVE: g_tx_drive = constrain(g_tx_drive + (int)d, 0, 15); g_tx_drive_dirty = true; break;
      case UI_MIC: g_wm_pga = constrain(g_wm_pga + 2 * (int)d, 0, 63); g_wm_mic_req = 1; break;   // 1,5 dB/pas, se aplică pe I2C
    }
  }
  if (g_ui_ready) {                                   // butonul e pe DCLK: doar după configurare
    // detectarea rețelei RC: la pornire, apoi la 5 s cât timp nu e găsită (dacă se montează cu placa pornită)
    static uint32_t t_det = 0; static bool keys_init = false;
    if (g_keys_pause) return;
    if (!keys_init || (!g_keys_rc && gpio_get(PIN_ENC_SW) && millis() - t_det > 5000)) {
      keys_detect(); keys_init = true; t_det = millis();
    }
    static uint32_t t_key = 0, t_kdown = 0; static int key = K_NONE, key_prev = K_NONE, key_raw = K_NONE;
    static bool band_long = false;
    uint32_t t = millis();
    if (t - t_key >= 25) {                            // două citiri identice la rând = apăsare stabilă
      t_key = t;
      int k = keys_read();
      if (k == key_raw) key = k;
      key_raw = k; g_key_last = k;
    }
    if (key != key_prev) {                            // MOD/FILTRU/PAS la apăsare; BANDA la eliberare (lung = înapoi)
      if (key == K_MOD)  { g_mode = (g_mode + 1) % MODE_N; g_mode_dirty = true; }
      if (key == K_STEP) step_idx = (step_idx + 1) % 5;
      if (key == K_BAND) { t_kdown = t; band_long = false; }
      if (key_prev == K_BAND && !band_long) band_step(1);
      if (key != K_NONE) { t_act = t; if (g_keys_rc && !g_stream && !cat_active()) Serial.printf("tasta %s: %lu us\n", KEY_NAME[key], (unsigned long)g_key_us); }
      key_prev = key;
    }
    if (key == K_BAND && !band_long && t - t_kdown > 700) { band_step(-1); band_long = true; }
    g_ptt_key = key == K_FILT;                        // PTT: emite cât timp e ținut (4,7 kΩ)
    bool now_down = key == K_SW;
    if (now_down && !down) { t_down = t; long_done = false; }
    if (now_down && !long_done && t - t_down > 700) { ui_sel = UI_FREQ; long_done = true; t_act = t; }
    if (!now_down && down && !long_done && t - t_down > 30) { ui_sel = (ui_sel + 1) % UI_N; t_act = t; }
    down = now_down;
  }
  // BOOT ținut apăsat = PTT (citirea oprește scurt accesul la flash, deci doar la 40 ms)
  static uint32_t t_boot = 0;
  if (g_boot_ptt && g_ui_ready && millis() - t_boot >= 40) { t_boot = millis(); g_ptt_boot = BOOTSEL; }
  if (!g_boot_ptt) g_ptt_boot = false;
  if (ui_sel != UI_FREQ && millis() - t_act > 6000) ui_sel = UI_FREQ;
  if (!g_tx_on) settings_poll();                      // nu se scrie în flash în timpul emisiei


  // spectru: cere un bloc nou, procesează-l când e gata
  if (cap_ready) spectrum_update();
  else if (!cap_req) { cap_n = 0; __dmb(); cap_req = true; }

  if (millis() - t_draw < 80) return;
  t_draw = millis();
  if (!gpio_get(PIN_OLED_SDA)) i2c_runtime_recover();   // nu desena pe o magistrală blocată (fiecare transfer ar expira)
  if ((int32_t)(g_oled_test_until - millis()) > 0) {  // model de test: chenar + linii la rândurile 0/2/4 și 59/61/63
    oled.clearBuffer();
    oled.drawFrame(0, 0, 128, 64);
    for (int y : { 2, 4, 59, 61 }) oled.drawHLine(20, y, 88);
    oled.setFont(u8g2_font_5x7_tf);
    oled.drawStr(4, 8, "0"); oled.drawStr(112, 8, "2/4");
    oled.drawStr(30, 36, "TEST OLED 128x64");
    oled.drawStr(4, 62, "63"); oled.drawStr(104, 58, "59/61");
    oled.sendBuffer();
    return;
  }
  char l[24];
  uint32_t f = g_freq;
  oled.clearBuffer();
  // rândul de sus: frecvența (mare-mică) + modul
  oled.setFont(u8g2_font_6x10_tf);
  snprintf(l, sizeof(l), "%lu.%03lu.%02lu", (unsigned long)(f / 1000000), (unsigned long)(f / 1000 % 1000),
           (unsigned long)(f % 1000 / 10));
  oled.drawStr(0, 9, l);
  if (ui_sel == UI_FREQ) oled.drawHLine(0, 10, (int)strlen(l) * 6);
  oled.setFont(u8g2_font_5x7_tf);
  int bd = band_of(f);
  draw_item(58, 8, bd >= 0 ? BANDS[bd].name : "--", ui_sel == UI_BAND);
  draw_item(82, 8, MODE_NAME[g_mode], ui_sel == UI_MODE);
  snprintf(l, sizeof(l), "%4.0f", g_smeter_db); oled.drawStr(108, 8, l);

  spectrum_draw();
  if (g_tx_on) {                                     // indicator de emisie peste numele benzii
    oled.setDrawColor(1); oled.drawBox(92, 0, 36, 11); oled.setDrawColor(0);
    oled.setFont(u8g2_font_6x10_tf); oled.drawStr(95, 9, g_tune_req ? "TUNE" : " TX "); oled.setDrawColor(1);
    oled.setFont(u8g2_font_5x7_tf);
  }

  // rândul de jos: pas, volum, câștig, filtru — sau squelch / NR când sunt selectate în meniu
  int x = 0;
  if (ui_sel == UI_MIC) {                            // sensibilitatea microfonului: câștig analogic total în codec
    static const int BOOST_DB[4] = { 0, 13, 20, 29 };
    float db = BOOST_DB[g_wm_boost & 3] + (-17.25f + 0.75f * g_wm_pga);
    snprintf(l, sizeof(l), "MIC %+.1fdB", (double)db);
    x = draw_item(1, 63, l, true);
    float pk = g_tx_on ? 20 * log10f(g_tx_mic_pk + 1e-9f) : -99;   // în emisie: vârful vocii
    if (g_tx_on) { snprintf(l, sizeof(l), "%.0fdBFS", (double)pk); oled.drawStr(x + 2, 63, l); g_tx_mic_pk = 0; }   // vârf pe ultimul cadru
    oled.sendBuffer();
    return;
  }
  if (ui_sel == UI_DRIVE) {                          // puterea de emisie: nivelul drive 0..15 trimis FPGA-ului
    snprintf(l, sizeof(l), "DRIVE %d/15", g_tx_drive);
    x = draw_item(1, 63, l, true);
    for (int k = 0; k < 15; k++) {                   // bară: 15 trepte de 2 px
      if (k < g_tx_drive) oled.drawBox(x + 2 + k * 3, 57, 2, 6); else oled.drawPixel(x + 2 + k * 3, 62);
    }
    oled.sendBuffer();
    return;
  }
  if (ui_sel == UI_CAL) {                            // calibrare: ppm + decalajul SAM (rotește până ajunge ~0)
    snprintf(l, sizeof(l), "CAL %+.1fppm", g_cal_ppb / 1000.0);
    x = draw_item(1, 63, l, true);
    if (g_mode == MODE_SAM) { snprintf(l, sizeof(l), "%+.0fHz", (double)g_sam_offset_hz); oled.drawStr(x + 2, 63, l); }
    else oled.drawStr(x + 2, 63, "->SAM");
    oled.sendBuffer();
    return;
  }
  if (ui_sel == UI_SQL || ui_sel == UI_NR) {
    if (g_sql) snprintf(l, sizeof(l), "SQL +%d", g_sql); else snprintf(l, sizeof(l), "SQL off");
    x = draw_item(1, 63, l, ui_sel == UI_SQL);
    if (g_nr) snprintf(l, sizeof(l), "NR %d", g_nr); else snprintf(l, sizeof(l), "NR off");
    draw_item(x + 6, 63, l, ui_sel == UI_NR);
    oled.sendBuffer();
    return;
  }
  uint32_t st = STEPS[step_idx];
  if (st >= 1000) snprintf(l, sizeof(l), "P%luk", (unsigned long)(st / 1000)); else snprintf(l, sizeof(l), "P%lu", (unsigned long)st);
  x = draw_item(x + 1, 63, l, ui_sel == UI_STEP);
  snprintf(l, sizeof(l), "V%d", g_vol);           x = draw_item(x, 63, l, ui_sel == UI_VOL);
  snprintf(l, sizeof(l), "G%+d", g_gain_db);      x = draw_item(x, 63, l, ui_sel == UI_GAIN);
  int bw = bw_hz(g_mode);
  if (bw >= 1000) snprintf(l, sizeof(l), "F%d.%d", bw / 1000, bw % 1000 / 100); else snprintf(l, sizeof(l), "F%d", bw);
  x = draw_item(x, 63, l, ui_sel == UI_FILT);
  if (g_nr) oled.drawStr(101, 63, "N");             // reducerea de zgomot pornită
  if (g_sql) draw_item(108, 63, "Q", !g_sq_open);     // squelch: „Q” inversat = închis
  if (g_fpga != 1) oled.drawStr(113, 63, "ERR");
  else if (g_keys_rc) oled.drawStr(118, 63, "RC");   // rețeaua de butoane detectată
  oled.sendBuffer();
}
