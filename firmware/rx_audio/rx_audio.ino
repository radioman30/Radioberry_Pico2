// rx_audio — receptor SSB/CW/AM: Radioberry v2 (CL025) + Waveshare RP2350-PiZero + PCM5102A
//
// Lanț: FPGA (gateware PIO 75.2 din RagchewBerry/WP3DN, 4 linii + meta, citit cu PIO + DMA) -> IQ 48 kHz
//       -> decimare /4 (12 kHz) -> filtru complex trece-bandă -> demodulare -> AGC -> I2S 12 kHz.
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
#define PIN_I2S_DATA       14   //  14    4   7   PCM5102A DIN
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
static const char *KEY_NAME[] = { "-", "ENCODER", "MOD", "BANDA", "FILTRU", "PAS" };      // diagnosticul „kd" ține nucleul 1 departe de pin
static volatile int      g_vol = 30;                // 0..100
static volatile int      g_gain_db = 20;            // câștig LNA AD9866: -12..+48 dB
static volatile bool     g_gain_dirty = false;
static int               g_gain_sent = 0;           // câștigul trimis efectiv la FPGA (pentru squelch)
static volatile bool     g_freq_dirty = true, g_mode_dirty = true;
static volatile int      g_fpga = 0;                // 1 = OK
static volatile uint8_t  g_gw_major = 0, g_gw_minor = 0;
static volatile float    g_smeter_db = -140;
static volatile uint32_t g_iq_rate = 0, g_audio_drop = 0, g_audio_under = 0;
static volatile bool     g_ui_ready = false;        // nucleul 1 poate lua pinul DCLK ca buton

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
// Ieșire audio (PCM5102A, I2S 48 kHz). De înlocuit aici pentru CM108AH.
// Demodulatorul dă 12 kHz; ieșirea e la 48 kHz (interpolare liniară ×4), ca în audio_test validat:
// cu SCK la GND, PCM5102A își face ceasul din BCK prin PLL, iar la 12 kHz (BCK 384 kHz) nu e sigur
// că se sincronizează — pe placă, la 12 kHz se auzea doar zgomot și cu volumul pe 0.
// ============================================================
static const int FS_OUT = 48000, UPS = FS_OUT / FS;
static I2S i2s(OUTPUT);
#define AQ 4096
static int16_t aq[AQ]; static int aq_w = 0, aq_r = 0;
static float up_prev = 0;

static void audio_out_begin() {
  i2s.setBCLK(PIN_I2S_BCK);
  i2s.setDATA(PIN_I2S_DATA);
  i2s.setBitsPerSample(16);
  i2s.setBuffers(8, 256);
  i2s.begin(FS_OUT);
}
static void audio_out_push(float x) {
  if (x > 1) x = 1; if (x < -1) x = -1;
  int n = (aq_w - aq_r + AQ) % AQ;
  if (n >= AQ - UPS) { g_audio_drop++; up_prev = x; return; }   // ceasul FPGA > ceasul I2S: aruncă
  for (int k = 1; k <= UPS; k++) {
    float y = up_prev + (x - up_prev) * k / UPS;
    aq[aq_w] = (int16_t)(y * 32000); aq_w = (aq_w + 1) % AQ;
  }
  up_prev = x;
}
static void audio_out_pump() {
  while (aq_r != aq_w && i2s.availableForWrite() >= 4) {
    int16_t s = aq[aq_r]; aq_r = (aq_r + 1) % AQ;
    i2s.write16(s, s);
  }
  if (i2s.getUnderflow()) g_audio_under++;
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
  if (dsp_push(fi, fq, &out)) { audio_out_push(out); usb_audio_push12k(g_dsp_raw); }
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
// Nucleul 0
// ============================================================
static void radio_start() {
  gpio_init(PIN_RX_CLK); gpio_set_dir(PIN_RX_CLK, GPIO_OUT); gpio_put(PIN_RX_CLK, 0);  // jos ÎNAINTE de încărcare
  gpio_init(PIN_RX_RDY); gpio_set_dir(PIN_RX_RDY, GPIO_IN);
  for (int k = 0; k < 4; k++) { gpio_init(PIN_RX_D0 + k); gpio_set_dir(PIN_RX_D0 + k, GPIO_IN); }
  for (uint8_t p : FREED_GP) { gpio_init(p); gpio_set_dir(p, GPIO_IN); }
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
  } else if ((a == 'T' && b == 'X') || (a == 'R' && b == 'X')) { /* fără emisie */ }
  else if (!n) Serial.print("?;");                  // interogare necunoscută
  // setare necunoscută: Kenwood nu răspunde nimic
}

static void handle_cmd(const char *s) {
  if (s[0] == 'i') { pin_activity(); return; }
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
  if (now - t_rate >= 1000) { t_rate = now; g_iq_rate = cnt_iq; cnt_iq = 0; }
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
  int8_t   sql; uint8_t nr; uint8_t rsv[2]; int32_t cal_ppb;    // cal_ppb: fost rezervă (0)
  uint32_t band_f[NBANDS];
  uint8_t  band_mode[NBANDS];
  uint32_t sum;
};
static const uint32_t SET_MAGIC = 0x52425333;          // "RBS3"; schimbă-l când se schimbă structura
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
  for (int m = 0; m < MODE_N; m++) x.bw[m] = g_bw_idx[m];
  for (int b = 0; b < NBANDS; b++) { x.band_f[b] = BANDS[b].f; x.band_mode[b] = BANDS[b].mode; }
  x.sum = settings_sum(x);
}

static void settings_load() {
  EEPROM.begin(sizeof(Settings) < 256 ? 256 : 512);
  Settings x; EEPROM.get(0, x);
  if (x.magic == SET_MAGIC && x.sum == settings_sum(x)) {     // altfel: prima pornire, rămân valorile implicite
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

void setup1() {
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), enc_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), enc_isr, CHANGE);
  Wire1.setSDA(PIN_OLED_SDA); Wire1.setSCL(PIN_OLED_SCL); Wire1.setClock(400000);
  oled.begin();
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
static const uint32_t KEY_LIM_US[] = { 100, 1500, 5000, 10000, 20000 };  // SW | MOD | BANDA | FILTRU | PAS
static const int      KEY_OF[]     = { K_SW, K_MOD, K_BAND, K_FILT, K_STEP };

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

enum { UI_FREQ, UI_STEP, UI_MODE, UI_FILT, UI_BAND, UI_VOL, UI_GAIN, UI_SQL, UI_NR, UI_CAL, UI_N };
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
      if (key == K_FILT) { int m = g_mode; g_bw_idx[m] = (g_bw_idx[m] + 1) % bw_count(m); g_mode_dirty = true; }
      if (key == K_STEP) step_idx = (step_idx + 1) % 5;
      if (key == K_BAND) { t_kdown = t; band_long = false; }
      if (key_prev == K_BAND && !band_long) band_step(1);
      if (key != K_NONE) { t_act = t; if (g_keys_rc && !g_stream && !cat_active()) Serial.printf("tasta %s: %lu us\n", KEY_NAME[key], (unsigned long)g_key_us); }
      key_prev = key;
    }
    if (key == K_BAND && !band_long && t - t_kdown > 700) { band_step(-1); band_long = true; }
    bool now_down = key == K_SW;
    if (now_down && !down) { t_down = t; long_done = false; }
    if (now_down && !long_done && t - t_down > 700) { ui_sel = UI_FREQ; long_done = true; t_act = t; }
    if (!now_down && down && !long_done && t - t_down > 30) { ui_sel = (ui_sel + 1) % UI_N; t_act = t; }
    down = now_down;
  }
  if (ui_sel != UI_FREQ && millis() - t_act > 6000) ui_sel = UI_FREQ;
  settings_poll();


  // spectru: cere un bloc nou, procesează-l când e gata
  if (cap_ready) spectrum_update();
  else if (!cap_req) { cap_n = 0; __dmb(); cap_req = true; }

  if (millis() - t_draw < 80) return;
  t_draw = millis();
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

  // rândul de jos: pas, volum, câștig, filtru — sau squelch / NR când sunt selectate în meniu
  int x = 0;
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
