// rx_audio — receptor SSB/CW/AM: Radioberry v2 (CL025) + Waveshare RP2350-PiZero + PCM5102A
//
// Lanț: FPGA (gateware Hermes-Lite 2 radioberry_cl025 73.3, protocol clasic Pi 4) -> IQ 48 kHz
//       -> decimare /4 (12 kHz) -> filtru complex trece-bandă -> demodulare -> AGC -> I2S 12 kHz.
// Protocol și capcane: PROTOCOL.md §6. Doar pe RP2350-PiZero (Radioberry înfipt în header).
//
// UI (meniu cu un singur buton): apăsare scurtă = parametrul următor (FRECV > PAS > MOD > FILTRU > BANDA > VOL > GAIN);
//     rotire = schimbă parametrul evidențiat; apăsare lungă sau 6 s fără atingere = înapoi la FRECV.
//     Fiecare bandă își ține minte ultima frecvență și ultimul mod.
// 4 butoane (MOD, BANDA, FILTRU, PAS) pe același pin cu apăsarea encoderului (GP24, pin 18), fără ADC (pe
//     header nu e niciun pin analogic): rețea RC, butonul se recunoaște după timpul de descărcare. WIRING.md §0.6.
//     Rețeaua se detectează singură la pornire; fără ea, pinul 18 rămâne buton simplu, ca înainte.
// Setările (frecvență, mod, pas, filtre, volum, câștig, memoria benzilor) se salvează în flash la 5 s
//     după ultima schimbare și se reîncarcă la pornire.
// USB (115200): s stare | f<Hz> frecvența | m<0-3> modul | v<0-100> volumul | g<-12..48> câștig RX dB | k butoane (diagnostic) | w<Hz> lățimea filtrului | x inversează IQ (LSB<->USB)
//
// Ieșirea audio e izolată în audio_out_*(), ca să poată fi înlocuită (ex. CM108AH pe USB).

#include <Arduino.h>
#include <Wire.h>
#include <I2S.h>
#include <U8g2lib.h>
#include <math.h>
#include <EEPROM.h>
struct Settings;                    // definită mai jos; aici doar pt. prototipurile generate de Arduino
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "gateware_cl025.h"

#if !defined(ARDUINO_WAVESHARE_RP2350_PIZERO)
#error "rx_audio e doar pentru Waveshare RP2350-PiZero (FQBN rp2040:rp2040:waveshare_rp2350_pizero)"
#endif

// ============================================================
// Pini RP2350-PiZero cu Radioberry înfipt (protocol clasic). „hdr" = pinul fizic al header-ului.
// Liberi de FPGA (netlist Radioberry.net): BCM 0,1,2,3,14,15 (+BCM4 intrare FPGA nefolosită,
// +BCM24/DCLK după configurare). INTERZIS: BCM17, BCM21 (ieșiri FPGA pi_cwl/pi_cwr).
// ============================================================
//                                   GP   BCM hdr
#define PIN_FPGA_NCONFIG   27   //  27   27  13
#define PIN_FPGA_DATA0     13   //  13   13  33  (după configurare = linie de date RX)
#define PIN_FPGA_DCLK      24   //  24   24  18  (după configurare = apăsarea encoderului)
#define PIN_FPGA_NSTATUS   26   //  26   26  37
#define PIN_FPGA_CONFDONE  22   //  22   22  15
#define PIN_RX_CLK          6   //   6    6  31
#define PIN_RX_RDY         25   //  25   25  22
static const uint8_t RX_GP[8] = { 23, 20, 19, 18, 16, 13, 9, 15 };   // BCM 23,20,19,18,16,13,12,5
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
enum { MODE_USB, MODE_LSB, MODE_CW, MODE_AM, MODE_N };
static const char *MODE_NAME[MODE_N] = { "USB", "LSB", "CW", "AM" };
static volatile uint32_t g_freq = 7074000;
static volatile int      g_mode = MODE_USB;
// Radioberry dă spectrul în oglindă față de convenția I+jQ (LSB apărea ca USB): Q se neagă la intrare.
static volatile bool     g_iq_inv = true;
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
  gpio_set_dir(PIN_FPGA_DATA0, GPIO_IN);            // BCM13 devine linie de date RX
  gpio_set_dir(PIN_FPGA_DCLK, GPIO_IN);             // BCM24 devine butonul encoderului
  gpio_pull_up(PIN_FPGA_DCLK);
  return 1;
}

// ============================================================
// SPI control: mod 3, 6 octeți [stare, C0, C1..C4]; răspuns: [4].[5] = versiune
// ============================================================
static const uint32_t RB_REG0 = 0x00000004;         // 48 kHz, 1 RX, DUPLEX=1 (altfel RX1 = frecv. TX)
// Câștig RX: adresa 0x0A (C0 = 0x14), cmd_data[6:0] = 0x40 | (dB + 12). La pornire gateware-ul e pe
// 0x40 = -12 dB (minimul) -> fără comanda asta receptorul e practic surd (ad9866ctrl.v).
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
static volatile int g_bw_idx[MODE_N] = { 2, 2, 1, 1 };     // USB/LSB 2,5 kHz, CW 500 Hz, AM 8 kHz

static int bw_count(int mode) {
  return mode == MODE_CW ? 3 : mode == MODE_AM ? 2 : 4;
}
static int bw_hz(int mode) {
  int i = g_bw_idx[mode];
  return mode == MODE_CW ? BW_CW[i] : mode == MODE_AM ? BW_AM[i] : BW_SSB[i];
}
// marginile benzii trecute, în Hz față de frecvența acordată
static void filter_edges(int mode, float *lo, float *hi) {
  float bw = bw_hz(mode);
  switch (mode) {
    case MODE_USB: *lo = 250;         *hi = 250 + bw; break;   // SSB: de la 250 Hz în sus
    case MODE_LSB: *lo = -250 - bw;   *hi = -250;     break;
    case MODE_CW:  *lo = 700 - bw / 2; *hi = 700 + bw / 2; break;
    default:       *lo = -bw / 2;     *hi = bw / 2;   break;   // AM
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
    hb_r[k] = h[k] * cosf(ph);
    hb_i[k] = h[k] * sinf(ph);
  }
}

// un eșantion IQ la 48 kHz -> 0 sau 1 eșantion audio (la 12 kHz)
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
  if (mode == MODE_AM) {
    for (int k = 0; k < NBP; k++) { yr += hb_r[k] * ri[k]; yi += hb_r[k] * rq[k]; }
  } else {
    for (int k = 0; k < NBP; k++) { yr += hb_r[k] * ri[k] - hb_i[k] * rq[k]; yi += hb_r[k] * rq[k] + hb_i[k] * ri[k]; }
  }
  float p = yr * yr + yi * yi;                       // putere în banda filtrului (S-metru)
  sm_acc += p; if (++sm_n >= FS / 10) { g_smeter_db = 10 * log10f(sm_acc / sm_n + 1e-20f); sm_acc = 0; sm_n = 0; }

  float a;
  if (mode == MODE_AM) {
    float env = sqrtf(p);
    a = env - dc_x + 0.995f * dc_y;                  // blocare DC
    dc_x = env; dc_y = a;
  } else {
    a = yr;                                          // SSB/CW: partea reală
  }
  // AGC: atac instant, revenire ~0,5 s
  float m = fabsf(a);
  agc_env = (m > agc_env) ? m : agc_env * 0.99983f;
  if (agc_env < 3e-6f) agc_env = 3e-6f;
  float g = 0.3f / agc_env;
  *out = a * g * (g_vol / 100.0f);
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
// RX clasic: RDY -> 63 eșantioane × 6 octeți (Q anterior hi,mid,lo | I hi,mid,lo)
// ============================================================
static inline uint8_t rx_byte(uint32_t a) {
  return (uint8_t)((((a >> 23) & 1) << 7) | (((a >> 20) & 1) << 6) | (((a >> 19) & 1) << 5) |
                   (((a >> 18) & 1) << 4) | (((a >> 16) & 1) << 3) | (((a >> 13) & 1) << 2) |
                   (((a >>  9) & 1) << 1) |  ((a >> 15) & 1));
}
static uint32_t cnt_iq = 0;

// Captură pentru spectru: nucleul 1 cere un bloc, nucleul 0 copiază 256 de perechi IQ consecutive.
#define FFT_N 256
static float cap_i[FFT_N], cap_q[FFT_N];
static volatile bool cap_req = false, cap_ready = false;
static int cap_n = 0;
static int32_t prev_i = 0;

static void rx_poll() {
  if (!gpio_get(PIN_RX_RDY)) return;
  for (int s = 0; s < 63; s++) {
    uint8_t b[6];
    for (int i = 0; i < 6; i++) {
      gpio_put(PIN_RX_CLK, (i & 1) ? 0 : 1);
      busy_wait_at_least_cycles(15);                 // ~100 ns
      b[i] = rx_byte(gpio_get_all());
    }
    int32_t q = (int32_t)(((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8)) >> 8;
    int32_t i = (int32_t)(((uint32_t)b[3] << 24) | ((uint32_t)b[4] << 16) | ((uint32_t)b[5] << 8)) >> 8;
    // cadrul n aduce Q[n-1] și I[n]: perechea corectă e (I din cadrul anterior, Q din cadrul curent).
    // O decalare I/Q de un eșantion strică suprimarea benzii laterale opuse la SSB.
    float out;
    float fi = prev_i / 8388608.0f, fq = (g_iq_inv ? -q : q) / 8388608.0f;
    if (dsp_push(fi, fq, &out)) audio_out_push(out);
    if (cap_req) {
      cap_i[cap_n] = fi; cap_q[cap_n] = fq;
      if (++cap_n >= FFT_N) { cap_n = 0; __dmb(); cap_req = false; cap_ready = true; }
    }
    prev_i = i;
    cnt_iq++;
  }
}

// ============================================================
// Nucleul 0
// ============================================================
static void radio_start() {
  gpio_init(PIN_RX_CLK); gpio_set_dir(PIN_RX_CLK, GPIO_OUT); gpio_put(PIN_RX_CLK, 0);  // jos ÎNAINTE de încărcare
  gpio_init(PIN_RX_RDY); gpio_set_dir(PIN_RX_RDY, GPIO_IN); gpio_pull_up(PIN_RX_RDY);
  for (int k = 0; k < 8; k++) { gpio_init(RX_GP[k]); gpio_set_dir(RX_GP[k], GPIO_IN); }
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
    delay(200);                                      // ieșirea din reset înainte de primul cadru SPI
    rb_cmd(0x00, RB_REG0); rb_cmd(0x02, g_freq); rb_cmd(0x04, g_freq); rb_cmd(0x14, rb_gain_word());
  }
  g_ui_ready = true;
}

static void settings_load();

void setup() {
  settings_load();
  Serial.begin(115200);
  design_lowpass(h_dec, NDEC, 4500, FS_IN);
  design_bandpass(g_mode); g_mode_dirty = false;
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

static void handle_cmd(const char *s) {
  if (s[0] == 'i') { pin_activity(); return; }
  if (s[0] == 'z') { scan_start(s + 1); return; }
  long v = atol(s + 1);
  if (s[0] == 'f' && v >= 10000 && v <= 30000000) { g_freq = v; g_freq_dirty = true; }
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
  if (s[0] == 'g' && v >= -12 && v <= 48)         { g_gain_db = v; g_gain_dirty = true; }
  Serial.printf("FPGA %s gw %u.%u | %lu Hz %s vol %d gain %+d dB | IQ %lu/s | S %.1f dBFS | drop %lu under %lu\n",
                g_fpga == 1 ? "OK" : "ERR", g_gw_major, g_gw_minor, (unsigned long)g_freq, MODE_NAME[g_mode],
                g_vol, g_gain_db, (unsigned long)g_iq_rate, g_smeter_db, (unsigned long)g_audio_drop, (unsigned long)g_audio_under);
}

void loop() {
  static uint32_t t_keep = 0, t_rate = 0; static int keep = 0;
  static char line[24]; static int len = 0;

  if (g_fpga == 1) rx_poll();
  audio_out_pump();

  if (g_mode_dirty) { g_mode_dirty = false; design_bandpass(g_mode); }
  if (g_freq_dirty && g_fpga == 1) { g_freq_dirty = false; rb_cmd(0x02, g_freq); rb_cmd(0x04, g_freq); }
  if (g_gain_dirty && g_fpga == 1) { g_gain_dirty = false; rb_cmd(0x14, rb_gain_word()); }

  uint32_t now = millis();
  if (g_fpga == 1 && now - t_keep >= 100) {          // comenzi retrimise ciclic: reg0, TX, RX1, câștig
    t_keep = now;
    switch (keep) {
      case 0: rb_cmd(0x00, RB_REG0); break;
      case 1: rb_cmd(0x02, g_freq); break;
      case 2: rb_cmd(0x04, g_freq); break;
      default: rb_cmd(0x14, rb_gain_word()); break;
    }
    keep = (keep + 1) % 4;
  }
  if (now - t_rate >= 1000) { t_rate = now; g_iq_rate = cnt_iq; cnt_iq = 0; }
  scan_tick();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { line[len] = 0; if (len) handle_cmd(line); len = 0; }
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
  uint8_t  bw[MODE_N];
  uint32_t band_f[NBANDS];
  uint8_t  band_mode[NBANDS];
  uint32_t sum;
};
static const uint32_t SET_MAGIC = 0x52425331;          // "RBS1"; schimbă-l când se schimbă structura
static Settings set_saved;

static uint32_t settings_sum(const Settings &x) {
  const uint8_t *b = (const uint8_t *)&x; uint32_t h = 2166136261u;
  for (size_t k = 0; k < offsetof(Settings, sum); k++) h = (h ^ b[k]) * 16777619u;   // FNV-1a
  return h;
}

static void settings_snapshot(Settings &x) {
  memset(&x, 0, sizeof(x));
  x.magic = SET_MAGIC; x.freq = g_freq; x.mode = g_mode; x.step = step_idx;
  x.vol = g_vol; x.gain = g_gain_db;
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

enum { UI_FREQ, UI_STEP, UI_MODE, UI_FILT, UI_BAND, UI_VOL, UI_GAIN, UI_N };
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
      if (key != K_NONE) { t_act = t; if (g_keys_rc) Serial.printf("tasta %s: %lu us\n", KEY_NAME[key], (unsigned long)g_key_us); }
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

  // rândul de jos: pas, volum, câștig
  int x = 0;
  uint32_t st = STEPS[step_idx];
  if (st >= 1000) snprintf(l, sizeof(l), "P%luk", (unsigned long)(st / 1000)); else snprintf(l, sizeof(l), "P%lu", (unsigned long)st);
  x = draw_item(x + 1, 63, l, ui_sel == UI_STEP);
  snprintf(l, sizeof(l), "V%d", g_vol);           x = draw_item(x, 63, l, ui_sel == UI_VOL);
  snprintf(l, sizeof(l), "G%+d", g_gain_db);      x = draw_item(x, 63, l, ui_sel == UI_GAIN);
  int bw = bw_hz(g_mode);
  if (bw >= 1000) snprintf(l, sizeof(l), "F%d.%d", bw / 1000, bw % 1000 / 100); else snprintf(l, sizeof(l), "F%d", bw);
  x = draw_item(x, 63, l, ui_sel == UI_FILT);
  if (g_fpga != 1) oled.drawStr(113, 63, "ERR");
  else if (g_keys_rc) oled.drawStr(118, 63, "RC");   // rețeaua de butoane detectată
  oled.sendBuffer();
}
