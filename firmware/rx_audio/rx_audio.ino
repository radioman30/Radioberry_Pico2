// rx_audio — receptor SSB/CW/AM: Radioberry v2 (CL025) + Waveshare RP2350-PiZero + PCM5102A
//
// Lanț: FPGA (gateware Hermes-Lite 2 radioberry_cl025 73.3, protocol clasic Pi 4) -> IQ 48 kHz
//       -> decimare /4 (12 kHz) -> filtru complex trece-bandă -> demodulare -> AGC -> I2S 12 kHz.
// Protocol și capcane: PROTOCOL.md §6. Doar pe RP2350-PiZero (Radioberry înfipt în header).
//
// UI: encoder = acord; apăsare scurtă = pasul (10 Hz..100 kHz); apăsare lungă = modul (USB/LSB/CW/AM).
// USB (115200): s stare | f<Hz> frecvența | m<0-3> modul | v<0-100> volumul
//
// Ieșirea audio e izolată în audio_out_*(), ca să poată fi înlocuită (ex. CM108AH pe USB).

#include <Arduino.h>
#include <Wire.h>
#include <I2S.h>
#include <U8g2lib.h>
#include <math.h>
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
static volatile int      g_vol = 30;                // 0..100
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

// filtru complex: trece-jos de lățime bw/2 mutat la f0 (f0<0 = banda de jos, pt. LSB)
static void design_bandpass(int mode) {
  float f0, half;
  switch (mode) {
    case MODE_USB: f0 =  1550; half = 1300; break;  // 250..2850 Hz
    case MODE_LSB: f0 = -1550; half = 1300; break;
    case MODE_CW:  f0 =   700; half =  250; break;  // 450..950 Hz
    default:       f0 =     0; half = 4000; break;  // AM ±4 kHz
  }
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
// Ieșire audio (PCM5102A, I2S 12 kHz). De înlocuit aici pentru CM108AH.
// ============================================================
static I2S i2s(OUTPUT);
#define AQ 1024
static int16_t aq[AQ]; static int aq_w = 0, aq_r = 0;

static void audio_out_begin() {
  i2s.setBCLK(PIN_I2S_BCK);
  i2s.setDATA(PIN_I2S_DATA);
  i2s.setBitsPerSample(16);
  i2s.setBuffers(8, 64);
  i2s.begin(FS);
}
static void audio_out_push(float x) {
  if (x > 1) x = 1; if (x < -1) x = -1;
  int n = (aq_w - aq_r + AQ) % AQ;
  if (n >= AQ - 1) { g_audio_drop++; return; }      // ceasul FPGA > ceasul I2S: aruncă
  aq[aq_w] = (int16_t)(x * 32000); aq_w = (aq_w + 1) % AQ;
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
    if (dsp_push(prev_i / 8388608.0f, q / 8388608.0f, &out)) audio_out_push(out);
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
    rb_cmd(0x00, RB_REG0); rb_cmd(0x02, g_freq); rb_cmd(0x04, g_freq);
  }
  g_ui_ready = true;
}

void setup() {
  Serial.begin(115200);
  design_lowpass(h_dec, NDEC, 4500, FS_IN);
  design_bandpass(g_mode); g_mode_dirty = false;
  audio_out_begin();
  radio_start();
}

static void handle_cmd(const char *s) {
  long v = atol(s + 1);
  if (s[0] == 'f' && v >= 10000 && v <= 30000000) { g_freq = v; g_freq_dirty = true; }
  if (s[0] == 'm' && v >= 0 && v < MODE_N)        { g_mode = v; g_mode_dirty = true; }
  if (s[0] == 'v' && v >= 0 && v <= 100)          g_vol = v;
  Serial.printf("FPGA %s gw %u.%u | %lu Hz %s vol %d | IQ %lu/s | S %.1f dBFS | drop %lu under %lu\n",
                g_fpga == 1 ? "OK" : "ERR", g_gw_major, g_gw_minor, (unsigned long)g_freq, MODE_NAME[g_mode],
                g_vol, (unsigned long)g_iq_rate, g_smeter_db, (unsigned long)g_audio_drop, (unsigned long)g_audio_under);
}

void loop() {
  static uint32_t t_keep = 0, t_rate = 0; static int keep = 0;
  static char line[24]; static int len = 0;

  if (g_fpga == 1) rx_poll();
  audio_out_pump();

  if (g_mode_dirty) { g_mode_dirty = false; design_bandpass(g_mode); }
  if (g_freq_dirty && g_fpga == 1) { g_freq_dirty = false; rb_cmd(0x02, g_freq); rb_cmd(0x04, g_freq); }

  uint32_t now = millis();
  if (g_fpga == 1 && now - t_keep >= 100) {          // comenzi retrimise ciclic: reg0, TX, RX1
    t_keep = now;
    if (keep == 0) rb_cmd(0x00, RB_REG0); else if (keep == 1) rb_cmd(0x02, g_freq); else rb_cmd(0x04, g_freq);
    keep = (keep + 1) % 3;
  }
  if (now - t_rate >= 1000) { t_rate = now; g_iq_rate = cnt_iq; cnt_iq = 0; }

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

static void enc_isr() {
  static uint8_t prev = 0; static int8_t acc = 0;
  static const int8_t TAB[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };
  uint8_t cur = (gpio_get(PIN_ENC_A) << 1) | gpio_get(PIN_ENC_B);
  acc += TAB[(prev << 2) | cur]; prev = cur;
  if (acc >= 4) { enc_delta++; acc = 0; }
  if (acc <= -4) { enc_delta--; acc = 0; }
}

void setup1() {
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), enc_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), enc_isr, CHANGE);
  Wire1.setSDA(PIN_OLED_SDA); Wire1.setSCL(PIN_OLED_SCL); Wire1.setClock(400000);
  oled.begin();
}

void loop1() {
  static uint32_t t_draw = 0, t_down = 0; static bool down = false, long_done = false;

  int32_t d; noInterrupts(); d = enc_delta; enc_delta = 0; interrupts();
  if (d) {
    int64_t f = (int64_t)g_freq + (int64_t)d * STEPS[step_idx];
    if (f < 10000) f = 10000; if (f > 30000000) f = 30000000;
    g_freq = (uint32_t)f; g_freq_dirty = true;
  }
  if (g_ui_ready) {                                   // butonul e pe DCLK: doar după configurare
    bool now_down = !gpio_get(PIN_ENC_SW); uint32_t t = millis();
    if (now_down && !down) { t_down = t; long_done = false; }
    if (now_down && !long_done && t - t_down > 700) { g_mode = (g_mode + 1) % MODE_N; g_mode_dirty = true; long_done = true; }
    if (!now_down && down && !long_done && t - t_down > 30) step_idx = (step_idx + 1) % 5;
    down = now_down;
  }

  if (millis() - t_draw < 120) return;
  t_draw = millis();
  char l[24];
  uint32_t f = g_freq;
  oled.clearBuffer();
  oled.setFont(u8g2_font_10x20_tf);
  snprintf(l, sizeof(l), "%2lu.%03lu.%02lu", (unsigned long)(f / 1000000), (unsigned long)(f / 1000 % 1000),
           (unsigned long)(f % 1000 / 10));
  oled.drawStr(0, 18, l);
  oled.setFont(u8g2_font_6x10_tf);
  snprintf(l, sizeof(l), "%s  pas %lu", MODE_NAME[g_mode], (unsigned long)STEPS[step_idx]);
  oled.drawStr(0, 32, l);
  // S-metru: -130..-40 dBFS pe 100 px
  float s = g_smeter_db; int w = (int)((s + 130) * 100 / 90); if (w < 0) w = 0; if (w > 100) w = 100;
  oled.drawFrame(0, 38, 102, 8); oled.drawBox(1, 39, w, 6);
  snprintf(l, sizeof(l), "%4.0f", s); oled.drawStr(104, 46, l);
  snprintf(l, sizeof(l), "%s gw%u.%u vol%d", g_fpga == 1 ? "RB" : "ERR", g_gw_major, g_gw_minor, g_vol);
  oled.drawStr(0, 60, l);
  oled.sendBuffer();
}
