// audio_test — test PCM5102A + căști pe Raspberry Pi Pico 2, fără Radioberry
//
// Verifică DAC-ul, cablajul I2S, OLED-ul, encoderul și butoanele înainte de partea radio.
// Pinii sunt cei din pins.h (aceiași ca în rb_bringup): I2S DATA GP6, BCK GP7, LRCK GP8.
//
// Moduri (BUTON 1 trece la următorul):
//   1 TON STEREO   sinus pe ambele canale
//   2 TON STANGA   sinus doar pe stânga   } verifică că L/R nu sunt inversate
//   3 TON DREAPTA  sinus doar pe dreapta  }
//   4 BALEIAJ      100 Hz -> 10 kHz în 5 s, logaritmic (urechea + răspunsul căștilor)
//   5 CW           ton manipulat „VVV" în Morse, ca un semnal CW
//   6 LINISTE      zero digital (zgomotul de fond al DAC-ului)
// Encoder = frecvența tonului; apăsare encoder = pasul (10/100/1000 Hz);
// BUTON 2 = volumul (-40/-30/-20/-12/-6 dB); pe RP2350-PiZero = apăsare LUNGĂ pe encoder. Pornește la -30 dB: PCM5102A dă 2,1 Vrms
// la maxim, prea tare direct în căști.
//
// USB (115200): stare o dată pe secundă + comenzi  m<n> mod, f<Hz> frecvență, v<dB> volum.

#include <Arduino.h>
#include <Wire.h>
#include <I2S.h>
#include <U8g2lib.h>
#include <math.h>
#include "pins.h"

static const int FS = 48000;

// ---------------- stare partajată între nuclee ----------------
enum { M_STEREO, M_LEFT, M_RIGHT, M_SWEEP, M_CW, M_SILENCE, M_COUNT };
static const char *MODE_NAME[M_COUNT] = { "TON STEREO", "TON STANGA", "TON DREAPTA",
                                          "BALEIAJ", "CW  VVV", "LINISTE" };
static const int VOL_DB[] = { -40, -30, -20, -12, -6 };
static const int VOL_N = sizeof(VOL_DB) / sizeof(VOL_DB[0]);

static volatile int      g_mode = M_STEREO;
static volatile uint32_t g_freq = 1000;
static volatile int      g_vol_idx = 1;            // -30 dB la pornire
static volatile uint32_t g_underflows = 0;
static volatile uint32_t g_sweep_hz = 100;

// ---------------- sinteza (nucleul 0) ----------------
#define SINE_BITS 10
#define SINE_N    (1 << SINE_BITS)
static int16_t sine_tab[SINE_N];

static I2S i2s(OUTPUT);

static inline uint32_t phase_inc(float hz) {
  return (uint32_t)(hz * 4294967296.0f / FS);
}

// Morse „VVV  " cu unitate de 60 ms (≈20 WPM); 1 = ton, 0 = pauză, câte o unitate
static const char *CW_PATTERN = "10101011100010101011100010101011100000000";

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < SINE_N; i++)
    sine_tab[i] = (int16_t)lrintf(32767.0f * sinf(2.0f * (float)M_PI * i / SINE_N));

  i2s.setBCLK(PIN_I2S_BCK);                        // LRCK = BCK + 1 = GP8
  i2s.setDATA(PIN_I2S_DATA);
  i2s.setBitsPerSample(16);
  i2s.setBuffers(6, 256);
  if (!i2s.begin(FS)) {
    while (1) { Serial.println("I2S nu porneste - pini?"); delay(1000); }
  }
}

void loop() {
  static uint32_t phase = 0;
  static uint32_t n = 0;                           // eșantioane de la începutul modului
  static int last_mode = -1;
  static float gain = 0;
  static int last_vol = -1;
  static uint32_t t_stat = 0;
  static float env = 0;                            // anvelopă CW (fără click-uri)

  int mode = g_mode;
  if (mode != last_mode) { last_mode = mode; n = 0; phase = 0; }
  int vi = g_vol_idx;
  if (vi != last_vol) { last_vol = vi; gain = powf(10.0f, VOL_DB[vi] / 20.0f); }

  // umple bufferul I2S cât are loc
  while (i2s.availableForWrite() >= 4) {
    float hz = (float)g_freq;
    float a = 1.0f;                                // amplitudine relativă
    if (mode == M_SWEEP) {
      float t = (float)(n % (5 * FS)) / (5 * FS);  // 0..1 la fiecare 5 s
      hz = 100.0f * powf(100.0f, t);               // 100 Hz .. 10 kHz logaritmic
      if ((n & 1023) == 0) g_sweep_hz = (uint32_t)hz;
    } else if (mode == M_CW) {
      uint32_t unit = (n / (FS * 60 / 1000)) % strlen(CW_PATTERN);
      float target = CW_PATTERN[unit] == '1' ? 1.0f : 0.0f;
      env += (target - env) * 0.0025f;             // ~8 ms rampă
      a = env;
    } else if (mode == M_SILENCE) {
      a = 0.0f;
    }

    phase += phase_inc(hz);
    int16_t s = (int16_t)(sine_tab[phase >> (32 - SINE_BITS)] * gain * a);
    int16_t l = (mode == M_RIGHT) ? 0 : s;
    int16_t r = (mode == M_LEFT)  ? 0 : s;
    i2s.write16(l, r);
    n++;
  }
  if (i2s.getUnderflow()) g_underflows++;

  // comenzi USB
  while (Serial.available()) {
    static char buf[16]; static int len = 0;
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf[len] = 0;
      if (len > 1) {
        long v = atol(buf + 1);
        if (buf[0] == 'm' && v >= 1 && v <= M_COUNT) g_mode = v - 1;
        if (buf[0] == 'f' && v >= 20 && v <= 20000) g_freq = v;
        if (buf[0] == 'v') for (int i = 0; i < VOL_N; i++) if (VOL_DB[i] == v) g_vol_idx = i;
      }
      len = 0;
    } else if (len < 15) buf[len++] = c;
  }

  uint32_t now = millis();
  if (now - t_stat >= 1000) {
    t_stat = now;
    Serial.printf("mod %s | %lu Hz | %d dB | underflow %lu\n", MODE_NAME[g_mode],
                  (unsigned long)(g_mode == M_SWEEP ? g_sweep_hz : g_freq),
                  VOL_DB[g_vol_idx], (unsigned long)g_underflows);
  }
}

// ---------------- UI (nucleul 1): OLED + encoder + butoane ----------------
U8G2_SH1106_128X64_NONAME_F_2ND_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
static volatile int32_t enc_delta = 0;
static const uint32_t STEPS[] = { 10, 100, 1000 };
static int step_idx = 1;

static void enc_isr() {
  static uint8_t prev = 0;
  static int8_t acc = 0;
  static const int8_t TAB[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };
  uint8_t cur = (gpio_get(PIN_ENC_A) << 1) | gpio_get(PIN_ENC_B);
  acc += TAB[(prev << 2) | cur];
  prev = cur;
  if (acc >= 4)  { enc_delta++; acc = 0; }
  if (acc <= -4) { enc_delta--; acc = 0; }
}

static bool pressed(int pin, uint32_t &t_last, bool &was_down) {
  bool down = !digitalRead(pin);
  bool edge = down && !was_down && millis() - t_last > 30;
  if (edge) t_last = millis();
  was_down = down;
  return edge;
}


// Apăsare encoder: 1 = scurtă, 2 = lungă (>700 ms). Lunga ține locul BUTONULUI 2 pe plăcile
// fără el (RP2350-PiZero: doar 4 pini liberi pentru UI).
static int enc_sw_event() {
  static bool down = false, long_sent = false;
  static uint32_t t_down = 0;
  bool now_down = !digitalRead(PIN_ENC_SW);
  uint32_t t = millis();
  int ev = 0;
  if (now_down && !down) { t_down = t; long_sent = false; }
  if (now_down && !long_sent && t - t_down > 700) { ev = 2; long_sent = true; }
  if (!now_down && down && !long_sent && t - t_down > 30) ev = 1;
  down = now_down;
  return ev;
}

void setup1() {
  static const int UI_PINS[] = { PIN_ENC_A, PIN_ENC_B, PIN_ENC_SW, PIN_BTN1, PIN_BTN2 };
  for (int p : UI_PINS) if (p >= 0) pinMode(p, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), enc_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), enc_isr, CHANGE);
  Wire1.setSDA(PIN_OLED_SDA);
  Wire1.setSCL(PIN_OLED_SCL);
  Wire1.setClock(400000);
  oled.begin();
}

void loop1() {
  static uint32_t t_draw = 0, t_b1 = 0, t_b2 = 0;
  static bool b1_d = false, b2_d = false;

  int32_t d;
  noInterrupts(); d = enc_delta; enc_delta = 0; interrupts();
  if (d) {
    int32_t f = (int32_t)g_freq + d * (int32_t)STEPS[step_idx];
    g_freq = (uint32_t)constrain(f, 20, 20000);
  }
  int sw = enc_sw_event();
  if (sw == 1) step_idx = (step_idx + 1) % 3;
  if (pressed(PIN_BTN1, t_b1, b1_d))   g_mode = (g_mode + 1) % M_COUNT;
  if ((PIN_BTN2 >= 0 && pressed(PIN_BTN2, t_b2, b2_d)) || (PIN_BTN2 < 0 && sw == 2))
    g_vol_idx = (g_vol_idx + 1) % VOL_N;

  if (millis() - t_draw < 100) return;
  t_draw = millis();
  char l[24];
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(0, 10, "TEST AUDIO " RB_BOARD_NAME);
  oled.setFont(u8g2_font_10x20_tf);
  oled.drawStr(0, 32, MODE_NAME[g_mode]);
  oled.setFont(u8g2_font_6x10_tf);
  snprintf(l, sizeof(l), "%lu Hz  pas %lu",
           (unsigned long)(g_mode == M_SWEEP ? g_sweep_hz : g_freq), (unsigned long)STEPS[step_idx]);
  oled.drawStr(0, 46, l);
  snprintf(l, sizeof(l), "vol %d dB  ufl %lu", VOL_DB[g_vol_idx], (unsigned long)g_underflows);
  oled.drawStr(0, 58, l);
  oled.sendBuffer();
}
