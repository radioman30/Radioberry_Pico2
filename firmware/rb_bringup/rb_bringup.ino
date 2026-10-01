// rb_bringup — firmware de bring-up Radioberry v2 (CL025) pe Raspberry Pi Pico 2
//
// Etapa: verificarea legăturii cu FPGA-ul, fără DSP. Face:
//   1. încarcă gateware-ul (dacă gateware_cl025.h există, vezi tools/make_gateware_header.py)
//   2. citește versiunea gateware-ului pe SPI  -> primul test că SPI-ul merge
//   3. setează 48 kHz, 1 receptor, frecvența RX1
//   4. citește fluxul IQ cu PIO + DMA și verifică marcajele (meta 0 = I, 1 = Q)
//   5. USB: statistici o dată pe secundă; la cerere, IQ brut int16 pentru tools/iq_fft.py
//
// Nucleul 0: FPGA, SPI, IQ, USB.   Nucleul 1: OLED, encoder, butoane.
// Protocolul și pinii: PROTOCOL.md, pins.h.
//
// Comenzi USB (terminal 115200, linie terminată cu Enter):
//   f <Hz>   frecvența RX1          b   flux IQ binar on/off
//   r        reîncarcă FPGA          s   stare
//   d <n>    divizorul PIO RX (implicit 4)

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pins.h"

#if __has_include("gateware_cl025.h")
#include "gateware_cl025.h"
#define HAVE_GATEWARE 1
#else
#define HAVE_GATEWARE 0
#endif

// ============================================================
// Stare partajată între nuclee
// ============================================================
static volatile uint32_t g_freq_hz   = 7074000;
static volatile bool     g_freq_dirty = true;     // nucleul 1 cere retrimiterea frecvenței
static volatile bool     g_reload_req = false;
static volatile bool     g_bin_stream = false;
static volatile int      g_fpga_state = 0;        // 0 neîncărcat, 1 OK, -1 eroare, 2 fără gateware
static volatile uint8_t  g_gw_major = 0, g_gw_minor = 0, g_gw_fpga = 0, g_gw_status = 0;
static volatile uint32_t g_words_s = 0, g_pairs_s = 0, g_sync_err_s = 0;
static volatile bool     g_synced = false;
static volatile int32_t  g_last_i = 0, g_last_q = 0;
static volatile uint32_t g_rx_clkdiv = 4;

// ============================================================
// 1. Încărcare gateware — passive serial, LSB primul (rb2-load-fpga.c)
// ============================================================
static inline void dclk_pulse() {
  gpio_put(PIN_FPGA_DCLK, 1);
  __asm volatile("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
  gpio_put(PIN_FPGA_DCLK, 0);
  __asm volatile("nop\nnop\nnop\nnop");
}

static int fpga_load() {
#if !HAVE_GATEWARE
  return 2;
#else
  gpio_put(PIN_FPGA_NCONFIG, 0);
  gpio_put(PIN_FPGA_DATA0, 0);
  gpio_put(PIN_FPGA_DCLK, 0);
  delay(10);
  gpio_put(PIN_FPGA_NCONFIG, 1);
  uint32_t t0 = millis();
  while (!gpio_get(PIN_FPGA_NSTATUS)) {
    if (millis() - t0 > 2000) return -1;          // nSTATUS n-a urcat
  }
  for (uint32_t n = 0; n < GATEWARE_LEN; n++) {
    uint8_t b = GATEWARE[n];
    for (int i = 0; i < 8; i++) {
      gpio_put(PIN_FPGA_DATA0, (b >> i) & 1);
      dclk_pulse();
    }
  }
  if (!gpio_get(PIN_FPGA_NSTATUS) || !gpio_get(PIN_FPGA_CONFDONE)) return -1;
  dclk_pulse();                                    // 2 pulsuri de inițializare
  dclk_pulse();
  return 1;
#endif
}

// ============================================================
// 2. Control SPI — 6 octeți: [stare, C0, C1..C4] (protocol HPSDR 1)
//    stare: bit0 running, bit1 CWX, bit2 pa_temp_ok
//    răspuns: [3]&3 = tip FPGA, [4].[5] = versiune gateware
// ============================================================
static const uint8_t RB_STATUS = 0x05;             // running + pa_temp_ok

static void rb_cmd(uint8_t c0, uint32_t data) {
  uint8_t tx[6] = { RB_STATUS, c0, (uint8_t)(data >> 24), (uint8_t)(data >> 16),
                    (uint8_t)(data >> 8), (uint8_t)data };
  uint8_t rx[6];
  gpio_put(PIN_SPI_CE0, 0);
  spi_write_read_blocking(RB_SPI, tx, rx, 6);
  gpio_put(PIN_SPI_CE0, 1);
  g_gw_status = rx[0];
  g_gw_fpga   = rx[3] & 0x03;
  g_gw_major  = rx[4];
  g_gw_minor  = rx[5];
}

static void rb_configure() {
  rb_cmd(0x00, 0x00000000);          // C0=0: 48 kHz (C1[1:0]=0), 1 receptor (C4[5:3]=0)
  rb_cmd(0x02, g_freq_hz);           // adresa 1: frecvența TX (ținută egală cu RX)
  rb_cmd(0x04, g_freq_hz);           // adresa 2: frecvența RX1
}

// ============================================================
// 3. RX IQ: PIO (programul PA3GSB, rb2-rx-stream.c) + DMA ping-pong
// ============================================================
static const uint16_t rx_iq_program_instr[] = {
  0x000f,  //  0: jmp    15
  0xbb42,  //  1: nop           side 1 [3]
  0x5004,  //  2: in pins, 4    side 0
  0xbb42, 0x5004, 0xbb42, 0x5004, 0xbb42, 0x5004,
  0xbb42, 0x5004, 0xbb42, 0x5004, 0xbb42, 0x5004,   // 3..14: încă 6 nibble-uri
  0xba42,  // 15: nop           side 1 [2]
  0xb242,  // 16: nop           side 0 [2]
  0x00c1,  // 17: jmp pin, 1    (RDY=1 -> citește un cuvânt de 28 biți)
};
static const struct pio_program rx_iq_program = {
  .instructions = rx_iq_program_instr,
  .length = 18,
  .origin = -1,
};

static PIO  rx_pio = pio0;
static uint rx_sm, rx_off;
static int  dma_ch[2];
#define DMA_WORDS 2048
static uint32_t dma_buf[2][DMA_WORDS];
static volatile uint32_t buf_ready_mask = 0;       // bit b = bufferul b e plin
static volatile uint32_t dma_overruns = 0;

static void dma_irq_handler() {
  for (int b = 0; b < 2; b++) {
    if (dma_channel_get_irq1_status(dma_ch[b])) {
      dma_channel_acknowledge_irq1(dma_ch[b]);
      if (buf_ready_mask & (1u << b)) dma_overruns++;  // bufferul nu fusese procesat
      buf_ready_mask |= (1u << b);
      // re-armează canalul terminat (pornește când îl înlănțuie celălalt)
      dma_channel_set_write_addr(dma_ch[b], dma_buf[b], false);
      dma_channel_set_trans_count(dma_ch[b], DMA_WORDS, false);
    }
  }
}

static bool rx_start() {
  if (!pio_can_add_program(rx_pio, &rx_iq_program)) { Serial.println("[RX] PIO: fara loc pt. program"); return false; }
  int sm = pio_claim_unused_sm(rx_pio, false);
  if (sm < 0) { Serial.println("[RX] PIO: niciun state machine liber"); return false; }
  rx_sm  = (uint)sm;
  rx_off = pio_add_program(rx_pio, &rx_iq_program);

  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_wrap(&c, rx_off + 0, rx_off + 17);
  sm_config_set_sideset(&c, 2, true, false);       // 1 bit + „opt"
  sm_config_set_sideset_pins(&c, PIN_RX_CLK);
  sm_config_set_in_pins(&c, PIN_RX_D0);
  sm_config_set_jmp_pin(&c, PIN_RX_RDY);
  sm_config_set_in_shift(&c, false, true, 28);     // stânga, autopush la 28 biți
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
  sm_config_set_clkdiv_int_frac(&c, g_rx_clkdiv, 0);

  pio_gpio_init(rx_pio, PIN_RX_CLK);
  pio_sm_set_consecutive_pindirs(rx_pio, rx_sm, PIN_RX_CLK, 1, true);
  for (int p = PIN_RX_D0; p <= PIN_RX_D3; p++) pio_gpio_init(rx_pio, p);
  pio_sm_set_consecutive_pindirs(rx_pio, rx_sm, PIN_RX_D0, 4, false);
  gpio_init(PIN_RX_RDY);
  gpio_set_dir(PIN_RX_RDY, GPIO_IN);

  pio_sm_init(rx_pio, rx_sm, rx_off, &c);

  for (int b = 0; b < 2; b++) {
    dma_ch[b] = dma_claim_unused_channel(false);
    if (dma_ch[b] < 0) { Serial.println("[RX] DMA: niciun canal liber"); return false; }
  }
  for (int b = 0; b < 2; b++) {
    dma_channel_config dc = dma_channel_get_default_config(dma_ch[b]);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, false);
    channel_config_set_write_increment(&dc, true);
    channel_config_set_dreq(&dc, pio_get_dreq(rx_pio, rx_sm, false));
    channel_config_set_chain_to(&dc, dma_ch[b ^ 1]);
    dma_channel_configure(dma_ch[b], &dc, dma_buf[b], &rx_pio->rxf[rx_sm], DMA_WORDS, false);
    dma_channel_set_irq1_enabled(dma_ch[b], true);
  }
  irq_add_shared_handler(DMA_IRQ_1, dma_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
  irq_set_enabled(DMA_IRQ_1, true);
  dma_channel_start(dma_ch[0]);
  pio_sm_set_enabled(rx_pio, rx_sm, true);
  return true;
}

// ============================================================
// 4. Procesarea cuvintelor: meta = biții 27..24, eșantion = 23..0 (cu semn)
// ============================================================
static uint32_t cnt_words = 0, cnt_pairs = 0, cnt_sync_err = 0;
static bool synced = false;
static uint8_t expect = 0;
static int32_t cur_i = 0;
static int16_t usb_out[512];
static int usb_n = 0;

static void process_buffer(const uint32_t *w, int n) {
  for (int k = 0; k < n; k++) {
    uint32_t v = w[k];
    uint8_t meta = (v >> 24) & 0x0F;
    int32_t s = (int32_t)(v << 8) >> 8;           // extensie de semn pe 24 biți
    cnt_words++;
    if (!synced) {
      if (meta != 0) continue;
      synced = true;
      expect = 0;
    }
    if (meta != expect) { synced = false; cnt_sync_err++; continue; }
    if (expect == 0) {
      cur_i = s;
    } else {
      cnt_pairs++;
      g_last_i = cur_i;
      g_last_q = s;
      if (g_bin_stream) {
        usb_out[usb_n++] = (int16_t)(cur_i >> 8);
        usb_out[usb_n++] = (int16_t)(s >> 8);
        if (usb_n >= 512) {
          Serial.write((const uint8_t *)usb_out, sizeof(usb_out));
          usb_n = 0;
        }
      }
    }
    expect ^= 1;                                   // 1 receptor: I, Q, I, Q ...
  }
  g_synced = synced;
}

// ============================================================
// USB: comenzi text
// ============================================================
static char cmd_line[48];
static int cmd_len = 0;

static void print_status() {
  static const char *FPGA_STATE[] = { "neincarcat", "OK", "fara gateware" };
  int st = g_fpga_state;
  Serial.printf("FPGA: %s | gateware %u.%u tip %u stare 0x%02X | RX %lu Hz | div %lu\n",
                st < 0 ? "EROARE" : FPGA_STATE[st], g_gw_major, g_gw_minor, g_gw_fpga,
                g_gw_status, (unsigned long)g_freq_hz, (unsigned long)g_rx_clkdiv);
}

static void handle_command(const char *s) {
  if (s[0] == 'f') { g_freq_hz = strtoul(s + 1, nullptr, 10); g_freq_dirty = true; }
  else if (s[0] == 'b') { g_bin_stream = !g_bin_stream; usb_n = 0; }
  else if (s[0] == 'r') { g_reload_req = true; }
  else if (s[0] == 'd') {
    uint32_t d = strtoul(s + 1, nullptr, 10);
    if (d >= 1 && d <= 255) { g_rx_clkdiv = d; pio_sm_set_clkdiv_int_frac(rx_pio, rx_sm, d, 0); }
  }
  if (!g_bin_stream) print_status();
}

static void poll_usb_commands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (cmd_len) { cmd_line[cmd_len] = 0; handle_command(cmd_line); cmd_len = 0; }
    } else if (cmd_len < (int)sizeof(cmd_line) - 1) {
      cmd_line[cmd_len++] = c;
    }
  }
}

static void do_fpga_bringup() {
  g_fpga_state = fpga_load();
  if (g_fpga_state == 1 || g_fpga_state == 2) rb_configure();
}

// ============================================================
// Nucleul 0
// ============================================================
void setup() {
  Serial.begin(115200);
  uint32_t t_usb = millis();
  while (!Serial && millis() - t_usb < 3000) delay(10);   // lasă timp terminalului
  Serial.printf("\nrb_bringup pe %s\n", RB_BOARD_NAME);

  gpio_init(PIN_FPGA_NCONFIG);  gpio_set_dir(PIN_FPGA_NCONFIG, GPIO_OUT); gpio_put(PIN_FPGA_NCONFIG, 1);
  gpio_init(PIN_FPGA_DATA0);    gpio_set_dir(PIN_FPGA_DATA0, GPIO_OUT);
  gpio_init(PIN_FPGA_DCLK);     gpio_set_dir(PIN_FPGA_DCLK, GPIO_OUT);
  gpio_init(PIN_FPGA_NSTATUS);  gpio_set_dir(PIN_FPGA_NSTATUS, GPIO_IN);  gpio_pull_down(PIN_FPGA_NSTATUS);
  gpio_init(PIN_FPGA_CONFDONE); gpio_set_dir(PIN_FPGA_CONFDONE, GPIO_IN); gpio_pull_down(PIN_FPGA_CONFDONE);

  spi_init(RB_SPI, 1000000);                         // 1 MHz la început (PA3GSB: 10 MHz)
  spi_set_format(RB_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
  gpio_set_function(PIN_SPI_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SPI_SCK,  GPIO_FUNC_SPI);
  gpio_set_function(PIN_SPI_MOSI, GPIO_FUNC_SPI);
  gpio_init(PIN_SPI_CE0); gpio_set_dir(PIN_SPI_CE0, GPIO_OUT); gpio_put(PIN_SPI_CE0, 1);
  gpio_init(PIN_SPI_CE1); gpio_set_dir(PIN_SPI_CE1, GPIO_OUT); gpio_put(PIN_SPI_CE1, 1);

  Serial.println("[1] FPGA: incarc gateware...");
  do_fpga_bringup();
  Serial.printf("[1] FPGA: stare %d (1=OK, -1=eroare: nSTATUS/CONF_DONE, 2=fara gateware)\n", g_fpga_state);
  g_freq_dirty = false;
  Serial.println("[2] RX: pornesc PIO + DMA...");
  Serial.println(rx_start() ? "[2] RX: pornit" : "[2] RX: EROARE");
  print_status();
}

void loop() {
  static uint32_t t_stat = 0, t_keepalive = 0;
  static uint8_t keep_idx = 0;

  // buffere DMA pline
  for (int b = 0; b < 2; b++) {
    if (buf_ready_mask & (1u << b)) {
      process_buffer(dma_buf[b], DMA_WORDS);
      noInterrupts(); buf_ready_mask &= ~(1u << b); interrupts();
    }
  }

  poll_usb_commands();

  if (g_reload_req) { g_reload_req = false; do_fpga_bringup(); }
  if (g_freq_dirty) { g_freq_dirty = false; rb_cmd(0x02, g_freq_hz); rb_cmd(0x04, g_freq_hz); }

  // comenzile se retrimit ciclic, ca în firmware-ul PA3GSB (una per pachet RX)
  uint32_t now = millis();
  if (now - t_keepalive >= 100 && g_fpga_state > 0) {
    t_keepalive = now;
    if (keep_idx == 0) rb_cmd(0x00, 0x00000000); else rb_cmd(0x04, g_freq_hz);
    keep_idx ^= 1;
  }

  if (now - t_stat >= 1000) {
    t_stat = now;
    g_words_s = cnt_words; g_pairs_s = cnt_pairs; g_sync_err_s = cnt_sync_err;
    cnt_words = cnt_pairs = cnt_sync_err = 0;
    if (!g_bin_stream) {
      Serial.printf("cuvinte/s %lu | perechi IQ/s %lu | erori sync %lu | overrun %lu | %s | I %ld Q %ld\n",
                    (unsigned long)g_words_s, (unsigned long)g_pairs_s, (unsigned long)g_sync_err_s,
                    (unsigned long)dma_overruns, g_synced ? "SYNC" : "fara sync",
                    (long)g_last_i, (long)g_last_q);
    }
  }
}

// ============================================================
// Nucleul 1: OLED + encoder + butoane
// ============================================================
U8G2_SH1106_128X64_NONAME_F_2ND_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

static volatile int32_t enc_delta = 0;
static const uint32_t STEPS[] = { 10, 100, 1000, 10000, 100000 };
static int step_idx = 2;

static void enc_isr() {
  static uint8_t prev = 0;
  static int8_t acc = 0;
  static const int8_t TAB[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };
  uint8_t cur = (gpio_get(PIN_ENC_A) << 1) | gpio_get(PIN_ENC_B);
  acc += TAB[(prev << 2) | cur];
  prev = cur;
  if (acc >= 4)  { enc_delta++; acc = 0; }        // un pas = un ciclu complet (4 tranziții)
  if (acc <= -4) { enc_delta--; acc = 0; }
}

static bool button_pressed(int pin, uint32_t &t_last, bool &was_down) {
  bool down = !digitalRead(pin);
  uint32_t now = millis();
  bool edge = false;
  if (down && !was_down && now - t_last > 30) { edge = true; t_last = now; }
  if (down != was_down) was_down = down;
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
    int64_t f = (int64_t)g_freq_hz + (int64_t)d * STEPS[step_idx];
    if (f < 10000) f = 10000;
    if (f > 30000000) f = 30000000;
    g_freq_hz = (uint32_t)f;
    g_freq_dirty = true;
  }
  int sw = enc_sw_event();
  if (sw == 1) step_idx = (step_idx + 1) % 5;
  if (button_pressed(PIN_BTN1, t_b1, b1_d))   g_bin_stream = !g_bin_stream;
  if ((PIN_BTN2 >= 0 && button_pressed(PIN_BTN2, t_b2, b2_d)) || (PIN_BTN2 < 0 && sw == 2))
    g_reload_req = true;

  uint32_t now = millis();
  if (now - t_draw < 200) return;
  t_draw = now;

  char l[24];
  oled.clearBuffer();
  oled.setFont(u8g2_font_10x20_tf);
  uint32_t f = g_freq_hz;
  snprintf(l, sizeof(l), "%2lu.%03lu.%02lu", (unsigned long)(f / 1000000),
           (unsigned long)(f / 1000 % 1000), (unsigned long)(f % 1000 / 10));
  oled.drawStr(0, 18, l);
  oled.setFont(u8g2_font_6x10_tf);
  snprintf(l, sizeof(l), "pas %lu Hz", (unsigned long)STEPS[step_idx]);
  oled.drawStr(0, 32, l);
  int st = g_fpga_state;
  snprintf(l, sizeof(l), "FPGA %s gw %u.%u", st == 1 ? "OK" : st == 2 ? "lipsa" : st < 0 ? "ERR" : "-",
           g_gw_major, g_gw_minor);
  oled.drawStr(0, 44, l);
  snprintf(l, sizeof(l), "%lu IQ/s %s", (unsigned long)g_pairs_s, g_synced ? "SYNC" : "nosync");
  oled.drawStr(0, 54, l);
  snprintf(l, sizeof(l), "err %lu%s", (unsigned long)g_sync_err_s, g_bin_stream ? "  USB-IQ" : "");
  oled.drawStr(0, 64, l);
  oled.sendBuffer();
}
