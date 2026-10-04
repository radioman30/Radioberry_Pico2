// rb_bringup_pio — test al căii PIO (4 biți + meta) cu gateware-ul din RagchewBerry (WP3DN)
//
// De ce: rb_bringup a mers pe PIO cu gateware-ul din SBC/rpi-5/archive — FPGA-ul se configura, dar
// SPI dădea 0 și RDY nu urca. RagchewBerry (github.com/wp3dn/RagchewBerry) rulează pe același
// hardware (RP2350-PiZero + Radioberry CL025) cu ACELAȘI program PIO și aceiași pini, dar cu
// propriul .rbf. Dacă merge aici, rx_audio poate trece de pe cele 8 linii clasice pe 4 linii PIO
// (se eliberează BCM16/BCM23 pentru I2S) și avem calea spre TX (rb-tx.pio la Juan).
//
// Pași (secvența din main.c RagchewBerry, plus lecțiile din PROTOCOL.md §6):
//   1. ceasul RX jos ÎNAINTE de încărcare, apoi gateware-ul (passive serial, LSB primul)
//   2. 500 ms, apoi citește versiunea pe SPI până e stabilă de 3 ori (ca rb_radio_detect)
//   3. reg 0 cu DUPLEX (0x00000004), frecvențele TX și RX1, retrimise ciclic la 100 ms
//   4. PIO RX (divizor 2, ca la Juan) + DMA ping-pong; verifică meta 0 = I, 1 = Q
//   5. o dată pe secundă: perechi IQ/s, erori de sync, nivel RMS în dBFS
//
// Câștigul LNA (0x14) NU se trimite la pornire — Juan nu-l trimite, iar harta registrelor din
// gateware-ul lui nu e confirmată. Se testează de mână cu 'g<dB>' și se urmărește dBFS-ul.
//
// Comenzi USB (115200, Enter):
//   f<Hz>  frecvența RX1/TX     g<dB>  câștig LNA -12..48 (0x14, ca în rx_audio)
//   b      flux IQ int16 on/off (tools/iq_fft.py)    s  stare    r  reîncarcă FPGA
//   d<n>   divizorul PIO RX     w  cuvinte brute     p  pini + diagnostic încărcare
//   v      5 citiri SPI brute

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <math.h>
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
static volatile uint32_t g_freq_hz    = 7074000;
static volatile bool     g_freq_dirty = false;
static volatile bool     g_reload_req = false;
static volatile bool     g_bin_stream = false;
static volatile int      g_fpga_state = 0;        // 0 neîncărcat, 1 OK, -1 eroare, -2 FPGA absent, 2 fără gateware
static volatile bool     g_detected   = false;    // versiunea citită stabil pe SPI
static volatile uint8_t  g_gw_major = 0, g_gw_minor = 0, g_gw_fpga = 0, g_gw_rx = 0, g_gw_tx = 0;
static volatile uint32_t g_pairs_s = 0, g_sync_err_s = 0;
static volatile bool     g_synced = false;
static volatile float    g_dbfs = -200.0f;
static volatile uint32_t g_rx_clkdiv = 2;
static volatile int      g_gain_db = -100;        // -100 = netrimis

// ============================================================
// 1. Încărcare gateware — passive serial, LSB primul
// ============================================================
static inline void dclk_pulse() {
  gpio_put(PIN_FPGA_DCLK, 1);
  __asm volatile("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
  gpio_put(PIN_FPGA_DCLK, 0);
  __asm volatile("nop\nnop\nnop\nnop");
}

// Pull-up-urile host-ului fac ca nSTATUS/CONF_DONE să citească 1 și fără FPGA; un FPGA alimentat
// le trage însă la 0 cât timp nCONFIG=0 — asta e testul care nu poate fi păcălit.
static int dg_nstatus_low = -1, dg_confdone_low = -1, dg_confdone_before = -1, dg_confdone_after = -1;
static uint32_t dg_nstatus_rise_us = 0, dg_load_ms = 0;

static int fpga_load() {
#if !HAVE_GATEWARE
  return 2;
#else
  uint32_t t_start = millis();
  gpio_set_dir(PIN_FPGA_DATA0, GPIO_OUT);
  gpio_put(PIN_FPGA_NCONFIG, 0);
  gpio_put(PIN_FPGA_DATA0, 0);
  gpio_put(PIN_FPGA_DCLK, 0);
  delay(10);
  dg_nstatus_low  = gpio_get(PIN_FPGA_NSTATUS);    // așteptat 0
  dg_confdone_low = gpio_get(PIN_FPGA_CONFDONE);   // așteptat 0
  gpio_put(PIN_FPGA_NCONFIG, 1);
  uint32_t t0 = micros();
  while (!gpio_get(PIN_FPGA_NSTATUS)) {
    if (micros() - t0 > 2000000) return -1;       // nSTATUS n-a urcat
  }
  dg_nstatus_rise_us = micros() - t0;
  dg_confdone_before = gpio_get(PIN_FPGA_CONFDONE);
  for (uint32_t n = 0; n < GATEWARE_LEN; n++) {
    uint8_t b = GATEWARE[n];
    for (int i = 0; i < 8; i++) {
      gpio_put(PIN_FPGA_DATA0, (b >> i) & 1);
      dclk_pulse();
    }
  }
  dg_confdone_after = gpio_get(PIN_FPGA_CONFDONE);
  dg_load_ms = millis() - t_start;
  if (!gpio_get(PIN_FPGA_NSTATUS) || !gpio_get(PIN_FPGA_CONFDONE)) return -1;
  if (dg_nstatus_low != 0 || dg_confdone_before != 0) return -2;   // nimeni nu trage liniile
  dclk_pulse();                                    // 2 pulsuri de inițializare
  dclk_pulse();
  return 1;
#endif
}

// ============================================================
// 2. Control SPI — 6 octeți: [stare, C0, C1..C4] (HPSDR protocol 1)
//    răspuns: [3] = tip FPGA (biți 1:0), nr. receptoare (5:2), nr. emițătoare (7:6); [4].[5] = versiune
// ============================================================
static const uint8_t  RB_STATUS = 0x05;            // running + pa_temp_ok (ca rb_radio_write_control)
static const uint32_t RB_REG0   = 0x00000004;      // 48 kHz, 1 receptor, DUPLEX (PROTOCOL.md §6)

static void spi_xfer(const uint8_t *tx, uint8_t *rx) {
  gpio_put(PIN_SPI_CE0, 0);
  spi_write_read_blocking(RB_SPI, tx, rx, 6);
  gpio_put(PIN_SPI_CE0, 1);
}

static void rb_cmd(uint8_t c0, uint32_t data) {
  uint8_t tx[6] = { RB_STATUS, c0, (uint8_t)(data >> 24), (uint8_t)(data >> 16),
                    (uint8_t)(data >> 8), (uint8_t)data };
  uint8_t rx[6];
  spi_xfer(tx, rx);
}

// ca rb_radio_detect(50, 3, 100): cadre cu zero până când versiunea (≠ 0, ≠ FF) e aceeași de 3 ori
static bool rb_detect() {
  uint8_t tx[6] = { 0 }, rx[6] = { 0 }, last_major = 0, last_minor = 0;
  unsigned stable = 0;
  for (unsigned n = 0; n < 50 && stable < 3; n++) {
    spi_xfer(tx, rx);
    uint8_t major = rx[4], minor = rx[5];
    if (major != 0 && major != 0xFF && major == last_major && minor == last_minor) stable++;
    else { stable = 0; last_major = major; last_minor = minor; }
    delay(100);
  }
  if (stable < 3) return false;
  g_gw_major = last_major;
  g_gw_minor = last_minor;
  g_gw_fpga  = rx[3] & 0x03;
  g_gw_rx    = (rx[3] & 0x3C) >> 2;
  g_gw_tx    = (rx[3] & 0xC0) >> 6;
  return true;
}

static void rb_set_freq() {
  rb_cmd(0x02, g_freq_hz);                         // adresa 1: TX (ținută egală cu RX)
  rb_cmd(0x04, g_freq_hz);                         // adresa 2: RX1
}

static void rb_set_gain(int db) {                  // ca în rx_audio: adresa 0x0A, 0x40 | (dB + 12)
  rb_cmd(0x14, 0x40u | (uint32_t)(db + 12));
}

// ============================================================
// 3. RX IQ: PIO + DMA ping-pong (programul PA3GSB, identic cu rb-rx.pio din RagchewBerry)
// ============================================================
static const uint16_t rx_iq_program_instr[] = {
  0x000f,  //  0: jmp    15
  0xbb42,  //  1: nop           side 1 [3]
  0x5004,  //  2: in pins, 4    side 0         <- meta
  0xbb42, 0x5004, 0xbb42, 0x5004, 0xbb42, 0x5004,
  0xbb42, 0x5004, 0xbb42, 0x5004, 0xbb42, 0x5004,   // 3..14: 6 nibble-uri = 24 biți eșantion
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
static bool rx_running = false;
static int  dma_ch[2];
#define DMA_WORDS 2048
static uint32_t dma_buf[2][DMA_WORDS];
static volatile uint32_t buf_ready_mask = 0;
static volatile uint32_t dma_overruns = 0;

static void dma_irq_handler() {
  for (int b = 0; b < 2; b++) {
    if (dma_channel_get_irq1_status(dma_ch[b])) {
      dma_channel_acknowledge_irq1(dma_ch[b]);
      if (buf_ready_mask & (1u << b)) dma_overruns++;
      buf_ready_mask |= (1u << b);
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

  pio_sm_set_pins_with_mask(rx_pio, rx_sm, 0, 1u << PIN_RX_CLK);   // ceasul rămâne jos la predare
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
  rx_running = true;
  return true;
}

// ============================================================
// 4. Procesarea cuvintelor: meta = biții 27..24, eșantion = 23..0 (cu semn)
// ============================================================
static uint32_t cnt_pairs = 0, cnt_sync_err = 0;
static double   acc_pow = 0.0;                     // Σ(I² + Q²), normalizat la 2^23
static bool synced = false;
static uint8_t expect = 0;
static int32_t cur_i = 0;
static int16_t usb_out[512];
static int usb_n = 0;

static void process_buffer(const uint32_t *w, int n) {
  for (int k = 0; k < n; k++) {
    uint32_t v = w[k];
    uint8_t meta = (v >> 24) & 0x0F;
    int32_t s = (int32_t)(v << 8) >> 8;            // extensie de semn pe 24 biți
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
      float fi = cur_i * (1.0f / 8388608.0f), fq = s * (1.0f / 8388608.0f);
      acc_pow += fi * fi + fq * fq;
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

static const char *fpga_state_str(int st) {
  switch (st) {
    case 1:  return "OK";
    case 2:  return "fara gateware";
    case -1: return "EROARE (nSTATUS/CONF_DONE)";
    case -2: return "NU RASPUNDE (alimentare/conectare?)";
    default: return "neincarcat";
  }
}

static void print_status() {
  Serial.printf("FPGA: %s | %s gateware %u.%u tip %u RX %u TX %u | %lu Hz | div %lu | LNA %s\n",
                fpga_state_str(g_fpga_state), g_detected ? "detectat" : "NEDETECTAT",
                g_gw_major, g_gw_minor, g_gw_fpga, g_gw_rx, g_gw_tx,
                (unsigned long)g_freq_hz, (unsigned long)g_rx_clkdiv,
                g_gain_db == -100 ? "netrimis" : String(g_gain_db).c_str());
}

static void print_load_diag() {
  Serial.printf("incarcare: cu nCONFIG=0 -> nSTATUS=%d CONF_DONE=%d (asteptat 0 0) | "
                "nSTATUS urca dupa %lu us | CONF_DONE inainte=%d dupa=%d (asteptat 0 -> 1) | %lu ms\n",
                dg_nstatus_low, dg_confdone_low, (unsigned long)dg_nstatus_rise_us,
                dg_confdone_before, dg_confdone_after, (unsigned long)dg_load_ms);
}

static void print_pins() {
  Serial.printf("pini: nSTATUS=%d CONF_DONE=%d nCONFIG=%d | RDY=%d D3..D0=%d%d%d%d | MISO=%d\n",
                gpio_get(PIN_FPGA_NSTATUS), gpio_get(PIN_FPGA_CONFDONE), gpio_get(PIN_FPGA_NCONFIG),
                gpio_get(PIN_RX_RDY), gpio_get(PIN_RX_D3), gpio_get(PIN_RX_D2), gpio_get(PIN_RX_D1),
                gpio_get(PIN_RX_D0), gpio_get(PIN_SPI_MISO));
}

static void spi_probe() {
  for (int k = 0; k < 5; k++) {
    uint8_t tx[6] = { RB_STATUS, 0x00, 0, 0, 0, (uint8_t)RB_REG0 }, rx[6];
    spi_xfer(tx, rx);
    Serial.printf("SPI %d: %02X %02X %02X %02X %02X %02X\n", k, rx[0], rx[1], rx[2], rx[3], rx[4], rx[5]);
    delay(20);
  }
}

static void dump_words() {
  Serial.print("cuvinte RX (meta|esantion): ");
  for (int k = 0; k < 16; k++) {
    uint32_t v = dma_buf[0][k];
    Serial.printf("%X|%06lX ", (unsigned)((v >> 24) & 0xF), (unsigned long)(v & 0xFFFFFF));
  }
  Serial.println();
}

static void handle_command(const char *s) {
  switch (s[0]) {
    case 'p': print_pins(); print_load_diag(); return;
    case 'w': dump_words(); return;
    case 'v': spi_probe(); return;
    case 'f': g_freq_hz = strtoul(s + 1, nullptr, 10); g_freq_dirty = true; break;
    case 'b': g_bin_stream = !g_bin_stream; usb_n = 0; break;
    case 'r': g_reload_req = true; break;
    case 'g': {
      int db = atoi(s + 1);
      if (db >= -12 && db <= 48) { g_gain_db = db; rb_set_gain(db); }
      break;
    }
    case 'd': {
      uint32_t d = strtoul(s + 1, nullptr, 10);
      if (d >= 1 && d <= 255) { g_rx_clkdiv = d; if (rx_running) pio_sm_set_clkdiv_int_frac(rx_pio, rx_sm, d, 0); }
      break;
    }
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
  if (rx_running) pio_sm_set_enabled(rx_pio, rx_sm, false);
  // Ceasul RX pe nivel jos ÎNAINTE de încărcare: gateware-ul numără separat fronturile up/down;
  // un front în plus după reset decalează contoarele (PROTOCOL.md §6).
  gpio_init(PIN_RX_CLK); gpio_set_dir(PIN_RX_CLK, GPIO_OUT); gpio_put(PIN_RX_CLK, 0);
  g_detected = false;
  g_fpga_state = fpga_load();
  if (g_fpga_state == 1) {
    gpio_set_dir(PIN_FPGA_DATA0, GPIO_IN);
    delay(500);                                    // ca la Juan: 500 ms până la prima comandă SPI
    g_detected = rb_detect();
  }
  if (g_fpga_state == 1 || g_fpga_state == 2) {
    rb_cmd(0x00, RB_REG0);
    delay(2);
    rb_set_freq();
    if (g_gain_db != -100) rb_set_gain(g_gain_db);
  }
  if (rx_running) {                                // la reîncărcare: PIO-ul reia ceasul
    pio_gpio_init(rx_pio, PIN_RX_CLK);
    pio_sm_clear_fifos(rx_pio, rx_sm);
    pio_sm_restart(rx_pio, rx_sm);
    pio_sm_exec(rx_pio, rx_sm, pio_encode_jmp(rx_off));
    pio_sm_set_enabled(rx_pio, rx_sm, true);
  }
}

// ============================================================
// Nucleul 0
// ============================================================
void setup() {
  Serial.begin(115200);
  uint32_t t_usb = millis();
  while (!Serial && millis() - t_usb < 3000) delay(10);
  Serial.printf("\nrb_bringup_pio pe %s\n", RB_BOARD_NAME);
#if HAVE_GATEWARE
  Serial.printf("gateware: %lu octeti, sha256 %.16s...\n", (unsigned long)GATEWARE_LEN, GATEWARE_SHA256);
#else
  Serial.println("gateware: LIPSA (tools/make_gateware_header.py ... rb_bringup_pio)");
#endif

  gpio_init(PIN_FPGA_NCONFIG);  gpio_set_dir(PIN_FPGA_NCONFIG, GPIO_OUT); gpio_put(PIN_FPGA_NCONFIG, 1);
  gpio_init(PIN_FPGA_DATA0);    gpio_set_dir(PIN_FPGA_DATA0, GPIO_OUT);
  gpio_init(PIN_FPGA_DCLK);     gpio_set_dir(PIN_FPGA_DCLK, GPIO_OUT);
  gpio_init(PIN_FPGA_NSTATUS);  gpio_set_dir(PIN_FPGA_NSTATUS, GPIO_IN);  gpio_pull_up(PIN_FPGA_NSTATUS);
  gpio_init(PIN_FPGA_CONFDONE); gpio_set_dir(PIN_FPGA_CONFDONE, GPIO_IN); gpio_pull_up(PIN_FPGA_CONFDONE);

  spi_init(RB_SPI, 10000000);                      // 10 MHz, ca radio_spi.c al lui Juan
  spi_set_format(RB_SPI, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);   // mod 3
  gpio_set_function(PIN_SPI_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SPI_SCK,  GPIO_FUNC_SPI);
  gpio_set_function(PIN_SPI_MOSI, GPIO_FUNC_SPI);
  gpio_init(PIN_SPI_CE0); gpio_set_dir(PIN_SPI_CE0, GPIO_OUT); gpio_put(PIN_SPI_CE0, 1);
  gpio_init(PIN_SPI_CE1); gpio_set_dir(PIN_SPI_CE1, GPIO_OUT); gpio_put(PIN_SPI_CE1, 1);

  Serial.println("[1] FPGA: incarc gateware...");
  do_fpga_bringup();
  Serial.printf("[1] FPGA: %s\n", fpga_state_str(g_fpga_state));
  print_load_diag();
  if (g_fpga_state == 1)
    Serial.printf("[2] SPI: %s\n", g_detected ? "versiune stabila" : "versiunea NU s-a stabilizat (vezi 'v')");
  print_pins();
  Serial.println("[3] RX: pornesc PIO + DMA...");
  Serial.println(rx_start() ? "[3] RX: pornit" : "[3] RX: EROARE");
  print_status();
}

void loop() {
  static uint32_t t_stat = 0, t_keepalive = 0;
  static uint8_t keep_idx = 0;

  for (int b = 0; b < 2; b++) {
    if (buf_ready_mask & (1u << b)) {
      process_buffer(dma_buf[b], DMA_WORDS);
      noInterrupts(); buf_ready_mask &= ~(1u << b); interrupts();
    }
  }

  poll_usb_commands();

  if (g_reload_req) { g_reload_req = false; do_fpga_bringup(); }
  if (g_freq_dirty) { g_freq_dirty = false; rb_set_freq(); }

  // comenzile se retrimit ciclic, ca în firmware-ul PA3GSB
  uint32_t now = millis();
  if (now - t_keepalive >= 100 && g_fpga_state > 0) {
    t_keepalive = now;
    if (keep_idx == 0) rb_cmd(0x00, RB_REG0);
    else if (keep_idx == 1) rb_cmd(0x02, g_freq_hz);
    else if (keep_idx == 2) rb_cmd(0x04, g_freq_hz);
    else if (g_gain_db != -100) rb_set_gain(g_gain_db);
    keep_idx = (keep_idx + 1) % 4;
  }

  if (now - t_stat >= 1000) {
    t_stat = now;
    g_pairs_s = cnt_pairs; g_sync_err_s = cnt_sync_err;
    g_dbfs = cnt_pairs ? 10.0f * log10f((float)(acc_pow / cnt_pairs) + 1e-20f) : -200.0f;
    cnt_pairs = cnt_sync_err = 0; acc_pow = 0.0;
    if (!g_bin_stream) {
      Serial.printf("perechi IQ/s %lu | erori sync %lu | overrun %lu | %s | %.1f dBFS\n",
                    (unsigned long)g_pairs_s, (unsigned long)g_sync_err_s,
                    (unsigned long)dma_overruns, g_synced ? "SYNC" : "fara sync", g_dbfs);
    }
  }
}

// ============================================================
// Nucleul 1: OLED + encoder (frecvență, pas) — doar afișaj de stare
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
  if (acc >= 4)  { enc_delta++; acc = 0; }
  if (acc <= -4) { enc_delta--; acc = 0; }
}

// Apăsare encoder: 1 = scurtă (pas), 2 = lungă >700 ms (reîncarcă FPGA)
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
  static uint32_t t_draw = 0;

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
  if (sw == 2) g_reload_req = true;

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
  snprintf(l, sizeof(l), "PIO  pas %lu Hz", (unsigned long)STEPS[step_idx]);
  oled.drawStr(0, 30, l);
  int st = g_fpga_state;
  snprintf(l, sizeof(l), "FPGA %s gw %u.%u", st == 1 ? (g_detected ? "OK" : "nodet") : st == 2 ? "lipsa" :
           st == -2 ? "NU RASP" : st < 0 ? "ERR" : "-", g_gw_major, g_gw_minor);
  oled.drawStr(0, 41, l);
  snprintf(l, sizeof(l), "%lu IQ/s %s", (unsigned long)g_pairs_s, g_synced ? "SYNC" : "nosync");
  oled.drawStr(0, 52, l);
  snprintf(l, sizeof(l), "%.1f dBFS err %lu", g_dbfs, (unsigned long)g_sync_err_s);
  oled.drawStr(0, 63, l);
  oled.sendBuffer();
}
