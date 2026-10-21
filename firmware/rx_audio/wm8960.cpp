// WM8960 — vezi wm8960.h. Secvența de alimentare (anti-pocnet) și calea microfonului după
// pa3gsb/RP2350-SDR-CW-Interface (wm8960.c) și driverul Linux Waveshare (referinte/WM8960-Audio-HAT).
#include "wm8960.h"

enum {
  R_LINVOL = 0x00, R_RINVOL = 0x01, R_LOUT1 = 0x02, R_ROUT1 = 0x03, R_CLOCK1 = 0x04, R_DACCTL1 = 0x05,
  R_IFACE1 = 0x07, R_CLOCK2 = 0x08, R_IFACE2 = 0x09, R_LDAC = 0x0A, R_RDAC = 0x0B, R_RESET = 0x0F,
  R_LADC = 0x15, R_RADC = 0x16, R_ADDCTL2 = 0x18, R_POWER1 = 0x19, R_POWER2 = 0x1A, R_APOP1 = 0x1C,
  R_LINPATH = 0x20, R_RINPATH = 0x21, R_LOUTMIX = 0x22, R_ROUTMIX = 0x25, R_LOUT2 = 0x28, R_ROUT2 = 0x29,
  R_POWER3 = 0x2F, R_CLASSD1 = 0x31, R_PLL1 = 0x34, R_PLL2 = 0x35, R_PLL3 = 0x36, R_PLL4 = 0x37,
};
// POWER1
#define P1_VMID50K  (1u << 7)
#define P1_VMID250K (2u << 7)
#define P1_VREF     (1u << 6)
#define P1_AINL     (1u << 5)
#define P1_AINR     (1u << 4)
#define P1_ADCL     (1u << 3)
#define P1_ADCR     (1u << 2)
#define P1_MICB     (1u << 1)
// POWER2
#define P2_DACL     (1u << 8)
#define P2_DACR     (1u << 7)
#define P2_LOUT1    (1u << 6)
#define P2_ROUT1    (1u << 5)
#define P2_SPKL     (1u << 4)
#define P2_SPKR     (1u << 3)
#define P2_PLL      (1u << 0)
// POWER3
#define P3_LMIC     (1u << 5)
#define P3_RMIC     (1u << 4)
#define P3_LOMIX    (1u << 3)
#define P3_ROMIX    (1u << 2)
// APOP1
#define AP_POBCTRL  (1u << 7)
#define AP_BUFDCOPEN (1u << 4)
#define AP_BUFIOEN  (1u << 3)
#define AP_SOFT_ST  (1u << 2)

#define VU (1u << 8)                                  // „volume update": scris pe dreapta, aplică ambele canale

static bool spk_on = false;
int wm8960_fail_reg = -1, wm8960_fail_code = 0;

static bool wr(TwoWire &w, uint8_t reg, uint16_t v) {
  // cuvânt de 16 biți: adresa în biții 15:9, valoarea în 8:0 (codecul nu se poate citi)
  for (int t = 0; t < 3; t++) {                       // reîncercare: un NACK izolat nu strică pornirea
    w.beginTransmission(WM8960_ADDR);
    w.write((uint8_t)((reg << 1) | ((v >> 8) & 1)));
    w.write((uint8_t)(v & 0xFF));
    int e = w.endTransmission();
    if (e == 0) return true;
    if (wm8960_fail_reg < 0) { wm8960_fail_reg = reg; wm8960_fail_code = e; }
    delay(2);
  }
  return false;
}

static uint16_t power2() { return P2_DACL | P2_DACR | P2_LOUT1 | P2_ROUT1 | P2_PLL | (spk_on ? (P2_SPKL | P2_SPKR) : 0); }

bool wm8960_probe(TwoWire &w) {
  w.beginTransmission(WM8960_ADDR);
  return w.endTransmission() == 0;
}

bool wm8960_init(TwoWire &w) {
  wm8960_fail_reg = -1;
  bool ok = wr(w, R_RESET, 0);
  delay(20);
  ok &= wr(w, R_DACCTL1, 1u << 3);                    // DAC mut pe toată pornirea
  // polarizare cu anti-pocnet: VMID rapid (2×50k), VREF, apoi VMID normal (2×250k)
  ok &= wr(w, R_APOP1, AP_POBCTRL | AP_SOFT_ST | AP_BUFDCOPEN | AP_BUFIOEN);
  ok &= wr(w, R_POWER1, P1_VMID50K);
  delay(100);
  ok &= wr(w, R_POWER1, P1_VMID50K | P1_VREF);
  ok &= wr(w, R_APOP1, AP_BUFIOEN);

  // PLL: MCLK 24 MHz /2 = 12 MHz; R = 8,192 (N 8, K 0x3126E9) -> 98,304 MHz /4 = 24,576; SYSCLKDIV /2 = 12,288 MHz
  ok &= wr(w, R_CLOCK1, 0);                           // ceasul de pe MCLK cât se programează PLL-ul
  ok &= wr(w, R_POWER2, 0);
  ok &= wr(w, R_PLL2, 0x31); ok &= wr(w, R_PLL3, 0x26); ok &= wr(w, R_PLL4, 0xE9);
  ok &= wr(w, R_PLL1, (1u << 5) | (1u << 4) | 8);     // SDM (fracționar) | prescaler /2 | N = 8
  ok &= wr(w, R_POWER2, P2_PLL);
  delay(20);                                          // PLL-ul se prinde în < 10 ms
  ok &= wr(w, R_CLOCK1, (2u << 1) | 1);               // CLKSEL = PLL, SYSCLKDIV /2, ADCDIV = DACDIV = /1 (fs = SYSCLK/256)

  // interfață: master, I2S, 16 biți; BCLK = SYSCLK/8 = 1,536 MHz (32 BCLK pe cadru)
  ok &= wr(w, R_CLOCK2, (7u << 6) | 7);               // DCLKDIV /16 (difuzor clasa D), BCLKDIV /8
  ok &= wr(w, R_IFACE2, 1u << 6);                     // ALRCGPIO: ADC-ul folosește tot DACLRC (HAT-ul scoate un singur LRCLK)
  ok &= wr(w, R_ADDCTL2, 1u << 2);                    // LRCM: LRCLK oprit doar când și ADC și DAC sunt oprite
  ok &= wr(w, R_IFACE1, (1u << 6) | 2);               // MS = master, WL 16 biți, FORMAT = I2S

  // redare: DAC 0 dB -> mixerele de ieșire -> căști
  ok &= wr(w, R_LDAC, 0xFF); ok &= wr(w, R_RDAC, VU | 0xFF);
  ok &= wr(w, R_LOUTMIX, 1u << 8); ok &= wr(w, R_ROUTMIX, 1u << 8);   // LD2LO / RD2RO
  ok &= wr(w, R_CLASSD1, 0);                          // difuzorul pornește oprit
  ok &= wr(w, R_POWER2, power2());

  // înregistrare: LINPUT1/RINPUT1 (microfoanele MEMS ale HAT-ului) -> boost +29 dB -> PGA -> ADC 0 dB
  ok &= wr(w, R_LINPATH, (1u << 8) | (3u << 4) | (1u << 3));          // LMN1 | LMICBOOST 29 dB | LMIC2B
  ok &= wr(w, R_RINPATH, (1u << 8) | (3u << 4) | (1u << 3));
  ok &= wr(w, R_LINVOL, 0x027); ok &= wr(w, R_RINVOL, VU | 0x027);    // PGA +12 dB (ca setarea Waveshare)
  ok &= wr(w, R_LADC, 0xC3);  ok &= wr(w, R_RADC, VU | 0xC3);         // ADC 0 dB
  ok &= wr(w, R_POWER3, P3_LMIC | P3_RMIC | P3_LOMIX | P3_ROMIX);
  ok &= wr(w, R_POWER1, P1_VMID250K | P1_VREF | P1_AINL | P1_AINR | P1_ADCL | P1_ADCR | P1_MICB);

  ok &= wm8960_set_hp(w, 0x73);                       // căști -6 dB la pornire; volumul fin rămâne digital (g_vol)
  ok &= wm8960_set_spk(w, 0);
  delay(5);
  ok &= wr(w, R_DACCTL1, 0);                          // DAC pornit
  return ok;
}

bool wm8960_set_hp(TwoWire &w, int reg) {
  if (reg < 0x30) reg = 0; if (reg > 0x7F) reg = 0x7F;  // sub 0x30 = mut
  return wr(w, R_LOUT1, reg) && wr(w, R_ROUT1, VU | reg);
}

bool wm8960_set_spk(TwoWire &w, int reg) {
  if (reg < 0x30) reg = 0; if (reg > 0x7F) reg = 0x7F;
  spk_on = reg != 0;
  bool ok = wr(w, R_LOUT2, reg) && wr(w, R_ROUT2, VU | reg);
  ok &= wr(w, R_CLASSD1, spk_on ? 0xC0 | 0x37 : 0x37); // SPK_OP_EN L+R (biții 7:6); restul = valoarea implicită
  ok &= wr(w, R_POWER2, power2());
  return ok;
}

bool wm8960_set_mic(TwoWire &w, int pga, int boost) {
  pga = constrain(pga, 0, 63); boost = constrain(boost, 0, 3);
  uint16_t path = (1u << 8) | ((uint16_t)boost << 4) | (1u << 3);
  return wr(w, R_LINPATH, path) && wr(w, R_RINPATH, path) &&
         wr(w, R_LINVOL, pga) && wr(w, R_RINVOL, VU | pga);
}
