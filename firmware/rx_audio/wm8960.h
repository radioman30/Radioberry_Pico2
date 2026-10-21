// WM8960 (Waveshare WM8960 Audio HAT): registre prin I2C, codec MASTER de ceas.
// HAT-ul are cristal propriu de 24 MHz pe MCLK -> PLL -> SYSCLK 12,288 MHz -> LRCLK 48 kHz, BCLK 1,536 MHz
// (32 BCLK/cadru, 16 biți/canal, format I2S). RP2350 e slave: citește BCLK/LRCLK, scrie DACDAT, citește ADCDAT.
// De ce master: ceasul I2S al Pico și cristalul codecului ar fi asincrone -> țăcănit periodic în DAC.
// Toate funcțiile scriu pe I2C: se apelează DOAR de pe nucleul 1 (care ține și OLED-ul pe același bus).
#pragma once
#include <Arduino.h>
#include <Wire.h>

#define WM8960_ADDR 0x1A
extern int wm8960_fail_reg, wm8960_fail_code;  // prima scriere eșuată (registru, cod Wire), -1 = niciuna

bool wm8960_probe(TwoWire &w);                       // răspunde la 0x1A?
bool wm8960_init(TwoWire &w);                        // pornire completă: PLL, master I2S, DAC->căști, ADC<-microfoane
bool wm8960_set_hp(TwoWire &w, int reg);             // volum căști, registru 0x30..0x7F (0x79 = 0 dB, 1 dB/pas), 0 = mut
bool wm8960_set_spk(TwoWire &w, int reg);            // volum difuzor, la fel; 0 = difuzor oprit
bool wm8960_set_mic(TwoWire &w, int pga, int boost); // PGA 0..63 (-17,25 + 0,75·pga dB), boost 0..3 (0/13/20/29 dB)
