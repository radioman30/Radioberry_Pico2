// pins.h — harta de pini Radioberry_Pico2 pe Raspberry Pi Pico 2 (RP2350A, 26 GPIO utilizabile)
//
// Coloana „RB" = GPIO BCM de pe header-ul Pi, adică pinul pe care îl așteaptă Radioberry
// (din driverul PA3GSB pentru Pi 5, pio-mode — vezi PROTOCOL.md). Pe Pico se leagă cu fire.
// Toți cei 26 de GPIO sunt folosiți; consola e pe USB CDC, nu pe UART.
#pragma once

// ---------- Radioberry: încărcare gateware (passive serial) ----------
#define PIN_FPGA_NCONFIG   0   // RB 27  ieșire
#define PIN_FPGA_DATA0     1   // RB 13  ieșire (LSB primul)
#define PIN_FPGA_DCLK     22   // RB 24  ieșire
#define PIN_FPGA_NSTATUS  26   // RB 26  intrare
#define PIN_FPGA_CONFDONE 27   // RB 22  intrare

// ---------- Radioberry: flux IQ RX (PIO) ----------
// D0..D3 trebuie să fie CONSECUTIVI (in pins, 4 de la IN_BASE)
#define PIN_RX_D0          2   // RB 18
#define PIN_RX_D1          3   // RB 19
#define PIN_RX_D2          4   // RB 20
#define PIN_RX_D3          5   // RB 21
#define PIN_RX_RDY         9   // RB 25  intrare (jmp pin): eșantion disponibil
#define PIN_RX_CLK        13   // RB 6   ieșire (side-set): ceas generat de host

// ---------- Radioberry: control (SPI0, mod 0) ----------
#define PIN_SPI_MISO      16   // RB 9
#define PIN_SPI_CE0       17   // RB 8   comenzi (CS manual)
#define PIN_SPI_SCK       18   // RB 11
#define PIN_SPI_MOSI      19   // RB 10
#define PIN_SPI_CE1       28   // RB 7   canalul TX la PA3GSB; ținut inactiv (sus)

// ---------- Audio: PCM5102A (I2S prin PIO, pentru etapele următoare) ----------
#define PIN_I2S_DATA       6
#define PIN_I2S_BCK        7
#define PIN_I2S_LRCK       8   // = BCK + 1 (convenția pico-extras)

// ---------- UI ----------
#define PIN_OLED_SDA      14   // I2C1, SH1106 1.3" la 0x3C
#define PIN_OLED_SCL      15
#define PIN_ENC_A         10   // encoder: acord
#define PIN_ENC_B         11
#define PIN_ENC_SW        12   // apăsare encoder: pasul de acord
#define PIN_BTN1          20   // bring-up: pornește/oprește fluxul IQ binar pe USB
#define PIN_BTN2          21   // bring-up: reîncarcă FPGA-ul
// encoder + butoane: pull-up intern, contact spre GND
