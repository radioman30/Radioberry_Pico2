// pins.h — harta de pini Radioberry_Pico2, pentru două plăci (alese automat la compilare):
//   * Raspberry Pi Pico 2          (rpipico2)                — bring-up cu fire
//   * Waveshare RP2350-PiZero      (waveshare_rp2350_pizero) — Radioberry înfipt în header
//
// Radioberry folosește GPIO-urile BCM ale header-ului Pi (driverul PA3GSB pio-mode, PROTOCOL.md).
// Pe PiZero, maparea header -> GPIO e din schema Waveshare (WIRING.md §0.3); nu e 1:1 la
// SPI (BCM 9/10/11 -> GP12/11/10 = SPI1 hardware), BCM 4/5/12/14/15 -> GP14/15/9/4/5.
// Fișierul e identic în firmware/rb_bringup și firmware/audio_test — ține-le sincronizate.
#pragma once

#if defined(ARDUINO_WAVESHARE_RP2350_PIZERO)
// =====================================================================================
//  Waveshare RP2350-PiZero (RP2350B). Coloana „hdr" = pinul fizic al header-ului J5.
// =====================================================================================
#define RB_BOARD_NAME "RP2350-PiZero"

// Radioberry: gateware (passive serial)                       BCM  hdr
#define PIN_FPGA_NCONFIG  27   // ieșire                         27   13
#define PIN_FPGA_DATA0    13   // ieșire, LSB primul             13   33
#define PIN_FPGA_DCLK     24   // ieșire                         24   18
#define PIN_FPGA_NSTATUS  26   // intrare                        26   37
#define PIN_FPGA_CONFDONE 22   // intrare                        22   15
// Radioberry: flux IQ RX (PIO), D0..D3 consecutivi
#define PIN_RX_D0         18   //                                18   12
#define PIN_RX_D1         19   //                                19   35
#define PIN_RX_D2         20   //                                20   38
#define PIN_RX_D3         21   //                                21   40
#define PIN_RX_RDY        25   // intrare (jmp pin)              25   22
#define PIN_RX_CLK         6   // ieșire (side-set)               6   31
// Radioberry: control — SPI1 hardware pe PiZero
#define RB_SPI          spi1
#define PIN_SPI_MISO      12   //                                 9   21
#define PIN_SPI_CE0        8   // CS manual                       8   24
#define PIN_SPI_SCK       10   //                                11   23
#define PIN_SPI_MOSI      11   //                                10   19
#define PIN_SPI_CE1        7   // ținut sus                       7   26
// (rezervat TX, mai târziu: TX_RDY GP9 = BCM12 hdr 32, TX_DATA GP15 = BCM5 hdr 29,
//  TX_CLK GP14 = BCM4 hdr 7 — nu le folosi pentru altceva)

// Liberi rămași pe header cu Radioberry înfipt: GP0,1,2,3,4,5,16,17,23
// Audio I2S (PCM5102A sau, mai târziu, Teensy) — LRCK = BCK + 1
#define PIN_I2S_BCK       16   //                                16   36
#define PIN_I2S_LRCK      17   //                                17   11
#define PIN_I2S_DATA      23   //                                23   16
// OLED SH1106, I2C1                                            BCM  hdr
#define PIN_OLED_SDA       2   //                                 2    3
#define PIN_OLED_SCL       3   //                                 3    5
// Encoder + buton (doar 4 pini liberi): BUTON 2 = apăsare LUNGĂ pe encoder
#define PIN_ENC_A          4   //                                14    8
#define PIN_ENC_B          5   //                                15   10
#define PIN_ENC_SW         0   // ID_SD la Pi                     0   27
#define PIN_BTN1           1   // ID_SC la Pi                     1   28
#define PIN_BTN2          -1   // nu există -> apăsare lungă ENC_SW

#else
// =====================================================================================
//  Raspberry Pi Pico 2 (RP2350A, 26 GPIO, toți folosiți). „RB" = GPIO BCM Radioberry.
// =====================================================================================
#define RB_BOARD_NAME "Pico 2"

#define PIN_FPGA_NCONFIG   0   // RB 27  ieșire
#define PIN_FPGA_DATA0     1   // RB 13  ieșire (LSB primul)
#define PIN_FPGA_DCLK     22   // RB 24  ieșire
#define PIN_FPGA_NSTATUS  26   // RB 26  intrare
#define PIN_FPGA_CONFDONE 27   // RB 22  intrare

#define PIN_RX_D0          2   // RB 18
#define PIN_RX_D1          3   // RB 19
#define PIN_RX_D2          4   // RB 20
#define PIN_RX_D3          5   // RB 21
#define PIN_RX_RDY         9   // RB 25  intrare (jmp pin)
#define PIN_RX_CLK        13   // RB 6   ieșire (side-set)

#define RB_SPI          spi0
#define PIN_SPI_MISO      16   // RB 9
#define PIN_SPI_CE0       17   // RB 8   CS manual
#define PIN_SPI_SCK       18   // RB 11
#define PIN_SPI_MOSI      19   // RB 10
#define PIN_SPI_CE1       28   // RB 7   ținut sus

#define PIN_I2S_DATA       6
#define PIN_I2S_BCK        7
#define PIN_I2S_LRCK       8   // = BCK + 1

#define PIN_OLED_SDA      14   // I2C1
#define PIN_OLED_SCL      15
#define PIN_ENC_A         10
#define PIN_ENC_B         11
#define PIN_ENC_SW        12
#define PIN_BTN1          20
#define PIN_BTN2          21
#endif
// encoder + butoane: pull-up intern, contact spre GND
