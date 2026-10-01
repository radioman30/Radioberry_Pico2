// usb_audio — placa apare în Windows ca microfon USB (UAC2, 48 kHz, 16 biți, mono) cu audio demodulat.
// Necesită stiva USB „Adafruit TinyUSB" și flag-urile CFG_TUD_AUDIO_* la compilare (vezi README, „Compilare").
#pragma once
#include <stdint.h>

void usb_audio_begin();              // în setup(), înainte de Serial.begin()
void usb_audio_push12k(float x);     // un eșantion demodulat la 12 kHz, nivel ~±1 (fără volumul căștilor)
bool usb_audio_streaming();          // PC-ul citește acum de pe „microfon"
uint32_t usb_audio_drops();          // eșantioane aruncate (FIFO USB plin)
