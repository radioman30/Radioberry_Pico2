// usb_audio — microfon USB (UAC2) cu audio demodulat, pentru WSJT-X, fldigi, înregistrare.
//
// Ceasul: FPGA-ul dă ~48006 eșantioane/s, USB-ul consumă exact 48000. Endpoint-ul e asincron, iar driverul
// TinyUSB (CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL) trimite 47, 48 sau 49 de eșantioane pe cadru după cât de plin
// e FIFO-ul, deci diferența se absoarbe singură; dacă FIFO-ul se umple, se aruncă eșantioane (contor).
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include "usb_audio.h"

static const uint32_t FS_USB = 48000;
static const int UPS = 4;                            // 12 kHz -> 48 kHz, interpolare liniară

// entitățile din TUD_AUDIO_MIC_ONE_CH_DESCRIPTOR
enum { ENT_INPUT_TERM = 0x01, ENT_FEATURE = 0x02, ENT_CLOCK = 0x04 };

static bool     mute = false;
static int16_t  volume = 0;                          // dB/256, doar raportat (nivelul îl dă receptorul)
static volatile bool streaming = false;
static uint32_t drops = 0;

class MicInterface : public Adafruit_USBD_Interface {
public:
  uint16_t getInterfaceDescriptor(uint8_t, uint8_t *buf, uint16_t bufsize) override {
    const uint16_t len = TUD_AUDIO_MIC_ONE_CH_DESC_LEN;
    if (!buf) return len;
    if (bufsize < len) return 0;
    uint8_t itf = TinyUSBDevice.allocInterface(2);   // control + streaming
    uint8_t ep_in = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
    uint8_t d[] = { TUD_AUDIO_MIC_ONE_CH_DESCRIPTOR(itf, _strid, 2, 16, ep_in, CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX) };
    memcpy(buf, d, len);
    return len;
  }
};
static MicInterface mic;

void usb_audio_begin() {
  if (!TinyUSBDevice.isInitialized()) TinyUSBDevice.begin(0);
  // PID propriu: 000F e PID-ul BOOTSEL al RP2350 și Windows încurca driverele (vezi memoria proiectului)
  TinyUSBDevice.setID(0x2E8A, 0x10F1);
  TinyUSBDevice.setProductDescriptor("Radioberry RX");
  mic.setStringDescriptor("Radioberry RX audio");
  TinyUSBDevice.addInterface(mic);
  if (TinyUSBDevice.mounted()) { TinyUSBDevice.detach(); delay(10); TinyUSBDevice.attach(); }
}

bool usb_audio_streaming() { return streaming; }
uint32_t usb_audio_drops() { return drops; }

void usb_audio_push12k(float x) {
  static float prev = 0;
  static int16_t buf[48];
  static int n = 0;
  if (!streaming) { prev = x; n = 0; return; }
  for (int k = 1; k <= UPS; k++) {
    float y = (prev + (x - prev) * k / UPS) * 32000.0f;
    if (y > 32767) y = 32767; if (y < -32768) y = -32768;
    buf[n++] = mute ? 0 : (int16_t)y;
  }
  prev = x;
  if (n >= 48) {                                     // ~1 ms de audio odată
    uint16_t w = tud_audio_write(buf, n * 2);
    if (w < n * 2) drops += (n * 2 - w) / 2;
    n = 0;
  }
}

// ---------------- callback-uri TinyUSB (cereri de control UAC2) ----------------

bool tud_audio_set_itf_cb(uint8_t, tusb_control_request_t const *p_request) {
  streaming = TU_U16_LOW(p_request->wValue) != 0;    // alt 1 = PC-ul pornește fluxul
  return true;
}

bool tud_audio_set_itf_close_ep_cb(uint8_t, tusb_control_request_t const *) {
  streaming = false;
  return true;
}

bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  uint8_t ctrl = TU_U16_HIGH(p_request->wValue);
  uint8_t entity = TU_U16_HIGH(p_request->wIndex);

  if (entity == ENT_CLOCK) {
    if (ctrl == AUDIO_CS_CTRL_SAM_FREQ) {
      if (p_request->bRequest == AUDIO_CS_REQ_CUR) {
        audio_control_cur_4_t cur = { .bCur = (int32_t)FS_USB };
        // prin tud_audio_buffer_...: driverul reține rata și calculează mărimea pachetelor
        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &cur, sizeof(cur));
      }
      if (p_request->bRequest == AUDIO_CS_REQ_RANGE) {
        audio_control_range_4_n_t(1) rng;
        rng.wNumSubRanges = 1;
        rng.subrange[0].bMin = FS_USB; rng.subrange[0].bMax = FS_USB; rng.subrange[0].bRes = 0;
        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &rng, sizeof(rng));
      }
    }
    if (ctrl == AUDIO_CS_CTRL_CLK_VALID && p_request->bRequest == AUDIO_CS_REQ_CUR) {
      uint8_t valid = 1;
      return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &valid, 1);
    }
    return false;
  }
  if (entity == ENT_INPUT_TERM && ctrl == AUDIO_TE_CTRL_CONNECTOR) {
    audio_desc_channel_cluster_t c;
    c.bNrChannels = 1; c.bmChannelConfig = (audio_channel_config_t)0; c.iChannelNames = 0;
    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &c, sizeof(c));
  }
  if (entity == ENT_FEATURE) {
    if (ctrl == AUDIO_FU_CTRL_MUTE && p_request->bRequest == AUDIO_CS_REQ_CUR) {
      uint8_t m = mute;
      return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &m, 1);
    }
    if (ctrl == AUDIO_FU_CTRL_VOLUME) {
      if (p_request->bRequest == AUDIO_CS_REQ_CUR)
        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &volume, sizeof(volume));
      if (p_request->bRequest == AUDIO_CS_REQ_RANGE) {
        audio_control_range_2_n_t(1) rng;
        rng.wNumSubRanges = 1;
        rng.subrange[0].bMin = -90 * 256; rng.subrange[0].bMax = 0; rng.subrange[0].bRes = 256;
        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &rng, sizeof(rng));
      }
    }
  }
  return false;
}

bool tud_audio_set_req_entity_cb(uint8_t, tusb_control_request_t const *p_request, uint8_t *buf) {
  uint8_t ctrl = TU_U16_HIGH(p_request->wValue);
  uint8_t entity = TU_U16_HIGH(p_request->wIndex);
  if (p_request->bRequest != AUDIO_CS_REQ_CUR) return false;
  if (entity == ENT_FEATURE && ctrl == AUDIO_FU_CTRL_MUTE) { mute = buf[0]; return true; }
  if (entity == ENT_FEATURE && ctrl == AUDIO_FU_CTRL_VOLUME) { memcpy(&volume, buf, 2); return true; }
  if (entity == ENT_CLOCK && ctrl == AUDIO_CS_CTRL_SAM_FREQ) return true;   // rată fixă
  return false;
}
