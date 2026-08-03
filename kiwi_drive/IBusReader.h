#ifndef IBUS_READER_H
#define IBUS_READER_H

// ============================================================
// IBusReader.h — Minimal FlySky iBUS protocol parser
//
// iBUS frame format (32 bytes total, sent ~every 7ms):
//   byte 0      : 0x20  (frame length, fixed)
//   byte 1      : 0x40  (command byte, fixed)
//   bytes 2..29 : 14 channels x 2 bytes, little-endian, 1000-2000us
//   bytes 30-31 : checksum = 0xFFFF - (sum of bytes 0..29)
//
// No external library dependency — just a HardwareSerial.
// ============================================================

#include <Arduino.h>

#define IBUS_FRAME_LEN     32
#define IBUS_MAX_CHANNELS  14

class IBusReader {
  public:
    IBusReader(HardwareSerial &serialPort) : _serial(serialPort) {
      for (int i = 0; i < IBUS_MAX_CHANNELS; i++) _channels[i] = 1500;
      _lastFrameMs = 0;
      _state = 0;
      _idx = 0;
    }

    void begin(unsigned long baud) {
      _serial.begin(baud);
    }

    // Call every loop() iteration — non-blocking, consumes whatever
    // bytes are currently in the UART buffer.
    void update() {
      while (_serial.available()) {
        uint8_t b = _serial.read();

        switch (_state) {
          case 0: // waiting for length byte
            if (b == 0x20) { _buf[0] = b; _idx = 1; _state = 1; }
            break;

          case 1: // waiting for command byte
            if (b == 0x40) { _buf[1] = b; _idx = 2; _state = 2; }
            else { _state = 0; } // resync
            break;

          case 2: // collecting payload + checksum
            _buf[_idx++] = b;
            if (_idx >= IBUS_FRAME_LEN) {
              _parseFrame();
              _state = 0;
              _idx = 0;
            }
            break;
        }
      }
    }

    // Raw channel value, ~1000-2000. Returns 1500 (center) if channel
    // index is out of range or no frame has ever been received.
    uint16_t channel(uint8_t ch) const {
      if (ch >= IBUS_MAX_CHANNELS) return 1500;
      return _channels[ch];
    }

    // True if we haven't seen a valid, checksum-passing frame recently.
    // Use this as a kill-switch condition (lost RC link / receiver unbound).
    bool isFailsafe(unsigned long timeoutMs) const {
      if (_lastFrameMs == 0) return true; // never synced
      return (millis() - _lastFrameMs) > timeoutMs;
    }

  private:
    HardwareSerial &_serial;
    uint8_t  _buf[IBUS_FRAME_LEN];
    uint8_t  _idx;
    uint8_t  _state;
    uint16_t _channels[IBUS_MAX_CHANNELS];
    unsigned long _lastFrameMs;

    void _parseFrame() {
      uint16_t checksum = 0xFFFF;
      for (int i = 0; i < IBUS_FRAME_LEN - 2; i++) checksum -= _buf[i];

      uint16_t rxChecksum = (uint16_t)_buf[30] | ((uint16_t)_buf[31] << 8);
      if (checksum != rxChecksum) return; // corrupt frame — drop silently, keep last good values

      for (int i = 0; i < IBUS_MAX_CHANNELS; i++) {
        _channels[i] = (uint16_t)_buf[2 + i * 2] | ((uint16_t)_buf[3 + i * 2] << 8);
      }
      _lastFrameMs = millis();
    }
};

#endif // IBUS_READER_H
