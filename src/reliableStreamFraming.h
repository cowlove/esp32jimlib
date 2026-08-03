#ifndef RELIABLESTREAMFRAMING_H
#define RELIABLESTREAMFRAMING_H

#include <stdint.h>
#include <stdio.h>
#include <string>

#include "Arduino_CRC32.h"

// Printable, self-synchronizing logical-packet framing:
//
//   ~SSLLLL<escaped-payload>CCCCCCCC
//
// SS is an 8-bit logical-packet sequence number and LLLL is the exact number
// of escaped payload bytes on the wire. '~' and '%' are encoded as %HH; all
// other payload bytes are unchanged. The CRC covers the binary sequence byte,
// the four ASCII length bytes, and the original decoded payload. The maximum
// escaped payload is therefore 65,535 bytes, independent of transport packet
// boundaries.
class ReliableStreamFraming {
public:
  static std::string encode(const uint8_t *payload, size_t payloadBytes,
      uint8_t sequence) {
    std::string escaped;
    escaped.reserve(payloadBytes);
    for (size_t i = 0; i < payloadBytes; ++i) {
      uint8_t c = payload[i];
      if (c == '~' || c == '%') {
        static const char hex[] = "0123456789ABCDEF";
        escaped += '%';
        escaped += hex[c >> 4];
        escaped += hex[c & 0x0f];
      } else {
        escaped += (char)c;
      }
      if (escaped.size() > 0xffff)
        return std::string();
    }

    char header[8];
    snprintf(header, sizeof(header), "~%02X%04X", sequence,
        (unsigned int)escaped.size());

    std::string crcInput(1, (char)sequence);
    crcInput.append(header + 3, 4);
    crcInput.append((const char *)payload, payloadBytes);
    Arduino_CRC32 crc32;
    char trailer[9];
    snprintf(trailer, sizeof(trailer), "%08lX",
        (unsigned long)crc32.calc((const uint8_t *)crcInput.data(),
            crcInput.size()));
    return std::string(header) + escaped + trailer;
  }

  static bool extract(std::string &wire, std::string &payload,
      uint8_t &sequence) {
    payload.clear();
    for (;;) {
      size_t start = wire.find('~');
      if (start == std::string::npos) {
        wire.clear();
        return false;
      }
      if (start != 0)
        wire.erase(0, start);
      if (wire.size() < 7)
        return false;

      uint32_t seq = 0, escapedBytes = 0;
      if (!parseHex(wire.data() + 1, 2, seq) ||
          !parseHex(wire.data() + 3, 4, escapedBytes)) {
        wire.erase(0, 1);
        continue;
      }

      const size_t frameBytes = 7 + (size_t)escapedBytes + 8;
      size_t restart = wire.find('~', 1);
      if (restart != std::string::npos && restart < frameBytes) {
        wire.erase(0, restart);
        continue;
      }
      if (wire.size() < frameBytes)
        return false;

      uint32_t expectedCrc = 0;
      if (!parseHex(wire.data() + 7 + escapedBytes, 8, expectedCrc)) {
        discardCandidate(wire);
        continue;
      }

      std::string decoded;
      if (!decode(wire.data() + 7, escapedBytes, decoded)) {
        discardCandidate(wire);
        continue;
      }

      uint8_t candidateSequence = (uint8_t)seq;
      std::string crcInput(1, (char)candidateSequence);
      crcInput.append(wire.data() + 3, 4);
      crcInput += decoded;
      Arduino_CRC32 crc32;
      uint32_t actualCrc = crc32.calc(
          (const uint8_t *)crcInput.data(), crcInput.size());
      wire.erase(0, frameBytes);
      if (actualCrc != expectedCrc)
        continue;
      sequence = candidateSequence;
      payload.swap(decoded);
      return true;
    }
  }

private:
  static int hexDigit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  }

  static bool parseHex(const char *text, size_t bytes, uint32_t &value) {
    value = 0;
    for (size_t i = 0; i < bytes; ++i) {
      int digit = hexDigit(text[i]);
      if (digit < 0) return false;
      value = (value << 4) | (uint32_t)digit;
    }
    return true;
  }

  static bool decode(const char *encoded, size_t encodedBytes,
      std::string &decoded) {
    decoded.clear();
    decoded.reserve(encodedBytes);
    for (size_t i = 0; i < encodedBytes; ++i) {
      uint8_t c = (uint8_t)encoded[i];
      if (c != '%') {
        if (c == '~') return false;
        decoded += (char)c;
        continue;
      }
      if (i + 2 >= encodedBytes) return false;
      int high = hexDigit(encoded[++i]);
      int low = hexDigit(encoded[++i]);
      if (high < 0 || low < 0) return false;
      uint8_t value = (uint8_t)((high << 4) | low);
      if (value != '~' && value != '%') return false;
      decoded += (char)value;
    }
    return true;
  }

  static void discardCandidate(std::string &wire) {
    size_t restart = wire.find('~', 1);
    if (restart == std::string::npos) wire.clear();
    else wire.erase(0, restart);
  }
};

#endif
