#ifndef RELIABLESTREAMFRAMING_H
#define RELIABLESTREAMFRAMING_H

#include <ctype.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <string>

#include "Arduino_CRC32.h"

// Printable framing added by ReliableStream:
//
//   ~<escaped-payload-byte-count-in-hex>|<escaped-payload>|<8-hex-digit-crc>
//
// '~', '|' and '%' are encoded as %HH in the payload. Other bytes, including
// newlines, NULs and control bytes, are left untouched. The byte count covers
// the escaped wire payload. It makes the frame independent of transport read
// and write boundaries without adding a binary or control-byte delimiter.
class ReliableStreamFraming {
public:
  static std::string encode(const uint8_t *payload, size_t payloadBytes) {
    std::string escaped;
    escaped.reserve(payloadBytes);
    for (size_t i = 0; i < payloadBytes; ++i) {
      uint8_t c = payload[i];
      if (c == '~' || c == '|' || c == '%') {
        static const char hex[] = "0123456789ABCDEF";
        escaped += '%';
        escaped += hex[c >> 4];
        escaped += hex[c & 0x0f];
      } else {
        escaped += (char)c;
      }
    }

    Arduino_CRC32 crc32;
    uint32_t crc = crc32.calc(payload, payloadBytes);
    char header[2 * sizeof(unsigned long) + 3];
    char trailer[11];
    snprintf(header, sizeof(header), "~%lX|", (unsigned long)escaped.size());
    snprintf(trailer, sizeof(trailer), "|%08lX", (unsigned long)crc);
    return std::string(header) + escaped + trailer;
  }

  // Extracts the next complete, valid packet. Invalid candidates are dropped
  // and parsing resumes at the next unescaped '~'. Returns false when no
  // complete valid packet is currently buffered.
  static bool extract(std::string &wire, std::string &payload) {
    payload.clear();
    for (;;) {
      size_t start = wire.find('~');
      if (start == std::string::npos) {
        wire.clear();
        return false;
      }
      if (start != 0)
        wire.erase(0, start);

      size_t separator = wire.find('|', 1);
      size_t restart = wire.find('~', 1);
      if (restart != std::string::npos &&
          (separator == std::string::npos || restart < separator)) {
        wire.erase(0, restart);
        continue;
      }
      if (separator == std::string::npos) {
        // A 32-bit encoded length needs no more than eight hexadecimal digits.
        if (wire.size() > 9)
          wire.erase(0, 1);
        else
          return false;
        continue;
      }
      if (separator == 1 || separator > 9) {
        wire.erase(0, 1);
        continue;
      }

      size_t escapedBytes = 0;
      bool validLength = true;
      for (size_t i = 1; i < separator; ++i) {
        int digit = hexDigit(wire[i]);
        if (digit < 0) {
          validLength = false;
          break;
        }
        escapedBytes = (escapedBytes << 4) | (size_t)digit;
      }
      if (!validLength) {
        wire.erase(0, 1);
        continue;
      }

      const size_t payloadStart = separator + 1;
      if (escapedBytes > SIZE_MAX - payloadStart - 9) {
        discardCandidate(wire);
        continue;
      }
      const size_t payloadEnd = payloadStart + escapedBytes;
      const size_t frameBytes = payloadEnd + 1 + 8;

      // '~' is never emitted inside an encoded frame. Seeing it before the
      // expected end proves that bytes were lost/corrupted, and gives us the
      // next synchronization point immediately.
      restart = wire.find('~', 1);
      if (restart != std::string::npos && restart < frameBytes) {
        wire.erase(0, restart);
        continue;
      }
      if (wire.size() < frameBytes)
        return false;
      if (wire[payloadEnd] != '|') {
        discardCandidate(wire);
        continue;
      }

      uint32_t expectedCrc = 0;
      bool validCrc = true;
      for (size_t i = payloadEnd + 1; i < frameBytes; ++i) {
        int digit = hexDigit(wire[i]);
        if (digit < 0) {
          validCrc = false;
          break;
        }
        expectedCrc = (expectedCrc << 4) | (uint32_t)digit;
      }

      std::string decoded;
      if (!validCrc || !decode(wire.data() + payloadStart, escapedBytes, decoded)) {
        discardCandidate(wire);
        continue;
      }

      Arduino_CRC32 crc32;
      uint32_t actualCrc = crc32.calc(
          (const uint8_t *)decoded.data(), decoded.size());
      wire.erase(0, frameBytes);
      if (actualCrc != expectedCrc)
        continue;
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

  static bool decode(const char *encoded, size_t encodedBytes,
      std::string &decoded) {
    decoded.clear();
    decoded.reserve(encodedBytes);
    for (size_t i = 0; i < encodedBytes; ++i) {
      uint8_t c = (uint8_t)encoded[i];
      if (c != '%') {
        // Reserved bytes can only appear through their escape sequence.
        if (c == '~' || c == '|')
          return false;
        decoded += (char)c;
        continue;
      }
      if (i + 2 >= encodedBytes)
        return false;
      int high = hexDigit(encoded[++i]);
      int low = hexDigit(encoded[++i]);
      if (high < 0 || low < 0)
        return false;
      uint8_t value = (uint8_t)((high << 4) | low);
      if (value != '~' && value != '|' && value != '%')
        return false;
      decoded += (char)value;
    }
    return true;
  }

  static void discardCandidate(std::string &wire) {
    size_t restart = wire.find('~', 1);
    if (restart == std::string::npos)
      wire.clear();
    else
      wire.erase(0, restart);
  }
};

#endif
