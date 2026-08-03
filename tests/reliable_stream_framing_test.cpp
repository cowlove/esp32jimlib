#include "reliableStreamFraming.h"

#include <assert.h>
#include <stdint.h>
#include <string>

using std::string;

static uint8_t nextSequence = 0;
static string encode(const string &payload) {
  return ReliableStreamFraming::encode(
      (const uint8_t *)payload.data(), payload.size(), nextSequence++);
}

int main() {
  const char bytes[] = {'t', 'e', 'x', 't', '~', '|', '%', '\n', '\0', 1};
  string original(bytes, sizeof(bytes));
  string frame = encode(original);
  assert(frame[0] == '~');
  assert(frame.find("%7E|%25") != string::npos);
  assert(frame.find("XOXEND") == string::npos);

  string wire;
  string decoded;
  uint8_t sequence;
  for (size_t i = 0; i + 1 < frame.size(); ++i) {
    wire += frame[i];
    assert(!ReliableStreamFraming::extract(wire, decoded, sequence));
  }
  wire += frame.back();
  assert(ReliableStreamFraming::extract(wire, decoded, sequence));
  assert(sequence == 0);
  assert(decoded == original);
  assert(wire.empty());

  // Coalesced transport reads still return exactly one logical packet at a
  // time and leave the next packet buffered.
  wire = encode("first") + encode("second");
  assert(ReliableStreamFraming::extract(wire, decoded, sequence));
  assert(decoded == "first");
  assert(ReliableStreamFraming::extract(wire, decoded, sequence));
  assert(decoded == "second");

  // Losing bytes from a fragmented logical packet rejects that packet. The
  // unescaped start marker of the following frame restores synchronization.
  string large(600, 'A');
  frame = encode(large);
  frame.erase(180, 75);
  wire = frame + encode("recovered");
  assert(ReliableStreamFraming::extract(wire, decoded, sequence));
  assert(decoded == "recovered");

  // A complete frame with corrupted data is rejected without poisoning the
  // valid frame after it.
  frame = encode("corrupt me");
  frame[8] ^= 1;
  wire = frame + encode("still good");
  assert(ReliableStreamFraming::extract(wire, decoded, sequence));
  assert(decoded == "still good");

  // Garbage and heartbeat text outside a frame are harmless.
  wire = "-1 ACK\nnoise" + encode("packet");
  assert(ReliableStreamFraming::extract(wire, decoded, sequence));
  assert(decoded == "packet");

  // Sequence wraps modulo 256 and is integrity protected.
  string wrapped = ReliableStreamFraming::encode(
      (const uint8_t *)"wrap", 4, 0xff);
  wire = wrapped;
  assert(ReliableStreamFraming::extract(wire, decoded, sequence));
  assert(sequence == 0xff && decoded == "wrap");
  wrapped[1] = '0';
  wire = wrapped + encode("after bad sequence");
  assert(ReliableStreamFraming::extract(wire, decoded, sequence));
  assert(decoded == "after bad sequence");

  // The four-digit field imposes a hard escaped-wire-payload ceiling.
  string maximum(65535, 'A');
  assert(!ReliableStreamFraming::encode(
      (const uint8_t *)maximum.data(), maximum.size(), 1).empty());
  maximum += 'A';
  assert(ReliableStreamFraming::encode(
      (const uint8_t *)maximum.data(), maximum.size(), 2).empty());

  return 0;
}
