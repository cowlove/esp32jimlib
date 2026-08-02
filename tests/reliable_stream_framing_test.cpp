#include "reliableStreamFraming.h"

#include <assert.h>
#include <stdint.h>
#include <string>

using std::string;

static string encode(const string &payload) {
  return ReliableStreamFraming::encode(
      (const uint8_t *)payload.data(), payload.size());
}

int main() {
  const char bytes[] = {'t', 'e', 'x', 't', '~', '|', '%', '\n', '\0', 1};
  string original(bytes, sizeof(bytes));
  string frame = encode(original);
  assert(frame[0] == '~');
  assert(frame.find("%7E%7C%25") != string::npos);
  assert(frame.find("XOXEND") == string::npos);

  string wire;
  string decoded;
  for (size_t i = 0; i + 1 < frame.size(); ++i) {
    wire += frame[i];
    assert(!ReliableStreamFraming::extract(wire, decoded));
  }
  wire += frame.back();
  assert(ReliableStreamFraming::extract(wire, decoded));
  assert(decoded == original);
  assert(wire.empty());

  // Coalesced transport reads still return exactly one logical packet at a
  // time and leave the next packet buffered.
  wire = encode("first") + encode("second");
  assert(ReliableStreamFraming::extract(wire, decoded));
  assert(decoded == "first");
  assert(ReliableStreamFraming::extract(wire, decoded));
  assert(decoded == "second");

  // Losing bytes from a fragmented logical packet rejects that packet. The
  // unescaped start marker of the following frame restores synchronization.
  string large(600, 'A');
  frame = encode(large);
  frame.erase(180, 75);
  wire = frame + encode("recovered");
  assert(ReliableStreamFraming::extract(wire, decoded));
  assert(decoded == "recovered");

  // A complete frame with corrupted data is rejected without poisoning the
  // valid frame after it.
  frame = encode("corrupt me");
  frame[frame.find('|') + 2] ^= 1;
  wire = frame + encode("still good");
  assert(ReliableStreamFraming::extract(wire, decoded));
  assert(decoded == "still good");

  // Garbage and heartbeat text outside a frame are harmless.
  wire = "-1 ACK\nnoise" + encode("packet");
  assert(ReliableStreamFraming::extract(wire, decoded));
  assert(decoded == "packet");

  return 0;
}
