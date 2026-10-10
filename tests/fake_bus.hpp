// A CanBus that records frames instead of touching hardware, plus a loopback
// harness that wires two Transports together. This is the whole reason the bus
// is an interface: the protocol logic is tested on the host, in milliseconds,
// with no target, no timing flake and no "works on my bench".

#pragma once

#include <cstdint>
#include <vector>

#include "j1939/transport.hpp"

class FakeBus final : public j1939::CanBus {
public:
  bool send(const j1939::CanFrame& frame) override {
    if (fail_after_ >= 0) {
      if (sent_ok_ >= fail_after_) return false;  // simulate a full mailbox
    }
    frames.push_back(frame);
    ++sent_ok_;
    return true;
  }

  // After this many successful sends, start refusing. -1 disables.
  void fail_after(int n) { fail_after_ = n; }
  void clear() { frames.clear(); sent_ok_ = 0; }

  std::vector<j1939::CanFrame> frames;

private:
  int fail_after_ = -1;
  int sent_ok_ = 0;
};

// Collects reassembled messages so tests can assert on the payload.
struct Sink {
  struct Message {
    j1939::pgn_t pgn;
    j1939::addr_t source;
    std::vector<std::uint8_t> data;
  };
  std::vector<Message> messages;

  static void handler(void* ctx, j1939::pgn_t pgn, j1939::addr_t src,
                      const std::uint8_t* data, std::size_t len) {
    auto* self = static_cast<Sink*>(ctx);
    self->messages.push_back(Message{pgn, src, {data, data + len}});
  }
};
