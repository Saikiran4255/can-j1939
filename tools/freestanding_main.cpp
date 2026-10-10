// Compiled with -fno-exceptions -fno-rtti -Werror -Os. Exercises every public
// entry point so nothing is dead-stripped before the symbol check runs.
//
// If this translation unit ends up referencing operator new, malloc or the
// unwinder, the build or tools/check_no_heap.sh fails. That is the point: the
// claim "no dynamic allocation" is enforced by the build, not by a README.

#include <cstdint>

#include "j1939/transport.hpp"

namespace {

class NullBus final : public j1939::CanBus {
public:
  bool send(const j1939::CanFrame& f) override {
    sink ^= f.id ^ f.data[0];
    return true;
  }
  std::uint32_t sink = 0;
};

NullBus g_bus;
j1939::Transport<2> g_transport(g_bus, 0x21);
std::uint8_t g_payload[200];
std::uint32_t g_observed = 0;

void on_message(void* ctx, j1939::pgn_t pgn, j1939::addr_t src,
                const std::uint8_t* data, std::size_t len) {
  (void)ctx;
  g_observed += pgn + src + static_cast<std::uint32_t>(len) + data[0];
}

}  // namespace

int main() {
  g_transport.set_handler(&on_message, nullptr);

  for (std::size_t i = 0; i < sizeof(g_payload); ++i) {
    g_payload[i] = static_cast<std::uint8_t>(i);
  }

  // Short frame, BAM and CMDT paths.
  g_transport.send(0x00FEDA, j1939::kBroadcastAddress, g_payload, 4, 0);
  g_transport.send(0x00FEDA, j1939::kBroadcastAddress, g_payload, 200, 0);
  g_transport.send(0x00FEDA, 0x17, g_payload, 200, 0);

  // Receive path, timers and the identifier codec.
  j1939::CanFrame f;
  f.id = j1939::encode(j1939::make_id(0x00EC00, 0x17, 0x21, 7));
  f.dlc = 8;
  f.data[0] = j1939::kCtrlRts;
  f.data[1] = 20;
  f.data[3] = 3;
  g_transport.on_frame(f, 0);

  for (std::uint32_t t = 0; t < 2000; t += 50) g_transport.tick(t);

  const j1939::CanId id = j1939::decode(0x0CF00400);
  return static_cast<int>((g_bus.sink ^ g_observed ^ id.pgn() ^
                           g_transport.active_sessions()) & 0x01);
}
