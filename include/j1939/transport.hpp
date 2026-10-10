// transport.hpp — SAE J1939-21 multi-packet transport, static allocation only.
//
// Classic CAN carries 8 bytes per frame. J1939 parameter groups can be up to
// 1785 bytes, so anything over 8 is fragmented into 7-byte chunks (the first
// byte of every data frame is the sequence number) and reassembled.
//
// Two protocols, and the difference matters:
//
//   BAM  (Broadcast Announce Message) - fire and forget to the global address.
//        TP.CM with control byte 32, then N data frames with no acknowledgement
//        and no flow control. The receiver has to keep up or lose the message.
//        Spec says 50..200 ms between frames; a sender that bursts faster will
//        silently break conforming receivers.
//
//   CMDT (Connection Mode Data Transfer) - point to point with handshaking.
//        RTS (16) -> CTS (17) -> data frames -> EndOfMsgAck (19). CTS carries
//        how many packets the receiver will accept and where to start, so the
//        receiver controls the rate. Either side can Abort (255) at any time.
//
// Design constraints, because this is meant for firmware:
//   - No heap. Sessions live in a fixed array sized at compile time.
//   - No exceptions, no RTTI. Errors are return values.
//   - No recursion, bounded loops only.
//   - Time is injected as a millisecond counter, so tests run instantly and
//     deterministically instead of sleeping.
//   - The bus is an interface, so the same code runs against a real driver on
//     target and a fake bus on the host.

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "j1939/can_id.hpp"

namespace j1939 {

// ----------------------------------------------------------- protocol numbers

inline constexpr pgn_t kPgnTpCm = 0x00EC00;  // 60416, connection management
inline constexpr pgn_t kPgnTpDt = 0x00EB00;  // 60160, data transfer

inline constexpr std::uint8_t kCtrlRts = 16;
inline constexpr std::uint8_t kCtrlCts = 17;
inline constexpr std::uint8_t kCtrlEndOfMsgAck = 19;
inline constexpr std::uint8_t kCtrlBam = 32;
inline constexpr std::uint8_t kCtrlAbort = 255;

inline constexpr std::size_t kBytesPerDataFrame = 7;
inline constexpr std::size_t kMaxPackets = 255;
inline constexpr std::size_t kMaxMessageBytes = kMaxPackets * kBytesPerDataFrame;  // 1785

// J1939-21 abort reasons. Sent on the wire, so the values are fixed.
enum class AbortReason : std::uint8_t {
  kBusy = 1,                  // already in a session with this node
  kResourcesFreed = 2,        // had resources, lost them
  kTimeout = 3,               // a timer expired
  kUnexpectedDataPacket = 4,  // sequence number out of order
};

enum class Status : std::uint8_t {
  kOk = 0,
  kNoSession,         // nothing matched this frame
  kNoFreeSession,     // static pool exhausted, caller should back off
  kMessageTooLarge,   // over 1785 bytes
  kBadSequence,       // data frame out of order
  kTimeout,
  kAborted,
  kBusError,          // the bus layer refused the frame
};

// Timing from J1939-21 table 10. Named rather than inlined so a deployment
// that has to tolerate a slow gateway can see exactly what it is relaxing.
struct Timeouts {
  std::uint32_t tr_ms = 200;   // Tr: response expected
  std::uint32_t th_ms = 500;   // Th: holding
  std::uint32_t t1_ms = 750;   // T1: between data frames, receiver side
  std::uint32_t t2_ms = 1250;  // T2: after CTS, waiting for first data
  std::uint32_t t3_ms = 1250;  // T3: after RTS, waiting for CTS
  std::uint32_t t4_ms = 1050;  // T4: waiting after a hold
  std::uint32_t bam_gap_ms = 50;  // minimum spacing between BAM data frames
};

// ------------------------------------------------------------- bus interface

// The one thing a platform must provide. On target this wraps the CAN
// peripheral; in tests it records frames in a vector. No virtual destructor
// needed because the transport never owns a bus.
class CanBus {
public:
  // Returns false if the frame could not be queued (mailbox full). The
  // transport treats that as a bus error rather than retrying blindly, so the
  // caller decides the backoff policy.
  virtual bool send(const CanFrame& frame) = 0;

protected:
  ~CanBus() = default;
};

// Called when a complete multi-packet message has been reassembled. A function
// pointer plus context, not std::function: no allocation, no indirect
// construction, and it works with -fno-exceptions.
using MessageHandler = void (*)(void* ctx, pgn_t pgn, addr_t source,
                                const std::uint8_t* data, std::size_t length);

// ------------------------------------------------------------------- session

namespace detail {

enum class SessionState : std::uint8_t {
  kIdle = 0,
  kRxBam,       // receiving broadcast, no flow control
  kRxCmdt,      // receiving point to point, we send CTS
  kTxBamData,   // sending broadcast data frames
  kTxWaitCts,   // sent RTS, waiting for CTS
  kTxCmdtData,  // sending data frames the CTS asked for
  kTxWaitAck,   // sent the last frame, waiting for EndOfMsgAck
};

struct Session {
  SessionState state = SessionState::kIdle;
  pgn_t pgn = 0;
  addr_t peer = kNullAddress;   // the other node
  addr_t local = kNullAddress;  // us
  std::uint16_t total_bytes = 0;
  std::uint8_t total_packets = 0;
  // Deliberately 16-bit although the wire field is 8-bit. At the 1785-byte
  // maximum there are exactly 255 packets, so a uint8_t counter wraps to 0
  // after the last one and the completion test (0 > 255) never fires. The
  // message then hangs until the session times out. Found by the
  // BamHandlesTheMaximumMessageSize test, which is why that test exists.
  std::uint16_t next_seq = 1;       // sequence numbers are 1-based
  std::uint8_t packets_this_burst = 0;  // CMDT: how many the CTS allowed
  std::uint8_t burst_remaining = 0;
  std::uint32_t deadline_ms = 0;
  std::uint8_t buffer[kMaxMessageBytes] = {};

  bool active() const noexcept { return state != SessionState::kIdle; }
  void reset() noexcept {
    state = SessionState::kIdle;
    peer = kNullAddress;
    local = kNullAddress;
    total_bytes = 0;
    total_packets = 0;
    next_seq = 1;
    packets_this_burst = 0;
    burst_remaining = 0;
    deadline_ms = 0;
    // buffer is deliberately not cleared: it is overwritten on use and
    // clearing 1785 bytes per session teardown is wasted cycles on an MCU.
  }
};

}  // namespace detail

// ----------------------------------------------------------------- transport

// MaxSessions is the only sizing decision. Each session costs about 1.8 KB,
// almost all of it the reassembly buffer, so a small MCU picks 2 and a gateway
// picks 16. Nothing here allocates at runtime.
template <std::size_t MaxSessions = 4>
class Transport {
public:
  static_assert(MaxSessions >= 1, "need at least one session slot");

  explicit Transport(CanBus& bus, addr_t own_address,
                     Timeouts timeouts = Timeouts{}) noexcept
      : bus_(bus), own_address_(own_address), timeouts_(timeouts) {}

  void set_handler(MessageHandler handler, void* ctx) noexcept {
    handler_ = handler;
    handler_ctx_ = ctx;
  }

  addr_t address() const noexcept { return own_address_; }

  // ---- transmit -----------------------------------------------------------

  // Send a parameter group of any length. Up to 8 bytes goes out as a single
  // frame; anything longer opens a transport session. destination ==
  // kBroadcastAddress selects BAM, otherwise CMDT.
  Status send(pgn_t pgn, addr_t destination, const std::uint8_t* data,
              std::size_t length, std::uint32_t now_ms,
              std::uint8_t priority = 6) noexcept {
    if (length > kMaxMessageBytes) return Status::kMessageTooLarge;

    if (length <= 8) {
      CanFrame f;
      f.id = encode(make_id(pgn, own_address_, destination, priority));
      f.dlc = static_cast<std::uint8_t>(length);
      for (std::size_t i = 0; i < length; ++i) f.data[i] = data[i];
      return bus_.send(f) ? Status::kOk : Status::kBusError;
    }

    detail::Session* s = acquire();
    if (s == nullptr) return Status::kNoFreeSession;

    s->pgn = pgn;
    s->peer = destination;
    s->local = own_address_;
    s->total_bytes = static_cast<std::uint16_t>(length);
    s->total_packets = packet_count(length);
    s->next_seq = 1;
    for (std::size_t i = 0; i < length; ++i) s->buffer[i] = data[i];

    const bool broadcast = (destination == kBroadcastAddress);
    if (!send_cm(broadcast ? kCtrlBam : kCtrlRts, *s)) {
      s->reset();
      return Status::kBusError;
    }

    if (broadcast) {
      s->state = detail::SessionState::kTxBamData;
      // First data frame goes out on the next tick, honouring the inter-frame
      // gap. Blasting it immediately is the most common BAM bug.
      s->deadline_ms = now_ms + timeouts_.bam_gap_ms;
    } else {
      s->state = detail::SessionState::kTxWaitCts;
      s->deadline_ms = now_ms + timeouts_.t3_ms;
    }
    return Status::kOk;
  }

  // ---- receive ------------------------------------------------------------

  // Feed every incoming frame here. Frames that are not transport frames, or
  // not for us, return kNoSession and cost one comparison.
  Status on_frame(const CanFrame& frame, std::uint32_t now_ms) noexcept {
    const CanId id = decode(frame.id);
    const pgn_t pgn = id.pgn();

    if (pgn == kPgnTpCm) return on_cm(id, frame, now_ms);
    if (pgn == kPgnTpDt) return on_dt(id, frame, now_ms);
    return Status::kNoSession;
  }

  // Drive timers and outbound data frames. Call from the main loop or a
  // periodic task; nothing here blocks.
  void tick(std::uint32_t now_ms) noexcept {
    for (auto& s : sessions_) {
      if (!s.active()) continue;
      if (static_cast<std::int32_t>(now_ms - s.deadline_ms) < 0) continue;

      switch (s.state) {
        case detail::SessionState::kTxBamData:
          send_next_data(s, now_ms, /*burst_limited=*/false);
          break;
        case detail::SessionState::kTxCmdtData:
          send_next_data(s, now_ms, /*burst_limited=*/true);
          break;
        default:
          // Every other state reaching its deadline is a timeout. Receivers
          // abort so the peer stops sending; senders just give up the slot.
          if (s.state == detail::SessionState::kRxCmdt) {
            send_abort(s, AbortReason::kTimeout);
          }
          s.reset();
          break;
      }
    }
  }

  std::size_t active_sessions() const noexcept {
    std::size_t n = 0;
    for (auto const& s : sessions_) {
      if (s.active()) ++n;
    }
    return n;
  }

private:
  static constexpr std::uint8_t packet_count(std::size_t length) noexcept {
    return static_cast<std::uint8_t>((length + kBytesPerDataFrame - 1) /
                                     kBytesPerDataFrame);
  }

  detail::Session* acquire() noexcept {
    for (auto& s : sessions_) {
      if (!s.active()) return &s;
    }
    return nullptr;
  }

  // A session is identified by the (peer, local) address pair, which is what
  // J1939 allows: one transport session per direction per node pair.
  detail::Session* find(addr_t peer, addr_t local) noexcept {
    for (auto& s : sessions_) {
      if (s.active() && s.peer == peer && s.local == local) return &s;
    }
    return nullptr;
  }

  bool send_cm(std::uint8_t control, const detail::Session& s) noexcept {
    CanFrame f;
    f.id = encode(make_id(kPgnTpCm, own_address_, s.peer, 7));
    f.dlc = 8;
    f.data[0] = control;
    f.data[1] = static_cast<std::uint8_t>(s.total_bytes & 0xFF);
    f.data[2] = static_cast<std::uint8_t>(s.total_bytes >> 8);
    f.data[3] = s.total_packets;
    f.data[4] = 0xFF;  // max packets per CTS, 0xFF = no limit
    put_pgn(f.data + 5, s.pgn);
    return bus_.send(f);
  }

  bool send_cts(const detail::Session& s, std::uint8_t num_packets,
                std::uint8_t next_seq) noexcept {
    CanFrame f;
    f.id = encode(make_id(kPgnTpCm, own_address_, s.peer, 7));
    f.dlc = 8;
    f.data[0] = kCtrlCts;
    f.data[1] = num_packets;
    f.data[2] = next_seq;
    f.data[3] = 0xFF;
    f.data[4] = 0xFF;
    put_pgn(f.data + 5, s.pgn);
    return bus_.send(f);
  }

  bool send_eom_ack(const detail::Session& s) noexcept {
    CanFrame f;
    f.id = encode(make_id(kPgnTpCm, own_address_, s.peer, 7));
    f.dlc = 8;
    f.data[0] = kCtrlEndOfMsgAck;
    f.data[1] = static_cast<std::uint8_t>(s.total_bytes & 0xFF);
    f.data[2] = static_cast<std::uint8_t>(s.total_bytes >> 8);
    f.data[3] = s.total_packets;
    f.data[4] = 0xFF;
    put_pgn(f.data + 5, s.pgn);
    return bus_.send(f);
  }

  bool send_abort(const detail::Session& s, AbortReason reason) noexcept {
    CanFrame f;
    f.id = encode(make_id(kPgnTpCm, own_address_, s.peer, 7));
    f.dlc = 8;
    f.data[0] = kCtrlAbort;
    f.data[1] = static_cast<std::uint8_t>(reason);
    f.data[2] = 0xFF;
    f.data[3] = 0xFF;
    f.data[4] = 0xFF;
    put_pgn(f.data + 5, s.pgn);
    return bus_.send(f);
  }

  static void put_pgn(std::uint8_t* out, pgn_t pgn) noexcept {
    out[0] = static_cast<std::uint8_t>(pgn & 0xFF);
    out[1] = static_cast<std::uint8_t>((pgn >> 8) & 0xFF);
    out[2] = static_cast<std::uint8_t>((pgn >> 16) & 0xFF);
  }
  static pgn_t get_pgn(const std::uint8_t* in) noexcept {
    return static_cast<pgn_t>(in[0]) | (static_cast<pgn_t>(in[1]) << 8) |
           (static_cast<pgn_t>(in[2]) << 16);
  }

  // Send one data frame and advance. Pads the final frame with 0xFF, which is
  // the J1939 "not available" value rather than zero: a receiver that ignores
  // total_bytes and reads the padding gets "no data" instead of a real 0.
  void send_next_data(detail::Session& s, std::uint32_t now_ms,
                      bool burst_limited) noexcept {
    CanFrame f;
    f.id = encode(make_id(kPgnTpDt, own_address_, s.peer, 7));
    f.dlc = 8;
    f.data[0] = static_cast<std::uint8_t>(s.next_seq);

    const std::size_t offset = (s.next_seq - 1) * kBytesPerDataFrame;
    for (std::size_t i = 0; i < kBytesPerDataFrame; ++i) {
      const std::size_t src = offset + i;
      f.data[1 + i] = (src < s.total_bytes) ? s.buffer[src] : 0xFF;
    }

    if (!bus_.send(f)) {
      s.reset();  // mailbox full mid-transfer: drop the session, let the peer time out
      return;
    }

    ++s.next_seq;
    if (burst_limited && s.burst_remaining > 0) --s.burst_remaining;

    if (s.next_seq > s.total_packets) {
      if (burst_limited) {
        s.state = detail::SessionState::kTxWaitAck;
        s.deadline_ms = now_ms + timeouts_.t3_ms;
      } else {
        s.reset();  // BAM is unacknowledged: once the last frame is out, done
      }
      return;
    }

    if (burst_limited && s.burst_remaining == 0) {
      // Burst exhausted. Wait for the next CTS rather than carrying on, which
      // is the whole point of CMDT flow control.
      s.deadline_ms = now_ms + timeouts_.t3_ms;
      return;
    }

    s.deadline_ms = now_ms + (burst_limited ? 0 : timeouts_.bam_gap_ms);
  }

  Status on_cm(const CanId& id, const CanFrame& frame,
               std::uint32_t now_ms) noexcept {
    if (frame.dlc < 8) return Status::kNoSession;

    const std::uint8_t control = frame.data[0];
    const addr_t from = id.source;
    const addr_t to = id.destination();
    const pgn_t embedded = get_pgn(frame.data + 5);

    // A TP.CM addressed to another node is not ours, except BAM which is
    // broadcast and therefore for everyone.
    if (control != kCtrlBam && to != own_address_) return Status::kNoSession;

    switch (control) {
      case kCtrlBam:
      case kCtrlRts: {
        const std::uint16_t total = static_cast<std::uint16_t>(
            frame.data[1] | (static_cast<std::uint16_t>(frame.data[2]) << 8));
        const std::uint8_t packets = frame.data[3];

        if (total == 0 || total > kMaxMessageBytes ||
            packets != packet_count(total)) {
          // Inconsistent announcement. Reject rather than trusting one field
          // over the other and reassembling garbage.
          return Status::kMessageTooLarge;
        }

        // Duplicate announcement from a peer we are already talking to:
        // J1939 says the new session replaces the old one.
        if (detail::Session* existing = find(from, to)) existing->reset();

        detail::Session* s = acquire();
        if (s == nullptr) {
          if (control == kCtrlRts) {
            detail::Session tmp;
            tmp.peer = from;
            tmp.pgn = embedded;
            send_abort(tmp, AbortReason::kBusy);
          }
          return Status::kNoFreeSession;
        }

        s->pgn = embedded;
        s->peer = from;
        s->local = to;
        s->total_bytes = total;
        s->total_packets = packets;
        s->next_seq = 1;

        if (control == kCtrlBam) {
          s->state = detail::SessionState::kRxBam;
          s->deadline_ms = now_ms + timeouts_.t1_ms;
        } else {
          s->state = detail::SessionState::kRxCmdt;
          // Accept the whole message in one burst. A receiver with less RAM
          // would send a smaller number here and issue further CTS frames.
          if (!send_cts(*s, packets, 1)) {
            s->reset();
            return Status::kBusError;
          }
          s->deadline_ms = now_ms + timeouts_.t2_ms;
        }
        return Status::kOk;
      }

      case kCtrlCts: {
        detail::Session* s = find(from, to);
        if (s == nullptr) return Status::kNoSession;
        if (s->state != detail::SessionState::kTxWaitCts &&
            s->state != detail::SessionState::kTxCmdtData) {
          return Status::kNoSession;
        }
        const std::uint8_t num = frame.data[1];
        const std::uint8_t next = frame.data[2];
        if (num == 0) {
          // CTS with zero packets is a legitimate "hold": the receiver is busy
          // and will send a real CTS later. Not an error.
          s->deadline_ms = now_ms + timeouts_.th_ms;
          return Status::kOk;
        }
        if (next == 0 || next > s->total_packets) return Status::kBadSequence;

        s->next_seq = next;
        s->burst_remaining = num;
        s->state = detail::SessionState::kTxCmdtData;
        s->deadline_ms = now_ms;  // start sending on the next tick
        return Status::kOk;
      }

      case kCtrlEndOfMsgAck: {
        detail::Session* s = find(from, to);
        if (s == nullptr) return Status::kNoSession;
        s->reset();
        return Status::kOk;
      }

      case kCtrlAbort: {
        detail::Session* s = find(from, to);
        if (s == nullptr) return Status::kNoSession;
        s->reset();
        return Status::kAborted;
      }

      default:
        return Status::kNoSession;
    }
  }

  Status on_dt(const CanId& id, const CanFrame& frame,
               std::uint32_t now_ms) noexcept {
    if (frame.dlc < 8) return Status::kNoSession;

    detail::Session* s = find(id.source, id.destination());
    if (s == nullptr) {
      // BAM data frames are broadcast, so the destination is 0xFF and the
      // session was opened against that. Try again that way.
      s = find(id.source, kBroadcastAddress);
    }
    if (s == nullptr) return Status::kNoSession;
    if (s->state != detail::SessionState::kRxBam &&
        s->state != detail::SessionState::kRxCmdt) {
      return Status::kNoSession;
    }

    const std::uint16_t seq = frame.data[0];
    if (seq != s->next_seq) {
      // Out of order. Recovering mid-stream is not worth the complexity on an
      // MCU: abort and let the sender start over.
      if (s->state == detail::SessionState::kRxCmdt) {
        send_abort(*s, AbortReason::kUnexpectedDataPacket);
      }
      s->reset();
      return Status::kBadSequence;
    }

    const std::size_t offset = (seq - 1) * kBytesPerDataFrame;
    for (std::size_t i = 0; i < kBytesPerDataFrame; ++i) {
      const std::size_t dst = offset + i;
      if (dst < s->total_bytes) s->buffer[dst] = frame.data[1 + i];
    }

    ++s->next_seq;
    s->deadline_ms = now_ms + timeouts_.t1_ms;

    if (s->next_seq > s->total_packets) {
      if (s->state == detail::SessionState::kRxCmdt) send_eom_ack(*s);
      if (handler_ != nullptr) {
        handler_(handler_ctx_, s->pgn, s->peer, s->buffer, s->total_bytes);
      }
      s->reset();
    }
    return Status::kOk;
  }

  CanBus& bus_;
  addr_t own_address_;
  Timeouts timeouts_;
  MessageHandler handler_ = nullptr;
  void* handler_ctx_ = nullptr;
  detail::Session sessions_[MaxSessions] = {};
};

}  // namespace j1939
