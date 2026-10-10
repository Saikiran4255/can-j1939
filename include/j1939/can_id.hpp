// can_id.hpp — SAE J1939 29-bit CAN identifier codec.
//
// J1939 packs five fields into the 29-bit extended CAN identifier:
//
//   bits 28..26  Priority            (3 bits, 0 = highest)
//   bit  25      Extended Data Page  (EDP)
//   bit  24      Data Page           (DP)
//   bits 23..16  PDU Format          (PF)
//   bits 15..8   PDU Specific        (PS)
//   bits  7..0   Source Address      (SA)
//
// The wrinkle that catches people out is that PF decides what PS means:
//
//   PF <  0xF0  (PDU1)  PS is a Destination Address. The PGN is PF << 8,
//                       with the low byte zeroed: the DA is addressing
//                       information, not part of the parameter number.
//   PF >= 0xF0  (PDU2)  PS is a Group Extension and IS part of the PGN,
//                       so the PGN is (PF << 8) | PS. PDU2 messages are
//                       always broadcast; there is no destination field.
//
// Getting that wrong means decoding a broadcast engine message as if it were
// addressed to node 0xF4, which is the classic J1939 integration bug.
//
// Header-only, no allocation, no exceptions. Everything here is constexpr so
// PGN tables can be built at compile time on a microcontroller.

#pragma once

#include <cstdint>

namespace j1939 {

using pgn_t = std::uint32_t;  // 18-bit Parameter Group Number
using addr_t = std::uint8_t;

inline constexpr addr_t kBroadcastAddress = 0xFF;  // "global", J1939-81
inline constexpr addr_t kNullAddress = 0xFE;       // claimed by nobody
inline constexpr std::uint8_t kPdu2Threshold = 0xF0;

// A decoded identifier. Trivially copyable, 8 bytes, fits in registers.
struct CanId {
  std::uint8_t priority = 6;  // 6 is the J1939 default for proprietary data
  bool extended_data_page = false;
  bool data_page = false;
  std::uint8_t pdu_format = 0;
  std::uint8_t pdu_specific = 0;
  addr_t source = kNullAddress;

  // True for PDU2, i.e. broadcast-only with the group extension in the PGN.
  constexpr bool is_pdu2() const noexcept { return pdu_format >= kPdu2Threshold; }

  // The 18-bit PGN. For PDU1 the low byte is zero by definition.
  constexpr pgn_t pgn() const noexcept {
    const pgn_t dp = (static_cast<pgn_t>(extended_data_page) << 1) |
                     static_cast<pgn_t>(data_page);
    const pgn_t base = (dp << 16) | (static_cast<pgn_t>(pdu_format) << 8);
    return is_pdu2() ? (base | pdu_specific) : base;
  }

  // Destination address, or the broadcast address for PDU2. Callers that treat
  // every PS field as a destination are the bug this function exists to avoid.
  constexpr addr_t destination() const noexcept {
    return is_pdu2() ? kBroadcastAddress : pdu_specific;
  }

  constexpr bool is_broadcast() const noexcept {
    return is_pdu2() || pdu_specific == kBroadcastAddress;
  }
};

// Pack into the raw 29-bit identifier.
constexpr std::uint32_t encode(const CanId& id) noexcept {
  return (static_cast<std::uint32_t>(id.priority & 0x07) << 26) |
         (static_cast<std::uint32_t>(id.extended_data_page ? 1u : 0u) << 25) |
         (static_cast<std::uint32_t>(id.data_page ? 1u : 0u) << 24) |
         (static_cast<std::uint32_t>(id.pdu_format) << 16) |
         (static_cast<std::uint32_t>(id.pdu_specific) << 8) |
         static_cast<std::uint32_t>(id.source);
}

// Unpack. Bits above 28 are ignored rather than rejected, since a CAN
// controller that hands us a wider word has already masked what matters.
constexpr CanId decode(std::uint32_t raw) noexcept {
  CanId id;
  id.priority = static_cast<std::uint8_t>((raw >> 26) & 0x07);
  id.extended_data_page = ((raw >> 25) & 0x01) != 0;
  id.data_page = ((raw >> 24) & 0x01) != 0;
  id.pdu_format = static_cast<std::uint8_t>((raw >> 16) & 0xFF);
  id.pdu_specific = static_cast<std::uint8_t>((raw >> 8) & 0xFF);
  id.source = static_cast<addr_t>(raw & 0xFF);
  return id;
}

// Build an identifier for a PGN, choosing PDU1 or PDU2 from the PGN itself so
// the caller cannot accidentally put a destination on a broadcast-only PGN.
constexpr CanId make_id(pgn_t pgn, addr_t source, addr_t destination,
                        std::uint8_t priority = 6) noexcept {
  CanId id;
  id.priority = priority;
  id.extended_data_page = ((pgn >> 17) & 0x01) != 0;
  id.data_page = ((pgn >> 16) & 0x01) != 0;
  id.pdu_format = static_cast<std::uint8_t>((pgn >> 8) & 0xFF);
  id.source = source;
  id.pdu_specific = (id.pdu_format >= kPdu2Threshold)
                        ? static_cast<std::uint8_t>(pgn & 0xFF)  // group extension
                        : destination;
  return id;
}

// A CAN frame as the driver layer sees it. Fixed 8-byte payload: classic CAN,
// not CAN FD, which is what J1939-21 transport exists to work around.
struct CanFrame {
  std::uint32_t id = 0;
  std::uint8_t dlc = 0;
  std::uint8_t data[8] = {};
};

}  // namespace j1939
