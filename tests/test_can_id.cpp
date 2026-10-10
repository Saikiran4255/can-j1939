#include <gtest/gtest.h>

#include "j1939/can_id.hpp"

using namespace j1939;

// PDU1 vs PDU2 is the distinction that breaks naive implementations, so it is
// tested first and hardest.

TEST(CanId, Pdu2PutsGroupExtensionInThePgn) {
  // PGN 61444 (0xF004) Electronic Engine Controller 1, the canonical PDU2
  // broadcast message. PF = 0xF0 >= 0xF0, so PS is part of the PGN.
  const CanId id = decode(0x0CF00400);
  EXPECT_TRUE(id.is_pdu2());
  EXPECT_EQ(id.pgn(), 0x00F004u);
  EXPECT_EQ(id.pgn(), 61444u);
  EXPECT_EQ(id.source, 0x00);
  EXPECT_EQ(id.priority, 3);
  // Critically: this is NOT addressed to node 0x04.
  EXPECT_EQ(id.destination(), kBroadcastAddress);
  EXPECT_TRUE(id.is_broadcast());
}

TEST(CanId, Pdu1TreatsPsAsDestinationAndZeroesThePgnLowByte) {
  // TP.CM (0xEC00) from 0x21 to 0x17. PF = 0xEC < 0xF0, so PS is the DA.
  const CanId id = decode(0x1CEC1721);
  EXPECT_FALSE(id.is_pdu2());
  EXPECT_EQ(id.pgn(), 0x00EC00u);  // low byte zero, not 0x17
  EXPECT_EQ(id.destination(), 0x17);
  EXPECT_EQ(id.source, 0x21);
  EXPECT_FALSE(id.is_broadcast());
}

TEST(CanId, RoundTripsEveryEncodableField) {
  for (std::uint8_t prio = 0; prio < 8; ++prio) {
    for (int pf : {0x00, 0x0F, 0xEB, 0xEC, 0xEF, 0xF0, 0xFE, 0xFF}) {
      CanId in;
      in.priority = prio;
      in.data_page = (pf & 1) != 0;
      in.extended_data_page = (pf & 2) != 0;
      in.pdu_format = static_cast<std::uint8_t>(pf);
      in.pdu_specific = 0x5A;
      in.source = 0xA5;

      const CanId out = decode(encode(in));
      EXPECT_EQ(out.priority, in.priority);
      EXPECT_EQ(out.data_page, in.data_page);
      EXPECT_EQ(out.extended_data_page, in.extended_data_page);
      EXPECT_EQ(out.pdu_format, in.pdu_format);
      EXPECT_EQ(out.pdu_specific, in.pdu_specific);
      EXPECT_EQ(out.source, in.source);
    }
  }
}

TEST(CanId, EncodedIdNeverExceeds29Bits) {
  CanId id;
  id.priority = 7;
  id.extended_data_page = true;
  id.data_page = true;
  id.pdu_format = 0xFF;
  id.pdu_specific = 0xFF;
  id.source = 0xFF;
  EXPECT_LE(encode(id), 0x1FFFFFFFu);
}

TEST(CanId, MakeIdIgnoresDestinationForBroadcastOnlyPgns) {
  // Caller passes a destination for a PDU2 PGN. It must be dropped, not
  // written over the group extension, or the PGN silently changes.
  const CanId id = make_id(61444, 0x12, 0x34);
  EXPECT_EQ(id.pgn(), 61444u);
  EXPECT_EQ(id.pdu_specific, 0x04);  // group extension preserved
  EXPECT_EQ(id.destination(), kBroadcastAddress);
}

TEST(CanId, MakeIdCarriesDestinationForAddressedPgns) {
  const CanId id = make_id(0x00EC00, 0x12, 0x34, 7);  // TP.CM, a PDU1 PGN
  EXPECT_EQ(id.pgn(), 0x00EC00u);
  EXPECT_EQ(id.destination(), 0x34);
  EXPECT_EQ(id.source, 0x12);
  EXPECT_EQ(id.priority, 7);
}

TEST(CanId, DataPageBitsReachThePgn) {
  // PGN 126208 (0x1EF00) has DP set. It must survive make_id -> encode ->
  // decode -> pgn() without losing the page bits.
  const CanId id = make_id(0x1EF00, 0x01, 0xFF);
  EXPECT_TRUE(id.data_page);
  EXPECT_EQ(decode(encode(id)).pgn(), 0x1EF00u);
}

TEST(CanId, EverythingIsUsableAtCompileTime) {
  // If this compiles, the codec is constexpr and PGN tables can live in flash.
  constexpr CanId id = make_id(61444, 0x00, 0xFF, 3);
  static_assert(id.pgn() == 61444u);
  static_assert(id.is_pdu2());
  static_assert(id.pdu_specific == 0x04);      // group extension, not the DA
  static_assert(encode(id) == 0x0CF00400u);    // prio 3, PF F0, GE 04, SA 00
  static_assert(decode(0x0CF00400u).pgn() == 61444u);
  SUCCEED();
}
