#include <gtest/gtest.h>

#include <numeric>
#include <vector>

#include "fake_bus.hpp"
#include "j1939/transport.hpp"

using namespace j1939;

namespace {

std::vector<std::uint8_t> ramp(std::size_t n) {
  std::vector<std::uint8_t> v(n);
  for (std::size_t i = 0; i < n; ++i) v[i] = static_cast<std::uint8_t>(i & 0xFF);
  return v;
}

// Pump every frame one bus produced into the other transport, then clear.
// Models a perfect bus: no loss, no reordering. Tests that need loss or
// reordering do it explicitly.
template <typename RxT>
void deliver(FakeBus& from, RxT& to, std::uint32_t now) {
  auto frames = from.frames;
  from.clear();
  for (auto const& f : frames) to.on_frame(f, now);
}

constexpr pgn_t kTestPgn = 0x00FEDA;  // a PDU2 PGN, 65242
constexpr addr_t kNodeA = 0x21;
constexpr addr_t kNodeB = 0x17;

}  // namespace

// --------------------------------------------------------------- single frame

TEST(Transport, ShortMessageGoesOutAsOneFrameWithNoSession) {
  FakeBus bus;
  Transport<2> t(bus, kNodeA);
  const std::uint8_t payload[3] = {1, 2, 3};

  ASSERT_EQ(t.send(kTestPgn, kBroadcastAddress, payload, 3, 0), Status::kOk);
  ASSERT_EQ(bus.frames.size(), 1u);
  EXPECT_EQ(bus.frames[0].dlc, 3);
  EXPECT_EQ(decode(bus.frames[0].id).pgn(), kTestPgn);
  EXPECT_EQ(t.active_sessions(), 0u) << "8 bytes or fewer must not open a session";
}

TEST(Transport, ExactlyEightBytesStillFitsInOneFrame) {
  FakeBus bus;
  Transport<2> t(bus, kNodeA);
  const auto p = ramp(8);
  ASSERT_EQ(t.send(kTestPgn, kBroadcastAddress, p.data(), 8, 0), Status::kOk);
  EXPECT_EQ(bus.frames.size(), 1u);
  EXPECT_EQ(t.active_sessions(), 0u);
}

TEST(Transport, NineBytesOpensATransportSession) {
  FakeBus bus;
  Transport<2> t(bus, kNodeA);
  const auto p = ramp(9);
  ASSERT_EQ(t.send(kTestPgn, kBroadcastAddress, p.data(), 9, 0), Status::kOk);
  ASSERT_EQ(bus.frames.size(), 1u) << "only the BAM announcement so far";
  EXPECT_EQ(bus.frames[0].data[0], kCtrlBam);
  EXPECT_EQ(t.active_sessions(), 1u);
}

TEST(Transport, RejectsMessagesOverTheProtocolLimit) {
  FakeBus bus;
  Transport<2> t(bus, kNodeA);
  const auto p = ramp(kMaxMessageBytes + 1);
  EXPECT_EQ(t.send(kTestPgn, kBroadcastAddress, p.data(), p.size(), 0),
            Status::kMessageTooLarge);
  EXPECT_TRUE(bus.frames.empty());
}

// ----------------------------------------------------------------------- BAM

TEST(Transport, BamRoundTripsAnArbitraryPayload) {
  FakeBus tx_bus, rx_bus;
  Transport<2> tx(tx_bus, kNodeA);
  Transport<2> rx(rx_bus, kNodeB);
  Sink sink;
  rx.set_handler(&Sink::handler, &sink);

  const auto payload = ramp(100);  // 15 packets
  ASSERT_EQ(tx.send(kTestPgn, kBroadcastAddress, payload.data(), payload.size(), 0),
            Status::kOk);

  // Run the clock forward in 50 ms steps, the BAM inter-frame gap.
  for (std::uint32_t now = 0; now <= 1000; now += 50) {
    tx.tick(now);
    deliver(tx_bus, rx, now);
    rx.tick(now);
  }

  ASSERT_EQ(sink.messages.size(), 1u);
  EXPECT_EQ(sink.messages[0].pgn, kTestPgn);
  EXPECT_EQ(sink.messages[0].source, kNodeA);
  EXPECT_EQ(sink.messages[0].data, payload);
  EXPECT_EQ(tx.active_sessions(), 0u);
  EXPECT_EQ(rx.active_sessions(), 0u);
}

TEST(Transport, BamHandlesTheMaximumMessageSize) {
  FakeBus tx_bus, rx_bus;
  Transport<2> tx(tx_bus, kNodeA);
  Transport<2> rx(rx_bus, kNodeB);
  Sink sink;
  rx.set_handler(&Sink::handler, &sink);

  const auto payload = ramp(kMaxMessageBytes);  // 1785 bytes, 255 packets
  ASSERT_EQ(tx.send(kTestPgn, kBroadcastAddress, payload.data(), payload.size(), 0),
            Status::kOk);

  for (std::uint32_t now = 0; now <= 20000; now += 50) {
    tx.tick(now);
    deliver(tx_bus, rx, now);
    rx.tick(now);
  }

  ASSERT_EQ(sink.messages.size(), 1u);
  EXPECT_EQ(sink.messages[0].data.size(), kMaxMessageBytes);
  EXPECT_EQ(sink.messages[0].data, payload);
}

TEST(Transport, BamSenderRespectsTheInterFrameGap) {
  FakeBus bus;
  Timeouts t;
  t.bam_gap_ms = 50;
  Transport<2> tx(bus, kNodeA, t);

  const auto payload = ramp(50);  // 8 packets
  ASSERT_EQ(tx.send(kTestPgn, kBroadcastAddress, payload.data(), payload.size(), 0),
            Status::kOk);
  const std::size_t after_announce = bus.frames.size();

  tx.tick(10);  // too early
  EXPECT_EQ(bus.frames.size(), after_announce) << "sent a data frame before the gap";

  tx.tick(50);  // gap elapsed
  EXPECT_EQ(bus.frames.size(), after_announce + 1);

  tx.tick(60);  // too early again
  EXPECT_EQ(bus.frames.size(), after_announce + 1);
}

TEST(Transport, BamPadsTheFinalFrameWithNotAvailable) {
  FakeBus bus;
  Transport<2> tx(bus, kNodeA);
  const auto payload = ramp(9);  // 2 packets: 7 bytes then 2 bytes + 5 padding
  ASSERT_EQ(tx.send(kTestPgn, kBroadcastAddress, payload.data(), 9, 0), Status::kOk);
  for (std::uint32_t now = 0; now <= 300; now += 50) tx.tick(now);

  ASSERT_GE(bus.frames.size(), 3u);
  const auto& last = bus.frames.back();
  EXPECT_EQ(last.data[0], 2);     // sequence number
  EXPECT_EQ(last.data[1], 7);     // payload byte 7
  EXPECT_EQ(last.data[2], 8);     // payload byte 8
  for (int i = 3; i < 8; ++i) {
    EXPECT_EQ(last.data[i], 0xFF) << "padding must be 0xFF, not 0, at byte " << i;
  }
}

// ---------------------------------------------------------------------- CMDT

TEST(Transport, CmdtCompletesTheFullHandshake) {
  FakeBus a_bus, b_bus;
  Transport<2> a(a_bus, kNodeA);
  Transport<2> b(b_bus, kNodeB);
  Sink sink;
  b.set_handler(&Sink::handler, &sink);

  const auto payload = ramp(60);  // 9 packets
  ASSERT_EQ(a.send(kTestPgn, kNodeB, payload.data(), payload.size(), 0), Status::kOk);

  // RTS -> CTS -> data -> EndOfMsgAck, pumped both ways.
  for (std::uint32_t now = 0; now <= 500; now += 10) {
    a.tick(now);
    deliver(a_bus, b, now);
    b.tick(now);
    deliver(b_bus, a, now);
  }

  ASSERT_EQ(sink.messages.size(), 1u);
  EXPECT_EQ(sink.messages[0].data, payload);
  EXPECT_EQ(a.active_sessions(), 0u) << "sender must close on EndOfMsgAck";
  EXPECT_EQ(b.active_sessions(), 0u);
}

TEST(Transport, CmdtReceiverSendsRtsThenCtsThenAck) {
  FakeBus a_bus, b_bus;
  Transport<2> a(a_bus, kNodeA);
  Transport<2> b(b_bus, kNodeB);
  Sink sink;
  b.set_handler(&Sink::handler, &sink);

  const auto payload = ramp(20);  // 3 packets
  ASSERT_EQ(a.send(kTestPgn, kNodeB, payload.data(), payload.size(), 0), Status::kOk);

  ASSERT_EQ(a_bus.frames.size(), 1u);
  EXPECT_EQ(a_bus.frames[0].data[0], kCtrlRts);
  deliver(a_bus, b, 0);

  ASSERT_EQ(b_bus.frames.size(), 1u);
  EXPECT_EQ(b_bus.frames[0].data[0], kCtrlCts);
  EXPECT_EQ(b_bus.frames[0].data[1], 3) << "CTS should allow all 3 packets";
  EXPECT_EQ(b_bus.frames[0].data[2], 1) << "starting at sequence 1";
  deliver(b_bus, a, 0);

  for (std::uint32_t now = 0; now <= 200; now += 10) {
    a.tick(now);
    deliver(a_bus, b, now);
  }

  bool saw_ack = false;
  for (auto const& f : b_bus.frames) {
    if (f.data[0] == kCtrlEndOfMsgAck) saw_ack = true;
  }
  EXPECT_TRUE(saw_ack);
  EXPECT_EQ(sink.messages.size(), 1u);
}

TEST(Transport, CtsWithZeroPacketsIsAHoldNotAnError) {
  FakeBus a_bus, b_bus;
  Transport<2> a(a_bus, kNodeA);
  const auto payload = ramp(30);
  ASSERT_EQ(a.send(kTestPgn, kNodeB, payload.data(), payload.size(), 0), Status::kOk);
  a_bus.clear();

  // Hand-build a hold CTS from B.
  CanFrame cts;
  cts.id = encode(make_id(0x00EC00, kNodeB, kNodeA, 7));
  cts.dlc = 8;
  cts.data[0] = kCtrlCts;
  cts.data[1] = 0;  // zero packets = hold
  cts.data[2] = 0xFF;
  cts.data[3] = 0xFF;
  cts.data[4] = 0xFF;
  cts.data[5] = static_cast<std::uint8_t>(kTestPgn & 0xFF);
  cts.data[6] = static_cast<std::uint8_t>((kTestPgn >> 8) & 0xFF);
  cts.data[7] = static_cast<std::uint8_t>((kTestPgn >> 16) & 0xFF);

  EXPECT_EQ(a.on_frame(cts, 100), Status::kOk);
  EXPECT_EQ(a.active_sessions(), 1u) << "a hold must not tear down the session";
  a.tick(150);
  EXPECT_TRUE(a_bus.frames.empty()) << "must not send data during a hold";
}

// -------------------------------------------------------- errors and limits

TEST(Transport, OutOfOrderDataFrameAbortsAndNotifiesThePeer) {
  FakeBus a_bus, b_bus;
  Transport<2> a(a_bus, kNodeA);
  Transport<2> b(b_bus, kNodeB);
  Sink sink;
  b.set_handler(&Sink::handler, &sink);

  const auto payload = ramp(30);  // 5 packets
  ASSERT_EQ(a.send(kTestPgn, kNodeB, payload.data(), payload.size(), 0), Status::kOk);
  deliver(a_bus, b, 0);   // RTS
  deliver(b_bus, a, 0);   // CTS

  a.tick(0);
  ASSERT_FALSE(a_bus.frames.empty());
  auto frames = a_bus.frames;
  a_bus.clear();
  b_bus.clear();

  // Deliver sequence 1, then jump to 3.
  ASSERT_EQ(b.on_frame(frames[0], 10), Status::kOk);
  CanFrame bad = frames[0];
  bad.data[0] = 3;
  EXPECT_EQ(b.on_frame(bad, 20), Status::kBadSequence);

  EXPECT_EQ(b.active_sessions(), 0u) << "receiver must drop the session";
  EXPECT_TRUE(sink.messages.empty()) << "must not deliver a partial message";

  bool saw_abort = false;
  for (auto const& f : b_bus.frames) {
    if (f.data[0] == kCtrlAbort) {
      saw_abort = true;
      EXPECT_EQ(f.data[1], static_cast<std::uint8_t>(AbortReason::kUnexpectedDataPacket));
    }
  }
  EXPECT_TRUE(saw_abort) << "peer was never told why we stopped";
}

TEST(Transport, RtsWithInconsistentLengthAndPacketCountIsRejected) {
  FakeBus bus;
  Transport<2> t(bus, kNodeB);
  Sink sink;
  t.set_handler(&Sink::handler, &sink);

  CanFrame rts;
  rts.id = encode(make_id(0x00EC00, kNodeA, kNodeB, 7));
  rts.dlc = 8;
  rts.data[0] = kCtrlRts;
  rts.data[1] = 100;  // claims 100 bytes
  rts.data[2] = 0;
  rts.data[3] = 2;    // but only 2 packets, which holds at most 14
  rts.data[4] = 0xFF;
  rts.data[5] = 0x00;
  rts.data[6] = 0xFE;
  rts.data[7] = 0x00;

  EXPECT_EQ(t.on_frame(rts, 0), Status::kMessageTooLarge);
  EXPECT_EQ(t.active_sessions(), 0u);
}

TEST(Transport, SessionPoolExhaustionIsReportedAndAbortsTheNewPeer) {
  FakeBus bus;
  Transport<1> t(bus, kNodeB);  // exactly one slot

  auto make_rts = [](addr_t from, addr_t to) {
    CanFrame f;
    f.id = encode(make_id(0x00EC00, from, to, 7));
    f.dlc = 8;
    f.data[0] = kCtrlRts;
    f.data[1] = 20;
    f.data[2] = 0;
    f.data[3] = 3;
    f.data[4] = 0xFF;
    f.data[5] = 0x00;
    f.data[6] = 0xFE;
    f.data[7] = 0x00;
    return f;
  };

  EXPECT_EQ(t.on_frame(make_rts(0x10, kNodeB), 0), Status::kOk);
  EXPECT_EQ(t.active_sessions(), 1u);

  bus.clear();
  EXPECT_EQ(t.on_frame(make_rts(0x11, kNodeB), 0), Status::kNoFreeSession);

  bool saw_busy_abort = false;
  for (auto const& f : bus.frames) {
    if (f.data[0] == kCtrlAbort &&
        f.data[1] == static_cast<std::uint8_t>(AbortReason::kBusy)) {
      saw_busy_abort = true;
    }
  }
  EXPECT_TRUE(saw_busy_abort) << "second peer must be told we are busy";
}

TEST(Transport, SenderReportsNoFreeSessionRatherThanOverwriting) {
  FakeBus bus;
  Transport<1> t(bus, kNodeA);
  const auto p = ramp(30);
  ASSERT_EQ(t.send(kTestPgn, kNodeB, p.data(), p.size(), 0), Status::kOk);
  EXPECT_EQ(t.send(kTestPgn, 0x33, p.data(), p.size(), 0), Status::kNoFreeSession);
  EXPECT_EQ(t.active_sessions(), 1u);
}

TEST(Transport, ReceiverTimesOutAndAbortsWhenDataStops) {
  FakeBus a_bus, b_bus;
  Transport<2> a(a_bus, kNodeA);
  Transport<2> b(b_bus, kNodeB);
  Sink sink;
  b.set_handler(&Sink::handler, &sink);

  const auto payload = ramp(60);
  ASSERT_EQ(a.send(kTestPgn, kNodeB, payload.data(), payload.size(), 0), Status::kOk);
  deliver(a_bus, b, 0);  // RTS in, CTS out
  b_bus.clear();

  // The sender goes silent. T2 is 1250 ms.
  b.tick(1000);
  EXPECT_EQ(b.active_sessions(), 1u) << "must not give up early";
  b.tick(1300);
  EXPECT_EQ(b.active_sessions(), 0u) << "must give up after T2";

  bool saw_timeout_abort = false;
  for (auto const& f : b_bus.frames) {
    if (f.data[0] == kCtrlAbort &&
        f.data[1] == static_cast<std::uint8_t>(AbortReason::kTimeout)) {
      saw_timeout_abort = true;
    }
  }
  EXPECT_TRUE(saw_timeout_abort);
  EXPECT_TRUE(sink.messages.empty());
}

TEST(Transport, AbortFromThePeerClosesOurSession) {
  FakeBus bus;
  Transport<2> t(bus, kNodeA);
  const auto p = ramp(30);
  ASSERT_EQ(t.send(kTestPgn, kNodeB, p.data(), p.size(), 0), Status::kOk);
  ASSERT_EQ(t.active_sessions(), 1u);

  CanFrame abort;
  abort.id = encode(make_id(0x00EC00, kNodeB, kNodeA, 7));
  abort.dlc = 8;
  abort.data[0] = kCtrlAbort;
  abort.data[1] = static_cast<std::uint8_t>(AbortReason::kResourcesFreed);
  for (int i = 2; i < 8; ++i) abort.data[i] = 0xFF;

  EXPECT_EQ(t.on_frame(abort, 10), Status::kAborted);
  EXPECT_EQ(t.active_sessions(), 0u);
}

TEST(Transport, BusFailureMidTransferReleasesTheSession) {
  FakeBus bus;
  Transport<2> t(bus, kNodeA);
  bus.fail_after(3);  // announcement plus two data frames, then the mailbox jams

  const auto p = ramp(100);
  ASSERT_EQ(t.send(kTestPgn, kBroadcastAddress, p.data(), p.size(), 0), Status::kOk);
  for (std::uint32_t now = 0; now <= 1000; now += 50) t.tick(now);

  EXPECT_EQ(t.active_sessions(), 0u) << "a jammed session would leak a slot forever";
}

TEST(Transport, SendReportsBusErrorWhenTheAnnouncementCannotGoOut) {
  FakeBus bus;
  Transport<2> t(bus, kNodeA);
  bus.fail_after(0);  // refuse everything
  const auto p = ramp(30);
  EXPECT_EQ(t.send(kTestPgn, kBroadcastAddress, p.data(), p.size(), 0),
            Status::kBusError);
  EXPECT_EQ(t.active_sessions(), 0u) << "failed start must not hold a slot";
}

TEST(Transport, TwoPeersAreReassembledIndependently) {
  FakeBus bus;
  Transport<4> rx(bus, kNodeB);
  Sink sink;
  rx.set_handler(&Sink::handler, &sink);

  FakeBus bus1, bus2;
  Transport<2> p1(bus1, 0x10);
  Transport<2> p2(bus2, 0x11);

  const auto a = ramp(40);
  std::vector<std::uint8_t> bpl(40, 0xAA);

  ASSERT_EQ(p1.send(kTestPgn, kNodeB, a.data(), a.size(), 0), Status::kOk);
  ASSERT_EQ(p2.send(kTestPgn, kNodeB, bpl.data(), bpl.size(), 0), Status::kOk);

  // Interleave the two senders' frames, which is what a real bus does.
  for (std::uint32_t now = 0; now <= 500; now += 10) {
    p1.tick(now);
    p2.tick(now);
    deliver(bus1, rx, now);
    deliver(bus2, rx, now);
    rx.tick(now);
    // CTS and acks go back on rx's bus; route them to both peers, each ignores
    // what is not addressed to it.
    auto back = bus.frames;
    bus.clear();
    for (auto const& f : back) {
      p1.on_frame(f, now);
      p2.on_frame(f, now);
    }
  }

  ASSERT_EQ(sink.messages.size(), 2u) << "interleaved sessions got crossed";
  const bool first_is_ramp = (sink.messages[0].data == a);
  EXPECT_EQ(sink.messages[first_is_ramp ? 0 : 1].data, a);
  EXPECT_EQ(sink.messages[first_is_ramp ? 1 : 0].data, bpl);
}

TEST(Transport, FramesForOtherNodesAreIgnoredCheaply) {
  FakeBus bus;
  Transport<2> t(bus, kNodeB);

  CanFrame rts;
  rts.id = encode(make_id(0x00EC00, kNodeA, 0x55, 7));  // addressed to 0x55
  rts.dlc = 8;
  rts.data[0] = kCtrlRts;
  rts.data[1] = 20;
  rts.data[2] = 0;
  rts.data[3] = 3;
  rts.data[4] = 0xFF;
  rts.data[5] = 0x00;
  rts.data[6] = 0xFE;
  rts.data[7] = 0x00;

  EXPECT_EQ(t.on_frame(rts, 0), Status::kNoSession);
  EXPECT_EQ(t.active_sessions(), 0u);
  EXPECT_TRUE(bus.frames.empty()) << "must not respond to another node's RTS";
}

TEST(Transport, NonTransportFramesAreNotClaimed) {
  FakeBus bus;
  Transport<2> t(bus, kNodeB);
  CanFrame eec1;
  eec1.id = encode(make_id(61444, kNodeA, kBroadcastAddress, 3));
  eec1.dlc = 8;
  EXPECT_EQ(t.on_frame(eec1, 0), Status::kNoSession);
}

TEST(Transport, DuplicateRtsFromTheSamePeerRestartsTheSession) {
  FakeBus bus;
  Transport<2> t(bus, kNodeB);
  Sink sink;
  t.set_handler(&Sink::handler, &sink);

  auto rts = [](std::uint8_t bytes, std::uint8_t packets) {
    CanFrame f;
    f.id = encode(make_id(0x00EC00, kNodeA, kNodeB, 7));
    f.dlc = 8;
    f.data[0] = kCtrlRts;
    f.data[1] = bytes;
    f.data[2] = 0;
    f.data[3] = packets;
    f.data[4] = 0xFF;
    f.data[5] = 0x00;
    f.data[6] = 0xFE;
    f.data[7] = 0x00;
    return f;
  };

  EXPECT_EQ(t.on_frame(rts(20, 3), 0), Status::kOk);
  EXPECT_EQ(t.on_frame(rts(40, 6), 10), Status::kOk);
  EXPECT_EQ(t.active_sessions(), 1u) << "restart must not leak a second slot";
}
