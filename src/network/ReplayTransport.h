// ReplayTransport.h — an INetworkTransport decorator that records the traffic a
// session delivered, or replays a recording as if the peer were live. It sits at
// the top of the transport stack: what it journals is exactly what the game
// consumed, after any latency simulation — the last deterministic boundary.
//
//   NONE    pure pass-through.
//   RECORD  passes everything through and journals inbound messages, connection
//           events and outbound sends, each stamped with the poll() call that
//           saw it.
//   PLAY    needs no inner transport and no live peer: the journal is the peer.
//           Inbound entries are delivered at their recorded poll index; sends go
//           nowhere but are compared against the journal, so the outbound stream
//           doubles as a determinism check and the first divergence is reported.
//
// Delivery is keyed by poll index, not the clock: replaying a session alongside
// the time and input recordings of the same run reproduces the identical poll
// sequence, which reproduces the timing.
#pragma once

#include "INetworkTransport.h"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <span>
#include <string>
#include <vector>

class ReplayTransport : public INetworkTransport {
public:
    enum class Mode { NONE, RECORD, PLAY };

    // PLAY takes no inner transport (pass nullptr); every other mode requires one.
    ReplayTransport(std::unique_ptr<INetworkTransport> inner, Mode mode,
                    const std::filesystem::path& filepath);
    ~ReplayTransport() override { shutdown(); }

    bool startServer(std::uint16_t port) override;
    bool connect(const std::string& ip, std::uint16_t port) override;
    void poll() override;
    void send(ConnectionId connection, std::span<const std::byte> data,
              bool reliable) override;
    std::vector<Event> drainEvents() override;
    std::vector<Message> drainMessages() override;
    bool isServer() const override;
    void shutdown() override;

private:
    enum class EntryKind : std::uint8_t { Connected = 0, Disconnected = 1, Inbound = 2, Sent = 3 };

    // Journal layout: a role byte, then entries of
    //   {u64 pollIndex, u8 kind, u64 connection, u8 reliable, u32 size, payload}.
    // The payload is the message bytes; for connection events, the detail text.
    struct Entry {
        std::uint64_t m_pollIndex{0};
        EntryKind m_kind{EntryKind::Inbound};
        ConnectionId m_connection{0};
        bool m_reliable{false};
        std::vector<std::byte> m_payload{};
    };

    void writeEntry(EntryKind kind, ConnectionId connection, bool reliable,
                    std::span<const std::byte> payload);
    void loadJournal();
    void playPoll();

    Mode m_mode;
    std::unique_ptr<INetworkTransport> m_inner;
    std::fstream m_file{};
    bool m_isServer{false};  // PLAY: the recorded role; other modes ask the inner transport
    std::uint64_t m_pollIndex{0};

    std::vector<Event> m_events{};
    std::vector<Message> m_messages{};

    // PLAY: the whole journal split by direction, consumed by cursor.
    std::vector<Entry> m_inbound{};
    std::vector<Entry> m_sent{};
    std::size_t m_inboundCursor{0};
    std::size_t m_sentCursor{0};
    bool m_divergenceReported{false};
    bool m_endReported{false};
};
