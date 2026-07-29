#include "ReplayTransport.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <stdexcept>

namespace {

// A self-produced journal never holds entries near this size; a larger count is a
// corrupt or truncated file, not data.
constexpr std::uint32_t s_maxEntryBytes{16u * 1024u * 1024u};

template <typename T>
void writeRaw(std::fstream& file, const T& value) {
    file.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

template <typename T>
bool readRaw(std::fstream& file, T& value) {
    file.read(reinterpret_cast<char*>(&value), sizeof(T));
    return file.good();
}

}  // namespace

ReplayTransport::ReplayTransport(std::unique_ptr<INetworkTransport> inner, Mode mode,
                                 const std::filesystem::path& filepath)
    : m_mode{mode}, m_inner{std::move(inner)} {
    // The journal stands in for the peer in PLAY; every other mode wraps a real one.
    assert((m_mode == Mode::PLAY) == (m_inner == nullptr));
    if (m_mode == Mode::NONE) {
        return;
    }
    if (m_mode == Mode::RECORD && !filepath.parent_path().empty()) {
        std::filesystem::create_directories(filepath.parent_path());
    }
    m_file.open(filepath, (m_mode == Mode::RECORD ? std::ios::out | std::ios::trunc
                                                  : std::ios::in) |
                              std::ios::binary);
    if (!m_file.is_open()) {
        throw std::runtime_error("Failed to open network journal: " + filepath.string());
    }
    if (m_mode == Mode::PLAY) {
        loadJournal();
        m_file.close();
        std::cout << "[replay] loaded " << (m_isServer ? "server" : "client")
                  << " journal: " << m_inbound.size() << " inbound, " << m_sent.size()
                  << " sent entries" << std::endl;
    }
}

bool ReplayTransport::startServer(std::uint16_t port) {
    if (m_mode == Mode::PLAY) {
        if (!m_isServer) {
            std::cout << "[replay] journal is a client recording, not a server's"
                      << std::endl;
            return false;
        }
        return true;
    }
    bool ok{m_inner->startServer(port)};
    if (m_mode == Mode::RECORD && ok) {
        writeRaw(m_file, std::uint8_t{1});
    }
    return ok;
}

bool ReplayTransport::connect(const std::string& ip, std::uint16_t port) {
    if (m_mode == Mode::PLAY) {
        if (m_isServer) {
            std::cout << "[replay] journal is a server recording, not a client's"
                      << std::endl;
            return false;
        }
        return true;
    }
    bool ok{m_inner->connect(ip, port)};
    if (m_mode == Mode::RECORD && ok) {
        writeRaw(m_file, std::uint8_t{0});
    }
    return ok;
}

void ReplayTransport::poll() {
    if (m_mode == Mode::NONE) {
        m_inner->poll();
        return;
    }
    m_pollIndex++;
    if (m_mode == Mode::PLAY) {
        playPoll();
        return;
    }
    m_inner->poll();
    for (Event& event : m_inner->drainEvents()) {
        EntryKind kind{event.m_type == EventType::Connected ? EntryKind::Connected
                                                            : EntryKind::Disconnected};
        writeEntry(kind, event.m_connection, false,
                   std::as_bytes(std::span{event.m_detail.data(), event.m_detail.size()}));
        m_events.push_back(std::move(event));
    }
    for (Message& message : m_inner->drainMessages()) {
        writeEntry(EntryKind::Inbound, message.m_connection, message.m_reliable,
                   message.m_data);
        m_messages.push_back(std::move(message));
    }
}

void ReplayTransport::playPoll() {
    while (m_inboundCursor < m_inbound.size() &&
           m_inbound[m_inboundCursor].m_pollIndex <= m_pollIndex) {
        Entry& entry{m_inbound[m_inboundCursor]};
        if (entry.m_kind == EntryKind::Inbound) {
            m_messages.push_back(
                Message{entry.m_connection, std::move(entry.m_payload), entry.m_reliable});
        } else {
            Event event{entry.m_kind == EntryKind::Connected ? EventType::Connected
                                                             : EventType::Disconnected,
                        entry.m_connection, std::string{}};
            event.m_detail.assign(reinterpret_cast<const char*>(entry.m_payload.data()),
                                  entry.m_payload.size());
            m_events.push_back(std::move(event));
        }
        m_inboundCursor++;
    }
    if (m_inboundCursor >= m_inbound.size() && !m_endReported) {
        std::cout << "[replay] journal exhausted; no further network input" << std::endl;
        m_endReported = true;
    }
}

void ReplayTransport::send(ConnectionId connection, std::span<const std::byte> data,
                           bool reliable) {
    if (m_mode == Mode::NONE) {
        m_inner->send(connection, data, reliable);
        return;
    }
    if (m_mode == Mode::RECORD) {
        writeEntry(EntryKind::Sent, connection, reliable, data);
        m_inner->send(connection, data, reliable);
        return;
    }
    // PLAY: nothing is sent; the outbound stream is a determinism check against the
    // recording. Only the first divergence is meaningful — after it everything
    // downstream differs — so checking stops there.
    if (m_divergenceReported) {
        return;
    }
    if (m_sentCursor >= m_sent.size()) {
        std::cout << "[replay] send diverged at poll " << m_pollIndex
                  << ": more sends than recorded" << std::endl;
        m_divergenceReported = true;
        return;
    }
    const Entry& expected{m_sent[m_sentCursor]};
    m_sentCursor++;
    bool matches{expected.m_pollIndex == m_pollIndex &&
                 expected.m_connection == connection && expected.m_reliable == reliable &&
                 expected.m_payload.size() == data.size() &&
                 std::equal(data.begin(), data.end(), expected.m_payload.begin())};
    if (!matches) {
        std::cout << "[replay] send diverged at poll " << m_pollIndex << " (recorded poll "
                  << expected.m_pollIndex << ", " << expected.m_payload.size() << " vs "
                  << data.size() << " bytes)" << std::endl;
        m_divergenceReported = true;
    }
}

std::vector<INetworkTransport::Event> ReplayTransport::drainEvents() {
    if (m_mode == Mode::NONE) {
        return m_inner->drainEvents();
    }
    std::vector<Event> events{std::move(m_events)};
    m_events.clear();
    return events;
}

std::vector<INetworkTransport::Message> ReplayTransport::drainMessages() {
    if (m_mode == Mode::NONE) {
        return m_inner->drainMessages();
    }
    std::vector<Message> messages{std::move(m_messages)};
    m_messages.clear();
    return messages;
}

bool ReplayTransport::isServer() const {
    return m_mode == Mode::PLAY ? m_isServer : m_inner->isServer();
}

void ReplayTransport::shutdown() {
    if (m_inner) {
        m_inner->shutdown();
    }
    if (m_file.is_open()) {
        m_file.close();
    }
}

void ReplayTransport::writeEntry(EntryKind kind, ConnectionId connection, bool reliable,
                                 std::span<const std::byte> payload) {
    writeRaw(m_file, m_pollIndex);
    writeRaw(m_file, static_cast<std::uint8_t>(kind));
    writeRaw(m_file, connection);
    writeRaw(m_file, static_cast<std::uint8_t>(reliable ? 1 : 0));
    writeRaw(m_file, static_cast<std::uint32_t>(payload.size()));
    if (!payload.empty()) {
        m_file.write(reinterpret_cast<const char*>(payload.data()),
                     static_cast<std::streamsize>(payload.size()));
    }
    m_file.flush();
}

void ReplayTransport::loadJournal() {
    std::uint8_t role{0};
    if (!readRaw(m_file, role)) {
        std::cout << "[replay] journal is empty" << std::endl;
        return;
    }
    m_isServer = role != 0;
    while (true) {
        Entry entry{};
        std::uint8_t kind{0};
        std::uint8_t reliable{0};
        std::uint32_t size{0};
        if (!readRaw(m_file, entry.m_pollIndex)) {
            break;  // clean end of journal
        }
        if (!readRaw(m_file, kind) || kind > static_cast<std::uint8_t>(EntryKind::Sent) ||
            !readRaw(m_file, entry.m_connection) || !readRaw(m_file, reliable) ||
            !readRaw(m_file, size) || size > s_maxEntryBytes) {
            std::cout << "[replay] journal truncated or corrupt; keeping "
                      << m_inbound.size() + m_sent.size() << " entries" << std::endl;
            break;
        }
        entry.m_payload.resize(size);
        if (size > 0 && !m_file.read(reinterpret_cast<char*>(entry.m_payload.data()),
                                     static_cast<std::streamsize>(size))) {
            std::cout << "[replay] journal truncated mid-entry; keeping "
                      << m_inbound.size() + m_sent.size() << " entries" << std::endl;
            break;
        }
        entry.m_kind = static_cast<EntryKind>(kind);
        entry.m_reliable = reliable != 0;
        (entry.m_kind == EntryKind::Sent ? m_sent : m_inbound).push_back(std::move(entry));
    }
}
