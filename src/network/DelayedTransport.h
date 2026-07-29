// DelayedTransport.h — an INetworkTransport decorator that holds inbound messages
// back by a configurable delay (± jitter, with an optional drop rate) so latency
// can be exercised on localhost. Channel semantics survive the simulation: jitter
// never reorders the reliable stream of a connection, and only unreliable messages
// drop. Backend-agnostic and seeded, so a run is reproducible. Connection events
// pass through immediately; only messages are delayed, which is what latency
// actually affects.
#pragma once

#include "INetworkTransport.h"

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <random>
#include <vector>

class DelayedTransport : public INetworkTransport {
public:
    DelayedTransport(std::unique_ptr<INetworkTransport> inner, double delayMs,
                     double jitterMs = 0.0, double dropRate = 0.0,
                     std::uint32_t seed = 1u);

    bool startServer(std::uint16_t port) override { return m_inner->startServer(port); }
    bool connect(const std::string& ip, std::uint16_t port) override {
        return m_inner->connect(ip, port);
    }
    void poll() override;
    void send(ConnectionId connection, std::span<const std::byte> data,
              bool reliable) override {
        m_inner->send(connection, data, reliable);
    }
    std::vector<Event> drainEvents() override { return m_inner->drainEvents(); }
    std::vector<Message> drainMessages() override;
    bool isServer() const override { return m_inner->isServer(); }
    void shutdown() override { m_inner->shutdown(); }

private:
    using Clock = std::chrono::steady_clock;

    struct Held {
        Clock::time_point m_releaseAt{};
        Message m_message{};
    };

    std::unique_ptr<INetworkTransport> m_inner;
    double m_delayMs;
    double m_jitterMs;
    double m_dropRate;
    std::mt19937 m_rng;
    std::vector<Held> m_held{};
    // Newest scheduled release per connection's reliable stream; later reliable
    // messages never release earlier (jitter must not reorder that channel).
    std::map<ConnectionId, Clock::time_point> m_lastReliableRelease{};
};
