// GnsTransport.h — INetworkTransport backed by Valve GameNetworkingSockets (the
// open-source standalone library). All Valve types live behind a pimpl in the .cpp.
#pragma once

#include "INetworkTransport.h"

#include <memory>

class GnsTransport : public INetworkTransport {
public:
    // Initializes the GameNetworkingSockets library; throws std::runtime_error on
    // failure. Only one instance may exist at a time (global library init and a
    // static callback trampoline).
    GnsTransport();
    ~GnsTransport() override;

    GnsTransport(const GnsTransport&) = delete;
    GnsTransport& operator=(const GnsTransport&) = delete;

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
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
