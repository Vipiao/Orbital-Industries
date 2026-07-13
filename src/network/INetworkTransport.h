// INetworkTransport.h — abstract network transport. Game code talks only to this
// interface; backend-specific types (e.g. Valve GameNetworkingSockets) must never
// appear here, so the backend can be swapped without touching callers.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

class INetworkTransport {
public:
    // Opaque connection handle. A client has exactly one; a server gets one per
    // remote client. 0 is reserved as "invalid".
    using ConnectionId = std::uint64_t;

    enum class EventType { Connected, Disconnected };

    struct Event {
        EventType m_type{EventType::Connected};
        ConnectionId m_connection{0};
        std::string m_detail{};  // human-readable description, for logging only
    };

    struct Message {
        ConnectionId m_connection{0};
        std::vector<std::byte> m_data{};
    };

    virtual ~INetworkTransport() = default;

    // Exactly one of these is called, once, before the first poll().
    virtual bool startServer(std::uint16_t port) = 0;
    virtual bool connect(const std::string& ip, std::uint16_t port) = 0;

    // Called once per frame: pumps the backend and refills the queues below.
    virtual void poll() = 0;

    virtual void send(ConnectionId connection, std::span<const std::byte> data,
                      bool reliable) = 0;

    // Return everything accumulated since the last call and clear the queue.
    virtual std::vector<Event> drainEvents() = 0;
    virtual std::vector<Message> drainMessages() = 0;

    virtual bool isServer() const = 0;

    // Closes all connections and the listen socket; idempotent; the destructor
    // of an implementation must call it.
    virtual void shutdown() = 0;
};
