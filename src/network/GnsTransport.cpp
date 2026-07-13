#include "GnsTransport.h"

#include <steam/isteamnetworkingutils.h>
#include <steam/steamnetworkingsockets.h>

#include <algorithm>
#include <cassert>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <utility>

struct GnsTransport::Impl {
    ISteamNetworkingSockets* m_interface{nullptr};
    HSteamListenSocket m_listenSocket{k_HSteamListenSocket_Invalid};
    HSteamNetPollGroup m_pollGroup{k_HSteamNetPollGroup_Invalid};
    HSteamNetConnection m_clientConnection{k_HSteamNetConnection_Invalid};
    std::vector<HSteamNetConnection> m_acceptedConnections{};
    bool m_isServer{false};
    std::vector<Event> m_events{};
    std::vector<Message> m_messages{};

    // GNS delivers status changes through a plain function pointer with no user
    // data, so the trampoline reaches the instance through this static. It limits
    // the process to one live GnsTransport, which the constructor enforces.
    static Impl* s_callbackInstance;

    static void onStatusChanged(SteamNetConnectionStatusChangedCallback_t* info) {
        s_callbackInstance->handleStatusChanged(info);
    }

    void handleStatusChanged(SteamNetConnectionStatusChangedCallback_t* info) {
        switch (info->m_info.m_eState) {
        case k_ESteamNetworkingConnectionState_Connecting:
            // Incoming connection on the server; a client's own outgoing attempt
            // also passes through this state and needs no action.
            if (m_isServer &&
                info->m_info.m_hListenSocket != k_HSteamListenSocket_Invalid) {
                if (m_interface->AcceptConnection(info->m_hConn) != k_EResultOK) {
                    m_interface->CloseConnection(info->m_hConn, 0, nullptr, false);
                    break;
                }
                m_interface->SetConnectionPollGroup(info->m_hConn, m_pollGroup);
                m_acceptedConnections.push_back(info->m_hConn);
            }
            break;

        case k_ESteamNetworkingConnectionState_Connected:
            m_events.push_back(Event{EventType::Connected,
                                     ConnectionId{info->m_hConn},
                                     std::string{info->m_info.m_szConnectionDescription}});
            break;

        case k_ESteamNetworkingConnectionState_ClosedByPeer:
        case k_ESteamNetworkingConnectionState_ProblemDetectedLocally:
            // Only report connections the game ever saw as live or pending; then
            // always close to free the handle.
            if (info->m_eOldState == k_ESteamNetworkingConnectionState_Connected ||
                info->m_eOldState == k_ESteamNetworkingConnectionState_Connecting) {
                m_events.push_back(Event{EventType::Disconnected,
                                         ConnectionId{info->m_hConn},
                                         std::string{info->m_info.m_szEndDebug}});
            }
            forgetConnection(info->m_hConn);
            m_interface->CloseConnection(info->m_hConn, 0, nullptr, false);
            break;

        default:
            break;
        }
    }

    void forgetConnection(HSteamNetConnection connection) {
        std::erase(m_acceptedConnections, connection);
        if (m_clientConnection == connection) {
            m_clientConnection = k_HSteamNetConnection_Invalid;
        }
    }

    void receiveInto(std::vector<Message>& out, bool fromPollGroup) {
        while (true) {
            ISteamNetworkingMessage* incoming{nullptr};
            int count{fromPollGroup
                          ? m_interface->ReceiveMessagesOnPollGroup(m_pollGroup,
                                                                    &incoming, 1)
                          : m_interface->ReceiveMessagesOnConnection(m_clientConnection,
                                                                     &incoming, 1)};
            if (count <= 0) {
                break;
            }
            Message message{ConnectionId{incoming->m_conn},
                            std::vector<std::byte>(incoming->m_cbSize)};
            std::memcpy(message.m_data.data(), incoming->m_pData,
                        static_cast<std::size_t>(incoming->m_cbSize));
            out.push_back(std::move(message));
            incoming->Release();
        }
    }
};

GnsTransport::Impl* GnsTransport::Impl::s_callbackInstance{nullptr};

namespace {

void debugOutput(ESteamNetworkingSocketsDebugOutputType type, const char* message) {
    (void)type;
    std::cout << "[gns] " << message << std::endl;
}

}  // namespace

GnsTransport::GnsTransport() : m_impl{std::make_unique<Impl>()} {
    if (Impl::s_callbackInstance != nullptr) {
        throw std::runtime_error{"Only one GnsTransport instance is supported"};
    }
    SteamDatagramErrMsg errMsg;
    if (!GameNetworkingSockets_Init(nullptr, errMsg)) {
        throw std::runtime_error{std::string{"GameNetworkingSockets_Init failed: "} +
                                 errMsg};
    }
    SteamNetworkingUtils()->SetDebugOutputFunction(
        k_ESteamNetworkingSocketsDebugOutputType_Important, debugOutput);
    m_impl->m_interface = SteamNetworkingSockets();
    Impl::s_callbackInstance = m_impl.get();
}

GnsTransport::~GnsTransport() {
    shutdown();
    Impl::s_callbackInstance = nullptr;
    GameNetworkingSockets_Kill();
}

bool GnsTransport::startServer(std::uint16_t port) {
    // Exactly one of startServer/connect, exactly once.
    assert(m_impl->m_listenSocket == k_HSteamListenSocket_Invalid &&
           m_impl->m_clientConnection == k_HSteamNetConnection_Invalid);
    m_impl->m_isServer = true;
    SteamNetworkingIPAddr address{};
    address.Clear();
    address.m_port = port;
    SteamNetworkingConfigValue_t options{};
    options.SetPtr(k_ESteamNetworkingConfig_Callback_ConnectionStatusChanged,
                   reinterpret_cast<void*>(&Impl::onStatusChanged));
    m_impl->m_listenSocket = m_impl->m_interface->CreateListenSocketIP(address, 1,
                                                                       &options);
    if (m_impl->m_listenSocket == k_HSteamListenSocket_Invalid) {
        std::cerr << "Failed to listen on port " << port << std::endl;
        return false;
    }
    m_impl->m_pollGroup = m_impl->m_interface->CreatePollGroup();
    std::cout << "Listening on port " << port << std::endl;
    return true;
}

bool GnsTransport::connect(const std::string& ip, std::uint16_t port) {
    // Exactly one of startServer/connect, exactly once.
    assert(m_impl->m_listenSocket == k_HSteamListenSocket_Invalid &&
           m_impl->m_clientConnection == k_HSteamNetConnection_Invalid);
    m_impl->m_isServer = false;
    SteamNetworkingIPAddr address{};
    address.Clear();
    if (!address.ParseString(ip.c_str())) {
        std::cerr << "Invalid IP address: " << ip << std::endl;
        return false;
    }
    address.m_port = port;
    SteamNetworkingConfigValue_t options{};
    options.SetPtr(k_ESteamNetworkingConfig_Callback_ConnectionStatusChanged,
                   reinterpret_cast<void*>(&Impl::onStatusChanged));
    m_impl->m_clientConnection = m_impl->m_interface->ConnectByIPAddress(address, 1,
                                                                         &options);
    if (m_impl->m_clientConnection == k_HSteamNetConnection_Invalid) {
        std::cerr << "Failed to start connecting to " << ip << ":" << port << std::endl;
        return false;
    }
    std::cout << "Connecting to " << ip << ":" << port << "..." << std::endl;
    return true;
}

void GnsTransport::poll() {
    m_impl->m_interface->RunCallbacks();
    if (m_impl->m_isServer) {
        if (m_impl->m_pollGroup != k_HSteamNetPollGroup_Invalid) {
            m_impl->receiveInto(m_impl->m_messages, true);
        }
    } else if (m_impl->m_clientConnection != k_HSteamNetConnection_Invalid) {
        m_impl->receiveInto(m_impl->m_messages, false);
    }
}

void GnsTransport::send(ConnectionId connection, std::span<const std::byte> data,
                        bool reliable) {
    assert(connection != ConnectionId{0});
    m_impl->m_interface->SendMessageToConnection(
        static_cast<HSteamNetConnection>(connection), data.data(),
        static_cast<uint32>(data.size()),
        reliable ? k_nSteamNetworkingSend_Reliable : k_nSteamNetworkingSend_Unreliable,
        nullptr);
}

std::vector<INetworkTransport::Event> GnsTransport::drainEvents() {
    return std::exchange(m_impl->m_events, {});
}

std::vector<INetworkTransport::Message> GnsTransport::drainMessages() {
    return std::exchange(m_impl->m_messages, {});
}

bool GnsTransport::isServer() const {
    return m_impl->m_isServer;
}

void GnsTransport::shutdown() {
    for (HSteamNetConnection connection : m_impl->m_acceptedConnections) {
        m_impl->m_interface->CloseConnection(connection, 0, "shutdown", true);
    }
    m_impl->m_acceptedConnections.clear();
    if (m_impl->m_clientConnection != k_HSteamNetConnection_Invalid) {
        m_impl->m_interface->CloseConnection(m_impl->m_clientConnection, 0, "shutdown",
                                             true);
        m_impl->m_clientConnection = k_HSteamNetConnection_Invalid;
    }
    if (m_impl->m_listenSocket != k_HSteamListenSocket_Invalid) {
        m_impl->m_interface->CloseListenSocket(m_impl->m_listenSocket);
        m_impl->m_listenSocket = k_HSteamListenSocket_Invalid;
    }
    if (m_impl->m_pollGroup != k_HSteamNetPollGroup_Invalid) {
        m_impl->m_interface->DestroyPollGroup(m_impl->m_pollGroup);
        m_impl->m_pollGroup = k_HSteamNetPollGroup_Invalid;
    }
}
