// NetworkStartupConfig.h — role and address chosen at startup, before the game runs.
#pragma once

#include <cstdint>
#include <string>

struct NetworkStartupConfig {
    enum class Role { Server, Client };

    Role m_role{Role::Server};
    std::string m_ip{"127.0.0.1"};  // client only: server address to connect to
    std::uint16_t m_port{27020};
};
