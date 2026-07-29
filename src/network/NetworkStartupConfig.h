// NetworkStartupConfig.h — role and address chosen at startup, before the game runs.
#pragma once

#include <cstdint>
#include <string>

struct NetworkStartupConfig {
    enum class Role { Server, Client };

    Role m_role{Role::Server};
    std::string m_ip{"127.0.0.1"};  // client only: server address to connect to
    std::uint16_t m_port{27020};

    // Testing knob: hold inbound messages back by this many milliseconds to
    // simulate latency on localhost. 0 disables the simulation entirely.
    double m_simulatedDelayMs{50.0};

    // Testing knob: add up to this many milliseconds of random jitter on top of
    // the simulated delay so arrivals are not perfectly spaced.
    double m_simulatedJitterMs{20.0};

    // Testing knob: render a debug sphere at the client's prediction anchor (the
    // server's version of the locally controlled character).
    static constexpr bool s_showPredictionAnchor{true};
};
